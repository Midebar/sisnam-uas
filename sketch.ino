#include <Wire.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>

#if defined(ESP32)
  #include "freertos/FreeRTOS.h"
  #include "freertos/task.h"
  #include "freertos/semphr.h"
#else
  #include <Arduino_FreeRTOS.h>
  #include <semphr.h>
#endif

// ===== Message Struct =====

// T7-K2: Intelligent Visitor Management System (IVMS)
typedef struct __attribute__((packed)) {
  char uid[32];
  char card_event[8];   // "ENTER"|"EXIT"|"RE-ENTER"|"RE-EXIT"|"UNKNOWN"
  char place[8];        // Last entry location
  uint8_t authorized;   // bool
  uint32_t utc_time;    // Last entry time
  uint8_t isLoopReport; // bool
  int32_t duration;     // Stay duration
} IVMSPacket;

// T8-K5: Smart Automated Door & Distance Monitoring System (SAD-DMS)
typedef struct __attribute__((packed)) {
  char dist_event[7];     // "OPEN" | "CLOSED"
  float distance_cm;  // -1 kalau gagal baca
  uint8_t dist_code; // 0=none, 1=OPENED, 2=CLOSED
  uint32_t ts_dist;     // timestamp = millis()
} SADDMSPacket;

// T9-K11: Smart Touchless Access Control System (STACS)
typedef struct {
  int id_kelompok;
  char hand_event[20];  // "CLOSED" | "OPEN"
  int total_akses;  // X times the door has been opened   
} STACSPacket;

// ===== LCD =====
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);
volatile bool displayInterrupt = false;

// LCD Init Helper -- Hard-Coded Specs
void initLCD() {
  Wire.begin(21, 22);   // 21 to SDA, 22 to SCL
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("ESP32 READY");

  delay(500);
}

void scrollText(const char* text, int col = 0, int row = 0, int maxLen = 16, int scrollDelay = 150) {
  int len = strlen(text);
  if (len <= maxLen) {
    lcd.setCursor(col, row);
    lcd.print("                ");
    lcd.setCursor(col, row);
    lcd.print(text);
    vTaskDelay(pdMS_TO_TICKS(scrollDelay));
    return;
  }

  for (int offset = 0; offset <= len - maxLen; offset++) {
    lcd.setCursor(col, row);
    for (int slot = 0; slot < maxLen; slot++) {
      lcd.print(text[offset + slot]);
    }
    vTaskDelay(pdMS_TO_TICKS(scrollDelay));
    if (displayInterrupt) return;
  }

  for (int slot = 0; slot < maxLen; slot++) {
    if (displayInterrupt) return;
    lcd.print(text[slot]);
  }

  if (displayInterrupt) return;
  lcd.setCursor(col, row);
  lcd.print("                ");
  if (displayInterrupt) return;
  lcd.setCursor(col, row);
  lcd.print(text);
  vTaskDelay(pdMS_TO_TICKS(scrollDelay));
}

void setupTimeUTC() {
  configTime(3600*7, 0, "pool.ntp.org", "time.nist.gov");

  Serial.print("Syncing time");
  time_t now;

  while (true) {
    time(&now);
    if (now > 1767225600) {   // ~2023+
      break;
    }
    Serial.print(".");
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }

  Serial.println("\nTime synced (UTC)");
}

uint64_t getTime() {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return ((uint64_t)tv.tv_sec + 3600 * 7) * 1000LL + (tv.tv_usec / 1000);
}

void formatTime(char* out, uint64_t ts_ms, size_t len) {
  uint64_t sec = ts_ms / 1000;
  uint64_t min = sec / 60;
  uint64_t hour = min / 60;

  sec %= 60;
  min %= 60;
  hour %= 24;

  snprintf(out, len, "%02llu:%02llu:%02llu", hour, min, sec);
}

// ===== Global =====
typedef struct {
  // T7-K2: Intelligent Visitor Management System (IVMS)
  char card_event[8];
  uint64_t ts_card;   // Last entry time (UTC Time)
  int32_t duration;   // Stay duration
  char place[8];      // Last entry location

  // T8-K5: Smart Automated Door & Distance Monitoring System (SAD-DMS)
  char dist_event[7];
  float distance_cm;
  uint64_t ts_dist;   // Last entry time millis()
  
  // T9-K11: Smart Touchless Access Control System (STACS)
  char hand_event[20];
  int total_akses;
  uint64_t ts_hand;   // Last entry time millis()

} SystemState;

SystemState currentState;
SemaphoreHandle_t xMutex;

// // ===== TEST =====
// IVMSPacket IVMSOut;
// SADDMSPacket SADDMSOut;
// STACSPacket STACSOut;
// // ===== END TEST =====

TaskHandle_t displayTaskHandle = NULL;

// For wokwi simulation
uint8_t selfMac[6];

// ===== RUNTIME HELPER FUNCTIONS =====

// // ===== TEST =====
// void safe_esp_now_send(const uint8_t *peer_addr, const uint8_t *data, size_t len) {
//     // Check if the destination MAC matches our own MAC
//     bool isSelf = true;
//     for(int i = 0; i < 6; i++) {
//         if(peer_addr[i] != selfMac[i]) {
//             isSelf = false;
//             break;
//         }
//     }

//     if (isSelf) {
//         // Bypass the radio and call the receiver function directly
//         OnDataRecv(NULL, data, len); 
//     } else {
//         // Send over the actual airwaves
//         esp_now_send(peer_addr, data, len);
//     }
// }
// // ===== END TEST =====

void OnDataRecv(const esp_now_recv_info *mac, const uint8_t *incomingData, int len) {
  Serial.println(len);
  displayInterrupt = true;
  if (displayTaskHandle != NULL) {
    xTaskNotifyGive(displayTaskHandle);
  }

  if (len == sizeof(IVMSPacket)) {
    IVMSPacket incoming;
    memcpy(&incoming, incomingData, sizeof(incoming));
    if (incoming.isLoopReport == 0) { // only grab when it's an instant notification
      if (xSemaphoreTake(xMutex, 0) == pdTRUE) {
        strcpy(currentState.card_event, incoming.card_event);
        currentState.ts_card = getTime(); // UTC-time is in seconds, want ms
        currentState.duration = incoming.duration;
        strcpy(currentState.place, incoming.place);
        xSemaphoreGive(xMutex);
      }
    }

  } else if (len == sizeof(SADDMSPacket)) {
    SADDMSPacket incoming;
    memcpy(&incoming, incomingData, sizeof(incoming));
    if (xSemaphoreTake(xMutex, 0) == pdTRUE) {
      strcpy(currentState.dist_event, incoming.dist_event);
      currentState.distance_cm = incoming.distance_cm;
      currentState.ts_dist = getTime();
      xSemaphoreGive(xMutex);
    }

  } else if (len == sizeof(STACSPacket)) {
    STACSPacket incoming;
    memcpy(&incoming, incomingData, sizeof(incoming));
    if (xSemaphoreTake(xMutex, 0) == pdTRUE) {
      strcpy(currentState.hand_event, incoming.hand_event);
      currentState.total_akses = incoming.total_akses;
      currentState.ts_hand = getTime();
      xSemaphoreGive(xMutex);
    }

  } else {
    Serial.println("⚠️ Unknown packet format!");
  }
}

// ===== RTOS =====
void TaskDisplayMain (void *pvParameters) {
  // Local var
  (void) pvParameters;

  int type = 0; // 1=IVMs | 2=SADDMS | 3=STACS
  int lastReceivedType = 0;
  SystemState localCopy;
  char tsAsChar[9];
  char longRow[64];    // long on purpose (for scrolling)

  localCopy = currentState; // initial

  for (;;) {
    displayInterrupt = false;
    if (xSemaphoreTake(xMutex, (TickType_t)100) == pdTRUE) {
      SystemState temp = localCopy;
      localCopy = currentState;

      if (strcmp(temp.card_event, localCopy.card_event) != 0) {
        type = 1;
      } else if (strcmp(temp.dist_event, localCopy.dist_event) != 0) {
        type = 2;
      } else if (strcmp(temp.hand_event, localCopy.hand_event) != 0) {
        type = 3;
      } 

      xSemaphoreGive(xMutex);
    }

    // LCD Cleanup
    lcd.setCursor(0, 0);
    lcd.print("                ");
    lcd.setCursor(0, 1);
    lcd.print("                ");

    switch (type) {
      // ===== IVMS =====
      case 1:
        lcd.setCursor(0, 0);
        lcd.print("IVMS:");
        lcd.print(localCopy.card_event);

        formatTime(tsAsChar, localCopy.ts_card, sizeof(tsAsChar));
        snprintf(
          longRow, sizeof(longRow),
          "AT %s %s UTC",
          // "AT %s %s DUR:%ds",
          localCopy.place,
          tsAsChar
          // tsAsChar,
          // localCopy.duration
        );
        scrollText(longRow, 0, 1, 16, 100);
        
        lastReceivedType = type;
        // Serial.println(lastReceivedType);
        type = 0;
        break;

      // ===== SAD-DMS =====
      case 2:
        lcd.setCursor(0, 0);
        lcd.print("SADDMS:");
        lcd.print(localCopy.dist_event);

        formatTime(tsAsChar, localCopy.ts_dist, sizeof(tsAsChar));

        snprintf(
          longRow, sizeof(longRow),
          "Distance: %dcm %s",
          (int)localCopy.distance_cm,
          tsAsChar
        );

        scrollText(longRow, 0, 1, 16, 100);
        
        lastReceivedType = type;
        // Serial.println(lastReceivedType);
        type = 0;
        break;

      // ===== STACS =====
      case 3:
        lcd.setCursor(0, 0);
        lcd.print("STACS:");
        lcd.print(localCopy.hand_event);

        formatTime(tsAsChar, localCopy.ts_hand, sizeof(tsAsChar));
        
        snprintf(
          longRow, sizeof(longRow),
          "Count: %d %s",
          localCopy.total_akses,
          tsAsChar
        );

        scrollText(longRow, 0, 1, 16, 100);
        
        lastReceivedType = type;
        // Serial.println(lastReceivedType);
        type = 0;
        break;

      default:
        lcd.setCursor(0, 0);
        lcd.print("WAITING DATA...");

        if (lastReceivedType != 0) {
          lcd.setCursor(0, 1);
          lcd.print("Last:");
          char tempLongRow[64];
          switch (lastReceivedType) {
            case 1:
              snprintf(
                tempLongRow, sizeof(tempLongRow),
                "IVMS | %s",
                longRow
              );
              break;
            case 2:
              snprintf(
                tempLongRow, sizeof(tempLongRow),
                "SADDMS | %s",
                longRow
              );
              break;
            case 3:
              snprintf(
                tempLongRow, sizeof(tempLongRow),
                "STACS | %s",
                longRow
              );
              break;
            default:
              strcpy(
                tempLongRow,
                "Unknown Pointer Error"
              );
              break;
          }

          scrollText(tempLongRow, 5, 1, 11, 100);
        }
        
        break;
    }

    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(666)); // dattebayo
  }
}

// ===== Main =====
void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  // Get MAC
  esp_wifi_get_mac(WIFI_IF_STA, selfMac);
  Serial.printf("My MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
    selfMac[0], selfMac[1], selfMac[2],
    selfMac[3], selfMac[4], selfMac[5]);

  // Init LCD
  initLCD();

  // Sync Time
  WiFi.begin("AdkA55", "boukendabouken");
  
  // unsigned long startAttempt = millis();
  // while ((WiFi.status() != WL_CONNECTED) && ((millis() - startAttempt) < 10000)) {
  //   delay(500);
  //   lcd.setCursor(0, 0);
  //   lcd.print("Connecting WIFI ");
  // }

  // if (WiFi.status() != WL_CONNECTED) {
  //   // OFFLINE: Force clock to 0
  //   struct timeval tv = { .tv_sec = 0, .tv_usec = 0 };
  //   settimeofday(&tv, NULL);
  //   Serial.println("NTP Failed: Time set to 00:00:00");
  // } else {
  //   // ONLINE: Sync with NTP
  //   lcd.setCursor(0, 0);
  //   lcd.print("Syncing Time... ");
  //   setupTimeUTC();
  //   WiFi.disconnect(true);
  // }

  // Sync Time
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi");

  setupTimeUTC();
  setenv("TZ", "WIB-7", 1);
  tzset();

  WiFi.disconnect(true);
  Serial.println("Wifi disconnected!");
  
  WiFi.mode(WIFI_STA);
  delay(500);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

  // // ===== TEST =====
  // // Add self as peer
  // esp_now_peer_info_t peerInfo = {};
  // memcpy(peerInfo.peer_addr, selfMac, 6);
  // peerInfo.channel = 0;
  // peerInfo.encrypt = false;

  // if (esp_now_add_peer(&peerInfo) != ESP_OK) {
  //   Serial.println("Failed to add self as peer");
  //   return;
  // }
  // // ===== END TEST =====

  // Default Global State
  // T7-K2: Intelligent Visitor Management System (IVMS)
  strcpy(currentState.card_event, "RE-ENTER");
  currentState.ts_card = 0;   // Last entry time (UTC Time)
  currentState.duration = 0;   // Stay duration
  strcpy(currentState.place, "HOME");      // Last entry location

  // T8-K5: Smart Automated Door & Distance Monitoring System (SAD-DMS)
  strcpy(currentState.dist_event, "CLOSED");
  currentState.distance_cm = 0;
  currentState.ts_dist = millis();   // Last entry time millis()
  
  // T9-K11: Smart Touchless Access Control System (STACS)
  strcpy(currentState.hand_event, "CLOSED");
  currentState.total_akses = 0;
  currentState.ts_hand = millis();   // Last entry time millis()

  // Task Init
  xMutex = xSemaphoreCreateMutex();

  // xTaskCreate(TaskDisplayMain, "DisplayMain", 2048, NULL, 1, NULL);
  xTaskCreate(
    TaskDisplayMain,
    "DisplayMain",
    2048,
    NULL,
    1,
    &displayTaskHandle
  );

}

void loop() {
  // // ===== TEST =====
  // delay(4267);

  // // Mockup T7-K2: Intelligent Visitor Management System (IVMS)
  // strcpy(IVMSOut.uid, "A1B2C3D4");
  // strcpy(IVMSOut.card_event, (strcmp(currentState.card_event, "ENTER") == 0) ? "EXIT" : "ENTER");
  // strcpy(IVMSOut.place, (strcmp(currentState.place, "HOME") == 0) ? "GATE" : "HOME");
  // IVMSOut.authorized = 1;
  // IVMSOut.utc_time = millis() / 1000;   // fake UTC
  // IVMSOut.isLoopReport = 0;              // instant notification
  // IVMSOut.duration = currentState.duration + 1;

  // safe_esp_now_send(selfMac, (uint8_t*)&IVMSOut, sizeof(IVMSOut));
  // delay(4267);
   
  // // Mockup T8-K5: Smart Automated Door & Distance Monitoring System (SAD-DMS)
  // strcpy(SADDMSOut.dist_event, (strcmp(currentState.dist_event, "OPEN") == 0) ? "CLOSED" : "OPEN");
  // SADDMSOut.distance_cm = currentState.distance_cm + 1; 
  // SADDMSOut.dist_code = (strcmp(SADDMSOut.dist_event, "OPEN") == 0) ? 1 : 2; // 0=none, 1=OPENED, 2=CLOSED
  // SADDMSOut.ts_dist = millis();  // timestamp = millis()

  // safe_esp_now_send(selfMac, (uint8_t*)&SADDMSOut, sizeof(SADDMSOut));
  // delay(4267);

  // // Mockup T9-K11: Smart Touchless Access Control System (STACS)
  // STACSOut.id_kelompok = 9;
  // strcpy(STACSOut.hand_event, (strcmp(currentState.hand_event, "OPEN") == 0) ? "CLOSED" : "OPEN");
  // STACSOut.total_akses = currentState.total_akses + 1;  // X times the door has been opened
  
  // safe_esp_now_send(selfMac, (uint8_t*)&STACSOut, sizeof(STACSOut));
  // // ===== END TEST =====
}