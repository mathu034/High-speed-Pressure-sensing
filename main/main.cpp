#include <Wire.h>
#include <esp_now.h>
#include <WiFi.h>
#include "TE_SM9000_series_1.2.1.h"
#include "Arduino.h"

extern "C" {
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
}

#define TCAADDR 0x70
#define SDA 20
#define SCL 21

static uint32_t total_reading=0;
//static uint8_t sensor_count = 19;
TimerHandle_t one_sec_timer;

SM9000_sensor T9333(-125, 125, -26215, 26214);

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
    uint8_t id;
    float pressure;
} struct_message;

// Create a struct_message called myData
struct_message myData;

// Peer info
esp_now_peer_info_t peerInfo;

// Replace with the MAC Address of your receiver
uint8_t broadcastAddress[] = {0x40, 0x4C, 0xCA, 0x44, 0xE0, 0x4C}; // Update with the actual MAC address

//40:4C:CA:44:E0:4C

extern "C"{
    void app_main(void);    
    void pressure_sensor(void);
    void myTimerCallback(TimerHandle_t xTimer);
    void tcaselect(uint8_t sensor_num);
}

void tcaselect(uint8_t bus) {
  const byte TCA_BASE_ADDR = 0x70;

  byte mux_index = bus / 8;
  byte mux_channel = bus % 8;

  byte address = TCA_BASE_ADDR + mux_index;

  static byte previousAddress = 0xFF;
  if (address != previousAddress) {
    if (previousAddress != 0xFF) {
      Wire.beginTransmission(previousAddress);
      Wire.write(0);
      Wire.endTransmission();
    }
    previousAddress = address;
  }

  Wire.beginTransmission(address);
  Wire.write(1 << mux_channel);
  Wire.endTransmission();
}

void myTimerCallback(TimerHandle_t xTimer) {
  if ((uint32_t)pvTimerGetTimerID(xTimer) == 1) {
    Serial.println("ZERO");
    Serial.println(total_reading);
    
    total_reading=0;
  }
}

void pressure_sensor(void * pvParameters){
  for(;;) {
    for(uint8_t sensor_num=0; sensor_num<16; sensor_num++){
        total_reading++;      //total readings       
        tcaselect(sensor_num); 
        
        T9333.readData();
       // Serial.print("Sensor ");
       // Serial.print(sensor_num);
       // Serial.print(" = ");
        //Serial.println(T9333.getPressure());
        float pressure = T9333.getPressure();
        
        // Prepare data to send
        myData.id = sensor_num;
        myData.pressure = pressure;
        
        // Send message via ESP-NOW
        esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
      
        if (result == ESP_OK) {
         //   Serial.println("Sent with success");
        } else {
         //   Serial.println("Error sending the data");
        }
    }
   // vTaskDelay(1000/portTICK_PERIOD_MS);
  }
}

void setupEspNow() {
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
      Serial.println("Error initializing ESP-NOW");
      return;
  }
  Serial.println("ESP-NOW Initialized");

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
      Serial.println("Failed to add peer");
      return;
  }
}

void app_main(void){
  initArduino();
  Serial.begin(115200);
  Wire.begin(SDA, SCL, 400000);

  setupEspNow();
    
  TaskHandle_t xHandle = NULL;

  one_sec_timer = xTimerCreate("One second timer", 1000 / portTICK_PERIOD_MS, pdTRUE,(void *)1,myTimerCallback);
  if(one_sec_timer==NULL){
    Serial.println("Could not create the timer");
  }
  else{
    Serial.println("Timer started");
    xTimerStart(one_sec_timer, portMAX_DELAY);
  }

  xTaskCreatePinnedToCore(pressure_sensor, "Read pressure",4096, (void *)1, 1, &xHandle, 0);
}
