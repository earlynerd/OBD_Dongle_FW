#include <Arduino.h>
#include <ESP32CAN.h>
#include <CAN_config.h>
#include <SPI.h>
#include <SD.h>

#define SD_MOSI 23
#define SD_MISO 19
#define SD_SCK 18
#define SD_CS 16
#define SD_CD 17

#define CAN_RX 4
#define CAN_TX 5
#define CAN_SLNT 22

#define PWR_FAULT 21
#define VBAT_DIV 27 // 40.2k upper, 10k lower

CAN_device_t CAN_cfg; // CAN Config
// unsigned long previousMillis = 0; // will store last time a CAN Message was send
// const int interval = 1000;        // interval at which send CAN Messages (milliseconds)
const int rx_queue_size = 40; // Receive Queue size

QueueHandle_t loggerqueue;
TaskHandle_t loggingtask;
TaskHandle_t canbustask;

void loggerTask(void *pvParameter)
{
  CAN_frame_t rx_frame;
  char buf[256];
  uint32_t messagecounter = 0;
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  SD.begin(SD_CS, SPI, 16000000);
  while (digitalRead(SD_CD))
  {
    vTaskDelay(1000);
  }
  vTaskDelay(500);
  File logfile = SD.open("/logfile.csv", FILE_APPEND, true);

  if (!logfile)
  {
    
      Serial.println("SD card unmountable. log task exit.");
    vTaskDelete(NULL);
    // task delete self. no card to log into.
  }
  if (logfile.size() == 0)
  {
    logfile.printf("frame count,milliseconds,format,type,Message ID,DLC,Data\r\n");
  }
  
    Serial.println("SD init Success, logfile open.");
  snprintf(buf, 256, "Card size: %lluB, type %u, used %lluB, log size%lu\r\n", SD.cardSize(), SD.cardType(), SD.usedBytes(), logfile.size());
  
    Serial.print(buf);
    logfile.flush();
  //log.close();

  while (1)
  {
    //File logfile = SD.open("/logfile.csv", FILE_APPEND);
    if (xQueueReceive(loggerqueue, &rx_frame, portMAX_DELAY) == pdTRUE)
    {
      
      messagecounter++;
      logfile.printf("%lu,%lu,", messagecounter, millis());
  
      if (rx_frame.FIR.B.FF == CAN_frame_std)
      {
        logfile.printf("standard,");
      }
      else
      {
        logfile.printf("extended,");
      }
      if (rx_frame.FIR.B.RTR == CAN_RTR)
      {
        logfile.printf("RTR,0x%08X,%d,\r\n", rx_frame.MsgID, rx_frame.FIR.B.DLC);
      }
      else
      {
        logfile.printf("data,0x%08X,%d,0x", rx_frame.MsgID, rx_frame.FIR.B.DLC);
        for (int i = 0; i < rx_frame.FIR.B.DLC; i++)
        {
          logfile.printf("%02X", rx_frame.data.u8[i]);
        }
        logfile.printf("\r\n");
      }
      logfile.flush();
    }
  }
}

void canRxTask(void *pvParameter)
{
  char buf[256];
  CAN_frame_t rx_frame;

  // unsigned long currentMillis = millis();
  while (true)
  {
    // Receive next CAN frame from queue
    if (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 3 * portTICK_PERIOD_MS) == pdTRUE)
    {
      xQueueSend(loggerqueue, &rx_frame, 0);
      if (rx_frame.FIR.B.FF == CAN_frame_std)
      {
        
          Serial.print("New standard frame");
      }
      else
      {
        
          Serial.print("New extended frame");
      }

      if (rx_frame.FIR.B.RTR == CAN_RTR)
      {
        snprintf(buf, 256, " RTR from 0x%08X, DLC %d\r\n", rx_frame.MsgID, rx_frame.FIR.B.DLC);
        
          Serial.print(buf);
      }
      else
      {
        snprintf(buf, 256, " from 0x%08X, DLC %d, Data ", rx_frame.MsgID, rx_frame.FIR.B.DLC);
        
          Serial.print(buf);
        for (int i = 0; i < rx_frame.FIR.B.DLC; i++)
        {
          snprintf(buf, 256, "0x%02X ", rx_frame.data.u8[i]);
          
            Serial.print(buf);
        }
        
          Serial.println();
      }
    }
    // Send CAN Message
    /*
    if (currentMillis - previousMillis >= interval)
    {
      previousMillis = currentMillis;
      CAN_frame_t tx_frame;
      tx_frame.FIR.B.FF = CAN_frame_std;
      tx_frame.MsgID = 0x001;
      tx_frame.FIR.B.DLC = 8;
      tx_frame.data.u8[0] = 0x00;
      tx_frame.data.u8[1] = 0x01;
      tx_frame.data.u8[2] = 0x02;
      tx_frame.data.u8[3] = 0x03;
      tx_frame.data.u8[4] = 0x04;
      tx_frame.data.u8[5] = 0x05;
      tx_frame.data.u8[6] = 0x06;
      tx_frame.data.u8[7] = 0x07;
      ESP32Can.CANWriteFrame(&tx_frame);
    }
    */
  }
}

void setup()
{
  Serial.begin(1000000);
  
    Serial.println("CAN Bus Logger");
  pinMode(CAN_SLNT, OUTPUT);
  digitalWrite(CAN_SLNT, HIGH);
  pinMode(PWR_FAULT, INPUT_PULLUP);
  pinMode(VBAT_DIV, INPUT);
  pinMode(SD_CD, INPUT_PULLUP);
  CAN_cfg.speed = CAN_SPEED_500KBPS;
  CAN_cfg.tx_pin_id = GPIO_NUM_5;
  CAN_cfg.rx_pin_id = GPIO_NUM_4;
  CAN_cfg.rx_queue = xQueueCreate(rx_queue_size, sizeof(CAN_frame_t));
  // Init CAN Module
  if (ESP32Can.CANInit() == 0)
    
      Serial.println("CAN bus initialized.");

  loggerqueue = xQueueCreate(rx_queue_size, sizeof(CAN_frame_t));
  xTaskCreate(loggerTask, "logger task", 40000, nullptr, 1, &loggingtask);
  xTaskCreate(canRxTask, "canbus Rx task", 40000, nullptr, 1, &canbustask);
}

void loop()
{
  vTaskDelete(NULL);
}