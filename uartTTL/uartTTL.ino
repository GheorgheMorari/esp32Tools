#include <HardwareSerial.h>

#define BAUD_RATE 115200
#define SERIAL2_RX 16
#define SERIAL2_TX 17

unsigned char receiveByte = 0;
unsigned char transmitByte = 0;
TaskHandle_t usbToUartTask;
TaskHandle_t uartToUsbTask;

HardwareSerial SerialPort(2); 

void uartToUsb(void* params){
  Serial.println("UartToUsb started");
  while(1){   
    while (Serial2.available() > 0) {
      Serial2.readBytes(&receiveByte, 1);
      Serial.write(&receiveByte, 1);
    }
    vTaskDelay(0); //Here to prevent reset by the watchdog timer
  }
}


void usbToUart(void* params){
  Serial.println("UsbToUart started");
  while(1){
    while (Serial.available() > 0) {
      Serial.readBytes(&transmitByte, 1);
      Serial2.write(&transmitByte, 1);
    }
    vTaskDelay(0); //Here to prevent reset by the watchdog timer
  }
}

void setup()  
{
  Serial2.begin(BAUD_RATE, SERIAL_8N1, 16, 17); 

  Serial.begin(BAUD_RATE);


  xTaskCreatePinnedToCore(
                    uartToUsb,      /* Task function. */
                    "uartToUsb",    /* name of task. */
                    1000,          /* Stack size of task */
                    NULL,           /* parameter of the task */
                    1,              /* priority of the task */
                    &uartToUsbTask, /* Task handle to keep track of created task */
                    0);             /* pin task to core 0 */
  xTaskCreatePinnedToCore(
                    usbToUart,      /* Task function. */
                    "usbToUart",    /* name of task. */
                    1000,          /* Stack size of task */
                    NULL,           /* parameter of the task */
                    1,              /* priority of the task */
                    &usbToUartTask, /* Task handle to keep track of created task */
                    1);             /* pin task to core 1 */
}
void loop() {vTaskDelay(0);}
