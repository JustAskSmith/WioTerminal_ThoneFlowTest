/*
  Reconfigures the left Grove connector as a UART named Serial3
  Left side Grove connector shares pins with I2C1 of 40 pin connector.
  Do not use the default wire (I2C) library as these pins are now a UART.
*/

#include <wiring_private.h>

#define SERIAL3_BAUD 19200

uint8_t data[8]; //  holds one frame of bytes from the serial input

#define headerByte 0xFE
#define footerByte 0xAA

// reassign sercom3 as a UART using the I2C pins on the left Grove connector
static Uart Serial3(&sercom3, PIN_WIRE_SCL, PIN_WIRE_SDA, SERCOM_RX_PAD_1, UART_TX_PAD_0);

void SERCOM3_0_Handler()
{
  Serial3.IrqHandler();
}

void SERCOM3_1_Handler()
{
  Serial3.IrqHandler();
}

void SERCOM3_2_Handler()
{
  Serial3.IrqHandler();
}

void SERCOM3_3_Handler()
{
  Serial3.IrqHandler();
}

void setup()
{
  Serial.begin(115200);
  while (!Serial) ;
  // Serial.println("Serial started");
  Serial3.begin(SERIAL3_BAUD);
   while (!Serial3);
  // Serial.println("Serial3 Started");

  
  pinPeripheral(PIN_WIRE_SCL, PIO_SERCOM_ALT);
  pinPeripheral(PIN_WIRE_SDA, PIO_SERCOM_ALT);
  // Serial.println("Pin maps set");
}



void loop() {

  // Read from Serial1, send to Serial:
  if (Serial3.available() > 0)
   {
     int inbyte = Serial3.read();
     if (inbyte == headerByte)
     {
      int i;   
      for (i=0; i < 9; i++)
      {
        data[i] = inbyte;
        while(!Serial3.available());
        int inbyte = Serial3.read();

      }
     }
    
  }
}
