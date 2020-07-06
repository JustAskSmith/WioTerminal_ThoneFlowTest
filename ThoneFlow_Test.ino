/*
  Reconfigures the left Grove connector as a UART named Serial3
  Left side Grove connector shares pins with I2C1 of 40 pin connector.
  Do not use the default wire (I2C) library as these pins are now a UART.
*/

#include <wiring_private.h>

#define SERIAL_OUTPUT
#define SERIAL3_BAUD 19200
#define HEADER_BYTE 0xFE
#define FOOTER_BYTE 0xAA

uint8_t data[8]; //  holds one frame of bytes from the serial input
int16_t xMotion, yMotion;
uint8_t surfaceQuality;

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
  #ifdef SERIAL_OUTPUT  
    Serial.begin(115200);
    while (!Serial) ;
    Serial.println("Serial started");
  #endif

  Serial3.begin(SERIAL3_BAUD);
   while (!Serial3);
  
  pinPeripheral(PIN_WIRE_SCL, PIO_SERCOM_ALT);
  pinPeripheral(PIN_WIRE_SDA, PIO_SERCOM_ALT);
  // Serial.println("Pin maps set");
}

void loop()
 {

  // Read from Serial1, send to Serial:
  if (Serial3.available() > 0)
   {
     int inbyte = Serial3.read();

      if (inbyte == HEADER_BYTE)  // header detected
      {
        int i = 0;
        data[i] = inbyte;
        for (i=1; i < 9; i++)
        {
          while(!Serial3.available()) // only wait 500 milliseconds for the next byte
          {

          }
          int inbyte = Serial3.read();  
          data[i] = inbyte;
        }
      }

      processFrame();  
   }
}

void processFrame()
{
  // check for proper header/footer framing
  if (data[8] != FOOTER_BYTE)  
  
  {
    #ifdef SERIAL_OUTPUT
      Serial.print("Bad frame detected ");
        serialPrintFrame();
      Serial.println("- incorrect footer byte");
    #endif  
    return;
  }
  if (data[1] != 4)
  {
    #ifdef SERIAL_OUTPUT
      Serial.print("Bad frame detected ");
      serialPrintFrame();
      Serial.println("- incorrect byte count");
    #endif  
    return;
  }
  if (!testChecksum())
  {
    #ifdef SERIAL_OUTPUT
      Serial.print("Bad frame detected ");
      serialPrintFrame();
      Serial.println("- checksum failure");
    #endif  
    return;
  } 
  // tests passed we have a valid frame
  xMotion = (data[2] << 8) | data[3];
  yMotion = (data[4] << 8) | data[5];
  surfaceQuality =  data[7];
  
  #ifdef SERIAL_OUTPUT
    Serial.print("X Motion= ");
    Serial.println(xMotion);
    Serial.print("Y Motion= ");
    Serial.println(yMotion);
    Serial.print("Surface quality= ");
    Serial.println(surfaceQuality);
  #endif
}

bool testChecksum()
{
    return true;
}

void serialPrintFrame()
{
  int i;
  for (i=0; i < 9; i++)
  {
    Serial.print(data[i]);
    Serial.print(" ");
  }

}
