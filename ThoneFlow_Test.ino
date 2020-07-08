/*
  Reconfigures the left Grove connector as a UART named Serial3
  Left side Grove connector shares pins with I2C1 of 40 pin connector.
  Do not use the default wire (I2C) library as these pins are now a UART.
*/

#include <wiring_private.h>
#include <TFT_eSPI.h>

#define SERIAL_OUTPUT
#define SERIAL3_BAUD 19200
#define HEADER_BYTE 0xFE
#define FOOTER_BYTE 0xAA
#define VECTOR_CENTER_X 160 // center point for the movement vector
#define VECTOR_CENTER_Y 120
#define VECTOR_MAX_DELTA 118  // maximum X or Y displacement of the movement vector to stay on screen
#define SCALE 5 // sensitivity for drawing the vector
#define SPIKE_FILTER // uncomment to use a filter to remove spikes above SPIKE_SIZE times bigger than the last reading
#define SPIKE_SIZE 1.7

uint8_t data [9]; //  holds one frame of bytes from the serial input
int16_t xMotion, yMotion;
uint8_t surfaceQuality;


// reassign sercom3 as a UART using the I2C pins on the left Grove connector
static Uart Serial3(&sercom3, PIN_WIRE_SCL, PIN_WIRE_SDA, SERCOM_RX_PAD_1, UART_TX_PAD_0);

TFT_eSPI tft = TFT_eSPI();

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
  
  // intialize display
  tft.begin();
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);
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
          while(!Serial3.available())
          {

          }
          int inbyte = Serial3.read();  
          data[i] = inbyte;
        }
      }
      processFrame();
      drawVector(xMotion, yMotion, SCALE);
   }
}

void processFrame()
{
  #ifdef SPIKE_FILTER
    static double lastX,lastY;
  #endif
  
  int16_t thisX,thisY;

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
  thisX = ((int16_t)data[3] << 8) | data[2];
  thisY = ((int16_t)data[5] << 8) | data[4];
 
  #ifdef SPIKE_FILTER
    if ((abs(thisX) > (abs(lastX) * SPIKE_SIZE)) && (abs(lastX) > 0))
    {
      xMotion = lastX;
    }
    else
    {
      xMotion = thisX;
    }
    

    if ((abs(thisY) > (abs(lastY) * SPIKE_SIZE)) && (abs(lastY) > 0))
    {
      yMotion = lastY;
    }
    else
    {
      yMotion - thisY;
    }
    
    
    lastX = (double)thisX;
    lastY = (double)thisY;

  #endif


   surfaceQuality = data[7];

  
  #ifdef SERIAL_OUTPUT
    serialPrintFrame();
    Serial.println();
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
  uint8_t sum = 0;
  sum = data[2] + data[3] + data[4] + data[5];
  return (sum == data[6]);
}

void serialPrintFrame()
{
  int j;
  for (j=0; j < 9; j++)
  {
    Serial.printf("%02X", data[j]);
    Serial.print(" ");
  }

}

void drawVector(int xMotion, int yMotion, int scale)
{
  static int lastX = 0, lastY = 0;
  
  if (lastX) // don't run for the very first call of this function
  {
    tft.drawLine(VECTOR_CENTER_X, VECTOR_CENTER_Y, lastX, lastY,TFT_BLACK); // erase the last line
  }

  lastX = constrain(VECTOR_CENTER_X + int(xMotion * scale), VECTOR_CENTER_X - VECTOR_MAX_DELTA, VECTOR_CENTER_X + VECTOR_MAX_DELTA); // constrain to screen space
  lastY = constrain(VECTOR_CENTER_Y + int(yMotion * scale), VECTOR_CENTER_Y - VECTOR_MAX_DELTA, VECTOR_CENTER_Y + VECTOR_MAX_DELTA);

  tft.drawLine(VECTOR_CENTER_X, VECTOR_CENTER_Y, lastX, lastY,TFT_WHITE);
}