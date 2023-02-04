// CAN Receive Example
//

#define BAMOCAR_TRANSMISSION_ADDR 0x181
#define BAMOCAR_RECEIVE_ADDR 0x201

#include <mcp_can.h>
#include <SPI.h>

long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
char msgString[128]; // Array to store serial string

#define CAN0_INT 25 // Set INT to pin 2
MCP_CAN CAN0(26);   // Set CS to pin 10

void can_read_task(void *pvParameters);
void can_requestData();

void setup()
{
  Serial.begin(115200);

  SPI.begin(18, 19, 23, 26); // Initiate SPI bus

  // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK)
    Serial.println("MCP2515 Initialized Successfully!");
  else
    Serial.println("Error Initializing MCP2515...");

  CAN0.setMode(MCP_NORMAL); // Set operation mode to normal so the MCP2515 sends acks to received data.

  pinMode(CAN0_INT, INPUT); // Configuring pin for /INT input

  Serial.println("MCP2515 Library Receive Example...");

  xTaskCreatePinnedToCore(can_read_task, "can_read_task", 10000, NULL, 1, NULL, 1);
  Serial.println("Can read task created");
}

void loop()
{
  can_requestData();
  delay(2000);
}

void can_read_task(void *pvParameters)
{
  while (1)
  {
    delay(100);
    if (!digitalRead(CAN0_INT)) // If CAN0_INT pin is low, read receive buffer
    {
      CAN0.readMsgBuf(&rxId, &len, rxBuf); // Read data: len = data length, buf = data byte(s)

      if ((rxId & 0x80000000) == 0x80000000) // Determine if ID is standard (11 bits) or extended (29 bits)
        sprintf(msgString, "Extended ID: 0x%.8lX  DLC: %1d  Data:", (rxId & 0x1FFFFFFF), len);
      else
        sprintf(msgString, "Standard ID: 0x%.3lX       DLC: %1d  Data:", rxId, len);

      Serial.print(msgString);

      if ((rxId & 0x40000000) == 0x40000000)
      { // Determine if message is a remote request frame.
        sprintf(msgString, " REMOTE REQUEST FRAME");
        Serial.print(msgString);
      }
      else
      {
        for (byte i = 0; i < len; i++)
        {
          sprintf(msgString, " 0x%.2X", rxBuf[i]);
          Serial.print(msgString);
        }
      }

      Serial.println();
    }
  }
}

void can_requestData()
{
  unsigned char tBuf[3] = {0x3D, 0x40, 0x00};
  int res = CAN0.sendMsgBuf(BAMOCAR_RECEIVE_ADDR, 0, 3, tBuf);
  if (res == CAN_OK)
    Serial.println("Message Sent Successfully!");
  else
    ESP_LOGE("CAN", "Error Sending Message... Error Code: %d", res);
}