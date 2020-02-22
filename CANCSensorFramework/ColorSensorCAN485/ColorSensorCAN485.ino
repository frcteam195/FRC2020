/*
 * CAN port receiver example
 * Repeatedly transmits an array of test data to the CAN port
 */

#include <ASTCanLib.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <avr/wdt.h>

#define TEAM_DEVICE_TYPE (uint32_t)10
#define TEAM_MANUFACTURER_TYPE (uint32_t)8

#define DEVICE_ID (uint32_t)1

#define FULL_DEVICE_ID (TEAM_DEVICE_TYPE << 24) | (TEAM_MANUFACTURER_TYPE << 16) | (DEVICE_ID & 0x3F)

#define API_NONE 0
#define API_SET_PERIODIC_RATE 1
#define API_COLOR_SENSOR_DATA 2

#define MESSAGE_PROTOCOL  1         // CAN protocol (0: CAN 2.0A, 1: CAN 2.0B)
#define MESSAGE_LENGTH    8         // Data length: 8 bytes
#define MESSAGE_RTR       0         // rtr bit

// CAN message object
st_cmd_t txMsg;

// Transmit buffer
uint8_t txBuffer[8] = {};

float r, g, b;

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_4X);

void clearTxBuffer() {
  for (int i=0; i<8; i++){
    txBuffer[i] = 0;
  }
}

void setTxAPICommand(uint32_t apiCommand) {
  txMsg.id.ext |= (apiCommand & 0x3FF) << 6;
}

void setupMessage() {
  // Setup CAN packet.
  txMsg.ctrl.ide = MESSAGE_PROTOCOL;  // Set CAN protocol (0: CAN 2.0A, 1: CAN 2.0B)
  txMsg.id.ext   = FULL_DEVICE_ID;        // Set message ID
  txMsg.dlc      = MESSAGE_LENGTH;    // Data length: 8 bytes
  txMsg.ctrl.rtr = MESSAGE_RTR;       // Set rtr bit
  clearTxBuffer();
}

void setup() {
  canInit(1000000);                  // Initialise CAN port. must be before Serial.begin
  Serial.begin(1000000);             // start serial port
  txMsg.pt_data = &txBuffer[0];      // reference message data to transmit buffer

  wdt_enable(WDTO_500MS);
  if (!tcs.begin()) {
    Serial.println("No TCS34725 found ... check your connections. Rebooting...");
    while (1);
  }

  r = 0;
  g = 0;
  b = 0;
}

void loop() {
  wdt_reset();
  tcs.getRGB(&r, &g, &b);

  setupMessage();
  setTxAPICommand(API_COLOR_SENSOR_DATA);

  txBuffer[5] = (uint8_t)r;
  txBuffer[6] = (uint8_t)g;
  txBuffer[7] = (uint8_t)b;

  canTx();

  Serial.print(txMsg.id.ext, HEX);
  Serial.println(" sent");
  delay(100);
}

void canTx() {
  // Send command to the CAN port controller
  txMsg.cmd = CMD_TX_DATA;       // send message
  // Wait for the command to be accepted by the controller
  while(can_cmd(&txMsg) != CAN_CMD_ACCEPTED);
  // Wait for command to finish executing
  while(can_get_status(&txMsg) == CAN_STATUS_NOT_COMPLETED);
  // Transmit is now complete. Wait a bit and loop
}
