/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_MCP23X17.h>

/**
 * RP2040 Motor and Solenoid Controller Firmware with Limit Switches and RS485 Datagram Handling
 *
 * This firmware:
 *  - Configures local RP2040 pins for motor control (direction, enable, and PWM).
 *  - Sets up the MCP23S17 IO expander for limit switch inputs, solenoid/air-compressor MOSFET outputs, and RS485 LEDs.
 *  - Provides functions to drive motors to endstops (limit switches).
 *  - Receives RS485 datagrams (via Serial1) that control motors and solenoids (start, stop, direction).
 *  - Sends RS485 datagrams, if needed, to other devices.
 *
 * (c) 2023. Provided under GPL v3 or later.
 */

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_MCP23X17.h>
#include "hardware/uart.h"

// -----------------------------------------------------------------------------
// Instantiate the MCP23S17 for SPI communication
// -----------------------------------------------------------------------------
Adafruit_MCP23X17 mcp;  

// -----------------------------------------------------------------------------
// Local RP2040 Pin Assignments for Motor Control (H-Bridge)
// -----------------------------------------------------------------------------
#define M1PWM  13  // PWM signal for Motor 1
#define M2PWM  10  // PWM signal for Motor 2
#define M3PWM  6   // PWM signal for Motor 3
#define M4PWM  1   // PWM signal for Motor 4

#define M1INA  15  // Motor 1 direction control (IN A)
#define M1INB  12  // Motor 1 direction control (IN B)
#define M2INA  11  // Motor 2 direction control (IN A)
#define M2INB  8   // Motor 2 direction control (IN B)
#define M3INA  7   // Motor 3 direction control (IN A)
#define M3INB  4   // Motor 3 direction control (IN B)
#define M4INA  3   // Motor 4 direction control (IN A)
#define M4INB  0   // Motor 4 direction control (IN B)

#define M1ENA  14  // Enable pin for Motor 1
#define M2ENA  9   // Enable pin for Motor 2
#define M3ENA  5   // Enable pin for Motor 3
#define M4ENA  2   // Enable pin for Motor 4

// -----------------------------------------------------------------------------
// MCP23S17 Bank A - Limit Switch Pin Assignments (Inputs)
// -----------------------------------------------------------------------------
#define LIMIT_SWITCH_MIN_1  0  // GPA0 (Motor 1 min limit)
#define LIMIT_SWITCH_MIN_2  1  // GPA1 (Motor 2 min limit)
#define LIMIT_SWITCH_MIN_3  2  // GPA2 (Motor 3 min limit)
#define LIMIT_SWITCH_MIN_4  3  // GPA3 (Motor 4 min limit)
#define LIMIT_SWITCH_MAX_1  4  // GPA4 (Motor 1 max limit)
#define LIMIT_SWITCH_MAX_2  5  // GPA5 (Motor 2 max limit)
#define LIMIT_SWITCH_MAX_3  6  // GPA6 (Motor 3 max limit)
#define LIMIT_SWITCH_MAX_4  7  // GPA7 (Motor 4 max limit)

// -----------------------------------------------------------------------------
// MCP23S17 Bank B - Solenoid/Air, Power Good, RS485 LEDs
// -----------------------------------------------------------------------------
#define AIR_COMPRESSOR_MOSFET  8   // GPB0 (air compressor MOSFET)
#define SOLENOID_3_MOSFET      9   // GPB1 (solenoid #3)
#define SOLENOID_2_MOSFET      10  // GPB2 (solenoid #2)
#define SOLENOID_1_MOSFET      11  // GPB3 (solenoid #1)
#define SOLENOID_0_MOSFET      12  // GPB4 (solenoid #0)
#define POWER_GOOD_PIN         13  // GPB5 (3.3V DCDC regulator power-good, input)
#define RS485_TX_LED           14  // GPB6 (RS485 TX LED)
#define RS485_RX_LED           15  // GPB7 (RS485 RX LED)

// -----------------------------------------------------------------------------
// RS485 Datagram Definitions (matching previous firmware)
// -----------------------------------------------------------------------------
#define RS485_HEADER   0x7E   // Start byte
#define RS485_END_BYTE 0xFF   // End byte

// Command Types
#define CMD_START     0x01
#define CMD_STOP      0x02
#define CMD_MOTOR     0x03
#define CMD_SOLENOID  0x04

// -----------------------------------------------------------------------------
// RS485 Payload Structures
// -----------------------------------------------------------------------------
struct MotorControlPayload {
    uint8_t motorID;    // Motor ID (0..3)
    uint8_t direction;  // 0=Stop, 1=Forward, 2=Reverse
};

struct SolenoidControlPayload {
    uint8_t solenoidID; // 0..4 (0..3 = solenoids, 4 = air compressor)
    uint8_t state;      // 0=Off, 1=On
};

// Union for motor or solenoid payload
union PayloadData {
    MotorControlPayload motorControl;
    SolenoidControlPayload solenoidControl;
};

// Full RS485 Datagram structure
struct RS485Datagram {
    uint8_t startByte;    // e.g., 0x7E
    uint8_t address;      // target device address
    uint8_t command;      // e.g., CMD_MOTOR, CMD_SOLENOID
    uint8_t dataLength;   // length of payload
    PayloadData payload;  // motor or solenoid data
    uint8_t checksum;     
    uint8_t endByte;      // e.g., 0xFF
};

// -----------------------------------------------------------------------------
// Global Variables
// -----------------------------------------------------------------------------
String receivedCommand   = "";    // Buffer for incoming text from Serial1 (RS485)
bool   systemInitialized = false; // Whether the system is "active" (motors/solenoids allowed)

// -----------------------------------------------------------------------------
// Function Declarations
// -----------------------------------------------------------------------------
void setupMCP23S17();
void controlMOSFET(int solenoidID, bool state);
void controlMotor(int motorID, uint8_t direction, uint8_t pwmValue);
void stopMotor(int motorID);
bool readLimitSwitch(int motor, bool isMaxLimit);
void driveMotorToEndstop(int motor, int direction);
RS485Datagram parseRS485Datagram(const String &datagram);
void handleRS485Command();
void sendRS485Datagram(RS485Datagram& datagram);

// =============================================================================
// SETUP (Arduino entry point)
// =============================================================================
void setup() {
  // Start USB serial for debugging/logging
  Serial.begin(115200);
  Serial.println("RP2040 Motor Controller w/ Endstop + RS485 + Solenoids, Enhanced Setup");

  // --------------------------------------------------
  // Configure Local (RP2040) Pins for Motor Control
  // --------------------------------------------------
  pinMode(M1INA, OUTPUT);  digitalWrite(M1INA, LOW);
  pinMode(M1INB, OUTPUT);  digitalWrite(M1INB, LOW);
  pinMode(M1ENA, OUTPUT);  digitalWrite(M1ENA, LOW);
  pinMode(M1PWM, OUTPUT);  analogWrite(M1PWM, 0);

  pinMode(M2INA, OUTPUT);  digitalWrite(M2INA, LOW);
  pinMode(M2INB, OUTPUT);  digitalWrite(M2INB, LOW);
  pinMode(M2ENA, OUTPUT);  digitalWrite(M2ENA, LOW);
  pinMode(M2PWM, OUTPUT);  analogWrite(M2PWM, 0);

  pinMode(M3INA, OUTPUT);  digitalWrite(M3INA, LOW);
  pinMode(M3INB, OUTPUT);  digitalWrite(M3INB, LOW);
  pinMode(M3ENA, OUTPUT);  digitalWrite(M3ENA, LOW);
  pinMode(M3PWM, OUTPUT);  analogWrite(M3PWM, 0);

  pinMode(M4INA, OUTPUT);  digitalWrite(M4INA, LOW);
  pinMode(M4INB, OUTPUT);  digitalWrite(M4INB, LOW);
  pinMode(M4ENA, OUTPUT);  digitalWrite(M4ENA, LOW);
  pinMode(M4PWM, OUTPUT);  analogWrite(M4PWM, 0);

  // --------------------------------------------------
  // Initialize MCP23S17 (IO Expander) for limit switches, solenoids, LEDs
  // --------------------------------------------------
  setupMCP23S17();

  // --------------------------------------------------
  // (Optionally) Setup RS485 TX/RX pins, direction pins, etc.
  // If your board has them pre-mapped, you may not need to do anything extra.
  // e.g.: pinMode(uartTxPin, OUTPUT); pinMode(uartRxPin, INPUT);
  // ...
  // --------------------------------------------------

  // By default, system is off until we get a "START" command
  systemInitialized = false;
}

// =============================================================================
// LOOP (Arduino entry point)
// =============================================================================
void loop() {
  // Continuously handle any incoming RS485 commands
  handleRS485Command();

  // If system is not initialized, everything remains off
  if (!systemInitialized) {
    return;
  }

  // If desired, you can add any "automatic" motor sequences here
  // e.g. driveMotorToEndstop(0, 1); ...
}

// =============================================================================
// SETUP MCP23S17 - IO EXPANDER
// =============================================================================
void setupMCP23S17() {
  // Initialize MCP23S17 over SPI, address 0x20
  if (!mcp.begin_SPI(0x20, &SPI)) {
    Serial.println("Error: MCP23S17 not found at address 0x20 via SPI");
    while (true); // Halt
  }

  // Bank A (0..7) => limit switches (inputs, pull-up)
  for (uint8_t i = 0; i < 8; i++) {
    mcp.pinMode(i, INPUT_PULLUP);
  }

  // Bank B => solenoids & air compressor (0..4, outputs),
  //           power-good signal (5, input),
  //           RS485 LEDs (6,7 outputs).
  mcp.pinMode(AIR_COMPRESSOR_MOSFET, OUTPUT);
  mcp.pinMode(SOLENOID_3_MOSFET, OUTPUT);
  mcp.pinMode(SOLENOID_2_MOSFET, OUTPUT);
  mcp.pinMode(SOLENOID_1_MOSFET, OUTPUT);
  mcp.pinMode(SOLENOID_0_MOSFET, OUTPUT);

  mcp.pinMode(RS485_TX_LED, OUTPUT);
  mcp.pinMode(RS485_RX_LED, OUTPUT);

  mcp.pinMode(POWER_GOOD_PIN, INPUT);

  Serial.println("MCP23S17 configured for limit switches, solenoids, & RS485 LEDs.");
}

// =============================================================================
// SOLENOID / AIR COMPRESSOR CONTROL
// =============================================================================
void controlMOSFET(int solenoidID, bool state) {
  int pin;
  switch (solenoidID) {
    case 0: pin = SOLENOID_0_MOSFET; break;
    case 1: pin = SOLENOID_1_MOSFET; break;
    case 2: pin = SOLENOID_2_MOSFET; break;
    case 3: pin = SOLENOID_3_MOSFET; break;
    case 4: pin = AIR_COMPRESSOR_MOSFET; break;
    default:
      Serial.println("Invalid Solenoid/Air ID");
      return;
  }
  // High => ON, Low => OFF
  mcp.digitalWrite(pin, state ? HIGH : LOW);
}

// =============================================================================
// MOTOR CONTROL (H-BRIDGE)
// =============================================================================
void controlMotor(int motorID, uint8_t direction, uint8_t pwmValue) {
  int ina, inb, ena, pwmPin;
  switch (motorID) {
    case 0: ina=M1INA; inb=M1INB; ena=M1ENA; pwmPin=M1PWM; break;
    case 1: ina=M2INA; inb=M2INB; ena=M2ENA; pwmPin=M2PWM; break;
    case 2: ina=M3INA; inb=M3INB; ena=M3ENA; pwmPin=M3PWM; break;
    case 3: ina=M4INA; inb=M4INB; ena=M4ENA; pwmPin=M4PWM; break;
    default:
      Serial.println("Invalid Motor ID");
      return;
  }

  // Direction: 0=Stop, 1=Forward, 2=Reverse
  if (direction == 0) {
    // Stop
    digitalWrite(ena, LOW);
    digitalWrite(ina, LOW);
    digitalWrite(inb, LOW);
    analogWrite(pwmPin, 0);
  }
  else if (direction == 1) {
    // Forward
    digitalWrite(ena, HIGH);
    digitalWrite(ina, HIGH);
    digitalWrite(inb, LOW);
    analogWrite(pwmPin, pwmValue);
  }
  else if (direction == 2) {
    // Reverse
    digitalWrite(ena, HIGH);
    digitalWrite(ina, LOW);
    digitalWrite(inb, HIGH);
    analogWrite(pwmPin, pwmValue);
  }
}

// =============================================================================
// STOP MOTOR (CONVENIENCE)
// =============================================================================
void stopMotor(int motorID) {
  controlMotor(motorID, 0, 0);
}

// =============================================================================
// LIMIT SWITCH READING
// =============================================================================
bool readLimitSwitch(int motor, bool isMaxLimit) {
  int pin;
  switch (motor) {
    case 0: pin = isMaxLimit ? LIMIT_SWITCH_MAX_1 : LIMIT_SWITCH_MIN_1; break;
    case 1: pin = isMaxLimit ? LIMIT_SWITCH_MAX_2 : LIMIT_SWITCH_MIN_2; break;
    case 2: pin = isMaxLimit ? LIMIT_SWITCH_MAX_3 : LIMIT_SWITCH_MIN_3; break;
    case 3: pin = isMaxLimit ? LIMIT_SWITCH_MAX_4 : LIMIT_SWITCH_MIN_4; break;
    default:
      return false;
  }
  // Active-low => LOW means switch triggered
  return (mcp.digitalRead(pin) == LOW);
}

// =============================================================================
// DRIVE MOTOR UNTIL ENDSTOP
// =============================================================================
void driveMotorToEndstop(int motor, int direction) {
  // direction: 1=Forward, 2=Reverse
  bool minLimitReached = false;
  bool maxLimitReached = false;

  // Full speed
  controlMotor(motor, direction, 255);

  while (!minLimitReached && !maxLimitReached) {
    minLimitReached = readLimitSwitch(motor, false); // false => min limit
    maxLimitReached = readLimitSwitch(motor, true);  // true => max limit
    delay(10);
  }

  stopMotor(motor);

  if (minLimitReached) {
    Serial.printf("Motor %d: MIN limit reached.\n", motor);
  }
  if (maxLimitReached) {
    Serial.printf("Motor %d: MAX limit reached.\n", motor);
  }
}

// =============================================================================
// PARSE RS485 DATAGRAM
// =============================================================================
RS485Datagram parseRS485Datagram(const String &datagram) {
  RS485Datagram parsed;
  
  // Minimal parsing: indexes into string
  // Real code would confirm lengths, dataLength, etc.
  parsed.startByte   = datagram[0];
  parsed.address     = datagram[1];
  parsed.command     = datagram[2];
  parsed.dataLength  = datagram[3];

  if (parsed.command == CMD_MOTOR) {
    parsed.payload.motorControl.motorID   = datagram[4];
    parsed.payload.motorControl.direction = datagram[5];
  } 
  else if (parsed.command == CMD_SOLENOID) {
    parsed.payload.solenoidControl.solenoidID = datagram[4];
    parsed.payload.solenoidControl.state      = datagram[5];
  }

  parsed.checksum  = datagram[datagram.length() - 2];
  parsed.endByte   = datagram[datagram.length() - 1];
  
  return parsed;
}

// =============================================================================
// HANDLE RS485 COMMAND (READ + DISPATCH)
// =============================================================================
void handleRS485Command() {
  // If there's incoming data on Serial1
  if (Serial1.available()) {
    String rxData = Serial1.readStringUntil('\n');
    if (rxData.length() < 6) { 
      // Not enough length to form a valid datagram
      return;
    }

    RS485Datagram datagram = parseRS485Datagram(rxData);

    switch (datagram.command) {
      case CMD_START:
        systemInitialized = true;
        Serial.println("System Initialized (CMD_START).");
        break;

      case CMD_STOP:
        systemInitialized = false;
        Serial.println("System Stopped (CMD_STOP).");
        // Optionally stop all motors, turn off solenoids
        stopMotor(0); stopMotor(1); stopMotor(2); stopMotor(3);
        controlMOSFET(0, false);
        controlMOSFET(1, false);
        controlMOSFET(2, false);
        controlMOSFET(3, false);
        controlMOSFET(4, false); // Air compressor
        break;

      case CMD_MOTOR: {
        uint8_t mID = datagram.payload.motorControl.motorID;
        uint8_t dir = datagram.payload.motorControl.direction;
        // Control the motor (full speed)
        controlMotor(mID, dir, 255);
        Serial.printf("CMD_MOTOR: Motor %u, direction=%u\n", mID, dir);
      } break;

      case CMD_SOLENOID: {
        uint8_t solID = datagram.payload.solenoidControl.solenoidID;
        bool onOff     = (datagram.payload.solenoidControl.state == 0x01);
        controlMOSFET(solID, onOff);
        Serial.printf("CMD_SOLENOID: Solenoid %u => %s\n", solID, onOff?"ON":"OFF");
      } break;

      default:
        Serial.println("Unknown RS485 command received.");
        break;
    }
  }
}

// =============================================================================
// SEND RS485 DATAGRAM
// =============================================================================
void sendRS485Datagram(RS485Datagram& datagram) {
  // Convert datagram into a string or raw bytes as your protocol requires.
  // Here, we do a simplistic approach as ASCII chars.
  String data;
  data += (char)datagram.startByte;
  data += (char)datagram.address;
  data += (char)datagram.command;
  data += (char)datagram.dataLength;

  if (datagram.command == CMD_MOTOR) {
    data += (char)datagram.payload.motorControl.motorID;
    data += (char)datagram.payload.motorControl.direction;
  }
  else if (datagram.command == CMD_SOLENOID) {
    data += (char)datagram.payload.solenoidControl.solenoidID;
    data += (char)datagram.payload.solenoidControl.state;
  }

  data += (char)datagram.checksum;
  data += (char)datagram.endByte;

  // Transmit via Serial1
  Serial1.print(data);
}

