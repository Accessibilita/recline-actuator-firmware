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

/**
 * RP2040 Firmware: Motor & Solenoid Controller with:
 *   - Limit Switch Endstop Handling
 *   - Motor Current Sensing & Threshold Checking
 *   - RS485 Datagram Handling (Imported from Other Firmware)
 *   - Solenoid & Air Compressor Control via MCP23S17
 *
 * Detailed & Verbose Comments Included
 *
 * (c) 2023. Licensed under GPL v3 or later.
 */

#include <Wire.h>                  // For I2C (if needed)
#include <SPI.h>                   // For SPI communication
#include <Adafruit_MCP23X17.h>     // Adafruit library for MCP23S17 IO Expander
#include "hardware/uart.h"         // RP2040 hardware UART (for RS485 use)

// ---------------------------------------------------------------------------
// Instantiate MCP23S17 for SPI
// ---------------------------------------------------------------------------
Adafruit_MCP23X17 mcp;  

// ---------------------------------------------------------------------------
// RP2040 Pin Assignments for Motor (H-Bridge) Control
// ---------------------------------------------------------------------------
// Each motor has: PWM pin, two direction pins (INA/INB), and an enable pin (ENA).
#define M1PWM  13  // PWM output for Motor 1
#define M2PWM  10  // PWM output for Motor 2
#define M3PWM   6  // PWM output for Motor 3
#define M4PWM   1  // PWM output for Motor 4

#define M1INA  15  // Motor 1 direction: IN A
#define M1INB  12  // Motor 1 direction: IN B
#define M2INA  11  // Motor 2 direction: IN A
#define M2INB   8  // Motor 2 direction: IN B
#define M3INA   7  // Motor 3 direction: IN A
#define M3INB   4  // Motor 3 direction: IN B
#define M4INA   3  // Motor 4 direction: IN A
#define M4INB   0  // Motor 4 direction: IN B

#define M1ENA  14  // Enable pin for Motor 1
#define M2ENA   9  // Enable pin for Motor 2
#define M3ENA   5  // Enable pin for Motor 3
#define M4ENA   2  // Enable pin for Motor 4

// ---------------------------------------------------------------------------
// MCP23S17 Bank A: Limit Switch Pins (Inputs, Active-Low)
// ---------------------------------------------------------------------------
// The limit switches are connected to the IO expander's Bank A pins (GPA0..GPA7).
// Each motor has a min and a max limit switch.
#define LIMIT_SWITCH_MIN_1  0  // GPA0
#define LIMIT_SWITCH_MIN_2  1  // GPA1
#define LIMIT_SWITCH_MIN_3  2  // GPA2
#define LIMIT_SWITCH_MIN_4  3  // GPA3
#define LIMIT_SWITCH_MAX_1  4  // GPA4
#define LIMIT_SWITCH_MAX_2  5  // GPA5
#define LIMIT_SWITCH_MAX_3  6  // GPA6
#define LIMIT_SWITCH_MAX_4  7  // GPA7

// ---------------------------------------------------------------------------
// MCP23S17 Bank B: Solenoids, Air Compressor, Power Good, RS485 LEDs
// ---------------------------------------------------------------------------
// Each solenoid and the air compressor MOSFET are on outputs, plus pins for RS485 LEDs and the power good signal.
#define AIR_COMPRESSOR_MOSFET   8   // GPB0
#define SOLENOID_3_MOSFET       9   // GPB1
#define SOLENOID_2_MOSFET      10   // GPB2
#define SOLENOID_1_MOSFET      11   // GPB3
#define SOLENOID_0_MOSFET      12   // GPB4
#define POWER_GOOD_PIN         13   // GPB5 (Input, to read power good status)
#define RS485_TX_LED           14   // GPB6
#define RS485_RX_LED           15   // GPB7

// ---------------------------------------------------------------------------
// Current Sensing Inputs (RP2040 Analog Pins)
// ---------------------------------------------------------------------------
// Each motor has a current sense output that goes to an ADC pin on the RP2040.
#define CURRENT_SENSOR_1  26  // Motor 1 current sense -> GPIO26
#define CURRENT_SENSOR_2  27  // Motor 2 current sense -> GPIO27
#define CURRENT_SENSOR_3  28  // Motor 3 current sense -> GPIO28
#define CURRENT_SENSOR_4  29  // Motor 4 current sense -> GPIO29

// Current thresholds in amps (example values). If a motor’s current is outside
// these bounds, we stop the motor for safety / protection.
#define CURRENT_THRESHOLD_MIN_1  0.5f
#define CURRENT_THRESHOLD_MAX_1  3.0f
#define CURRENT_THRESHOLD_MIN_2  0.5f
#define CURRENT_THRESHOLD_MAX_2  3.0f
#define CURRENT_THRESHOLD_MIN_3  0.5f
#define CURRENT_THRESHOLD_MAX_3  3.0f
#define CURRENT_THRESHOLD_MIN_4  0.5f
#define CURRENT_THRESHOLD_MAX_4  3.0f

// ---------------------------------------------------------------------------
// RS485 Datagram Format (Matching the Other Firmware)
// ---------------------------------------------------------------------------
// These define how we parse incoming commands and structure outgoing ones.
#define RS485_HEADER   0x7E
#define RS485_END_BYTE 0xFF

// RS485 Command Types
#define CMD_START      0x01
#define CMD_STOP       0x02
#define CMD_MOTOR      0x03
#define CMD_SOLENOID   0x04

// Payload structures for motor or solenoid commands
struct MotorControlPayload {
    uint8_t motorID;    // 0..3 for four motors
    uint8_t direction;  // 0=Stop, 1=Forward, 2=Reverse
};

struct SolenoidControlPayload {
    uint8_t solenoidID; // 0..4 => 0..3 for solenoids, 4 for air compressor
    uint8_t state;      // 0=Off, 1=On
};

// Union to store either motor or solenoid payload
union PayloadData {
    MotorControlPayload       motorControl;
    SolenoidControlPayload    solenoidControl;
};

// RS485 Datagram structure
struct RS485Datagram {
    uint8_t  startByte;    // e.g., 0x7E
    uint8_t  address;      // device address
    uint8_t  command;      // e.g. CMD_MOTOR
    uint8_t  dataLength;   // how many bytes in payload
    PayloadData payload;   // either motor or solenoid data
    uint8_t  checksum;     
    uint8_t  endByte;      // e.g., 0xFF
};

// ---------------------------------------------------------------------------
// Global Variables
// ---------------------------------------------------------------------------
String receivedCommand   = "";    // Buffer for text from RS485 (Serial1)
bool   systemInitialized = false; // Indicates if the system is active

// ---------------------------------------------------------------------------
// Function Declarations
// ---------------------------------------------------------------------------
void setupMCP23S17();
void setupCurrentSensors();
void monitorCurrentDraw();

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
  // Start USB serial for debugging
  Serial.begin(115200);
  Serial.println("RP2040 Motor/Solenoid Controller w/ Limit Switch, Current Sense, RS485");

  // -------------------------
  // Configure Local RP2040 Pins for Motors
  // -------------------------
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

  // -------------------------
  // Initialize MCP23S17 (SPI IO Expander) for limit switches + solenoids
  // -------------------------
  setupMCP23S17();

  // -------------------------
  // Initialize Current Sensors (Analog inputs on RP2040)
  // -------------------------
  setupCurrentSensors();

  // System remains off until a "START" command is received over RS485
  systemInitialized = false;
}

// =============================================================================
// LOOP (Arduino entry point)
// =============================================================================
void loop() {
  // Check if there's an incoming RS485 command and process it
  handleRS485Command();

  // If system is not initialized, motors and solenoids stay off
  if (!systemInitialized) {
    return;
  }

  // If the system is active, monitor motor currents for safety
  monitorCurrentDraw();

  // You can add auto actions here, e.g. driving motors to endstops, etc.
  // Example:
  // driveMotorToEndstop(0, 1); ...
}

// =============================================================================
// setupMCP23S17 - Configure the IO Expander
// =============================================================================
void setupMCP23S17() {
  if (!mcp.begin_SPI(0x20, &SPI)) {
    Serial.println("Error: MCP23S17 not found at address 0x20 via SPI");
    while (true);
  }

  // Bank A: limit switches as inputs with pull-up
  for (uint8_t i = 0; i < 8; i++) {
    mcp.pinMode(i, INPUT_PULLUP);
  }

  // Bank B: solenoids, air compressor (outputs), power good (input), RS485 LEDs (outputs)
  mcp.pinMode(AIR_COMPRESSOR_MOSFET, OUTPUT);
  mcp.pinMode(SOLENOID_3_MOSFET, OUTPUT);
  mcp.pinMode(SOLENOID_2_MOSFET, OUTPUT);
  mcp.pinMode(SOLENOID_1_MOSFET, OUTPUT);
  mcp.pinMode(SOLENOID_0_MOSFET, OUTPUT);

  mcp.pinMode(RS485_TX_LED, OUTPUT);
  mcp.pinMode(RS485_RX_LED, OUTPUT);

  mcp.pinMode(POWER_GOOD_PIN, INPUT);

  Serial.println("MCP23S17 configured for limit switches, solenoids, RS485 LEDs, etc.");
}

// =============================================================================
// setupCurrentSensors - Configure RP2040 ADC for Motor Current Sense
// =============================================================================
void setupCurrentSensors() {
  // If you're using Arduino-Pico core, you can do:
  analogReadResolution(12); // 12-bit => 0..4095
  // Make sure to pinMode them as input
  pinMode(CURRENT_SENSOR_1, INPUT);
  pinMode(CURRENT_SENSOR_2, INPUT);
  pinMode(CURRENT_SENSOR_3, INPUT);
  pinMode(CURRENT_SENSOR_4, INPUT);

  Serial.println("Analog inputs (GPIO26..29) configured for current sensing.");
}

// =============================================================================
// monitorCurrentDraw - Check if motors exceed current thresholds
// =============================================================================
void monitorCurrentDraw() {
  // Example: read each ADC, convert to "amps" using a scale factor
  // Here we do a simplified approach for demonstration

  float raw1 = analogRead(CURRENT_SENSOR_1);
  float raw2 = analogRead(CURRENT_SENSOR_2);
  float raw3 = analogRead(CURRENT_SENSOR_3);
  float raw4 = analogRead(CURRENT_SENSOR_4);

  // Convert raw ADC => voltage => approximate current
  // e.g. (ADC/4095)*3.3 => volts => amps if sense resistor is known
  float current1 = (raw1 / 4095.0f) * 3.3f;
  float current2 = (raw2 / 4095.0f) * 3.3f;
  float current3 = (raw3 / 4095.0f) * 3.3f;
  float current4 = (raw4 / 4095.0f) * 3.3f;

  // Print for debugging
  Serial.printf("Motor Currents => M1:%.2fA  M2:%.2fA  M3:%.2fA  M4:%.2fA\n",
                current1, current2, current3, current4);

  // Check Motor 1 thresholds
  if (current1 < CURRENT_THRESHOLD_MIN_1 || current1 > CURRENT_THRESHOLD_MAX_1) {
    Serial.println("WARNING: Motor 1 current out of range, stopping.");
    stopMotor(0);
  }
  // Check Motor 2 thresholds
  if (current2 < CURRENT_THRESHOLD_MIN_2 || current2 > CURRENT_THRESHOLD_MAX_2) {
    Serial.println("WARNING: Motor 2 current out of range, stopping.");
    stopMotor(1);
  }
  // Check Motor 3 thresholds
  if (current3 < CURRENT_THRESHOLD_MIN_3 || current3 > CURRENT_THRESHOLD_MAX_3) {
    Serial.println("WARNING: Motor 3 current out of range, stopping.");
    stopMotor(2);
  }
  // Check Motor 4 thresholds
  if (current4 < CURRENT_THRESHOLD_MIN_4 || current4 > CURRENT_THRESHOLD_MAX_4) {
    Serial.println("WARNING: Motor 4 current out of range, stopping.");
    stopMotor(3);
  }
}

// =============================================================================
// controlMOSFET - Turn a Solenoid or Air Compressor ON or OFF
// =============================================================================
void controlMOSFET(int solenoidID, bool state) {
  int pin;
  switch (solenoidID) {
    case 0: pin = SOLENOID_0_MOSFET;    break;
    case 1: pin = SOLENOID_1_MOSFET;    break;
    case 2: pin = SOLENOID_2_MOSFET;    break;
    case 3: pin = SOLENOID_3_MOSFET;    break;
    case 4: pin = AIR_COMPRESSOR_MOSFET;break;
    default:
      Serial.println("Invalid Solenoid/Air ID!");
      return;
  }
  // Use the MCP23S17 to drive the corresponding pin
  mcp.digitalWrite(pin, state ? HIGH : LOW);
}

// =============================================================================
// controlMotor - Basic H-Bridge Control with Direction & PWM
// =============================================================================
void controlMotor(int motorID, uint8_t direction, uint8_t pwmValue) {
  // direction => 0=Stop, 1=Forward, 2=Reverse
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

  if (direction == 0) {
    // Stop the motor
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
// stopMotor - Convenience function to instantly stop the motor
// =============================================================================
void stopMotor(int motorID) {
  // Simply calls controlMotor(...) with direction=0, speed=0
  controlMotor(motorID, 0, 0);
}

// =============================================================================
// readLimitSwitch - Query the MCP23S17 for a min or max limit switch
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
  // Active-low => LOW means triggered
  return (mcp.digitalRead(pin) == LOW);
}

// =============================================================================
// driveMotorToEndstop - Move motor until a limit switch triggers
// =============================================================================
void driveMotorToEndstop(int motor, int direction) {
  // direction=1 => forward, 2 => reverse
  bool minReached = false;
  bool maxReached = false;

  // Drive full speed in chosen direction
  controlMotor(motor, direction, 255);

  // Loop until min or max limit is triggered
  while (!minReached && !maxReached) {
    minReached = readLimitSwitch(motor, false);
    maxReached = readLimitSwitch(motor, true);
    delay(10);
  }

  // Stop once limit triggered
  stopMotor(motor);

  if (minReached) {
    Serial.printf("Motor %d: MIN endstop triggered.\n", motor);
  }
  if (maxReached) {
    Serial.printf("Motor %d: MAX endstop triggered.\n", motor);
  }
}

// =============================================================================
// parseRS485Datagram - Convert incoming ASCII string to RS485Datagram
// =============================================================================
RS485Datagram parseRS485Datagram(const String &datagram) {
  RS485Datagram parsed;
  if (datagram.length() < 7) {
    // Not enough data => return empty
    return parsed;
  }
  
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

  parsed.checksum = datagram[datagram.length()-2];
  parsed.endByte  = datagram[datagram.length()-1];

  return parsed;
}

// =============================================================================
// handleRS485Command - Listen on Serial1, parse commands, act
// =============================================================================
void handleRS485Command() {
  if (Serial1.available()) {
    // For demonstration, read until newline
    String rxData = Serial1.readStringUntil('\n');
    if (rxData.length() < 6) {
      // Not valid
      return;
    }
    RS485Datagram datagram = parseRS485Datagram(rxData);

    // Switch on command type
    switch (datagram.command) {
      case CMD_START:
        systemInitialized = true;
        Serial.println("System START => motors/solenoids enabled.");
        break;

      case CMD_STOP:
        systemInitialized = false;
        Serial.println("System STOP => shutting down motors/solenoids.");
        // Stop motors
        stopMotor(0); stopMotor(1); stopMotor(2); stopMotor(3);
        // Turn off solenoids/air
        controlMOSFET(0, false);
        controlMOSFET(1, false);
        controlMOSFET(2, false);
        controlMOSFET(3, false);
        controlMOSFET(4, false);
        break;

      case CMD_MOTOR: {
        // Motor command => motorID + direction
        uint8_t mID = datagram.payload.motorControl.motorID;
        uint8_t dir = datagram.payload.motorControl.direction;
        // Example => full speed
        controlMotor(mID, dir, 255);
        Serial.printf("CMD_MOTOR => Motor %u, Dir=%u\n", mID, dir);
      } break;

      case CMD_SOLENOID: {
        // Solenoid command => solenoidID + state
        uint8_t solID   = datagram.payload.solenoidControl.solenoidID;
        bool onOff      = (datagram.payload.solenoidControl.state == 0x01);
        controlMOSFET(solID, onOff);
        Serial.printf("CMD_SOLENOID => Solenoid %u => %s\n", solID, onOff?"ON":"OFF");
      } break;

      default:
        Serial.println("Unknown RS485 command or invalid data.");
        break;
    }
  }
}

// =============================================================================
// sendRS485Datagram - Construct & transmit a datagram over RS485
// =============================================================================
void sendRS485Datagram(RS485Datagram& datagram) {
  // Build ASCII-based or raw byte datagram
  String out;
  out += (char)datagram.startByte;
  out += (char)datagram.address;
  out += (char)datagram.command;
  out += (char)datagram.dataLength;

  if (datagram.command == CMD_MOTOR) {
    out += (char)datagram.payload.motorControl.motorID;
    out += (char)datagram.payload.motorControl.direction;
  }
  else if (datagram.command == CMD_SOLENOID) {
    out += (char)datagram.payload.solenoidControl.solenoidID;
    out += (char)datagram.payload.solenoidControl.state;
  }

  out += (char)datagram.checksum;
  out += (char)datagram.endByte;

  // Transmit
  Serial1.print(out);
}
