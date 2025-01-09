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

// MCP23S17T-E/ML IO Expander Configuration
Adafruit_MCP23X17 mcp;  // Create an instance of the MCP23X17 class for SPI communication

// Pin assignments for the motors and H-Bridge control
#define M1PWM 13   // PWM signal for Motor 1 (controls speed)
#define M2PWM 10   // PWM signal for Motor 2 (controls speed)
#define M3PWM 6    // PWM signal for Motor 3 (controls speed)
#define M4PWM 1    // PWM signal for Motor 4 (controls speed)

#define M1INA 15   // Motor 1 direction control (IN A pin of H-Bridge)
#define M1INB 12   // Motor 1 direction control (IN B pin of H-Bridge)
#define M2INA 11   // Motor 2 direction control (IN A pin of H-Bridge)
#define M2INB 8    // Motor 2 direction control (IN B pin of H-Bridge)
#define M3INA 7    // Motor 3 direction control (IN A pin of H-Bridge)
#define M3INB 4    // Motor 3 direction control (IN B pin of H-Bridge)
#define M4INA 3    // Motor 4 direction control (IN A pin of H-Bridge)
#define M4INB 0    // Motor 4 direction control (IN B pin of H-Bridge)

#define M1ENA 14   // Enable pin for Motor 1 H-Bridge (Turns motor on/off)
#define M2ENA 9    // Enable pin for Motor 2 H-Bridge (Turns motor on/off)
#define M3ENA 5    // Enable pin for Motor 3 H-Bridge (Turns motor on/off)
#define M4ENA 2    // Enable pin for Motor 4 H-Bridge (Turns motor on/off)

// Limit switch pin assignments for the MCP23S17T-E/ML (Bank A)
#define LIMIT_SWITCH_MIN_1 0  // GPA0 (Motor 1 Minimum Limit)
#define LIMIT_SWITCH_MIN_2 1  // GPA1 (Motor 2 Minimum Limit)
#define LIMIT_SWITCH_MIN_3 2  // GPA2 (Motor 3 Minimum Limit)
#define LIMIT_SWITCH_MIN_4 3  // GPA3 (Motor 4 Minimum Limit)
#define LIMIT_SWITCH_MAX_1 4  // GPA4 (Motor 1 Maximum Limit)
#define LIMIT_SWITCH_MAX_2 5  // GPA5 (Motor 2 Maximum Limit)
#define LIMIT_SWITCH_MAX_3 6  // GPA6 (Motor 3 Maximum Limit)
#define LIMIT_SWITCH_MAX_4 7  // GPA7 (Motor 4 Maximum Limit)

// New Pin assignments for controlling solenoids, air compressor, LEDs, and power monitoring (Bank B)
#define AIR_COMPRESSOR_MOSFET 8  // GPB0  (Control Air Compressor MOSFET)
#define SOLENOID_3_MOSFET 9     // GPB1  (Control Solenoid 3 MOSFET)
#define SOLENOID_2_MOSFET 10    // GPB2  (Control Solenoid 2 MOSFET)
#define SOLENOID_1_MOSFET 11    // GPB3  (Control Solenoid 1 MOSFET)
#define SOLENOID_0_MOSFET 12    // GPB4  (Control Solenoid 0 MOSFET)
#define POWER_GOOD_PIN 13       // GPB5  (3.3V DCDC Regulator Power Good Signal (Input))
#define RS485_TX_LED 14         // GPB6  (RS485 TX LED (Output))
#define RS485_RX_LED 15         // GPB7  (RS485 RX LED (Output))

// Variables for RS485 communication and command processing
String receivedCommand = "";  // To store the incoming RS485 command
bool systemInitialized = false; // Flag to track if the system is turned on by a valid command

// Initialize the MCP23S17T-E/ML and SPI
void setupMCP23S17() {
  // Initialize the MCP23S17 over SPI with the default address 0x20
  if (!mcp.begin_SPI(0x20, &SPI)) {  
    Serial.println("Error: MCP23S17T-E/ML not found");
    while (1);  // Stop execution if initialization fails
  }
  
  // Set all pins for limit switches (GPA0 to GPA7) as inputs with pull-ups enabled
  for (uint8_t i = 0; i < 8; i++) {
    mcp.pinMode(i, INPUT_PULLUP);  // Set the pins as INPUT_PULLUP for limit switches
  }

  // Set pins for solenoids, air compressor, and LEDs as outputs
  mcp.pinMode(AIR_COMPRESSOR_MOSFET, OUTPUT);
  mcp.pinMode(SOLENOID_3_MOSFET, OUTPUT);
  mcp.pinMode(SOLENOID_2_MOSFET, OUTPUT);
  mcp.pinMode(SOLENOID_1_MOSFET, OUTPUT);
  mcp.pinMode(SOLENOID_0_MOSFET, OUTPUT);
  mcp.pinMode(RS485_TX_LED, OUTPUT);
  mcp.pinMode(RS485_RX_LED, OUTPUT);

  // Set Power Good pin as input
  mcp.pinMode(POWER_GOOD_PIN, INPUT);
  
  Serial.println("MCP23S17T-E/ML initialized successfully");
}

// Function to control the solenoids and air compressor MOSFETs
void controlMOSFET(int mosfetPin, bool state) {
  // Set the state of the MOSFET (HIGH turns it on, LOW turns it off)
  mcp.digitalWrite(mosfetPin, state ? HIGH : LOW);
}

// Function to check the power good signal
bool checkPowerGood() {
  // Read the Power Good pin (LOW means power is good)
  return mcp.digitalRead(POWER_GOOD_PIN) == LOW;
}

// Function to control the RS485 TX LED
void toggleRS485TXLED(bool state) {
  // Turn the RS485 TX LED on or off based on the state
  mcp.digitalWrite(RS485_TX_LED, state ? HIGH : LOW);
}

// Function to control the RS485 RX LED
void toggleRS485RXLED(bool state) {
  // Turn the RS485 RX LED on or off based on the state
  mcp.digitalWrite(RS485_RX_LED, state ? HIGH : LOW);
}

// Function to read the state of the limit switches
bool readLimitSwitch(int motor, bool isMaxLimit) {
  // Determine the correct pin based on motor number and limit switch type (min/max)
  int pin;
  switch (motor) {
    case 1:
      pin = isMaxLimit ? LIMIT_SWITCH_MAX_1 : LIMIT_SWITCH_MIN_1;
      break;
    case 2:
      pin = isMaxLimit ? LIMIT_SWITCH_MAX_2 : LIMIT_SWITCH_MIN_2;
      break;
    case 3:
      pin = isMaxLimit ? LIMIT_SWITCH_MAX_3 : LIMIT_SWITCH_MIN_3;
      break;
    case 4:
      pin = isMaxLimit ? LIMIT_SWITCH_MAX_4 : LIMIT_SWITCH_MIN_4;
      break;
    default:
      return false;  // Invalid motor number
  }
  
  // Read the state of the pin (LOW means the switch is activated)
  return mcp.digitalRead(pin) == LOW;
}

// Function to drive a motor to the limit switch (either min or max) using MCP23S17T-E/ML
void driveMotorToEndstop(int motor, int direction) {
  bool minLimitReached = false;
  bool maxLimitReached = false;

  // Drive the motor in the specified direction (1 = forward, 0 = reverse)
  controlMotor(motor, direction, 255); // Full speed

  // Continue moving the motor until one of the limit switches is triggered
  while (!minLimitReached && !maxLimitReached) {
    minLimitReached = readLimitSwitch(motor, false);  // Check minimum limit switch
    maxLimitReached = readLimitSwitch(motor, true);   // Check maximum limit switch
    delay(10);  // Small delay to avoid excessive CPU usage
  }

  // Stop the motor once the limit switch is triggered
  stopMotor(motor);

  // Output to serial the result of which limit switch was triggered
  if (minLimitReached) {
    Serial.print("Motor ");
    Serial.print(motor);
    Serial.println(" reached minimum limit switch.");
  }
  if (maxLimitReached) {
    Serial.print("Motor ");
    Serial.print(motor);
    Serial.println(" reached maximum limit switch.");
  }
}

// Function to control the motor (existing, no changes)
void controlMotor(int motor, bool direction, int pwmValue) {
  switch (motor) {
    case 1:
      digitalWrite(M1INA, direction ? HIGH : LOW);
      digitalWrite(M1INB, direction ? LOW : HIGH);
      analogWrite(M1PWM, pwmValue);
      digitalWrite(M1ENA, HIGH);
      break;
    case 2:
      digitalWrite(M2INA, direction ? HIGH : LOW);
      digitalWrite(M2INB, direction ? LOW : HIGH);
      analogWrite(M2PWM, pwmValue);
      digitalWrite(M2ENA, HIGH);
      break;
    case 3:
      digitalWrite(M3INA, direction ? HIGH : LOW);
      digitalWrite(M3INB, direction ? LOW : HIGH);
      analogWrite(M3PWM, pwmValue);
      digitalWrite(M3ENA, HIGH);
      break;
    case 4:
      digitalWrite(M4INA, direction ? HIGH : LOW);
      digitalWrite(M4INB, direction ? LOW : HIGH);
      analogWrite(M4PWM, pwmValue);
      digitalWrite(M4ENA, HIGH);
      break;
  }
}

// Function to stop the motor (existing, no changes)
void stopMotor(int motor) {
  switch (motor) {
    case 1:
      digitalWrite(M1ENA, LOW);
      break;
    case 2:
      digitalWrite(M2ENA, LOW);
      break;
    case 3:
      digitalWrite(M3ENA, LOW);
      break;
    case 4:
      digitalWrite(M4ENA, LOW);
      break;
  }
}

// Function to listen for commands over RS485
void listenForCommands() {
  // Check if data is available to read from RS485
  if (Serial1.available()) {
    // Read the incoming data and store it in the receivedCommand string
    receivedCommand = Serial1.readStringUntil('\n');  // Assuming a newline separated command
    Serial.print("Received Command: ");
    Serial.println(receivedCommand);
    
    // Process the command based on its content (you can add specific logic here)
    if (receivedCommand == "START") {
      systemInitialized = true;  // Turn on the system
      Serial.println("System Initialized and Running.");
    } else if (receivedCommand == "STOP") {
      systemInitialized = false;  // Turn off the system
      Serial.println("System Stopped.");
      // Turn off all motors, solenoids, and compressors
      stopMotor(1);
      stopMotor(2);
      stopMotor(3);
      stopMotor(4);
      controlMOSFET(AIR_COMPRESSOR_MOSFET, false);
      controlMOSFET(SOLENOID_0_MOSFET, false);
      controlMOSFET(SOLENOID_1_MOSFET, false);
      controlMOSFET(SOLENOID_2_MOSFET, false);
      controlMOSFET(SOLENOID_3_MOSFET, false);
    }
  }
}

void setup() {
  // Start serial communication for debugging and monitoring
  Serial.begin(115200);  
  Serial.println("RP2040 Motor Controller Initialized");

  // Initialize the MCP23S17 for SPI communication
  setupMCP23S17();
}

void loop() {
  // Listen for commands over RS485 and process them
  listenForCommands();

  // Default system behavior: keep everything off unless "START" command received
  if (!systemInitialized) {
    // Everything should remain off (already set to off in the STOP command)
  } else {
    // Perform motor, solenoid, or compressor control based on received commands
    // Example: Move Motor 1 forward until the minimum limit switch is triggered
    driveMotorToEndstop(1, 1);  // Drive Motor 1 forward, stop at limit switch
    delay(1000);

    // Example: Move Motor 2 backward until the maximum limit switch is triggered
    driveMotorToEndstop(2, 0);  // Drive Motor 2 backward, stop at limit switch
    delay(1000);
  }
}
