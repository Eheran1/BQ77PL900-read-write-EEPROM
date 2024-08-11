/*
read and write BMS EEPROM via Teensy3.2

VSTARTUP = 7.5 V on PACK pin
If a voltage greater than VSTARTUP is applied to the PACK pin, then the bq77PL900 exits from shutdown and
enters normal mode.

From a shutdown situation, the bq77PL900 requires a voltage greater that the start-up voltage (VSTARTUP) applied
to the PACK pin to enable its integrated regulator and provide the regulator power source. Once the REG1 and
REG2 outputs become stable, the power source of the regulator is switched to BAT.
*/


#include <Wire.h>


// Define a structure for register settings
struct RegisterSetting {
  byte reg;
  byte value;
};

// Array of register settings
RegisterSetting registerSettings[] = {
  { 0x06, 0b01001100 },  // Register 0x06
  { 0x07, 0b00001110 },  // Register 0x07
  { 0x08, 0b00011111 },  // Register 0x08
  { 0x09, 0b00011111 },  // Register 0x09
  { 0x0A, 0b11111111 }   // Register 0x0A
};

const int numRegisters = sizeof(registerSettings) / sizeof(registerSettings[0]);
const int sdataPin = 18;  // hardware I2C on Teensy3.2
const int sclkPin = 19;
const int eepromPin = 10;        // Example pin for EEPROM enable
const int deviceAddress = 0x10;  // 7-bit address (0x20 shifted right by 1)
int verification = 0;
bool isVGOOD = 0;
const int numCells = 10;             // Number of cells (0 to 9)
const int stabilizationDelay = 100;  // Time to wait for ADC stabilization (ms)
const int averagingDuration = 100;   // Duration for averaging (ms)
#include <ADC.h>
ADC *adc = new ADC();  // adc object;


void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("running");
  delay(5000);
  Wire.begin();
  pinMode(eepromPin, OUTPUT);

  //https://github.com/pedvide/ADC/blob/master/examples/analogRead/analogRead.ino
  adc->adc0->setAveraging(32);                                          // set number of averages
  adc->adc0->setResolution(16);                                         // set bits of resolution
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_LOW_SPEED);  // change the conversion speed
  // it can be any of the ADC_MED_SPEED enum: VERY_LOW_SPEED, LOW_SPEED, MED_SPEED, HIGH_SPEED or VERY_HIGH_SPEED
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_LOW_SPEED);  // change the sampling speed


  delay(2000);
  readEEPROM();
  Serial.println("Starting programming");
  delay(1000);
  eepromProgrammingSequence();
  readEEPROM();
  // enter host mode
  writeRegister(0x02, 0b00000010);  // host mode 1
  // Set VAEN and ensure BAT and PACK are not set in register 0x03
  writeRegister(0x03, 0b00000001);  // Set VAEN (bit 0) to 1, others to 0
  writeRegister(0x04, 0b00000000);  // all balance off
  readEEPROM();
}

void loop() {

  selectAndReadCells();
  delay(1000);  // Delay for testing, remove as needed
}

void selectAndReadCells() {

  for (int cell = 0; cell < numCells; cell++) {
    byte cellSelectValue = cell & 0x0F;                           // Ensure only lower 4 bits are used
    byte cellSelRegisterValue = (0b0000 << 4) | cellSelectValue;  // CAL2–CAL0 = 000, CELL4–CELL1 = cell number
    writeRegister(0x05, cellSelRegisterValue);                    // Write to Cell_Sel register (0x05)

    delay(stabilizationDelay);             // Wait for the voltage to stabilize
    unsigned long startMillis = millis();  // Record the start time
    int analogValue = 0;                   // Variable to store the analog value
    int sampleCount = 0;                   // Variable to count the number of samples
    float Voltage_avg = 0;
    float Cell_Voltage = 0;
    int divisor = adc->adc0->getMaxValue();


    // Read analog value for 5 milliseconds
    while (millis() - startMillis < 5) {
      analogValue += analogRead(0);  // A0 = pin 14
      sampleCount++;                 // Increment sample count
    }

    // Calculate the average value
    if (sampleCount > 0) {
      analogValue /= sampleCount;
    }

    Voltage_avg = (analogValue * 3.3) / divisor;
    //Voltage = 0.975 - Cell_Voltage * 0.15;
    Cell_Voltage = (0.975 - Voltage_avg) / 0.15;
    Serial.print("Cell ");
    Serial.print(cell);
    Serial.print(": ");
    Serial.print(Cell_Voltage, 4);
    Serial.print(", raw: ");
    Serial.print(Voltage_avg, 4);
    Serial.println();
  }
  Serial.println();
}



void readEEPROM() {
  for (int reg = 0x00; reg <= 0x0B; reg++) {
    Wire.beginTransmission(deviceAddress);
    Wire.write(reg);              // Send the register address
    Wire.endTransmission(false);  // Keep the connection active

    Wire.requestFrom(deviceAddress, 1);
    if (Wire.available()) {
      byte value = Wire.read();
      Serial.print("Register 0x");
      Serial.print(reg, HEX);  // Print the register address in hex
      Serial.print(": ");
      Serial.print("0b");
      Serial.println(value, BIN);  // Print the EEPROM data in binary
    } else {
      Serial.print("Error: No response from register 0x");
      Serial.println(reg, HEX);
    }
  }
}


void writeRegister(byte reg, byte value) {
  Wire.beginTransmission(deviceAddress);
  Wire.write(reg);    // Specify register
  Wire.write(value);  // Write value to register
  Wire.endTransmission();
  delay(10);  // Small delay for stability
}

byte readRegister(byte reg) {
  Wire.beginTransmission(deviceAddress);
  Wire.write(reg);
  Wire.endTransmission(false);  // Keep the connection active

  Wire.requestFrom(deviceAddress, 1);
  if (Wire.available()) {
    return Wire.read();
  } else {
    Serial.println("Error: No response");
    return 0xFF;  // Return an error value if no response
  }
}

bool verifyRegister(byte reg, byte expectedValue) {
  byte actualValue = readRegister(reg);
  if (actualValue == expectedValue) {
    Serial.print("Register 0x");
    Serial.print(reg, HEX);
    Serial.println(" verified successfully.");
    return true;
  } else {
    Serial.print("Register 0x");
    Serial.print(reg, HEX);
    Serial.print(" verification failed. Expected: 0b");
    Serial.print(expectedValue, BIN);
    Serial.print(", Actual: 0b");
    Serial.println(actualValue, BIN);
    return false;
  }
}

void eepromProgrammingSequence() {
  // Sequence based on the flowchart provided in the datasheet

  // Step 1: Set BAT Voltage or PACK Voltage = 20 V
  // (Handle externally, not in code)

  // Step 2: Send OV/UV/SC/OL Register = Desired Value
  // Write and verify all registers
  for (int i = 0; i < numRegisters; i++) {
    byte reg = registerSettings[i].reg;
    byte value = registerSettings[i].value;

    // Write the value to the register
    writeRegister(reg, value);
  }

  // Step 3: Send I2C Command EEPROM Register = 0110 0010 (0x62)
  writeRegister(0x0B, 0x62);

  // Step 4: Verify the OV/UV/OL/SC Values
  verification = 0;  // Reset verification counter
  for (int i = 0; i < numRegisters; i++) {
    byte reg = registerSettings[i].reg;
    byte value = registerSettings[i].value;
    if (verifyRegister(reg, value)) {
      verification++;
    }
  }

  // only proceed if everything was correct
  if (verification == numRegisters) {
    // Step 5: Send I2C Command EEPROM Register = 0100 0001 (0x41)
    writeRegister(0x0B, 0x41);

    // Step 6: Wait 1 ms
    delay(1);

    // Step 7: Confirm the Status Register VGOOD = 1
    byte statusReg = readRegister(0x00);      // Read the Status register (0x00)
    isVGOOD = (statusReg & 0b00100000) != 0;  // Check if the VGOOD bit (bit 5) is set
    if (isVGOOD) {
      Serial.println("VGOOD is set");
    } else {
      Serial.println("Error: VGOOD is not set");
      return;
    }
  } else {
    Serial.println("Error: Verification 1 failed");
  }

  if (isVGOOD) {
    // Step 8: Set EEPROM Pin = VLOG Voltage (3.3 V or 5.5 V)
    digitalWrite(eepromPin, HIGH);

    // Step 9: Wait 100 ms
    delay(100);

    // Step 10: Send I2C Command EEPROM Register = 0000 0000 (0x00)
    writeRegister(0x0B, 0x00);

    // Step 11: Set EEPROM Pin = GND
    digitalWrite(eepromPin, LOW);

    // Step 12: Verify the OV/UV/OL/SC Value (implementation depends on available methods)
    verification = 0;  // Reset verification counter
    for (int i = 0; i < numRegisters; i++) {
      byte reg = registerSettings[i].reg;
      byte value = registerSettings[i].value;
      if (verifyRegister(reg, value)) {
        verification++;
      }
    }

    if (verification == numRegisters) {
      Serial.println("EEPROM Programming Complete");
    } else {
      Serial.println("Error: Verification 2 failed");
    }

  } else {
    Serial.println("Error: EEPROM Programming failed");
  }
}
