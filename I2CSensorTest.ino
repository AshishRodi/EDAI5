#include <Wire.h>

void setup() {
  Wire.begin();               // Initialize the I2C communication
  Serial.begin(9600);         // Start Serial communication at 9600 baud rate
  Serial.println("Starting I2C Scanner...");
}

void loop() {
  byte error, address;
  int nDevices = 0;

  Serial.println("Scanning for I2C devices...");

  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println(" !");
      nDevices++;
    } else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }

  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  } else {
    Serial.println("Scan complete.\n");
  }
  
  delay(5000);  // Wait for 5 seconds before the next scan
}
