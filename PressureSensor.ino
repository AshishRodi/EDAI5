#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

// Create an instance of the BMP280 sensor
Adafruit_BMP280 bmp; 

// Variables to store initial pressure and the last detected floor
float initialPressure = 0;
int lastFloor = 0;
int currentFloor = 0; // Initialize current floor to 0

// Threshold for floor change detection
const float pressureChangeThreshold = 5.0; // Threshold in hPa to consider as a floor change

void setup() {
  Serial.begin(9600);
  
  // Initialize the BMP280 sensor
  if (!bmp.begin(0x76)) {
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1);
  }

  // Optionally configure the sensor
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  // Store the initial pressure reading
  initialPressure = bmp.readPressure();
}

void loop() {
  // Read the current pressure from the BMP280 sensor
  float pressure = bmp.readPressure();
  
  // Calculate the change in pressure (in hPa)
  float pressureDifference = (pressure - initialPressure) / 100.0; // Convert from Pa to hPa

  // Convert the pressure difference to floor change
  // 12 hPa approximately corresponds to a height change of 100 meters
  // Reverse the sign to reflect climbing up as positive and going down as negative
  int newFloor = static_cast<int>(-(pressureDifference / 12.0 * 100.0 / 3.0));
  
  // Update the current floor only if it has changed significantly
  if (abs(newFloor - currentFloor) >= 1) { // Change threshold
    currentFloor = newFloor; // Update the current floor
    Serial.print("Current Floor: ");
    Serial.println(currentFloor);
  }

  // Print the current pressure for reference
  Serial.print("Pressure = ");
  Serial.print(pressure / 100.0); // Convert Pa to hPa
  Serial.println(" hPa");

  // Always print the current floor
  Serial.print("Displayed Current Floor: ");
  Serial.println(currentFloor);

  delay(2000);  // Delay for readability
}
