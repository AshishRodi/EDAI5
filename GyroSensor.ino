#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// Thresholds and constants
const int16_t accelThreshold = 23500;      // Acceleration threshold for fall detection
const int16_t gyroThreshold = 23500;       // Gyroscope threshold for fall detection
const unsigned long minFallDuration = 100; // Minimum duration (milliseconds) to consider it a fall
const unsigned long minStillDuration = 5000; // Minimum duration (milliseconds) to consider it motionless

// State variables
bool fallDetected = false;
unsigned long accelStartTime = 0;
unsigned long gyroStartTime = 0;
bool accelSpike = false;
bool gyroSpike = false;
bool isMotionless = false;
unsigned long stillStartTime = 0;

void setup() {
  Wire.begin();
  Serial.begin(9600);

  mpu.initialize();
  Serial.println("Initializing MPU6050...");

  if (mpu.testConnection()) {
    Serial.println("MPU6050 connection successful");
  } else {
    Serial.println("MPU6050 connection failed. Please check wiring and address.");
    while (1); // Halt if connection fails
  }
}

void loop() {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;

  // Read sensor data
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Check for significant acceleration spikes for fall detection
  if (abs(ax) > accelThreshold || abs(ay) > accelThreshold || abs(az) > accelThreshold) {
    if (!accelSpike) {
      accelSpike = true;
      accelStartTime = millis();  // Record the start time of the spike
    }
  } else {
    accelSpike = false;  // Reset if the acceleration is back to normal
  }

  // Check for significant gyroscope spikes for fall detection
  if (abs(gx) > gyroThreshold || abs(gy) > gyroThreshold || abs(gz) > gyroThreshold) {
    if (!gyroSpike) {
      gyroSpike = true;
      gyroStartTime = millis();  // Record the start time of the spike
    }
  } else {
    gyroSpike = false;  // Reset if the gyroscope values are back to normal
  }

  // Fall detection logic with time check
  if (accelSpike && gyroSpike && !fallDetected) {
    unsigned long currentTime = millis();
    // Check if the spikes have lasted long enough to be considered a fall
    if ((currentTime - accelStartTime >= minFallDuration) && (currentTime - gyroStartTime >= minFallDuration)) {
      fallDetected = true;  // Mark the fall as detected

      // Print the values that caused the detection
      Serial.println("Fall detected!");
      Serial.print("Accelerometer values: ax = "); Serial.print(ax);
      Serial.print(", ay = "); Serial.print(ay);
      Serial.print(", az = "); Serial.println(az);

      Serial.print("Gyroscope values: gx = "); Serial.print(gx);
      Serial.print(", gy = "); Serial.print(gy);
      Serial.print(", gz = "); Serial.println(gz);

      // Print time of detection
      Serial.print("Time to detect: ");
      Serial.print(currentTime - min(accelStartTime, gyroStartTime));  // Time taken to detect fall
      Serial.println(" ms");

      // Halt the program after detecting a fall
      while (1); // Infinite loop to stop further execution
    }
  } else {
    if (fallDetected) {
      Serial.println("Fall ended.");
      fallDetected = false;  // Reset fall detection after some time
    }
  }

  // Motionless detection logic
  if (abs(ax) < accelThreshold && abs(ay) < accelThreshold && abs(az) < accelThreshold &&
      abs(gx) < gyroThreshold && abs(gy) < gyroThreshold && abs(gz) < gyroThreshold) {
    
    // Person is motionless
    if (!isMotionless) {
      isMotionless = true;
      stillStartTime = millis();  // Record the start time of stillness
      //Serial.println("The person is not moving.");
    }

    // Check if the duration of stillness is enough to declare motionlessness
    if (isMotionless && (millis() - stillStartTime >= minStillDuration)) {
      Serial.println("The person is motionless.");
    }
    
  } else {
    // Person is moving
    if (isMotionless) {
      Serial.println("Person has started moving.");
    }
    isMotionless = false;  // Reset if there is movement
  }

  delay(10);  // Short delay to stabilize readings
}
