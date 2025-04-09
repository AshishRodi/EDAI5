#include <Wire.h>
#include <MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <avr/wdt.h>  // Watchdog Timer library
#include <SoftwareSerial.h>  // For GSM communication

#include "MAX30100_PulseOximeter.h"
#include <EEPROM.h>  // Include EEPROM library
const int initialPressureAddress = 0;  // EEPROM address to store the initial pressure

// Thresholds and Constants
#define REPORTING_PERIOD_MS 1000
#define WATCHDOG_TIMEOUT WDTO_8S  // 8-second timeout
const int16_t accelThreshold = 30000;
const int16_t gyroThreshold = 30000;
const unsigned long minFallDuration = 150;  // Minimum duration (ms) to consider a fall
const unsigned long minStillDuration = 20000;  // Minimum duration (ms) to consider motionlessness
const float pressureChangeThreshold = 5.0;  // Threshold for floor changes

// Flags and Variables
bool accelSpike = false, gyroSpike = false;
unsigned long accelStartTime = 0, gyroStartTime = 0;
unsigned long stillStartTime = 0;
bool messageSent = false;  // Flag to ensure SMS is sent only once after detection
float initialPressure = 0;
int currentFloor = 0;
unsigned long criticalStartTime = 0;  // Time when critical heart rate was first detected
bool isCritical = false;              // Flag for critical heart rate

unsigned long lastFallTime = 0;
unsigned long lastStillTime = 0;

// Sensors
Adafruit_BMP280 bmp;
MPU6050 mpu;
PulseOximeter pox;
uint32_t tsLastReport = 0;
// GSM Module
SoftwareSerial gsmSerial(7, 8);  // RX, TX pins for GSM module
String phoneNumber = "+919106114583";  // Replace with the recipient phone number


unsigned long lastHeartRateTime = 0; // Tracks the last time heart rate was read
const unsigned long heartRateInterval = 20000; // 20 seconds in milliseconds

// Function prototypes
void sendSMS(String message);
bool detectFall(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz);
bool detectMotionlessness(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz);
int detectFloor(float currentPressure);
void handleFall(int currentFloor, bool critical);
void handleMotionlessness(int currentFloor, bool critical);
void softReset();
void haltProgram();
void initializeSensors();

void setup() {
    pinMode(11, OUTPUT);
    Serial.begin(9600);
    Wire.begin();

    // Disable watchdog timer during setup
    Serial.println("Initializing MAX30100 Pulse Sensor...");

    // Initialize sensors
    
    // Initialize GSM module
    gsmSerial.begin(9600);
    delay(100);
    gsmSerial.println("AT");
    delay(100);
    if (gsmSerial.available()) {
        Serial.println(gsmSerial.readString());
    } else {
        Serial.println("GSM not responding. Check connection.");
    }
    wdt_disable();
    while (!pox.begin()) {
        Serial.println("Failed to initialize MAX30100! Retrying...");
        softReset();
    }
    pox.setIRLedCurrent(MAX30100_LED_CURR_33_8MA);
    //wdt_enable(WATCHDOG_TIMEOUT);

    // MPU6050 Initialization
    mpu.initialize();
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed.");
        while (1);
    }
    wdt_enable(WATCHDOG_TIMEOUT);
    initializeSensors();
    Serial.println("All Sensors successfully initialized");    
    wdt_enable(WDTO_8S); // Set a timeout of 8 seconds
    
}

void loop() {
    wdt_reset();
    //pox.update();
    uint32_t now = millis();
    unsigned long currentTime = millis();
    bool critical = isHeartRateCritical();
    if (critical) {
        if (criticalStartTime == 0) {
            criticalStartTime = now;  // Start tracking when the critical heart rate is first detected
        }
    } else {
        criticalStartTime = 0;  // Reset the timer if the heart rate is no longer critical
    }

    // If the heart rate has been critical for 10 seconds, mark it as critical
    if (critical && (now - criticalStartTime >= 10000)) { // 10 seconds
        if (!isCritical) { // Only update if it hasn't been marked as critical already
            isCritical = true;
            //Serial.println("ALERT! Heart rate is in a critical range for 10 seconds.");
        }
    } else {
        isCritical = false; // Reset if the heart rate goes back to normal before 10 seconds
        //Serial.println("Heart rate is normal.");
    }

    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    float pressure = bmp.readPressure();
    currentFloor = detectFloor(pressure);
    digitalWrite(11, LOW);
    if (detectFall(ax, ay, az, gx, gy, gz)) {
        digitalWrite(11, HIGH);
        String message = "ALERT: Fall detected.";
        if (isCritical) {
            message += " Critical condition!";
        }
        else
        {
          message += "Not Critical";
        }
        message += " Current Floor: " + String(currentFloor);
        
        if (!messageSent) {
            sendSMS(message);
            messageSent = true;
            lastFallTime = millis();
        }
        else
        {
          messageSent = false;
        }
        handleFall(currentFloor, isCritical);
        haltProgram();
    }

    if (detectMotionlessness(ax, ay, az, gx, gy, gz)) {
        digitalWrite(11, HIGH);
        String message = "ALERT: Motionlessness detected.";
        if (isCritical) {
            message += " Critical condition!";
        }
        else
        {
          message += "Not Critical";
        }
        message += " Current Floor: " + String(currentFloor);
        if (!messageSent) {
            sendSMS(message);
            messageSent = true;
            lastStillTime = millis();
        }
        else
        {
          messageSent = false;
        }
        handleMotionlessness(currentFloor, isCritical);
        haltProgram();
    }

    
    delay(10);
}

void sendSMS(String message) {
    gsmSerial.println("AT+CMGF=1");
    delay(100);
    gsmSerial.println("AT+CMGS=\"" + phoneNumber + "\"");
    delay(100);
    gsmSerial.println(message);
    delay(100);
    gsmSerial.write(26);
    Serial.println("SMS Sent");
}

bool detectFall(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz) {
    bool isAccelSpike = abs(ax) > accelThreshold || abs(ay) > accelThreshold || abs(az) > accelThreshold;
    bool isGyroSpike = abs(gx) > gyroThreshold || abs(gy) > gyroThreshold || abs(gz) > gyroThreshold;

    if (isAccelSpike || isGyroSpike) {
        if (!accelSpike && !gyroSpike) {
            accelStartTime = millis();
            gyroStartTime = millis();
            accelSpike = true;
            gyroSpike = true;
        }
    } else {
        accelSpike = false;
        gyroSpike = false;
    }

    return accelSpike && gyroSpike && (millis() - accelStartTime >= minFallDuration);
}

bool detectMotionlessness(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz) {
    bool isAccelStill = abs(ax) < accelThreshold && abs(ay) < accelThreshold && abs(az) < accelThreshold;
    bool isGyroStill = abs(gx) < gyroThreshold && abs(gy) < gyroThreshold && abs(gz) < gyroThreshold;

    if (isAccelStill && isGyroStill) {
        if (stillStartTime == 0) {
            stillStartTime = millis();
        }
        return millis() - stillStartTime >= minStillDuration;
    } else {
        stillStartTime = 0;
    }
    return false;
}

int detectFloor(float currentPressure) {
    float pressureDifference = (currentPressure - initialPressure) / 100.0;
    return round(pressureDifference);
}



void handleFall(int currentFloor, bool critical) {
    Serial.print("Fall detected on floor: ");
    Serial.println(currentFloor);
    if (critical) Serial.println("Critical condition");
    else
      Serial.println("Not Critical");
    digitalWrite(11, HIGH);
    digitalWrite(9, HIGH);
}

void handleMotionlessness(int currentFloor, bool critical) {
    Serial.print("Motionlessness detected on floor: ");
    Serial.println(currentFloor);
    if (critical) Serial.println("Critical condition");
    else
      Serial.println("Not critical");
    pox.update();
    digitalWrite(11, HIGH);
    digitalWrite(9, HIGH);
}

void haltProgram() {
    wdt_disable();
    Serial.println("halting program.");
    while (1);
}

void softReset() {
    pox.update();
    Serial.println("Performing soft reset...");
    wdt_enable(WDTO_15MS);
    while (true);
}

void initializeSensors() {
    // Initialize Pulse Oximeter
    //wdt_reset();
    
    // BMP280 Initialization
    if (!bmp.begin(0x76)) {
        Serial.println("BMP280 connection failed.");
        while (1);
    }
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16, Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_500);
    
    float storedPressure;
    EEPROM.get(initialPressureAddress, storedPressure);

    if (storedPressure == 0) {
        initialPressure = bmp.readPressure();
        EEPROM.put(initialPressureAddress, initialPressure);
        Serial.println("Initial pressure stored.");
    } else {
        initialPressure = storedPressure;
        Serial.println("Using stored initial pressure.");
    }
}

bool isHeartRateCritical() {
    pox.update(); // Update sensor data
    float bpm = pox.getHeartRate(); // Get the current heart rate
    Serial.println(bpm);
    if (bpm < 50 || bpm > 210) {
        return true;  
    } else {
        return false;
    } 
}