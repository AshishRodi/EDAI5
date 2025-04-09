#include <Wire.h>
#include "MAX30100_PulseOximeter.h"
#include <avr/wdt.h> // Include watchdog timer library for AVR boards

#define REPORTING_PERIOD_MS 1000
#define WATCHDOG_TIMEOUT WDTO_8S // 8-second timeout

PulseOximeter pox;

uint32_t tsLastReport = 0;

// Variables for critical heart rate detection
unsigned long criticalStartTime = 0;  // Time when critical heart rate was first detected
bool isCritical = false;              // Flag to track if heart rate is critical

// Watchdog Soft Reset
void softReset() {
    Serial.println("Performing soft reset...");
    wdt_enable(WDTO_15MS); // Set a short watchdog timer timeout for reset
    while (true);          // Wait for the watchdog timer to reset the board
}

// Initialize the Heart Rate Sensor
void setup() {
    // Temporarily disable watchdog timer during setup
    wdt_disable();

    Serial.begin(9600);
    Serial.println("Initializing MAX30100 Pulse Sensor...");

    // Initialize the Pulse Oximeter
    while (!pox.begin()) {
        Serial.println("FAILED to initialize! Retrying...");
        softReset();
    }
    Serial.println("Sensor initialized successfully.");

    // Set the LED current (you can adjust if needed)
    pox.setIRLedCurrent(MAX30100_LED_CURR_33_8MA);

    // Re-enable the watchdog timer after successful setup
    wdt_enable(WATCHDOG_TIMEOUT);
}

// Heart Rate Check Function
bool isHeartRateCritical() {
    pox.update(); // Update sensor data
    float bpm = pox.getHeartRate(); // Get the current heart rate

    if (bpm < 30 || bpm > 210) {
        return true;
    } else {
        return false;
    } 
}

// Main Loop
void loop() {
    // Reset the watchdog timer at the beginning of each loop
    wdt_reset();

    // Check heart rate periodically
    uint32_t now = millis();
    bool critical = isHeartRateCritical();

    // If the heart rate is critical, track the time
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
            Serial.println("ALERT! Heart rate is in a critical range for 10 seconds.");
        }
        else
        {
          Serial.println("Heart rate normal");
        }

    } else {
        isCritical = false; // Reset if the heart rate goes back to normal before 10 seconds
        //Serial.println("Heart rate is normal.");
    }
    delay(10);  // Small delay for stability
}
