#include <SoftwareSerial.h>

// Set your SIM800 module TX and RX pins here
SoftwareSerial sim800(2, 3); // RX, TX

void setup() {
  // Start communication with the SIM800 module and Serial Monitor
  sim800.begin(9600);
  Serial.begin(9600);
  
  delay(1000); // Allow module to initialize

  Serial.println("Sending SMS...");
  sendSMS("9106114583", "Hello! This is a test message from Arduino + SIM800C.");
  Serial.println("Message sent!");
}

void loop() {
  // Keep reading the incoming serial data from the SIM800C
  if (sim800.available()) {
    Serial.write(sim800.read());
  }
  if (Serial.available()) {
    sim800.write(Serial.read());
  }
}

void sendSMS(String phoneNumber, String message) {
  sim800.println("AT+CMGF=1");  // Set SMS mode to text
  delay(1000);

  sim800.print("AT+CMGS=\"");    // Command to send SMS
  sim800.print(phoneNumber);
  sim800.println("\"");
  delay(1000);

  sim800.print(message);         // The message to send
  delay(1000);

  sim800.write(26);              // ASCII code for CTRL+Z to send the message
  delay(1000);
}
