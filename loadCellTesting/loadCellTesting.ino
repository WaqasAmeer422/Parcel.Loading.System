#include <Arduino.h>
#include <HX711_ADC.h>

// Pins mapping for Seeed Studio XIAO ESP32C3:
// Scale 1:
const int dout_1 = 4; // Physical D2
const int sck_1 = 5;  // Physical D3

// Scale 2:
const int dout_2 = 3; // Physical D1
const int sck_2 = 2;  // Physical D0

// Create two separate load cell objects:
HX711_ADC LoadCell_1(dout_1, sck_1);
HX711_ADC LoadCell_2(dout_2, sck_2);

// --- UPDATE THESE VALUES WITH YOUR CALIBRATION FACTORS ---
// Set to 1.0 initially to print raw values. Once calibrated in grams, replace with your factors.
const float CAL_FACTOR_1 = 396.66; //405.02now
const float CAL_FACTOR_2 = 379.83; //383.05; //379.83; 

unsigned long lastPrintTime = 0;
const int printInterval = 500; // Print data every 500ms

void setup() {
  Serial.begin(115200);
  delay(1000); 
  
  Serial.println("\n--- Starting HX711_ADC Dual Cell Test ---");

  // Initialize both HX711 modules
  LoadCell_1.begin();
  LoadCell_2.begin();

  // Stabilize scales and tare (zero out the scales) on startup
  unsigned long stabilizingTime = 2000; 
  boolean performTare = true; 
  
  byte scale1_ready = 0;
  byte scale2_ready = 0;

  Serial.println("Stabilizing and taring both scales... Keep them empty.");

  // Startup and tare both modules simultaneously (non-blocking loop)
  while ((scale1_ready + scale2_ready) < 2) {
    if (!scale1_ready) scale1_ready = LoadCell_1.startMultiple(stabilizingTime, performTare);
    if (!scale2_ready) scale2_ready = LoadCell_2.startMultiple(stabilizingTime, performTare);
  }

  // Verify hardware responses
  if (LoadCell_1.getTareTimeoutFlag()) {
    Serial.println("Error: Timeout on Scale 1. Check your Scale 1 wiring!");
  }
  if (LoadCell_2.getTareTimeoutFlag()) {
    Serial.println("Error: Timeout on Scale 2. Check your Scale 2 wiring!");
  }

  if (LoadCell_1.getTareTimeoutFlag() || LoadCell_2.getTareTimeoutFlag()) {
    while (1); // Halt execution if connection failed
  }

  // Set calibration factors
  LoadCell_1.setCalFactor(CAL_FACTOR_1);
  LoadCell_2.setCalFactor(CAL_FACTOR_2);

  Serial.println("Startup complete! Reading values...");
  Serial.println("Send 't' to re-tare both scales.");
}

void loop() {
  static boolean newDataReady = false;

  // Continually check both scales for new data
  if (LoadCell_1.update()) {
    newDataReady = true;
  }
  LoadCell_2.update();

  // If new readings are ready, print weight in grams and kilograms
  if (newDataReady) {
    if (millis() - lastPrintTime >= printInterval) {
      float weight1_g = LoadCell_1.getData();
      float weight1_kg = weight1_g / 1000.0;

      float weight2_g = LoadCell_2.getData();
      float weight2_kg = weight2_g / 1000.0;
      
      // Serial.print("Scale 1: ");
      // Serial.print(weight1_g, 1);
      // Serial.print(" g (");
      // Serial.print(weight1_kg, 3);
      // Serial.print(" kg)"); 
      // Serial.print(" kg |  Scale 2: ");
      Serial.print("Scale 2: ");
      Serial.print(weight2_g, 1);
      Serial.print(" g (");
      Serial.print(weight2_kg, 3);
      Serial.println(" kg)");
      
      newDataReady = false;
      lastPrintTime = millis();
    }
  }

  // Check for incoming serial command 't' to tare
  if (Serial.available() > 0) {
    char inByte = Serial.read();
    if (inByte == 't' || inByte == 'T') {
      Serial.println("Taring both scales...");
      LoadCell_1.tareNoDelay();
      LoadCell_2.tareNoDelay();
    }
  }

  // Check if the non-blocking tare is completed for both
  if (LoadCell_1.getTareStatus() == true) {
    Serial.println("Scale 1 tare complete.");
  }
  if (LoadCell_2.getTareStatus() == true) {
    //Serial.println("Scale 2 tare complete.");
  }
}