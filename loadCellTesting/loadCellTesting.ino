#include <Arduino.h>
#include <HX711_ADC.h>

// Hardware connection pins (using explicit GPIO numbers to avoid board macro mapping issues):
// On Seeed Studio XIAO ESP32C3:
// - Physical Pin D2 is GPIO 4 (connects to HX711 DOUT/DT)
// - Physical Pin D3 is GPIO 5 (connects to HX711 SCK)
const int HX711_dout = 4; 
const int HX711_sck = 5;  

// HX711_ADC constructor:
HX711_ADC LoadCell(HX711_dout, HX711_sck);

// --- UPDATE THIS VALUE WITH YOUR CALCULATION ---
// Set to 1.0 initially to print raw values. Once calibrated in grams, replace with your factor.
const float CALIBRATION_FACTOR = 1.0; 

unsigned long lastPrintTime = 0;
const int printInterval = 500; // Print data every 500ms

void setup() {
  Serial.begin(115200);
  delay(1000); 
  
  Serial.println("\n--- Starting HX711_ADC Load Cell Test ---");

  // Initialize the HX711_ADC library
  LoadCell.begin();

  // Stabilize scale and tare (zero out the scale) on startup
  unsigned long stabilizingTime = 2000; 
  boolean performTare = true; 
  
  Serial.println("Stabilizing and taring... Keep the scale empty.");
  LoadCell.start(stabilizingTime, performTare);

  // Verify hardware response
  if (LoadCell.getTareTimeoutFlag() || LoadCell.getSignalTimeoutFlag()) {
    Serial.println("Error: Timeout. Check your MCU > HX711 wiring and pins!");
    while (1); // Halt execution if connection failed
  } else {
    // Set calibration factor.
    LoadCell.setCalFactor(CALIBRATION_FACTOR); 
    Serial.println("Startup complete! Reading values...");
    Serial.println("Send 't' to re-tare the scale.");
  }
}

void loop() {
  static boolean newDataReady = false;

  // The update() function checks if new data is available from the HX711 chip
  if (LoadCell.update()) {
    newDataReady = true;
  }

  // If a new reading is ready, print it in grams and kilograms
  if (newDataReady) {
    if (millis() - lastPrintTime >= printInterval) {
      float weight_g = LoadCell.getData();
      float weight_kg = weight_g / 1000.0;
      
      Serial.print("Weight: ");
      Serial.print(weight_g, 1);
      Serial.print(" g  |  ");
      Serial.print(weight_kg, 3);
      Serial.println(" kg");
      
      newDataReady = false;
      lastPrintTime = millis();
    }
  }

  // Check for incoming serial command 't' to tare
  if (Serial.available() > 0) {
    char inByte = Serial.read();
    if (inByte == 't' || inByte == 'T') {
      Serial.println("Taring...");
      LoadCell.tareNoDelay();
    }
  }

  // Check if the non-blocking tare is completed
  if (LoadCell.getTareStatus() == true) {
    Serial.println("Tare complete.");
  }
}