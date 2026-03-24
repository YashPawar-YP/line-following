/* * LSA08 Line Sensor Array - Analog Mode Test for ESP32
 * Reading line position through the AN pin (Set A)
 */

const int lsaAnalogPin = 34; // Using ADC1_CH6 on ESP32
int rawValue = 0;

void setup() {
  // Setup Serial communication at 115200 baud for USB monitoring
  Serial.begin(115200);
  delay(1000); 
  
  Serial.println("--- LSA08 Sensor Initialized ---");
  Serial.println("Move a black line across the sensor to see values change.");
  
  // ESP32 ADC resolution is 12-bit by default (0-4095)
  analogReadResolution(12);
}

void loop() {
  // Read the voltage level from the AN pin
  rawValue = analogRead(lsaAnalogPin);

  // Map the raw value to a 0-100 scale for easier understanding
  // 0 = Line is at one extreme end
  // 100 = Line is at the opposite extreme end
  // ~50 = Line is perfectly centered
  int position = map(rawValue, 0, 4095, 0, 100);

  // Output to Serial Monitor (USB)
  Serial.print("Raw ADC: ");
  Serial.print(rawValue);
  Serial.print(" | Line Position: ");
  Serial.print(position);
  Serial.println("%");

  // A small delay to make the Serial Monitor readable
  delay(100);
}
