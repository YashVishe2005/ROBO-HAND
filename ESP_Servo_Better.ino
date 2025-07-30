#include <ESP32Servo.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>

// Servo objects
Servo thumb1Servo;  // 0-90 degrees
Servo thumb2Servo;  // 0-180 degrees
Servo indexServo;
Servo middleServo;
Servo ringServo;
Servo pinkyServo;

// LCD setup (I2C address 0x27, 16x2 display)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Servo pins
const int THUMB1_PIN = 14;  // 0-90 degrees
const int THUMB2_PIN = 27;  // 0-180 degrees
const int INDEX_PIN = 26;
const int MIDDLE_PIN = 25;
const int RING_PIN = 33;
const int PINKY_PIN = 32;

// Current servo angles
int thumb1Angle = 0;    // Open position
int thumb2Angle = 0;    // Open position
int indexAngle = 0;     // Open position
int middleAngle = 0;    // Open position
int ringAngle = 0;      // Open position
int pinkyAngle = 0;     // Open position

// Previous servo angles for change detection
int prevThumb1Angle = -1;
int prevThumb2Angle = -1;
int prevIndexAngle = -1;
int prevMiddleAngle = -1;
int prevRingAngle = -1;
int prevPinkyAngle = -1;

// Display variables
unsigned long lastDisplayUpdate = 0;
const unsigned long DISPLAY_INTERVAL = 500; // Update display every 500ms (reduced frequency)

// Binary data parsing variables
byte incomingByte;
byte dataBuffer[8];
int bufferIndex = 0;
bool receivingData = false;

// Performance optimization
const int ANGLE_THRESHOLD = 2; // Only move servo if angle changed by more than 2 degrees
unsigned long lastServoUpdate = 0;
const unsigned long SERVO_UPDATE_INTERVAL = 15; // Minimum 15ms between servo updates

void setup() {
  Serial.begin(9600);
  // Serial.setRxBufferSize(512); // Smaller buffer to reduce latency
  
  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Hand Tracker");
  lcd.setCursor(0, 1);
  lcd.print("Starting");
  
  // Attach servos to pins
  thumb1Servo.attach(THUMB1_PIN, 544, 2400);  // 0-90 degrees
  thumb2Servo.attach(THUMB2_PIN, 700, 2300);  // 0-180 degrees
  indexServo.attach(INDEX_PIN, 700, 2300);
  middleServo.attach(MIDDLE_PIN, 700, 2300);
  ringServo.attach(RING_PIN, 700, 2300);
  pinkyServo.attach(PINKY_PIN, 700, 2300);
  
  resetServos();
  delay(1000);
  
  // Initial display
  updateDisplay();
  
  Serial.println("ESP32 Hand Tracker Ready");
  Serial.println("Expecting binary data: [255, T1, T2, I, M, R, P, 254]");
}

void loop() {
  // Process binary serial data
  if (Serial.available()) {
    incomingByte = Serial.read();
    
    if (!receivingData && incomingByte == 255) {
      // Start flag found
      receivingData = true;
      bufferIndex = 0;
    } else if (receivingData) {
      if (incomingByte == 254) {
        // End flag found
        if (bufferIndex == 6) {
          processBinaryData();
        }
        receivingData = false;
        bufferIndex = 0;
      } else if (bufferIndex < 6) {
        // Store data
        dataBuffer[bufferIndex] = incomingByte;
        bufferIndex++;
      }
    }
  }
  
  // Update LCD display (less frequently)
  updateDisplay();
  
  // Small delay to prevent overwhelming the system
  delayMicroseconds(100);
}

void processBinaryData() {
  // Extract angles from binary data
  int newThumb1 = constrain(dataBuffer[0], 0, 90);
  int newThumb2 = constrain(dataBuffer[1], 0, 180);
  int newIndex = constrain(dataBuffer[2], 0, 180);
  int newMiddle = constrain(dataBuffer[3], 0, 180);
  int newRing = constrain(dataBuffer[4], 0, 180);
  int newPinky = constrain(dataBuffer[5], 0, 180);
  
  // Update servos only if angles changed significantly
  unsigned long currentTime = millis();
  if (currentTime - lastServoUpdate >= SERVO_UPDATE_INTERVAL) {
    moveServoIfChanged(thumb1Servo, thumb1Angle, 90-newThumb1);
    moveServoIfChanged(thumb2Servo, thumb2Angle, newThumb2);
    moveServoIfChanged(indexServo, indexAngle, newIndex);
    moveServoIfChanged(middleServo, middleAngle, newMiddle);
    moveServoIfChanged(ringServo, ringAngle, newRing);
    moveServoIfChanged(pinkyServo, pinkyAngle, newPinky);
    
    lastServoUpdate = currentTime;
  }
  
  // Print to serial (less frequently)
  static unsigned long lastPrint = 0;
  if (currentTime - lastPrint > 200) { // Print every 200ms
    Serial.print("Angles: T1:");
    Serial.print(thumb1Angle);
    Serial.print(" T2:");
    Serial.print(thumb2Angle);
    Serial.print(" I:");
    Serial.print(indexAngle);
    Serial.print(" M:");
    Serial.print(middleAngle);
    Serial.print(" R:");
    Serial.print(ringAngle);
    Serial.print(" P:");
    Serial.println(pinkyAngle);
    lastPrint = currentTime;
  }
}

void moveServoIfChanged(Servo &servo, int &currentAngle, int newAngle) {
  // Only move servo if angle changed significantly
  if (abs(newAngle - currentAngle) >= ANGLE_THRESHOLD) {
    servo.write(newAngle);
    currentAngle = newAngle;
  }
}

void updateDisplay() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastDisplayUpdate >= DISPLAY_INTERVAL) {
    // Only update display if angles actually changed
    if (thumb1Angle != prevThumb1Angle || thumb2Angle != prevThumb2Angle ||
        indexAngle != prevIndexAngle || middleAngle != prevMiddleAngle ||
        ringAngle != prevRingAngle || pinkyAngle != prevPinkyAngle) {
      
      lastDisplayUpdate = currentTime;
      
      lcd.clear();
      
      // Display all servo angles on one screen
      lcd.setCursor(0, 0);
      lcd.print("T1:");
      lcd.print(thumb1Angle);
      lcd.print(" T2:");
      lcd.print(thumb2Angle);
      lcd.print(" I:");
      lcd.print(indexAngle);
      
      lcd.setCursor(0, 1);
      lcd.print("M:");
      lcd.print(middleAngle);
      lcd.print(" R:");
      lcd.print(ringAngle);
      lcd.print(" P:");
      lcd.print(pinkyAngle);
      
      // Store previous angles
      prevThumb1Angle = thumb1Angle;
      prevThumb2Angle = thumb2Angle;
      prevIndexAngle = indexAngle;
      prevMiddleAngle = middleAngle;
      prevRingAngle = ringAngle;
      prevPinkyAngle = pinkyAngle;
    }
  }
}

// Function to test servos individually (for debugging)
void testServos() {
  Serial.println("Testing servos...");
  
  // Test angles for different servos
  int testAngles[] = {0, 45, 90, 135, 180};
  
  for (int i = 0; i < 5; i++) {
    int angle = testAngles[i];
    
    // Thumb1 servo: 0-90 range only
    thumb1Servo.write(min(angle, 90));
    thumb1Angle = min(angle, 90);
    
    // All other servos: 0-180 range
    thumb2Servo.write(angle);
    thumb2Angle = angle;
    indexServo.write(angle);
    indexAngle = angle;
    middleServo.write(angle);
    middleAngle = angle;
    ringServo.write(angle);
    ringAngle = angle;
    pinkyServo.write(angle);
    pinkyAngle = angle;
    
    Serial.print("Test angle: ");
    Serial.print(angle);
    Serial.print(" (Thumb1: ");
    Serial.print(min(angle, 90));
    Serial.println(")");
    
    delay(500);
  }
  
  Serial.println("Servo test complete");
}

// Function to reset all servos to open position
void resetServos() {
  thumb1Angle = 90;    // Open position
  thumb2Angle = 0;    // Open position
  indexAngle = 0;     // Open position
  middleAngle = 0;    // Open position
  ringAngle = 0;      // Open position
  pinkyAngle = 0;     // Open position
  
  thumb1Servo.write(thumb1Angle);
  thumb2Servo.write(thumb2Angle);
  indexServo.write(indexAngle);
  middleServo.write(middleAngle);
  ringServo.write(ringAngle);
  pinkyServo.write(pinkyAngle);
  
  Serial.println("All servos reset to open positions (0 degrees)");
}