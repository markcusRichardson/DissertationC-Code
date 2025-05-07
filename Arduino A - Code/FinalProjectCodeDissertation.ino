
// ===================================================================
// ============= Declaring libraries and global variables =============
// ===================================================================

// ======== Libraries ========
#include <Wire.h>                // Wire.h is for the communication with the accelerometer
#include <FastLED.h>             // FastLED.h library is for the operation of the addressable LEDS
#include <SoftwareSerial.h>      // Softwareserial.h library is used to declare non serial pins the serial fucntion
#include <TinyGPSPlus.h>         // TinyGPSPlus.h library is ised to properly run the GPS functions


// ======== Accelerometer ========
const int MPU_ADDR = 0x68;
int16_t accelerometer_x, accelerometer_y, accelerometer_z;  // Values for accelerometer security system monitoring
char tmp_str[7];
int accX = 0;                   // Final value from accelerometer after conversion
int accY = 0;                   // Final value from accelerometer after conversion
int accZ = 0;                   // Final value from accelerometer after conversion
bool resetAlarmFlag = false;
const float ALARM_THRESHOLD = 1;



// ======== LEDS ========
CRGB leds_rear_left[10];        // LED strip facing the rear left side 10 leds
CRGB leds_rear_right[10];       // LED strip facing the rear right side 10 leds

CRGB leds_logo_left[18];        // LEDS inside the case for the logo left 18 leds
CRGB leds_logo_right[18];       // LEDS inside the case for the logo right 18 leds

CRGB leds_middle_left[21];      // LEDS in the middel facing down for under illunination left 21 leds
CRGB leds_middle_right[21];     // LEDS in the middel facing down for under illunination right 21 leds

CRGB leds_front_left[15];       // LEDS facing forward on the shock absorbers left 15 leds
CRGB leds_front_right[15];      // LEDS facing foward on the shcok absorber right 15 leds

uint8_t brightness_front;       // Serial comms brightness levels for the front leds
uint8_t brightness_middle;      // Serial comms brightness levels for the middle leds
uint8_t brightness_back;        // Serial comms brightness levels for the back leds
uint8_t brightness_logo;        // Serial comms brightness levels for the logo leds 

CRGBPalette16 currentPalette;   // pallette colours used for mode 4 party mode to ensure proper effects
TBlendType currentBlending = LINEARBLEND; // Blending for the color palletes



// ======== PINS allocation ========
const byte leds_rear_left_pin = 2;      // Pin allocation for rear left
const byte leds_rear_right_pin = 3;     // Pin allocation for rear right
const byte leds_logo_left_pin = 4;      // Pin allocation for logo left
const byte leds_logo_right_pin = 5;     // Pin allocation for logo right
const byte leds_middle_left_pin = 6;    // Pin allocation for middle left
const byte leds_middle_right_pin = 7;   // Pin allocation for middle right
const byte leds_front_left_pin = 9;     // Pin allocation for front left
const byte leds_front_right_pin = 8;    // Pin allocation for front right
const byte indicator_right = 10;        // Button input for indicators right
const byte indicator_left = 11;         // Button input for indicators left
const byte buzzerPin = 12;              // Buzzer pins output for security system
const byte lightSensor = A6;            // Pin for lightSensor input pin
const byte GPS_RX = A2;                 // GPS RX piun using SoftwareSerial
const byte GPS_TX = A1;                 // GPS TX piun using SoftwareSerial



// ======== Serial comms variables and booleans ========
bool Lock_Unlock_State = false;             // Boolean for System power on and off 
bool LockingOnce = true;                    // boolean to esnure locking happens once
bool UnLockingOnce = true;                  // boolean to ensure unlocking happens once
int Mode = 3;                               // Mode variables used tp switch between modes received from serial
const byte START_BYTE = 0xAA;               // Start byte for the serial comms to ensure robust communciation
const byte END_BYTE = 0xFF;                 // End byte for the serial comms to ensure robust communciation
SoftwareSerial gpsSerial(GPS_RX, GPS_TX);   // Setting up software serial for the gps rx and tx pins
bool Indicator_left = true;                 // Boolean used for proper functionality of indicators state left
bool Indicator_right = true;                // Boolean used for proper functionality of indicators state right
bool Alarm_bool = false;                    // Boolean used for the alarm activation and deactivation

unsigned long currentMillis;                // Currentmillis to measure time duration of certain fucntions
int i = 0;                                  // integer i used in loops 



// ======== GPS variables ========
float Longtitude = 200;         // Longitude reprenting 1 of the gps location values for security system
float Latitude = 200;           // Latitude reprenting 1 of the gps location values for security system
int Altitude = 50;         // Variable altitude form the gps 
int Speed = 20;            // Speed in KMH from the gps
int Satellites = 10;       // Amount of satellites connected to the gps at 1 time
int Date = 1;              // Date output fromt the gps
int Time = 2;              // Time hours output from the gps
int TimeM = 3;             // Time minutes output from the gps
TinyGPSPlus gps;                // Library for the gps to run smoothly



// ======== Light sensor ========
int OUTSIDE_BRIGHTNESS = 0;        // Brightness otuside measured by the light sensor
int brightnessEnvironment = 0;     // Brightness varibales used within the fucntion 
int LED_BRIGHTNESS = 0;            // brightness for the leds based on the outisde brightness





// ================================ SETUP stage ================================
void setup() {
// ======== Serial comms ========
  Serial.begin(9600);
  gpsSerial.begin(9600);

// ======== Accelerometer ========
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);  // Begins a transmission to the I2C slave 
  Wire.write(0x6B);                  // PWR_MGMT_1 register
  Wire.write(0);                     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);        // End the transmission

// ======== Leds ========

  FastLED.addLeds<WS2812, leds_rear_left_pin, GRB>(leds_rear_left, 10); // Setup LEDS leds_rear _left
  FastLED.addLeds<WS2812, leds_rear_right_pin, GRB>(leds_rear_right, 10); // Setup LEDS leds_rear_right

  FastLED.addLeds<WS2812, leds_logo_left_pin, GRB>(leds_logo_left, 21); // Setup LEDS leds_lgog_left
  FastLED.addLeds<WS2812, leds_logo_right_pin, GRB>(leds_logo_right, 21); // Setup LEDS leds_logo_right

  FastLED.addLeds<WS2812, leds_middle_left_pin, GRB>(leds_middle_left, 18); // Setup LEDS leds_middle_left
  FastLED.addLeds<WS2812, leds_middle_right_pin, GRB>(leds_middle_right, 18); // Setup LEDS leds_middel_right

  FastLED.addLeds<WS2812, leds_front_left_pin, GRB>(leds_front_left, 14); // Setup LEDS leds_front_left
  FastLED.addLeds<WS2812, leds_front_right_pin, GRB>(leds_front_right, 14); // Setup LEDS leds_front_right

  FastLED.setMaxPowerInVoltsAndMilliamps(5,2000); 

// ======== Buzzer ========
  pinMode(buzzerPin, OUTPUT); // Ouput pin for the buzzer for alarm
 

// ======== LightSensor ========
  pinMode(lightSensor, INPUT);  // Input pin of the light sensor to detetc light

// ======== Indicators ========
  pinMode(indicator_right, INPUT_PULLUP);
  pinMode(indicator_left, INPUT_PULLUP);

}




// ================================ Methods ================================


// ======== GPS decodingData ========
void gpsDecodeData(){
  while (gpsSerial.available()) {
    // Continuously read available bytes from the GPS module
    gps.encode(gpsSerial.read());  // Feed each byte into the TinyGPS parser

    // Once a new GPS location has been successfully parsed
    if(gps.location.isUpdated()) {
      // Update system variables with latest GPS data
      Longtitude = gps.location.lng();           // Longitude coordinate
      Latitude = gps.location.lat();             // Latitude coordinate
      Altitude = gps.altitude.meters();          // Altitude above sea level in meters
      Speed = gps.speed.kmph();                  // Speed in kilometers per hour
      Satellites = gps.satellites.value();       // Number of satellites in view
      Date = gps.date.day();                     // Current day of the month
      Time = gps.time.hour();                    // Hour from GPS time
      TimeM = gps.time.minute();                 // Minute from GPS time
    }
  }
}




// ======== Serial Encoding method ========
void Serial_Encode(){
  Serial.write(START_BYTE);  // Begin transmission with a start byte for packet recognition

  // Send each GPS and alarm data field as raw bytes (float/int/bool)
  Serial.write((byte*)&Longtitude, sizeof(Longtitude));  // Transmit longitude
  Serial.write((byte*)&Latitude, sizeof(Latitude));      // Transmit latitude
  Serial.write((byte*)&Altitude, sizeof(Altitude));      // Transmit altitude
  Serial.write((byte*)&Speed, sizeof(Speed));            // Transmit speed
  Serial.write((byte*)&Satellites, sizeof(Satellites));  // Transmit satellite count
  Serial.write((byte*)&Date, sizeof(Date));              // Transmit date (day only)
  Serial.write((byte*)&Time, sizeof(Time));              // Transmit hour
  Serial.write((byte*)&TimeM, sizeof(TimeM));            // Transmit minutes
  Serial.write((uint8_t)Alarm_bool);                     // Transmit current alarm status (0 or 1)

  Serial.write(END_BYTE);  // End transmission with an end byte for packet validation
}



// ======== Serial Decoding ========
void Serial_Decode(){
    if (Serial.available() >= 10) {  // Ensure enough bytes are available (Start + 8 Data + End)

        if (Serial.read() == START_BYTE) {  // Confirm the Start Byte to begin decoding
            byte buffer[8];  // Allocate buffer to store incoming bytes
            Serial.readBytes(buffer, 8);  // Read the full payload (8 bytes)

            byte endMarker = Serial.read();  // Check if the final byte is the END_BYTE
            if (endMarker == END_BYTE) {     // Only update values if packet is complete and valid
                Mode = buffer[0];                    // Operating mode (1–4)
                Lock_Unlock_State = buffer[1];       // System lock/unlock flag
                bool Alarm_bool_state = buffer[2];   // Incoming alarm flag (stored temporarily)
                brightness_front = buffer[3];        // Brightness for front LED group
                brightness_middle = buffer[4];       // Brightness for underglow LED group
                brightness_back = buffer[5];         // Brightness for rear LED group
                brightness_logo = buffer[6];         // Brightness for internal logo LEDs
                if (buffer[7] == 1) {
                  resetAlarmFlag = true;
                  }              // Final alarm state assigned
            } else {
                Serial.println("Error: End Byte Mismatch!");  // Log mismatch in packet end
            }
        } else {
            Serial.println("Error: Start Byte Mismatch!");  // Log mismatch in packet start
        }
    }
}

// ======== Accelerometer method ========
void accelerometerMonitor() {
  // Request 14 bytes from the MPU6050 starting at register 0x3B
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);                          // Set register pointer to start of acceleration data
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 7 * 2, true);   // Request 14 bytes (7 values * 2 bytes each)

  // Read raw high/low byte pairs and combine into 16-bit integers
  accelerometer_x = Wire.read() << 8 | Wire.read();  // X-axis acceleration
  accelerometer_y = Wire.read() << 8 | Wire.read();  // Y-axis acceleration
  accelerometer_z = Wire.read() << 8 | Wire.read();  // Z-axis acceleration

  // Convert raw readings to a simplified float representation
  accX = accelerometer_x / 10000;
  accY = accelerometer_y / 10000;
  accZ = accelerometer_z / 10000;
}




void lightSensorFunction() {
  // Read analog value from LDR circuit (0 = bright, 1023 = dark)
  brightnessEnvironment = analogRead(lightSensor);  // A6 reads 0–1023

  // Invert and scale: brighter environment = higher OUTSIDE_BRIGHTNESS
  // OUTSIDE_BRIGHTNESS will range from ~0.0 (dark) to ~1.00 (bright)
  OUTSIDE_BRIGHTNESS = 1.24 * (1 - (brightnessEnvironment / 1023.0));
  delay(10);
  // You can now use OUTSIDE_BRIGHTNESS * 255 to set LED brightness
}




void checkingUpdates() {
  // Step 0: Clean out any old or leftover bytes in the RX buffer
  // Step 1: Send GPS and alarm data to the Pi
  gpsDecodeData();    
  Serial_Encode();

  // Step 2: Wait up to 100ms for Pi to respond with ACK 'A'
  bool ackReceived = false;
  unsigned long startTime = millis();
  while (millis() - startTime < 100) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 'A') {
        ackReceived = true;
        break;
      }
    }
  }

  // Step 3: If ACK was received, decode Pi’s config values
  if (ackReceived) {
    delay(55);  // Pi waits 50ms before sending → give buffer time
    Serial_Decode();  // Your method that parses brightness, lock, etc.
  }


  delay(50);  // Small pause before the next loop starts
}





// ======== Indicator state method ========
bool readButton(int pin) {
  delay(5);  // Quick debounce delay to filter out mechanical bounce from button press
  return digitalRead(pin) == LOW;  // Return true if button is pressed (assuming active LOW logic)
}

void indicator_state(){
  // Continuously update the current state of the indicator buttons
  Indicator_left = readButton(indicator_left);    // Read and store left indicator button state
  Indicator_right = readButton(indicator_right);  // Read and store right indicator button state
}


// ======== Moves the streak forward, leaving LEDs ON ========
void applyPulsatingEffect(CRGB* leds, int pos, uint8_t brightness) {
  // Apply a yellow hue (CHSV 43 = yellow) at a specific LED index with dynamic brightness
  leds[pos] = CHSV(43, 255, brightness);  // Hue 43, max saturation, variable brightness
}


void applyPulsatingEffect1(CRGB* leds, int pos, uint8_t r, uint8_t g, uint8_t b) {
  // Apply a custom RGB color to a specific LED position
  leds[pos] = CRGB(r, g, b);  // Manual RGB input for more flexible color application
}



// ======== Moves the streak backward, turning LEDs OFF ========
void applyPulsatingEffectBack(CRGB* leds, int pos) {
  // Turn off a specific LED by setting it to black (off)
  leds[pos] = CRGB::Black;
}



void fillAllStrips(CRGB color) {
  // Apply a single color to all LED arrays in the system
  fill_solid(leds_rear_left, 10, color);
  fill_solid(leds_rear_right, 10, color);
  fill_solid(leds_logo_left, 18, color);
  fill_solid(leds_logo_right, 18, color);
  fill_solid(leds_middle_left, 21, color);
  fill_solid(leds_middle_right, 21, color);
  fill_solid(leds_front_left, 14, color);
  fill_solid(leds_front_right, 14, color);

}

void indicator_left_Active() {
  CHSV amber = CHSV(43, 255, 255);  // Amber color in HSV

  for (int i = 0; i < 21; i++) {
    // Front (up to 14 LEDs)
    if (i < 14) {
      leds_front_left[i] = amber;
      if (i > 0) leds_front_left[i - 1].fadeToBlackBy(100);
    }

    // Middle (21 LEDs)
    leds_middle_left[i] = amber;
    if (i > 0 && i - 1 < 21) leds_middle_left[i - 1].fadeToBlackBy(100);

    // Rear (up to 10 LEDs)
    if (i < 10) {
      leds_rear_left[i] = amber;
      if (i > 0) leds_rear_left[i - 1].fadeToBlackBy(100);
    }

    // Logo (up to 18 LEDs)
    if (i < 18) {
      leds_logo_left[i] = amber;
      if (i > 0) leds_logo_left[i - 1].fadeToBlackBy(100);
    }

    FastLED.show();
    delay(40);
  }

  // Fade out at the end for all left-side zones
  for (int f = 0; f < 4; f++) {
    fadeToBlackBy(leds_front_left, 14, 80);
    fadeToBlackBy(leds_middle_left, 21, 80);
    fadeToBlackBy(leds_rear_left, 10, 80);
    fadeToBlackBy(leds_logo_left, 18, 80);
    FastLED.show();
    delay(20);
  }

  FastLED.clear();
}

void indicator_right_Active() {
  CHSV amber = CHSV(43, 255, 255);  // Amber color in HSV

  for (int i = 0; i < 21; i++) {
    // Front (up to 14 LEDs)
    if (i < 14) {
      leds_front_right[i] = amber;
      if (i > 0) leds_front_right[i - 1].fadeToBlackBy(50);
    }

    // Middle (21 LEDs)
    leds_middle_right[i] = amber;
    if (i > 0 && i - 1 < 21) leds_middle_right[i - 1].fadeToBlackBy(50);

    // Rear (up to 10 LEDs)
    if (i < 10) {
      leds_rear_right[i] = amber;
      if (i > 0) leds_rear_right[i - 1].fadeToBlackBy(50);
    }

    // Logo (up to 18 LEDs)
    if (i < 18) {
      leds_logo_right[i] = amber;
      if (i > 0) leds_logo_right[i - 1].fadeToBlackBy(50);
    }

    FastLED.show();
    delay(40);
  }

  // Fade out for all right-side strips
  for (int f = 0; f < 4; f++) {
    fadeToBlackBy(leds_front_right, 14, 80);
    fadeToBlackBy(leds_middle_right, 21, 80);
    fadeToBlackBy(leds_rear_right, 10, 80);
    fadeToBlackBy(leds_logo_right, 18, 80);
    FastLED.show();
    delay(40);
  }

  FastLED.clear();
}


// ======== Locking LEDS method ========
void LockingUnLocking() {
  CHSV yellow = CHSV(43, 255, 255);  // Amber tone using HSV model
  CHSV black = CHSV(0,0,0);

  // First flash
  digitalWrite(buzzerPin, HIGH);
  fillAllStrips(yellow); FastLED.show();
  delay(200);  // Let buzzer sound
  digitalWrite(buzzerPin, LOW);
  fillAllStrips(black); FastLED.show();
  delay(200);

  // Second flash
  digitalWrite(buzzerPin, HIGH);
  fillAllStrips(yellow); FastLED.show();
  delay(200);  // Hold sound longer
  digitalWrite(buzzerPin, LOW);
  fillAllStrips(black); FastLED.show();

  // Final cleanup
  FastLED.clear();
  FastLED.show();
  delay(500);
}





// ======== Alarm method ========
void Alarm(void) {
  static int intensity = 0;        // Brightness level (0–255), retains value between calls
  static int direction = 5;        // Direction of brightness change (+5 or -5 per cycle)

  intensity += direction;          // Adjust brightness up or down

  // Reverse direction at boundaries to create a pulsing effect
  if (intensity <= 0 || intensity >= 255)
    direction = -direction;

  // Fill all LED strips with pulsing amber (Hue 43) using current intensity
  fillAllStrips(CHSV(43, 255, intensity));

  FastLED.show();                  // Push brightness update to hardware
  FastLED.clear();                 // Immediately clear for the next pulse frame
  delay(2);                        // Small delay for smooth visual rhythm
}





void FillSolidModesLeft() {
  // Rear Left: Red brightness controlled by brightness_back
  fill_solid(leds_rear_left, 10, CRGB(brightness_back, 0, 0));

  // Logo Left: Green brightness controlled by brightness_logo
  fill_solid(leds_logo_left, 18, CRGB(0, brightness_logo, 0));

  // Middle Under Left: Grayscale (white) controlled by brightness_middle
  fill_solid(leds_middle_left, 21, CRGB(brightness_middle, brightness_middle, brightness_middle));

  // Front Left: Grayscale (white) controlled by brightness_front
  fill_solid(leds_front_left, 14, CRGB(brightness_front, brightness_front, brightness_front));

  FastLED.show();  // Push updates to hardware
  delay(10);       // Small delay to stabilize LED updates
  
}


void FillSolidModesRight() {
  // Rear Right: Red brightness controlled by global LED_BRIGHTNESS
  fill_solid(leds_rear_right, 10, CRGB(brightness_back, 0, 0));

  // Logo Right: Green brightness controlled by brightness_logo
  fill_solid(leds_logo_right, 18, CRGB(0, brightness_logo, 0));

  // Middle Under Right: White light based on brightness_middle
  fill_solid(leds_middle_right, 21, CRGB(brightness_middle, brightness_middle, brightness_middle));

  // Front Right: White light based on brightness_front
  fill_solid(leds_front_right, 14, CRGB(brightness_front, brightness_front, brightness_front));

  FastLED.show();  // Show the effect on the LEDs
  delay(10);       // Delay for visual stability
  
}


void setAllStrips(int index, CRGB color) {
  // Set same LED index across all strips (if valid) to the specified color

  if (index < 10) {
    leds_rear_left[index] = color;
    leds_rear_right[index] = color;
  }

  if (index < 18) {
    leds_logo_left[index] = color;
    leds_logo_right[index] = color;
  }

  if (index < 21) {
    leds_middle_left[index] = color;
    leds_middle_right[index] = color;
  }

  if (index < 14) {
    leds_front_left[index] = color;
    leds_front_right[index] = color;
  }
}



  void Mode1() {
  checkingUpdates();
  lightSensorFunction();  // Read ambient light and calculate OUTSIDE_BRIGHTNESS (0.0 to ~1.24)
  
  // === Left-side logic ===
  if (Indicator_left == true) {
    // Calculate LED brightness from outside light level (scaled max = 255)
    LED_BRIGHTNESS = 255 * OUTSIDE_BRIGHTNESS;

    // Apply brightness and theme to left-side strips
    fill_solid(leds_rear_left, 10, CRGB(LED_BRIGHTNESS, 0, 0));                  // Red rear left
    fill_solid(leds_logo_left, 18, CRGB(0, 200, 0));                             // Green logo left
    fill_solid(leds_middle_left, 21, CRGB(LED_BRIGHTNESS, LED_BRIGHTNESS, LED_BRIGHTNESS));  // White underglow
    fill_solid(leds_front_left, 14, CRGB(LED_BRIGHTNESS, LED_BRIGHTNESS, LED_BRIGHTNESS));   // White front

    FastLED.show();     // Update LEDs with current brightness
    delay(10);          // Minor delay for smooth visual output
  }
  else if (Indicator_left == false) {
    // If indicator is active, play left-side indicator animation instead
    indicator_left_Active();
  }

  // === Right-side logic ===
  if (Indicator_right == true) {
    // Apply the same adaptive brightness to right-side strips
    fill_solid(leds_rear_right, 10, CRGB(LED_BRIGHTNESS, 0, 0));                 
    fill_solid(leds_logo_right, 18, CRGB(0, 200, 0));                            
    fill_solid(leds_middle_right, 21, CRGB(LED_BRIGHTNESS, LED_BRIGHTNESS, LED_BRIGHTNESS)); 
    fill_solid(leds_front_right, 14, CRGB(LED_BRIGHTNESS, LED_BRIGHTNESS, LED_BRIGHTNESS));  

    FastLED.show();
    delay(10);
  }
  else if (Indicator_right == false) {
    // If right indicator is triggered, override with animation
    indicator_right_Active();
  }

  indicator_state();    // Refresh indicator button states
  checkingUpdates();    // Sync with host controller (e.g. Raspberry Pi)
}


void Mode2() { // Custom
  checkingUpdates();
  // === LEFT SIDE ===
  if (Indicator_left == true) {
    // Apply user-defined brightness settings to left-side strips
    FillSolidModesLeft();  // Uses brightness_back, brightness_logo, brightness_middle, brightness_front
  }
  else if (Indicator_left == false) {
    // If left indicator is active, play indicator animation instead
    indicator_left_Active();
  }

  // === RIGHT SIDE ===
  if (Indicator_right == true) {
    // Apply user-defined brightness settings to right-side strips
    FillSolidModesRight();  // Uses LED_BRIGHTNESS and other globals
  }
  else if (Indicator_right == false) {
    // If right indicator is active, play indicator animation instead
    indicator_right_Active();
  }

  indicator_state();     // Refresh input states for next evaluation
  
}



void Mode3() {
  checkingUpdates();
  // === Enforce minimum brightness levels ===
  if (brightness_back < 50) {
    brightness_back = 50;  // Ensure rear LEDs are at least moderately visible
  }
  if (brightness_middle < 50) {
    brightness_middle = 50;  // Prevent underglow from being too dim
  }
  if (brightness_front < 50) {
    brightness_front = 50;  // Keep front lights visible for safety
  }
  if (brightness_logo < 50) {
    brightness_logo = 25;  // Keep front lights visible for safety
  }
  

  // === LEFT SIDE Logic ===
  if (Indicator_left == true) {
    FillSolidModesLeft();   // Use current (corrected) brightness values
    delay(50);              // Smooth visual transitions
  }
  else if (Indicator_left == false) {
    indicator_left_Active();  // Override with animated indicator pattern
  }

  // === RIGHT SIDE Logic ===
  if (Indicator_right == true) {
    FillSolidModesRight();  // Apply same brightness logic to right side
    delay(50);
  }
  else if (Indicator_right == false) {
    indicator_right_Active();  // Trigger animated indicator
  }

  indicator_state();     // Update indicator button status
  
}


// Displays a slow build-up animation using a color palette sweep
void lavaBuildUpEffect(CRGBPalette16 currentPalette) {
  uint8_t startIndex = 0;
  uint8_t brightness = 200;  // Overall brightness level

  for (int cycle = 0; cycle < 200; cycle++) {
    for (int i = 0; i < 21; i++) {
      // Get color from the palette using an offset index for scrolling effect
      CRGB color = ColorFromPalette(currentPalette, startIndex + i * 5, brightness, currentBlending);

      // Apply color to all strips with bounds checks
      if (i < 10) { leds_rear_left[i] = color; leds_rear_right[i] = color; }
      if (i < 18) { leds_logo_left[i] = color; leds_logo_right[i] = color; }
      leds_middle_left[i] = color;
      leds_middle_right[i] = color;
      if (i < 14) { leds_front_left[i] = color; leds_front_right[i] = color; }
    }

    FastLED.show();     // Push changes
    startIndex += 3;    // Shift palette to simulate movement
    delay(30);          // Control animation speed
  }
}

// Creates a bright white flash followed by a smooth fade to black
void whiteFlashFadeOutEffect() {
  fillAllStrips(CRGB::White);
  FastLED.show();
  delay(100);

  // Gradual fade out from white to black
  
    fillAllStrips(CRGB(0, 0, 0));
    FastLED.show();

  FastLED.clear();  // Ensure full reset
  FastLED.show();
}

// Simulates an arrow moving forward with a trail, from back to front
void arrowSweepBackToFrontEffect(int speed) {
  CHSV arrowColor = CHSV(0, 255, 255);  // Bright red
  int trail = 5;

  // Each section: rear, middle, logo, front
  // For each position, light up current pixel and turn off trailing one
  for (int i = 0; i < 10 + trail; i++) {
    if (i < 10) { leds_rear_left[i] = arrowColor; leds_rear_right[i] = arrowColor; }
    if (i >= trail && i - trail < 10) {
      leds_rear_left[i - trail] = CRGB::Black;
      leds_rear_right[i - trail] = CRGB::Black;
    }
    FastLED.show(); delay(speed);
  }

  // Repeat for middle
  for (int i = 0; i < 21 + trail; i++) {
    if (i < 21) { leds_middle_left[i] = arrowColor; leds_middle_right[i] = arrowColor; }
    if (i >= trail && i - trail < 21) {
      leds_middle_left[i - trail] = CRGB::Black;
      leds_middle_right[i - trail] = CRGB::Black;
    }
    FastLED.show(); delay(speed);
  }

  // Repeat for logo
  for (int i = 0; i < 18 + trail; i++) {
    if (i < 18) { leds_logo_left[i] = arrowColor; leds_logo_right[i] = arrowColor; }
    if (i >= trail && i - trail < 18) {
      leds_logo_left[i - trail] = CRGB::Black;
      leds_logo_right[i - trail] = CRGB::Black;
    }
    FastLED.show(); delay(speed);
  }

  // Repeat for front
  for (int i = 0; i < 14 + trail; i++) {
    if (i < 14) { leds_front_left[i] = arrowColor; leds_front_right[i] = arrowColor; }
    if (i >= trail && i - trail < 14) {
      leds_front_left[i - trail] = CRGB::Black;
      leds_front_right[i - trail] = CRGB::Black;
    }
    FastLED.show(); delay(speed);
  }
}


  
// Flashes random sections of LEDs in vibrant colors rapidly
void randomPartyStrobesEffect() {
  for (int j = 0; j < 30; j++) {
    // Generate random colors
    CRGB rearColor = CHSV(random8(), 255, 255);
    CRGB middleColor = CHSV(random8(), 255, 255);
    CRGB logoColor = CHSV(random8(), 255, 255);
    CRGB frontColor = CHSV(random8(), 255, 255);

    int Phase = random(1, 5);  // Choose a random section to flash

    switch(Phase) {
      case 1:
        fill_solid(leds_rear_left, 10, rearColor);
        fill_solid(leds_front_right, 14, frontColor);
        break;
      case 2:
        fill_solid(leds_middle_left, 21, middleColor);
        fill_solid(leds_logo_right, 18, logoColor);
        break;
      case 3:
        fill_solid(leds_logo_left, 18, logoColor);
        fill_solid(leds_middle_right, 21, middleColor);
        break;
      case 4:
        fill_solid(leds_front_left, 14, frontColor);
        fill_solid(leds_rear_right, 10, rearColor);
        break;
    }

    FastLED.show();
    delay(10 * random(1, 5));  // Random duration flash
    FastLED.clear();           // Blackout
    FastLED.show();
    delay(100);                // Time between strobes
  }
}
// Rapid, colorful electric storm effect for final visual climax
void finaleElectricPulseStorm() {
  for (int burst = 0; burst < 25; burst++) {
    for (int i = 0; i < 21; i++) {
      // Create vivid electric-like color
      CRGB pulseColor = CRGB(random8(100, 255), random8(0, 100), random8(150, 255));
      setAllStrips(i, pulseColor);  // Apply across all strips by index
    }
    FastLED.show();
    delay(random(30, 80));   // Variable pulse speed
    FastLED.clear(); FastLED.show();
    delay(random(20, 50));   // Blackout between pulses
  }

  // Finale: bright white flash and smooth fade to black
  fillAllStrips(CRGB::White);
  FastLED.show();
  delay(1000);

  for (int f = 0; f < 5; f++) {
    fadeToBlackBy(leds_rear_left, 10, 80);
    fadeToBlackBy(leds_rear_right, 10, 80);
    fadeToBlackBy(leds_logo_left, 18, 80);
    fadeToBlackBy(leds_logo_right, 18, 80);
    fadeToBlackBy(leds_middle_left, 21, 80);
    fadeToBlackBy(leds_middle_right, 21, 80);
    fadeToBlackBy(leds_front_left, 14, 80);
    fadeToBlackBy(leds_front_right, 14, 80);
    FastLED.show();
    delay(60);
  }

  FastLED.clear(); FastLED.show();
}


// Smooth flowing rainbow shimmer using HSV and index shifting
void rainbowShimmerSweep() {
  static uint8_t rainbowHue = 0;  // Keeps track of base hue for continuity

  for (int frame = 0; frame < 100; frame++) {
    for (int i = 0; i < 21; i++) {
      CRGB color = CHSV(rainbowHue + i * 5, 255, 255);  // Offset hue to get gradient
      setAllStrips(i, color);  // Apply to all LEDs at this index
    }

    FastLED.show();
    rainbowHue += 3;  // Slowly rotate hue for animated effect
    delay(25);        // Smooth pacing
  }
}


void Mode4() {
  static unsigned long lastStepTime = 0;  // Tracks time when last phase started
  static int phase = 0;                   // Which animation phase we’re in
  static bool firstRun = true;            // Ensures reset on mode entry

  unsigned long now = millis();

  // === Reset sequence on first entry ===
  if (firstRun) {
    phase = 0;
    lastStepTime = now;
    firstRun = false;
  }

  // === Sequential animation controller ===
  switch (phase) {

    case 0:
      // Step 1: Multi-palette flowing animation (Lava build-up)
      lavaBuildUpEffect(OceanColors_p);
      delay(100);
      whiteFlashFadeOutEffect();

      lavaBuildUpEffect(ForestColors_p);
      delay(100);
      whiteFlashFadeOutEffect();

      lavaBuildUpEffect(CloudColors_p);
      delay(100);
      whiteFlashFadeOutEffect();

      lavaBuildUpEffect(PartyColors_p);
      delay(100);
      whiteFlashFadeOutEffect();

      lavaBuildUpEffect(HeatColors_p);
      delay(100);
      whiteFlashFadeOutEffect();

      lavaBuildUpEffect(LavaColors_p);
      delay(100);
      whiteFlashFadeOutEffect();
      lastStepTime = millis();
      phase++;
      break;

    case 1:
      // Step 2: Simulated arrow sweep moving forward with increasing speed
      for (int i = 1; i < 15; i++) {
        int speed;
        if (i <= 6) {
          speed = 60 - i * 10;  // Speed up early passes
        }
        if(speed<=11){
          speed = 0.001;
        }
        arrowSweepBackToFrontEffect(speed);
      }
      lastStepTime = millis();
      phase++;
      break;

    case 2:
      // Step 3: Flash to white and fade out — high-impact visual
      whiteFlashFadeOutEffect();
      lastStepTime = millis();
      phase++;
      break;

    case 3:
      // Step 4: Fast strobe party lights with randomized sections and colors
      randomPartyStrobesEffect();
      lastStepTime = millis();
      phase++;
      break;

    case 4:
      // Step 5: Shimmering rainbow sweep using CHSV transitions
      rainbowShimmerSweep();
      lastStepTime = millis();
      phase++;
      break;

    case 5:
      // Final Step: Intense pulse storm and white finale
      finaleElectricPulseStorm();

      firstRun = true;  // Reset controller so it can restart next time Mode4 is re-entered
      break;
  }

  indicator_state();     // Keep indicator states current during show mode
  checkingUpdates();     // Accept new commands (e.g. to interrupt mode)

  Mode = 1;              // After one full cycle, return to Mode 1 automatically
}




// ================================ Loop main ================================
void loop() {
  if (Lock_Unlock_State) {  // System is in locked mode
    if (UnLockingOnce == true) {
      LockingUnLocking();        // Visual cue (LEDs flash amber then fade)
      UnLockingOnce = false;     // Ensure it's only triggered once
      LockingOnce = true;        // Reset flag for the next lock cycle
    }

    checkingUpdates();           // Still allow updates while locked
    accelerometerMonitor();      // Read movement from the accelerometer

    // If motion detected beyond threshold → trigger alarm
    if (accX > ALARM_THRESHOLD || accY > ALARM_THRESHOLD || accZ > ALARM_THRESHOLD) {
      Alarm_bool = true;

      while (Alarm_bool == true) {
  digitalWrite(buzzerPin, HIGH);

  for (i = 0; i <= 25; i++) {
    Alarm();           // Run LED effect
    delay(5);

    checkingUpdates(); // Allow updates from Pi

    // Exit alarm if reset or bike was unlocked
    if (resetAlarmFlag || Lock_Unlock_State == false) {
      Alarm_bool = false;
      resetAlarmFlag = false;
      break;
    }
  }

  digitalWrite(buzzerPin, LOW);  // Stop buzzer between cycles
  delay(100);
}

    }
  }

   else {  
    if (LockingOnce == true) {
      LockingUnLocking();          // Show unlock animation
      LockingOnce = false;         // Prevent replay
      UnLockingOnce = true;        // Reset for future lock
    }
    checkingUpdates();

    // === Select active operational mode based on `Mode` value ===
    switch (Mode) {
      case 1: Mode1(); break;
      case 2: Mode2(); break;
      case 3: Mode3(); break;
      case 4: Mode4(); break;
    }
  }
}


      

