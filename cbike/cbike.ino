#include <EEPROM.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


/* NOTES:

- The ADC pins on the STM32F103C8 is only 50 ohms, therefore a buffer with certainly be needed for many inputs

*/

/* UTILISED PINS - In top down order from microUSB & counter clockwise

  PB12 - Blue Button          (Interrupt: Falling Edge)
  PB13 - Yellow Button        (Interrupt: Falling Edge)
  PB14 - Red Button           (Interrupt: Falling Edge)
  PB15 - Hall Effect Sensor   (Interrupt: Falling Edge)
  PA8  - Left Blinker Light   (PWM 1kHz)
  PA9  - Right Blinker Light  (PWM 1kHz)
  PA10 - Front Headlight      (PWM 1kHz)
  PA15 - Right Blinker Switch (Interrupt: Falling Edge)
  PB3  - Left Blinker Switch  (Interrupt: Falling Edge)
  PB4  - SD Card Detect
  PB5  - OLED Reset           
  PB6  - OLED SCL             (I2C)
  PB7  - OLED SDA             (I2C)
  PA0  - System Light Sensor  (12-bit)  -- WAKE UP PIN -- Might want to free this pin up
  PA1  - System Temp Sensor   (12-bit)
  PA2  - Left Basket Temp     (12-bit)
  PA3  - Right Basket Temp    (12-bit)
  PA4  - SD Card Chip Select  (SPI)
  PA5  - SD Card Clock        (SPI)
  PA6  - SD Card MISO         (SPI)
  PA7  - SD Card MOSI         (SPI)
  PB0  - Battery Voltage      (12-bit)
  PB10 - Brake Button         (Interrupt: ???)
  PB11 - Power Button         (Interrupt: ???)

*/

/// DEFINED CONSTANTS
#define DEBOUNCE_TIME_MS 500
#define ODOMETER_ADDRESS 0
#define WHEEL_CIRCUM_CM 207.5
#define MAGNETS_PER_WHEEL 1

/// Variables
File sdLogFile;
String logFileName = "cBikeLog.csv";
String logString;

long prevCycleTime = 0;
long prevCycleTimeDiff;
float KPH = 0;
float MPH = 0;
float tripMetres = 0;
float tripMiles = 0;
float totalMiles = 0;

int batteryVoltageValue;
float batteryVoltage;

int lightSensorValue;
int systemTempValue;
int leftBasketTempValue;
int rightBasketTempValue;
bool SDCardPresent;

volatile bool powerState       = false;
volatile bool blueBtnPressed   = false;
volatile bool yellowBtnPressed = false;
volatile bool redBtnPressed    = false;
volatile bool leftBtnPressed   = false;
volatile bool rightBtnPressed  = false;
volatile int  hallDect         = 0;
volatile bool brakeBtnPressed  = false;

int ScreenState = 0;
long blinkerInitialTime; // in ms, to keep track of blinking
bool blinkerDirection;   // true = left, false = right

// Whether they are on or off
bool leftBlinkerState  = false;
bool rightBlinkerState = false;
bool headLightState    = false;

/// Functions

// ISRs
// TODO: Power Button ISR
void blueBtnISR(){
  blueBtnPressed = true;
}
void yellowBtnISR(){
  yellowBtnPressed = true;
}
void redBtnISR(){
  redBtnPressed = true;
}
void rightBtnISR(){
  rightBtnPressed = true;
}
void leftBtnISR(){
 leftBtnPressed = true;
}
void hallDectISR(){
  hallDect++;
}
void brakeBtnISR(){

}
void powerBtnISR(){

}

void processInterrupts(){
  if(blueBtnPressed == true){
    Serial.println("Blue Button was pressed!");
    blueBtnPressed = false;
  }

  if(yellowBtnPressed == true){
    Serial.println("Yellow Button was pressed!");
    yellowBtnPressed = false;
  }

  if(redBtnPressed == true){
    Serial.println("Red Button was pressed!");
    redBtnPressed = false;
  }

  if(rightBtnPressed == true){
    Serial.println("Right Button was pressed!");
    rightBtnPressed = false;
  }

  if(leftBtnPressed == true){
    Serial.println("Left Button was pressed!");
    leftBtnPressed = false;
  }

  if(hallDect > 0){
    prevCycleTimeDiff = millis() - prevCycleTime;                                     // Don't need this variable but why not
    tripMetres += hallDect*WHEEL_CIRCUM_CM/MAGNETS_PER_WHEEL;
    KPH = (hallDect*WHEEL_CIRCUM_CM/MAGNETS_PER_WHEEL)/(prevCycleTimeDiff/1000/3600); // Take the wheel circum * number of rotations / number of sensors on wheel / 1000ms in a second / 3600 seconds in an hour
    MPH = 0.6214*KPH;
    Serial.println("Hall Effected");
    hallDect = 0; // Reset the count of wheel cycles
  }
}

// Handles the timing for switching the states of the lights
void processBlinkers(){

}

void readSensors(){
  batteryVoltageValue  = analogRead(PB0);

  lightSensorValue     = analogRead(PA0);
  systemTempValue      = analogRead(PA1);
  leftBasketTempValue  = analogRead(PA2);
  rightBasketTempValue = analogRead(PA3);

  SDCardPresent        = digitalRead(PB4);
}

void calculateBatteryVoltage(){

}

// Handle PWM of the lights and their brightness from various inputs
void handleLights(){

}

void updateOLED(){

}

void processLogs(){
  
}

// TODO: Figure out noisy interrupts when touching things (I think)
void setupPins(){
  // Inputs
  pinMode(PB12, INPUT);  // Blue Btn
  pinMode(PB13, INPUT);  // Yellow Btn
  pinMode(PB14, INPUT);  // Red Btn
  pinMode(PA15, INPUT);  // Right Blinker
  pinMode(PB3,  INPUT);  // Left Blinker
  pinMode(PB4,  INPUT);  // SD Card Detect
  pinMode(PB15, INPUT);  // Hall Effect Sensor
  pinMode(PB10, INPUT);  // Brake Button
  pinMode(PB11, INPUT);  // Power Button
  
  
  // Analogue Pins don't need to be inputs?
  // PA0-3

  // Interrupts
  attachInterrupt(digitalPinToInterrupt(PB12), blueBtnISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(PB13), yellowBtnISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(PB14), redBtnISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(PA15), rightBtnISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(PB3),  leftBtnISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(PB15),  hallDectISR, FALLING);
  // TODO: Add Brake Interrupt
  // TODO: Add Power Interrupt

  // Outputs
  pinMode(PA8,  OUTPUT);  // Left Blinker PWM
  pinMode(PA9,  OUTPUT);  // Right Blinker PWM
  pinMode(PA10, OUTPUT);  // Headlight PWM
}

void readEEPROM(){
  EEPROM.get(0*sizeof(float), tripMetres); // Read back the trip meter in metres
  EEPROM.get(1*sizeof(float), tripMiles);  // Read back the trip miles meter in miles
  EEPROM.get(2*sizeof(float), totalMiles);  // Read back odometer value in miles
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  setupPins();
  readEEPROM();

}

void loop() {
  processInterrupts();
  processBlinkers();
  readSensors();
  calculateBatteryVoltage();
  handleLights();
  updateOLED();
  processLogs();
}
