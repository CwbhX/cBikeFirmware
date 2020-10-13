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
#define R1 150000
#define R2 12000
#define VCC 3.30

/// Variables

// SD Card / Logging Variables
File sdLogFile;
String logFileName = "cBikeLog.csv";
String logString;

// Trip Variables
long prevCycleTime = 0;
long prevCycleTimeDiff;
float KPH = 0;
float MPH = 0;
float tripMetres = 0;
float tripMiles = 0;
float totalMiles = 0;

// Battery Variables
int batteryVoltageValue;
float scaledVoltage;
float batteryVoltage;

// Sensor Variables
int lightSensorValue;
int systemTempValue;
int leftBasketTempValue;
int rightBasketTempValue;
bool SDCardPresent;

// ISR Variables
volatile bool powerState       = false;
volatile bool blueBtnPressed   = false;
volatile bool yellowBtnPressed = false;
volatile bool redBtnPressed    = false;
volatile bool leftBtnPressed   = false;
volatile bool rightBtnPressed  = false;
volatile int  hallDect         = 0;
volatile bool brakeBtnPressed  = false;

// Debounce Variables
long prevPowerBtnPress   = 0;
long prevBlueBtnPress    = 0;
long prevYellowBtnPress  = 0;
long prevRedBtnPress     = 0;
long prevLeftBtnPressed  = 0;
long prevRightBtnPressed = 0;
long prevBrakeBtnPressed = 0;

// Screen State Variable - For Menus and stuff
int ScreenState = 0;

int lightThreshold = 2500;
float PWMMultiplier;

// Blinker variables for producing a blinky blink
long blinkerInitialTime;  // in ms, to keep track of blinking
bool blinkerDirection;    // true = left, false = right
bool blinking;
int blinkTimerMultiplier; // this way I can do easy comparisons for every 500ms

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
  brakeBtnPressed = true;
}
void powerBtnISR(){
  powerState = true;
}

// Debounce Function that returns true if it's not a bounce, or false if it is
bool debounce(long &previousTime){
  if((millis() - previousTime) > DEBOUNCE_TIME_MS){
      Serial.println("Non-bounced Press");
      previousTime = millis();
     return true;
  }else{
    Serial.println("Bounce Detected");
    return false;
  }
}

// Remove bounciness from the button inputs
void processDebouncing(){
  if(blueBtnPressed == true){
    blueBtnPressed = debounce(prevBlueBtnPress);
  }

  if(yellowBtnPressed == true){
    yellowBtnPressed = debounce(prevYellowBtnPress);
  }

  if(redBtnPressed == true){
    redBtnPressed = debounce(prevRedBtnPress);
  }

  if(leftBtnPressed == true){
    leftBtnPressed = debounce(prevLeftBtnPressed);
  }

  if(rightBtnPressed == true){
    rightBtnPressed = debounce(prevRightBtnPressed);
  }

  if(brakeBtnPressed == true){
    brakeBtnPressed = debounce(prevBrakeBtnPressed);
  }
}

void processInterrupts(){
  if(blueBtnPressed == true){
    //Serial.println("Blue Button was pressed!");
    blueBtnPressed = false;
  }

  if(yellowBtnPressed == true){
    //Serial.println("Yellow Button was pressed!");
    yellowBtnPressed = false;
  }

  if(redBtnPressed == true){
    //Serial.println("Red Button was pressed!");
    redBtnPressed = false;
  }

  if(rightBtnPressed == true){
    //Serial.println("Right Button was pressed!");
    //rightBtnPressed = false;
  }

  if(leftBtnPressed == true){
    //Serial.println("Left Button was pressed!");
    //leftBtnPressed = false;
  }

  if(brakeBtnPressed == true){
    //Serial.println("Brake Button was pressed!");
    leftBtnPressed = false;
  }

  // Needs to be configured to operate at changing! not just falling edge and hopefully digitalread the new state
  if(powerState == true){
    Serial.println("Power state is true");
    powerState = false;
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
  if(leftBtnPressed || rightBtnPressed){
    blinkerInitialTime = millis();
    blinkTimerMultiplier = 1;
    blinking = true;

    Serial.println("Blinker Pressed");
    // Set the correct blinking direction
    if(leftBtnPressed){
      leftBtnPressed = false;
      blinkerDirection = true;
    }else{
      rightBtnPressed = false;
      blinkerDirection = false;
    }
  }

  // In case the interrupt is not called but we are still trying to blink 
  if(blinking){
    if((millis() - blinkerInitialTime)/blinkTimerMultiplier < 500){
      if(blinkTimerMultiplier % 2 == 0){ // If Even AKA the OFF cycle
        if(blinkerDirection) leftBlinkerState = false;
        else rightBlinkerState = false;
      }else{                             // If odd AKA the ON cycle
        if(blinkerDirection) leftBlinkerState = true;
        else rightBlinkerState = true;
      }
    }else{ // If it is beyond 500ms then we are on the next cycle
      blinkTimerMultiplier++;
    }

    // Turn off after 15 on/off cycles
    if(blinkTimerMultiplier >= 30){
      blinking = false;
      leftBlinkerState = false;
      rightBlinkerState = false;
    }
  }
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
  scaledVoltage  = (batteryVoltageValue/4095)*VCC;
  batteryVoltage = scaledVoltage/(R2/(R1+R2));
}

// Handle PWM of the lights and their brightness from various inputs
// TODO: Brake Light Intensity
// BUG: Changing Direction while other directional light is still on will keep it on!
// TODO: Lights remain off except when flashing. Need to respond to light and brakes in addition
void handleLights(){
  // Brightness control for ambient light
  if(lightSensorValue < lightThreshold) PWMMultiplier = 0.35;
  else PWMMultiplier = 1;

  //Left Blinker
  if(leftBlinkerState == true){
    //Serial.println("Turning on the left blinker");
    analogWrite(PA8, floor(205*PWMMultiplier));
  }else{
    //Serial.println("Turning off the left blinker");
    analogWrite(PA8, 0);
  }

  // Right Blinker
  if(rightBlinkerState == true){
    //Serial.println("Turning on the right blinker");
    analogWrite(PA9, floor(205*PWMMultiplier));
  }else{
    //Serial.println("Turning off the right blinker");
    analogWrite(PA9, 0);
  }

  // Headlight
  if(PWMMultiplier != 1){
    analogWrite(PA10, 225);
//    Serial.println(round(205*PWMMultiplier));
//    Serial.println(PWMMultiplier);
//    analogWrite(PA8, round(205*PWMMultiplier));
//    analogWrite(PA9, round(205*PWMMultiplier));
  }else{
    analogWrite(PA10, 0);
//    analogWrite(PA8, 205);
//    analogWrite(PA9, 205);
  }
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
  pinMode(PB8, INPUT);  // Right Blinker  -- NOTE: Original PA15 doesn't work?!
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
  attachInterrupt(digitalPinToInterrupt(PB8), rightBtnISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(PB3),  leftBtnISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(PB15), hallDectISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(PB10), brakeBtnISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(PB11), powerBtnISR, FALLING);
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
  //long inittime = micros();
  processDebouncing();        // NEDS TO BE IN FRON TOF PROCESSING
  processInterrupts();        // Process all received ISRs from previous loop -- NOTE: Not everything should be processed here as made clear by blinkers
  //////////////////////////////processDebouncing();        // Debounce all received inputs from ISRs before they are used in the loop
  processBlinkers();          // Proces the state of the blinkers
  readSensors();              // Read all sensors that do not have ISRs
  calculateBatteryVoltage();  // Calculate the system's voltage level
  handleLights();             // Handle the lights based off of sensors and blinker state
  updateOLED();               // Update the OLED with all the updated information
  processLogs();              // Log the information
  //Serial.println(micros()-inittime);
}
