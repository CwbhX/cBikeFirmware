#ifndef cbike
#define cbike

#include <EEPROM.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

/// DEFINED CONSTANTS
#define DEBOUNCE_TIME_MS 500
#define ODOMETER_ADDRESS 0
#define WHEEL_CIRCUM_CM 207.5
#define MAGNETS_PER_WHEEL 1
#define R1 150000.00
#define R2 12000.00
#define VCC 3.30
#define OLED_RESET PB5 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

/// Variables

// SD Card / Logging Variables
File sdLogFile;
String logFileName = "cBikeLog.csv";
String logString;

// OLED
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

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

bool powerBtnPressed;

// ISR Variables
volatile bool powerState       = false;
volatile bool blueBtnPressed   = false;
volatile bool yellowBtnPressed = false;
volatile bool redBtnPressed    = false;
volatile bool leftBtnPressed   = false;
volatile bool rightBtnPressed  = false;
volatile int  hallDect         = 0;
volatile bool brakeBtnChanged  = false;

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
bool brakeBtnPressed;
bool braking = false;

// Whether they are on or off
bool leftBlinkerState      = false;
bool rightBlinkerState     = false;
bool headlightState        = false;
bool leftBlinkerPrevState  = false;
bool rightBlinkerPrevState = false;
bool headLightState        = false;
bool prevLightState        = false;


#endif
