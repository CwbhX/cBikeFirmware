#include "cbike.h"


/* NOTES:

- The ADC pins on the STM32F103C8 is only 50 ohms, therefore a buffer with certainly be needed for many inputs
- PA15 is a JTAG pin and doesn't work with interrupts without special configuration

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

/// Functions

// ISRs
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
  powerBtnPressed = true;
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

  if(powerBtnPressed == true){
    powerBtnPressed = debounce(prevPowerBtnPress);
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

  /// THESE ARE PROCESSED IN THE PROCESSING OF THE BLINKERS
  // if(rightBtnPressed == true){
  //   //Serial.println("Right Button was pressed!");
  //   //rightBtnPressed = false;
  // }

  // if(leftBtnPressed == true){
  //   //Serial.println("Left Button was pressed!");
  //   //leftBtnPressed = false;
  // }

  // Perhaps the delay won't be noticable since it will only occur after a debounced change in state..?
  if(brakeBtnPressed == true){
    //Serial.println("Brake Button was pressed!");
//    Serial.print("Brake button is now: ");
//    Serial.println(digitalRead(PB10));
    delay(20);
    braking = !digitalRead(PB10);
  }

  // This and brake might be updated too fast, e.g. moving on the first one since power state is always 1?? at least with the breadboard switch
  // These should have a delay to reading them, but how to do that without blocking ? - Like the debouncing kind of?
  if(powerBtnPressed == true){
    Serial.println("Power Button was moved!");
    delay(10);
    powerState = !digitalRead(PB11);
    Serial.print("Power state is now: ");
    Serial.println(powerState);
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

      // Might have a weird bug where blinking ends and lights are briefly turned off and then on due to normal lighting resuming at the bottom of handle lights
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
  scaledVoltage  = ((float) batteryVoltageValue/4095)*VCC;

  batteryVoltage = scaledVoltage/(R2/(R1+R2));
}

// Handle PWM of the lights and their brightness from various inputs
// TODO: Brake Light Intensity
// BUG: Changing Direction while other directional light is still on will keep it on!
// TODO: Lights remain off except when flashing. Need to respond to light and brakes in addition
void handleLights(){
  // If we are turned on do the light handling code...
  if(powerState == 1){

    // Brightness control for ambient light
    if(lightSensorValue < lightThreshold)PWMMultiplier = 0.35;
    else PWMMultiplier = 1;

    // Headlight Control
    if(PWMMultiplier < 1){   // NIGHTTIME
      analogWrite(PA10, 240); // Headlight on when dark
      headlightState = true;
    }else{                    // DAYTIME
      analogWrite(PA10, 0);   // Turn off headlight
      headlightState = false;
    }

    // Left Blinker On - only if it is time to turn it back on for the on cycle and we haven't already turned it on this cycle
    if(leftBlinkerState == true && leftBlinkerPrevState == false){      // Only turn on if it was off before (FORESEEN BUG: WILL AFFECT BRAKING LIGHT)
        analogWrite(PA8, 200*PWMMultiplier);  // Normal brightness on left taillight
        leftBlinkerPrevState = true;
      }

    // Right Blinker On -  "
    if(rightBlinkerState == true && rightBlinkerPrevState == false){
        analogWrite(PA9, 200*PWMMultiplier);  // Normal brightness on right taillight
        rightBlinkerPrevState = true;
      }

    // Left Blinker Blinking off - only if it is time to turn it off and we haven't already turned it off this cycle
    if(leftBlinkerState == false && leftBlinkerPrevState == true){
      //Serial.println("Left Blinker Off");
      analogWrite(PA8, 0);
      leftBlinkerPrevState = false;
    }

    // Right Blinker Blinking off
    if(rightBlinkerState == false && rightBlinkerPrevState == true){
      //Serial.println("Right Blinker Off");
      analogWrite(PA9, 0);
      rightBlinkerPrevState = false;
    }

    // If we are not blinking, handle brake brightness and regular running taillights 
    if(blinking == false){
        // If braking - will get spammed a bit when braking is occuring, but it is simple code and don't have to mess with blinking much
      if(braking){
        PWMMultiplier += 0.2; // Make taillights brighter if braking
        leftBlinkerPrevState = false;
        rightBlinkerPrevState = false;
      }
      // Left Blinker On
      if(leftBlinkerPrevState == false){      // Only turn on if it was off before (FORESEEN BUG: WILL AFFECT BRAKING LIGHT)
        analogWrite(PA8, 200*PWMMultiplier);  // Normal brightness on left taillight
        leftBlinkerPrevState = true;
      }

      // Right Blinker On
      if(rightBlinkerPrevState == false){
        analogWrite(PA9, 200*PWMMultiplier);  // Normal brightness on right taillight
        rightBlinkerPrevState = true;
      }
    }

    prevLightState = true;
  } else if(powerState == 0 && prevLightState == true){  // Power is off so turn off the lights - previous state prevents us from mindlessly setting PWM off
    analogWrite(PA8, 0);   // Turn off left light
    analogWrite(PA9, 0);   // Turn off right light
    analogWrite(PA10, 0);  // Turn off head light

    prevLightState = false; // The previous light state is now off
  }

}

int count = 0;
void updateOLED(){
  // Clear Display
  display.clearDisplay();

  // If the display should be on... BUG: Random pixels on the bottom from incorrect chip will cause power draw
  if(powerState == true){
    // Speedometer
    if(MPH <= 9)display.setCursor(21,2);
    else display.setCursor(10,2);
    display.setTextSize(4);             // Normal 1:1 pixel scale
    display.println((int) round(MPH));
    display.setTextSize(1);
    display.println();


    display.print("Trip:");
    display.println(tripMiles);

    display.print("Total:");
    display.println(totalMiles);

    display.println();
    display.print("VBat:");
    display.println(batteryVoltage);

    display.println();
    display.print("Blinkers:");
    if(leftBlinkerState == true)       display.println("L");
    else if(rightBlinkerState == true) display.println("R");
    else display.println();

    display.print("Light:");
    if(headlightState)                display.println("On");
    else if(headlightState == false)  display.println("Off");

    if(SDCardPresent){
      Serial.println();
      Serial.println("Logging...");
    }
  }

  // Update Display
  display.display();
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
  attachInterrupt(digitalPinToInterrupt(PB10), brakeBtnISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PB11), powerBtnISR, CHANGE);
  // TODO: Add Power Interrupt

  // Outputs
  pinMode(PA8,  OUTPUT);  // Left Blinker PWM
  pinMode(PA9,  OUTPUT);  // Right Blinker PWM
  pinMode(PA10, OUTPUT);  // Headlight PWM
}

void setupOLED(){
    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3D)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  display.clearDisplay();
  display.setRotation(3);
  display.display();
  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
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
  
  setupOLED();

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
  count++;
}
