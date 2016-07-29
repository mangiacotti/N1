//Scott Mangiacotti
//Tucson, Arizona USA
//May 2016
//N1
//Version 1.0

#include <NewPing.h>
#include <EEPROM.h>

//Constants for I/O
int const US_TRIG_PIN = 8;
int const US_ECHO_PIN = 9;

int const O_RED_LED_PIN = 7;
int const O_YELLOW_LED_PIN = 4;
int const O_GREEN_LED_PIN = 2;
int const I_RED_BUTTON_PIN = 11;
int const I_YELLOW_BUTTON_PIN = 10;

//Constants
int const DEFAULT_ILLUM_SCANS = 5;  //number Loop() routine scans for LED on/off cycles

int const US_MAX_DIST = 500;  //ultra-sonic sensor: 500 centimeters = 196.85 inches = 16.40 feet

//Constants for NVM save addresses
int const MEM_ADDR_RED_DIST = 0;
int const MEM_ADDR_YEL_DIST = 2;

//Global variables for the ultrasonic sensor
NewPing gSonar(US_TRIG_PIN, US_ECHO_PIN, US_MAX_DIST);
int gSonarEchoTime;
int gSonarDistance_cm;

//General and misc global variables
bool gVerboseDiagMode = false;

//Variables for the sequential LED blinking
bool gRedBlink = false;
int gRedScanCounter = 0;
bool gRedOneShot = false;
int gRedButtonState_Last = 0;

bool gYellowBlink = false;
int gYellowScanCounter = 0;
bool gYellowOneShot = false;
int gYellowButtonState_Last = 0;

bool gGreenOneShot = false;

//Variables for red and yellow distance setting/threshold
int gYellowDistance_cm = 0;  //zero is invalid or not set yet
int gRedDistance_cm = 0; //zero is invalid or not set yet


//Runs once
void setup() 
{
  //Open a serial port
  Serial.begin(9600);
  
  //Setup digital outputs
  pinMode(O_RED_LED_PIN, OUTPUT);
  pinMode(O_YELLOW_LED_PIN, OUTPUT);
  pinMode(O_GREEN_LED_PIN, OUTPUT);
  
  pinMode(I_RED_BUTTON_PIN, INPUT);
  pinMode(I_YELLOW_BUTTON_PIN, INPUT);

  //Post product information to serial port
  reportProductInfo();

  //Read EEPROM stored distance settings
  readSettingsFromNVM();

}


//Runs continuously
void loop()
{
  //Serial port processing
  if (Serial.available() > 0)
  {
    int iControlCode;
    iControlCode = Serial.parseInt();
    processSerialMessage(iControlCode);

  }

  //Check if red distance needs setting
  if (gRedDistance_cm == 0)
  {
    blinkRedLED(DEFAULT_ILLUM_SCANS);
    gRedOneShot = false;
  }
  else
  {
    if (gRedOneShot == false)
    {
      gRedOneShot = true;
      digitalWrite(O_RED_LED_PIN, LOW);
    }
  }

  //Check if yellow distance needs setting
  if (gYellowDistance_cm == 0)
  {
    blinkYellowLED(DEFAULT_ILLUM_SCANS);
    gYellowOneShot = false;
  }
  else
  {
    if (gYellowOneShot == false)
    {
      gYellowOneShot = true;
      digitalWrite(O_YELLOW_LED_PIN, LOW);
    }
  }

  //Ping for object distance
  sonarPing();

  bool bSkipObjectCheckThisScan = false;

  //Read red distance setting button state
  int iRedButton;
  iRedButton = digitalRead(I_RED_BUTTON_PIN);

  //Read yellow distance setting button state
  int iYellowButton;
  iYellowButton = digitalRead(I_YELLOW_BUTTON_PIN);

  //Check if red button has been released (we will set distance on button_up detection)
  if (iRedButton != gRedButtonState_Last && iYellowButton == gYellowButtonState_Last) //red changed, yellow no change
  {
    if (iRedButton == LOW && iYellowButton == LOW)
    {
      gRedDistance_cm = gSonarDistance_cm;
      Serial.print("red LED distance set to ");
      Serial.print(gRedDistance_cm);
      Serial.println(" cm");

      writeSettingsToNVM();

      bSkipObjectCheckThisScan = true;
    }
  }

  //Check if yellow button has been released (we will set distance on button_up detection)
  if (iYellowButton != gYellowButtonState_Last  && iRedButton == gRedButtonState_Last) //yellow changed, red no change
  {
    if (iYellowButton == LOW && iRedButton == LOW)
    {
      gYellowDistance_cm = gSonarDistance_cm;
      Serial.print("yellow LED distance set to ");
      Serial.print(gYellowDistance_cm);
      Serial.println(" cm");

      writeSettingsToNVM();

      bSkipObjectCheckThisScan = true;
    }
  }

  //Check if both buttons have been released (we will clear settings on button_up detection of both buttons)
  if (iRedButton != gRedButtonState_Last && iYellowButton != gYellowButtonState_Last) //red changed, yellow changed
  {
    if (iYellowButton == LOW && iRedButton == LOW)
    {
      gRedDistance_cm = 0;
      gYellowDistance_cm = 0;
      digitalWrite(O_GREEN_LED_PIN, LOW);
      Serial.println("red and yellow distance settings cleared");
      writeSettingsToNVM();

      bSkipObjectCheckThisScan = true;
    }
  }

  //Set last state variable to current for next scan
  gRedButtonState_Last = iRedButton;
  gYellowButtonState_Last = iYellowButton;

  //Check for objects if we did not act on any push button activity this scan and red, yellow distance setpoints not zero
  if (bSkipObjectCheckThisScan == false && gRedDistance_cm != 0 && gYellowDistance_cm != 0)
  {
    checkForObjects();
  }

  //Give a little time back
  delay(25);
}


//Ping the ultra-sonic sensor for response time
//Convert response time into distance from sensor in both centimeters and inches
//Setting and measurement checks are handled in centimeters only (inches provided for information purposes only)
void sonarPing()
{
  int iSonarDist_inches = 0;
  
  //Ping for elapsed time for response in microseconds
  gSonarEchoTime = gSonar.ping();
  //gSonarEchoTime = gSonar.ping_median(5); //take 5 samples

  //Convert microsecond response time to centimeters
  gSonarDistance_cm = gSonar.convert_cm(gSonarEchoTime);

  //Convert microsecond response time to inches
  iSonarDist_inches = gSonar.convert_in(gSonarEchoTime);;

  //Post results
  if (gVerboseDiagMode == true)
  {
    Serial.print("echo time: ");
    Serial.print(gSonarEchoTime);
    Serial.print(", cm: ");
    Serial.print(gSonarDistance_cm);
    Serial.print(", in: ");
    Serial.print(iSonarDist_inches);
    Serial.print(", ft:  ");
    Serial.println(iSonarDist_inches/12.0);
  }
}


//Check the last ultra-sonic ping distance calculation and determine which of the three LED to energize
//based on previously set settings for red and yellow distance
void checkForObjects()
{
  if (gSonarDistance_cm !=  0 && gSonarDistance_cm <= gRedDistance_cm)
  { //is there an object within the pre-set distance for the red light
    //Energize the red LED and de-energize yellow and green
    digitalWrite(O_RED_LED_PIN, HIGH);
    digitalWrite(O_YELLOW_LED_PIN, LOW);
    digitalWrite(O_GREEN_LED_PIN, LOW);
    
    Serial.println("object detected within red light distance");
    
  }
  else if  (gSonarDistance_cm !=  0 && gSonarDistance_cm <= gYellowDistance_cm)
  { //is there an object that is not within red light distance but is within yellow
    //Energize the yellow LED and de-energize the red and green
    digitalWrite(O_RED_LED_PIN, LOW);
    digitalWrite(O_YELLOW_LED_PIN, HIGH);
    digitalWrite(O_GREEN_LED_PIN, LOW);

    Serial.println("object detected within yellow light distance");
    
  }
  else
  { //if not within red or yellow distance then energize the green light
    //Energize the green LED and de-energize the yellow and red
    digitalWrite(O_RED_LED_PIN, LOW);
    digitalWrite(O_YELLOW_LED_PIN, LOW);
    digitalWrite(O_GREEN_LED_PIN, HIGH);
    
  }
}


//Blink the red LED based on number of scans of SBC (non-blocking)
//Input parameter is number of scans of loop() routine for the LED to remain in a state before changing
//This method allows the remainder of the program to loop and not block while waiting to change state of LED
void blinkRedLED(int iNumScansToIllum)
{
  //Validate parameter
  if (iNumScansToIllum <= 0)
  {
    iNumScansToIllum = 5;
  }
  
  //Increment
  gRedScanCounter++;

  //Conditional
  if (gRedScanCounter >= iNumScansToIllum)
  {
    gRedScanCounter = 0;

     //Toggle the bit used to set or reset the output to the LED
    if (gRedBlink == false)
    {
      gRedBlink = true;
      digitalWrite(O_RED_LED_PIN, HIGH);
    }
    else
    {
      gRedBlink = false;
      digitalWrite(O_RED_LED_PIN, LOW);
    }
  }  
}


//Blink the yellow LED based on number of scans of SBC (non-blocking)
//Input parameter is number of scans of loop() routine for the LED to remain in a state before changing
//This method allows the remainder of the program to loop and not block while waiting to change state of LED
void blinkYellowLED(int iNumScansToIllum)
{
  //Validate parameter
  if (iNumScansToIllum <= 0)
  {
    iNumScansToIllum = 5;
  }
  
  //Increment
  gYellowScanCounter++;

  //Conditional
  if (gYellowScanCounter >= iNumScansToIllum)
  {
    gYellowScanCounter = 0;

     //Toggle the bit used to set or reset the output to the LED
    if (gYellowBlink == false)
    {
      gYellowBlink = true;
      digitalWrite(O_YELLOW_LED_PIN, HIGH);
    }
    else
    {
      gYellowBlink = false;
      digitalWrite(O_YELLOW_LED_PIN, LOW);
    }
  }  
}


//Process received messages from the serial port interface
//Input parameter iControlCode is the value received from the serial port to be processed
void processSerialMessage(int iControlCode)
{
  bool bMatchFound = false;
  
  //Report what we are doing
  Serial.print("processing control code: ");
  Serial.println(iControlCode);

  //Check for "start_diag" code
  if (iControlCode == 101)
  {
    bMatchFound = true;
    gVerboseDiagMode = true;
    Serial.println("diagnostics mode started");
  }

  //Check for "stop_diag" code
  if (iControlCode == 102)
  {
    bMatchFound = true;
    gVerboseDiagMode = false;
    Serial.println("diagnostics mode stopped");
  }

  //Check for "energize red LED" code
  if (iControlCode == 201)
  {
    bMatchFound = true;
    digitalWrite(O_RED_LED_PIN, HIGH);
    Serial.println("red LED energized");
  }

  //Check for "de-energize red LED" code
  if (iControlCode == 202)
  {
    bMatchFound = true;
    digitalWrite(O_RED_LED_PIN, LOW);
    Serial.println("red LED de-energized");
  }

  //Check for "energize yellow LED" code
  if (iControlCode == 301)
  {
    bMatchFound = true;
    digitalWrite(O_YELLOW_LED_PIN, HIGH);
    Serial.println("yellow LED energized");
  }

  //Check for "de-energize yellow LED" code
  if (iControlCode == 302)
  {
    bMatchFound = true;
    digitalWrite(O_YELLOW_LED_PIN, LOW);
    Serial.println("yellow LED de-energized");
  }

  //Check for "energize green LED" code
  if (iControlCode == 401)
  {
    bMatchFound = true;
    digitalWrite(O_GREEN_LED_PIN, HIGH);
    Serial.println("green LED energized");
  }

  //Check for "de-energize green LED" code
  if (iControlCode == 402)
  {
    bMatchFound = true;
    digitalWrite(O_GREEN_LED_PIN, LOW);
    Serial.println("green LED de-energized");
  }

  //Check for "version report" code
  if (iControlCode == 501)
  {
    bMatchFound = true;
    reportProductInfo();
  }

  //Report back on the same serial port if we did not find a matching/expected control code
  if (bMatchFound == false)
  {
    Serial.println("unrecognized control code");
  }
}


//Write values to EEPROM for red and yellow light distance setting
void writeSettingsToNVM()
{
  int iAddr;

  //Write red distance to non-volatile-memory
  iAddr = 0;
  EEPROM.put(iAddr, gRedDistance_cm);

  //Write yellow distance to non-volatile-memory
  iAddr += sizeof(int);
  EEPROM.put(iAddr, gYellowDistance_cm);

  //Post results
  Serial.println("settings successfully saved to NVM");
}


//Read values from EEPROM for red and yellow light distance setting
void readSettingsFromNVM()
{
  int iAddr;
  int iVal;

  //Read red distance from non-volatile-memory
  iAddr = 0;
  iVal = 0;
  EEPROM.get(iAddr, iVal);

  //Validate value
  if (iVal > 0 && iVal <= US_MAX_DIST)
  {
    gRedDistance_cm = iVal;
    Serial.print("red distance setting successfully read from NVM: ");
    Serial.print(gRedDistance_cm);
    Serial.println(" cm");
  }
  else
  {
    gRedDistance_cm = 0;
    Serial.print("failure reading red distance setting from NVM: ");
    Serial.println(iVal);
  }

  iAddr += sizeof(int);
  iVal = 0;
  EEPROM.get(iAddr, iVal);

  //Validate value
  if (iVal > 0 && iVal <= US_MAX_DIST)
  {
    gYellowDistance_cm = iVal;
    Serial.print("yellow distance setting successfully read from NVM: ");
    Serial.print(gYellowDistance_cm);
    Serial.println(" cm");
  }
  else
  {
    gYellowDistance_cm = 0;
    Serial.print("failure reading yellow distance setting from NVM: ");
    Serial.println(iVal);
  }
}


//Send product information to the serial port
void reportProductInfo()
{
  //Report product and other information to serial port
  Serial.println("N1 version 1.0");
  Serial.println("by zonatec software");
  Serial.println("tucson, arizona usa");
  Serial.println("june 2016");
  Serial.print("checksum ");
  Serial.println("cef2-2a2b-7360-8519-d7f7-3c71-040e-cc6e");
}


