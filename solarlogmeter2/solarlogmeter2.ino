/*
//*****************************************************************************
 
 SolarLogMeter (with weather measurements)						 
 		
 v. 3.2 - PV IV logging 
 2011-2015 - Nicola Ferralis - ferralis@mit.edu		
 
 With contribution from IVy: 
 created by Rupak Chakraborty and David Berney Needleman, MIT PV Lab.
			  
 This program (source code and binaries) is free software; 
 you can redistribute it and/or modify it under the terms of the
 GNU General Public License as published by the Free Software 
 Foundation, in version 3 of the License.
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You can find a complete copy of the GNU General Public License at:
 
 http://www.gnu.org/licenses/gpl.txt 
 
 //**********************************************************************************
 
 // WEATHER MEASUREMENTS
 1. Barometric pressure and temperature (via Bosch BMP085/BPMP180 Barometric sensor)
 2. 2 axis irradiation (mW/cm^2) via light sensors TAOS TSL230R.
 
 //**********************************************************************************
 
 User Notes for: 
 
 1. Controls a Microchip MCP4822 12-bit dual-voltage output DAC.
  The MCP4822 is SPI-compatible. To command it, hold the CS pin low and send 16 bits,
  4 configuration bits
   DAC selection: 0 = DACa, 1 = DACb
   empty
   gain selection: 1 = 1x, 0 = 2x with Vref = 2.048
   output shutdown: 1 = Vout is available, 0 = Vout is not available R = 500 k-ohms
    12 data bits (0 - 4096)

 2. The circuit:
  * DACa ouput through buffer amp - to A0 and DUT+
  * DACb output - to op amp negative terminal thru 3.3k resistor
  * CS - to digital pin 10  (SS pin)
  * SDI - to digital pin 11 (MOSI pin)
  * SCK - to digital pin 13 (SCK pin)
  * Vout of current meter - to op amp positive terminal through 3.3k resistor
  * Vout of op amp (current measurement) - to A1
 
 3. Resistors for voltage dividers are labeled Rv1 and Rv2, while shunts for current 
 measurements are labeled Ri. They are arrays of floats, with number defined by the
 variable numCell. Therefore the number of values in the arrays needs to be properly
 adjusted according to numCell. 
 
 4. Temperature measurement: The current code supports only one thermistor. The values
 need to be properly set according to the right thermistor.   
 
 5. SD Card:
 If using the the Adafruit Logging shield with an Arduino Mega, in the file:
 ~arduino/libraries/SD/utility/Sd2Card.h
 
 a. change the line: 
    #define MEGA_SOFT_SPI 0
    to 
    #define MEGA_SOFT_SPI 1

 b. and comment:

    #define USE_SPI_LIB
    in ~arduino/libraries/SD/utility/Sd2Card.h
    and ~arduino/libraries/SD/utility/Sd2Card.c
 
 Do not change the other pins!
 Also make sure the definition below (SDshield) is correctly set for the type of SD shield used.
 
 6. Temperature measurements:
 
 There are two ways temperature can be measured, using
 - thermistor (Vishay NTCLE100E3), pin 7 - (comment the definition of TBAR)
 - Bosch BMP085 Barometric sensor - via I2C bus (uncomment the definition of TBAR)
 You must select at build time the type of sensor you want to use.
 
 7. Irradiance measurements:
 - Pins 2, 4, 5, 6, 7 for vertical irradiance.
 - Pins 3, 22, 23, 24, 25 for horizontal irradiance.
 
 8. Sun calculator:
 - Time is always given in Standard Daylight Time form (no daylight saving).
 
 9. Config file (on SD card): SLM.CFG
 - Number of cells (default: 1)
 - Max Voltage (default: stopV = 4.1V)
 - Time between IV scans in minutes (default: 12 minutes)
 - Number of averages in voltage measurements (default: 80. Not used in v2)
 - Generic waiting time in millisec (default: 1000s ms)
 - Location Latitude (default: 42.36)
 - Location Longitude (default: -71.09)
 - Location Timezone from GMT (default: -5.00)  
 - Daylight Saving Time (default: 1, yes).
 
 //**************************************************************************************
 */

#include <SD.h> 
#include <Wire.h>
#include <SPI.h>
#include "RTClib.h"
#include "SLMtypes.h" 

//----------------------------------------------------------------------------
// Define divider of the SPI clock from the system clock.
// Default SPI clock (4MHz): "SPI_CLOCK_DIV4"
// For example: 
// "SPI_CLOCK_DIV2" sets the SPI clock to 1/2 of the system clock, twice the SPI clock 
// "SPI_CLOCK_DIV128" sets the SPI clock to 1/128 the system clock.
// Available dividers: 2, 4, 8, 16 (recommended), 32, 64, 128 
// (Change the last digits in the SPI_divider definition below)
// Comment SPIsetClock and SPI_divider to disable divider  
//----------------------------------------------------------------------------
#define SPIsetClock
#define SPI_divider SPI_CLOCK_DIV16

//--------------------------------------------------------------------------------
// Uncomment for use with Arduino DUE
//--------------------------------------------------------------------------------
//#define ArDUE

//--------------------------------------------------------------------------------
// Change this to match your SD shield or module;
// Arduino Ethernet shield: pin 4
// Adafruit SD shields and modules: pin 10 (also change MEGA_SOFT_SPI from 0 to 1
// Sparkfun SD shield: pin 8
//---------------------------------------------------------------------------------
#define SDshield 10 

//-------------------------------------------------
// Type of temperature measurement system used.
//  Comment for thermistor, uncomment for barometer
//-------------------------------------------------
#define TBAR 

//--------------------------------------------------
// Irradiance Measurement
// Comment to deactivate, uncomment to activate
//--------------------------------------------------
//#define IRR

//-------------------------------------------------------------------------------
// TO BE USED ONLY FOR CALIBRATION OF REAL TIME CLOCK. 
//  The next #define TIMECAL line should remain commented for normal operations
//  Uncomment ONLY for calibration of real time clock. 

//  Instructions:
//    0. Make sure the Real Time Clock hardware setup is in place. 
//    1. Compile this program with the "#define TIMECAL" line uncommented.
//    2. Upload but DO NOT open the Serial monitor.
//    3. Uncomment the line "#define TIMECAL".
//    4. Comment and upload.
//    5. Verify the correct date from the Serial monitor.
//-------------------------------------------------------------------------------
//#define TIMECAL  //Uncomment for calibration of real time clock. Otherwise leave commented.


//--------------------------------------------------------------------
//  Fixed vs variable amplification resistor for current measurement.
//  Comment for single, uncomment for variable
//--------------------------------------------------------------------
//#define MULTIR

//--------------------------------------------------------------------
//  Hard keys (external) for starting acquisition.
//  Comment to disable hard buttons
//--------------------------------------------------------------------
//#define HARDKEYS

//------------------
// Name and version 
//------------------

String nameProg = "SolarLogMeter";
String versProg = "3.2 - 20150721";
String developer = "Nicola Ferralis - ferralis@mit.edu";
char cfgFile[]="SLM.cfg";


//-------------------------------
// Boot mode
//-------------------------------

int bootMode = 1;    // set if automatic acquisition (0) or manual through serial commands (1)    

//-------------------------------
// Time and Location variables 
//-------------------------------

float latitude = 42.359757;        // MIT - Cambridge, MA - USA
float longitude = -71.093559;      // MIT - Cambridge, MA - USA
float timezone = -5;               // from GMT
int DST= 1;                        // Daylight Saving Time: 0-NO, 1-YES.

  //If you live in the southern hemisphere, it would probably be easier
  //for you if you make north as the direction where the azimuth equals
  //0 degrees. To do so, switch the 0 below with 180.  
float northOrSouth = 180;

//------------------
//Program variables
//------------------

void(* resetFunc) (void) = 0; //declare reset function @ address 0
int numCell = 1;        // Max number of cells to be measured

float currentOffset = 0.0;  // Offset in current measurement

 // resistor for current measurement A.
float Ri[] = {0.1, 0.1, 0.1, 0.1}; 

// resistor for Amplification of current measurement A (previously known as c1).
#ifdef MULTIR
float RAi[] = {67.4, 67.4, 67.4};
#endif

// This used to be called c1.
float RAmpI = 20.066; //Change this only when MULTIR is disabled (single amplification resistor, no transistor switching). 


// Applies the correction to the input voltage for a particular cell is the voltage divider is present (0), 
// or leave it with no correction (1).
int voltIN[]={0,0,0,0};  

// Because of the SD/RTC shield, the first available analog pin is #8
// However, with custom PC board, it can start at #0
int fp=0; 

float Vgain = 2.0;  // Multiplication factor for Voltage measurement (as set by IVy using either amplification or voltage divider).

float maxVolt = 0.0; // Max voltage on scale. Initialized later on setup according to AnalogReference.
float lowV = 1.0;     // min voltage for LED warning

int polar = 1;  // set to 1 for regular IV (positive currents), -1 for negative currents

int avNum = 10;     // number of averages to be taken over an analog input
int ledPin = 13;       // on actual arduino boards this is pre-hooked up.
const int singleIVPin = 31;     // the number of the pushbutton pin
const int multiIVPin = 33;     // the number of the pushbutton pin
const int stopIVPin = 35;     // the number of the pushbutton pin
const int GLED = 7;     // Green LED
const int RLED = 6;    // Red LED

#ifdef MULTIR
int TR1 = 5;    // digital pin for analog out for Amplification resistor R1.
int TR2 = 6;    // digital pin for analog out for Amplification resistor R2.
int TR3 = 7;    // digital pin for analog out for Amplification resistor R3.
#endif

/////////////////////////////////
// IVy specific variables
/////////////////////////////////

float precIoc = 4.0;  // minimum current (mA) to determine Voc
unsigned long restTime = 12;  //Time in between IV scans (minutes)
unsigned long restTimeSec = 0;  //
unsigned int delayTime = 10; // Generic time delay (ms). Fallback in case no SD card is found.

float refV = 1.0;
float startV = 0.0;
float stopV = 4.096;  //This is set by default as a fallback in case no SD card is found.

float currentLimit = 80;  //in mA, will take +/- currentLimit as upper and lower limits

int numPoints = 254;
// set pin 53 as the chip select for the DAC:
const int CS = 53;
// set A0 as input pin for current meter:
const int vMeter = A0;  //not used.
// set A1 as input pin for voltage:
const int iMeter = A1; //not used
// set A2 as input pin for DAC out reading:
const int DACa = A2;  //Not sure if needed
// designate channel A on DAC as sweep voltage, B as Op Amp input
// Gain is always 2x and Shutdown is always set high:
byte channelV = B00010000;
byte channelOpAmp = B10010000;

#ifdef TBAR
#else
int Tpin = 7;      // analog pin for T measurement
// temperature measurement
float Rtt = 33.0;  // Value thermistor used for T measurement (make sure you change the values for the thermistor). 
float Rtf = 976.0;   // Value used for voltage divider in T measurement.
#endif


//---------------------
// SD specific definitions
//-------------------------

const int chipSelect = SDshield;
boolean sd = true;     // enable SD support
boolean sds = false;    //switch SD on/off
char nameFile[13];
char nameFileA[4][13];


//-----------------------------
// SPI specific (digital pot)
//-----------------------------
// Obsolete on v2
// const int slaveSelectPin = 53;


//--------------------
// buttons definition
//--------------------

int singleIVbtn = 0;
int multiIVbtn = 0;
int stopIVbtn = 0;


//-------------------------------
// Misc. constants 
//-------------------------------

float pi = 3.14159265;

//---------------
//Real time chip
//---------------

RTC_DS1307 rtc; // define the Real Time Clock object
DateTime now;   // New RTC library


//---------------
//Barometric chip
//---------------

#ifdef TBAR
#define BMP085_ADDRESS 0x77  // I2C address of BMP085
const unsigned char OSS = 0;  // Oversampling Setting
// Calibration values
int ac1;
int ac2; 
int ac3; 
unsigned int ac4;
unsigned int ac5;
unsigned int ac6;
int b1; 
int b2;
int mb;
int mc;
int md;

// b5 is calculated in bmp085GetTemperature(...), this variable is also used in bmp085GetPressure(...)
// so ...Temperature(...) must be called before ...Pressure(...).
long b5; 

short temperature;
long pressure;
#endif

//----------------------------------------
// Definitions for the Barometric sensor
//----------------------------------------

// setup the TLS230R to Arduino mapping
#define TSL_FREQ_PIN 19 // output use digital pin2 for interrupt 
#define TSL_S0       28 
#define TSL_S1       26   
#define TSL_S2       24   
#define TSL_S3       22
#define TSL1_FREQ_PIN 18 // output use digital pin2 for interrupt 
#define TSL1_S0       36 
#define TSL1_S1       34   
#define TSL1_S2       32   
#define TSL1_S3       30
#define READ_TM 1000 // milleseconds between frequency calculations

int calcSensitivity[2];   
unsigned long sensitivityHighThresh = 2000;
unsigned long sensitivityLowThresh = 100000;

unsigned long pulseCount[] = {
  0, 0};
unsigned long currentTime0 = millis(); 
unsigned long startTime0 = currentTime0;
unsigned long currentTime1 = millis(); 
unsigned long startTime1 = currentTime1;
unsigned int tm_diff = 0;

long watt0 = 0;
long watt1 = 0;
long freq0 = 0;
long freq1 = 0;

// freq will be modified by the interrupt handler so needs to be volatile
// freq holds the latest frequency calculation
unsigned long frequency[2];
float uWattCm2[2];
volatile unsigned long curPulseCount[2];
unsigned int count=0;
unsigned int scale[2];   // holds the TLS scale value, see below


//////////////////////////////////////////////////////////
// SETUP 
///////////////////////////////////////////////////////////

void setup()
{ //----------------------------------------
  // Initialize serial port 
  //----------------------------------------
  Serial.begin(57600);
 
  //----------------------------------------
  // Set output for on board LED 
  //----------------------------------------
  pinMode(ledPin, OUTPUT);
  pinMode(GLED, OUTPUT);      
  pinMode(RLED, OUTPUT); 

  //----------------------------------------
  // set the pushbuttons pins as input:
  //----------------------------------------
  pinMode(singleIVPin, INPUT); 
  pinMode(multiIVPin, INPUT); 
  pinMode(stopIVPin, INPUT); 
  
  //----------------------------------------------------------
  // set the Transistor for Amplification Resistors as output
  //----------------------------------------------------------
  
#ifdef MULTIR
   pinMode(TR1, OUTPUT);  
   pinMode(TR2, OUTPUT);
   pinMode(TR3, OUTPUT);
   
   TRselect(1,0,0);
   Serial.println("Variable Amplification on current measurement enabled");
#endif

  //----------------------------------------------
  // initialize SPI: (and reduce SPI clock speed)
  //----------------------------------------------
  SPI.begin();
 
#ifdef SPIsetClock  
  SPI.setClockDivider(SPI_divider);
#endif
  //----------------------------------------
  //Initialize reference voltage
  //----------------------------------------
#ifdef ArDUE  // The arduino DUE only accepts the standard 3.3V.
  analogReference(AR_DEFAULT);
  maxVolt = 3.3;
#else        // For any board other than the Arduino Due
  analogReference(DEFAULT);  //0 to 5 V 
  maxVolt = 5.0;
  //analogReference(INTERNAL2V56); //0-2.56V  
  //maxVolt = 2.56;
  //analogReference(INTERNAL1V1); //0-1.1V
  //maxVolt = 1.1;
#endif  
  
  //----------------------------------------
  // Reset reference voltage DAC
  //----------------------------------------
  resetVOpAmp();
  
  //---------------------------------------------------
  //Stamp header with program details and data labels.
  //----------------------------------------
  delay(200);
  firstRunSerial();

  //----------------------------------------
  // get the time from the RTC
  //----------------------------------------
  Wire.begin();  
  rtc.begin();

  if (! rtc.isrunning()) {
    Serial.println("RTC is NOT running!");
  }

#ifdef TIMECAL
    rtc.adjust(DateTime(__DATE__, __TIME__));
    Serial.println("RTC is syncing!");
#endif

  //----------------------------------------  
  // Initialization SD card
  //----------------------------------------    
  Serial.print("Initializing SD card... ");
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(53, OUTPUT);    //Arduino boards

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present.");
    Serial.println("SD card support disabled.");
    Serial.println();
    sd=false;
    blinkLED(RLED, 3, 100);
  }
  else
  {
    Serial.println("Card initialized.");
    Serial.println();

    //----------------------------------------  
    // Reads or writes the preference file.
    //----------------------------------------  
    Pref();
    delay (100);

    // to use today's date as the filename:
    nameFile2(0,0).toCharArray(nameFile, 13);
    Serial.println();
    Serial.print("Saving full IV data into: ");
    Serial.println(nameFile);

    //nameFile2(1,0).toCharArray(nameFileA[0], 13);
    for(int i=0; i<numCell; i++)
    {
      nameFile2(1,i).toCharArray(nameFileA[i], 13);
      Serial.print("Saving summary cell #");
      Serial.print(i);
      Serial.print(" into: ");
      Serial.println(nameFileA[i]);
    }
    Serial.println();
  }

  //----------------------------------------
  // Initialize calibration of barometer
  //----------------------------------------
#ifdef TBAR
  bmp085Calibration();
#endif


#ifdef IRR
  //----------------------------------------
  // Initialize Irradiation sensors
  //---------------------------------------- 
  sensitivity(1, 0);
  sensitivity(1, 1);
  // Serial.print("Sensitivity 0: ");
  // Serial.println(calcSensitivity[0], DEC);
  // Serial.print("Sensitivity 1: ");
  // Serial.println(calcSensitivity[1], DEC);
  Serial.println("Sensitivity of light sensors initialized.");

  pinMode(TSL_FREQ_PIN, INPUT);
  pinMode(TSL_S0, OUTPUT); 
  pinMode(TSL_S1, OUTPUT);
  pinMode(TSL_S2, OUTPUT);
  pinMode(TSL_S3, OUTPUT);
  pinMode(TSL1_FREQ_PIN, INPUT);
  pinMode(TSL1_S0, OUTPUT); 
  pinMode(TSL1_S1, OUTPUT);
  pinMode(TSL1_S2, OUTPUT);
  pinMode(TSL1_S3, OUTPUT);

  digitalWrite(TSL_S2, HIGH);   // S2 and S3 HIGH = /100 output scaling
  digitalWrite(TSL_S3, HIGH);
  digitalWrite(TSL1_S2, HIGH);   // S2 and S3 HIGH = /100 output scaling
  digitalWrite(TSL1_S3, HIGH);
  scale[0] = 100; // set this to match TSL_S2 and TSL_S3
  scale[1] = 100;
#endif  

#ifdef IRR
  //Run a few measurement to get the baseline (first measurements are skewed)
  getIrradiance(2, 1);
#endif

  //----------------------------------------
  // Stamp date, time, temperature
  //----------------------------------------  
  NowSerial();

  //-----------------------------------------------------------
  // Blink green LED for the number of cells that are active. 
  //----------------------------------------------------------- 
  blinkLED(GLED, numCell, 100);

  serialMenu();

  if(sd==true)
  {
    analysisHeaderSD();
  }
}

///////////////////////////////////////////////////////////
// MAIN LOOP
///////////////////////////////////////////////////////////

void loop()
{ if(bootMode == 1) {
  
    // Accept input from serial 
    int inSerial = 0;    // variable for serial input
    int inSerial2 = 0;    // variable for serial input

    if (Serial.available() > 0) {
      inSerial = Serial.read();

      //Start (4) / stop (5) writing to SD 
      if(sd==true){
        if(inSerial==52)

        {
          sds=true;
          //headerSD();
          Serial.println("Starting data logging into SD card...");
        }
        if(inSerial==53)  
        {
          sds=false;
          Serial.println("Stopping data logging into SD card...");
        }
      }

      //start acquiring IV (1): Single  - (2): Sequence start - (3): sequence stop
      if(inSerial==49)
      { 
        Serial.println("Collecting Single IV");
        ivSingle();
        serialMenu();
       }

      if(inSerial==50)
      {        
        Serial.println("Collecting Sequence IV");
        inSerial2 = 0;

        while(inSerial2!=51)
        {
          inSerial2=Serial.read(); 
          ivSingle();

          Serial.print("Next IV sequence in: ");
          Serial.print(restTime);
          if(restTimeSec == 0) {
            Serial.println(" minutes");
            delay(restTime*60*1000);
          }
          else {
            Serial.println(" seconds");
          delay(restTime*1000);
          }
        }
        Serial.println("Stopping collection IV sequence"); 
      }

      // Print date/time/weather info (6)
      if(inSerial==54)  
      {
        NowSerial();
      }

      // Reset device (0)
      if(inSerial==48)  
      {
        Serial.println("Resetting device");
        Serial.println();
        resetFunc();
      }
    }
  
  /////////////////////////////////////////
  // Enable hardware starting buttons
  /////////////////////////////////////////
  
#ifdef HARDKEYS
    singleIVbtn = digitalRead(singleIVPin);
    multiIVbtn = digitalRead(multiIVPin);

    // check if the pushbutton is pressed.
    // if it is, the buttonState is HIGH:
    if (singleIVbtn == HIGH) {    
    // turn LED on:    

      blinkLED(GLED, 1, 500);
      blinkLED(RLED, 2, 500);

      sds=true;
      //headerSD();
      Serial.println("Starting data logging into SD card...");
      ivSingle();
      blinkLED(RLED, 3, 500);
    }

    if (multiIVbtn == HIGH) { 
      blinkLED(GLED, 2, 200); 
      while(stopIVbtn != HIGH){

        sds=true;
        //headerSD();
        Serial.println("Starting data logging into SD card...");
        blinkLED(RLED, 2, 500);
        ivSingle();

        blinkLED(RLED, 3, 500);
        delay(100);  
        stopIVbtn = digitalRead(stopIVPin);
        Serial.print("Next IV sequence in: ");
        Serial.print(restTime);
        if(restTimeSec == 0) {
            Serial.println(" minutes");
            delay(restTime*60*1000);
          }
          else {
            Serial.println(" seconds");
            delay(restTime*1000);
          }
      }


      blinkLED(GLED, 3, 200);
      stopIVbtn=0;
      sds=false;
      //headerSD();
      Serial.println("Stopping data logging into SD card...");

    }
#endif
}

 else {
      Serial.println("Starting data logging into SD card...");
      sds = true;
      
      while(bootMode == 0) {
          ivSingle();

          Serial.print("Next IV sequence in: ");
          Serial.print(restTime);
          if(restTimeSec == 0) {
            Serial.println(" minutes");
            delay(restTime*60*1000);
          }
          else {
            Serial.println(" seconds");
            delay(restTime*1000);
          }
        }    
      
  }


}


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// Additional libraries
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

/////////////////////////////
// IV acquisition
/////////////////////////////

void ivSingle() {
  header();

  File dataFile = SD.open(nameFile, FILE_WRITE);

  // if the file is available, write to it:
  if (!dataFile) {

    Serial.print("Error accessing the SD card. ");
    Serial.println(nameFile);
    sds=false;
    dataFile.close();
  } 

  if(sds==true)
  {
    headerSD(dataFile);
  }

  /////////////////////////////////
  // DATA ACQUISITION AND SAVING
  /////////////////////////////////
  
  float T = 0.0; 
  float P = 0.0;
  float irra0 = 0.0;
  float irra1 = 0.0;

#ifdef TBAR
  //Measure T and P at the beginning of the IV acquisition (Barometric).
  Wire.requestFrom(BMP085_ADDRESS, 1);
  if(Wire.available())
  {
    temperature = bmp085GetTemperature(bmp085ReadUT());
    pressure = bmp085GetPressure(bmp085ReadUP());
    T=temperature*0.10;
    P=pressure/100.00;
  }
#else
  //Measure T at the beginning of the IV acquisition (Thermistor).
  T=TC();  //when used with thermistor
#endif

#ifdef IRR
  //Measure irradiance at the beginnig of the IV acquisition.
  getIrradiance(1, 2);
  irra0=watt0;
  irra1=watt1;
#endif

  float V[numCell], I[numCell], Ic[numCell];
  float Voc[numCell], Ioc[numCell], Isc[numCell], Pmax[numCell], Vmax[numCell], Imax[numCell];
  
  int jmax;
  int ip=fp;

  //////////////////////////////
  //Measure IV
  //////////////////////////////

  //Setup for Isc, Pmax
  for (int i=0; i<numCell; i++)
  { 
    V[i] = 0.0;
    I[i] = 0.0;
    Ic[i]  = 0.0;
    Voc[i] = 0.0; 
    Isc[i] = 0.0;
    Ioc[i] = 0.0;
    Pmax[i] = 0.0;
    Vmax[i] = 0.0;
    Imax[i] = 0.0;
    jmax = 0;
  }

  int m1 = 0;

#ifdef MULTIR
  // Set first transistor for current amplification;
  TRselect(1, 0, 0);
  // Check for and set the proper transistor to use for current amplification
  TRcheck(0);
  TRcheck(1);
#endif

//////////////////////////////////////////////////////
// BEGINNING IVy code
//////////////////////////////////////////////////////

//boolean currentOverload = false; // indicates currentOverload during sweep
  
// create float variable for start and stop voltage DAC level:
  float startVLevelFloat = startV * 1000.0 + 0.5;
  float stopVLevelFloat = stopV * 1000.0 + 0.5;
  float stopVLevelFloatADC = stopV * 204.6;  // this is for ADC level
  
  // create increment to obtain numPoints number of data points
  float stepLevelFloat = (stopVLevelFloat - startVLevelFloat) / (numPoints - 1.0);
  
  // create int variables for start and stop voltage levels and stepLevel (truncates float):
  //int startVLevel = (int)startVLevelFloat;
  boolean stepIsExact = (stepLevelFloat == float((int)stepLevelFloat)); // determines whether an extra last point should be taken
  
  //more bit math place holders
 
  byte highEight;
  byte lowEight;

  // define current and voltage variables:
  float current = 0.0;
  float deviceVoltage = 0.0;
  //int sweep = 0; //voltage sweep function
  
  //Serial.println("DATA");  // signal to applet that data is coming
  // sweep voltage on channel A:
  for (int level = (int) startVLevelFloat; level < (int) stopVLevelFloat; level += (int)stepLevelFloat) {
    
    highEight = channelV | (byte) ((3840 & level) >> 8);
    lowEight = (byte) 255 & level;
    
    dacWrite(highEight, lowEight);
    
    // Add sequence number
    
    Serial.print(level/16);
    Serial.print(",");
    
    dataFile.print(level/16);
    dataFile.print(",");
    
    // Add date/time
    writeDateSerial();  
    if(sds==true) 
      {writeDateSD(dataFile);}
   
    ip=fp;
    for (int i=0; i<numCell; i++)
      {
      
      //convert digial voltage reading into device voltage reading.  
      //Divide by 1024 for 10 bit resolution, multiply by 5 to scale to the 5 volt max Arduino output.  
      //Then multiply by 2 because of the non-inverting amplifier with a gain of 2
    
      // 5.0 is now set as the AnalogReference(DEFAULT) or AnalogReference(1);
      // the 2 factor is now set initially in the definition parameters as Vgain
              
      // Voltage Measurement
      //V[i] = (float)analogRead(ip) / 1024.0 * 5.0 * 2;
      V[i] = avoltage(ip, maxVolt, Vgain, avNum);
      delay(2);
      
      // Current Measurement
      I[i] = currentRead(ip+1, maxVolt, polar, RAmpI)*1000/Ri[i]; //in mA
      Ic[i] = I[i] + currentOffset;
      
      delay(2);
      
      if(Ic[i]<precIoc && Ic[i] >=0.0)
        {Voc[i]=V[i];
        Ioc[i]=Ic[i];}
      
   // if (current < currentLimit && current > (-1)*currentLimit) {
      // send comma delimited output to Processing applet
    //   Serial.print(deviceVoltage,6);
    //   Serial.print(",");
   //    Serial.println(current,4);
   //    delay(10);
    //    }
        //else {
   //  Serial.println("Current overload.");
    // currentOverload = true;
   //break;
   //   }
   
      // Move to th next cell.  
      ip+=2;
    
      // Get Isc and extract Pmax   
      Isc[i]=max(Isc[i],Ic[i]); 

      if(Pmax[i]<=V[i]*Ic[i])
        {
        Pmax[i]=V[i]*Ic[i];
        Vmax[i]=V[i];
        Imax[i]=Ic[i];
        jmax=level/16;
        }
  
        // write data on Serial and SD
        if(sds==true)
          {writeIVSD(dataFile,V[i],I[i],Ic[i]);}

        writeIVSerial(V[i],I[i],Ic[i]);
        
      }
     dataFile.println();
     Serial.println();
     
     delay(delayTime);  
     
  //if (!currentOverload){
  //  Serial.println("Voltage sweep complete."); // successful sweep.
 // }
    }
  
    // end loop and shut down voltage out of DACa if current is too high
    resetVOpAmp();
 
  ////////////////////////
  // Save Voc
  /////////////////////////

  Serial.print("\"Voc\"");
  Serial.print(",");
  writeDateSerial();  

  if(sds==true)
  {
    dataFile.print("\"Voc\"");
    dataFile.print(",");
    writeDateSD(dataFile);
  }

  for (int i=0; i<numCell; i++) {

    writeIVSerial(Voc[i],Ioc[i]-currentOffset,Ioc[i]);
    // write data on Serial and SD
    if(sds==true)
      {writeIVSD(dataFile,Voc[i],Ioc[i]-currentOffset,Ioc[i]);}
  }
  

#ifdef TBAR
  //Measure T and P at the end of the IV acquisition (Barometric).
  Wire.requestFrom(BMP085_ADDRESS, 1);
  if(Wire.available())
  {
    temperature = bmp085GetTemperature(bmp085ReadUT());
    pressure = bmp085GetPressure(bmp085ReadUP());

    T=(T+temperature*0.10)/2;
    P=(P+pressure/100.00)/2;
  }

#else
  //Measure T and P at the end of the IV acquisition (Thermistor).
  T=(T+TC())/2; 
#endif

#ifdef IRR
  getIrradiance(1, 2);
  irra0=(irra0+watt0)/2;
  irra1=(irra1+watt1)/2;
#endif

Serial.println();
  Serial.print("\"Max Voltage (V)\",");
  Serial.println(stopV);
  Serial.print("\"Current Offset (mA)\",");
  Serial.println(currentOffset);
  Serial.print("\"Average temperature (C)\",");
  Serial.println(T);
#ifdef TBAR 
  Serial.print("\"Average pressure (mbar)\",");
  Serial.println(P); 
#endif
#ifdef IRR
  Serial.print("\"Vertical Irradiance (mW/cm^2)\",");
  Serial.println(irra0/1000.00);
  Serial.print("\"Horizontal Irradiance (mW/cm^2)\",");
  Serial.println(irra1/1000.00);
#endif   
   sunPos sPos=calcSunPos(now);
   Serial.print("\"Sun azimuth\",");
   Serial.println(sPos.altitude);
   Serial.print("\"Sun altitude\",");
   Serial.println(sPos.altitude);
   Serial.print("\"Latitude\",");
   Serial.println(latitude);
   Serial.print("\"Longitude\",");
   Serial.println(longitude);
   Serial.print("\"Time zone\",");
   Serial.println(timezone);
   Serial.println();
  
  if(sds==true)
  { dataFile.println();
    dataFile.print("\"Max Voltage (V)\",");
    dataFile.println(stopV);
    dataFile.print("\"Current Offset (mA)\",");
    dataFile.println(currentOffset);
    dataFile.print("\"Average temperature (C)\",");
    dataFile.println(T);
#ifdef TBAR
    dataFile.print("\"Average pressure (mbar)\",");
    dataFile.println(P);    
#endif
#ifdef IRR
    dataFile.print("\"Vertical Irradiance (mW/cm^2)\",");
    dataFile.println(irra0/1000.00);
    dataFile.print("\"Horizontal Irradiance (mW/cm^2)\",");
    dataFile.println(irra1/1000.00);
#endif    
    sunPos sPos=calcSunPos(now);
    dataFile.print("\"Sun azimuth\",");
    dataFile.println(sPos.altitude);
    dataFile.print("\"Sun altitude\",");
    dataFile.println(sPos.altitude);
    dataFile.print("\"Latitude\",");
    dataFile.println(latitude);
    dataFile.print("\"Longitude\",");
    dataFile.println(longitude);
    dataFile.print("\"Time zone\",");
    dataFile.println(timezone);
    dataFile.println();
    dataFile.close();
    Serial.println("Written on SD card");
    
  }
  
  delay(delayTime);
  

  ////////////////
  // Analyse data
  ////////////////


  analysisHeaderSerial();

  for (int i=0; i<numCell; i++) {

    analysisSerial(i, Voc[i], Isc[i], Vmax[i], Imax[i], Pmax[i], Vmax[i]*Imax[i]/(Voc[i]*Isc[i]*1000), jmax, T, P, irra0, irra1, now);

    if(sds==true)
    { File dataFile1 = SD.open(nameFileA[i], FILE_WRITE);
      analysisSD(dataFile1, i, Voc[i], Isc[i], Vmax[i], Imax[i], Pmax[i], Vmax[i]*Imax[i]/(Voc[i]*Isc[i]*1000), jmax, T, P, irra0, irra1, now);
     dataFile1.close();
    } 
  }
}

 //----------------------------------------  
  // Print Serial menu 
  //----------------------------------------  
  
void serialMenu(){  
  Serial.println("---------------------------------------------------------------------------");
  Serial.println("SELECT FROM THE FOLLOWING OPTIONS:");
  Serial.println("1: Collect single IV - 2: Collect sequence IV - 3: Stop sequence IV");
  Serial.println("4: Start writing data on SD - 5: Stop writing data on SD");
  Serial.println("6: Stamp loaction/time/date/solar info - 0: Reset");
  Serial.println("---------------------------------------------------------------------------");

  Serial.println();  

  delay(50);
}

//////////////////////////////////////////////
// Stamp data on Serial port
//////////////////////////////////////////////

void writeDateSerial(){
  // digital clock display of the time
  now = rtc.now();
  Serial.print("\"");
  Serial.print(now.year(), DEC );
  Serial.print("/");
  Serial.print(now.month(),DEC );
  Serial.print("/");
  Serial.print(now.day(), DEC);
  //Serial.print("\"");
  //Serial.print(" ");     // for regular serial
  //Serial.print(",\"");      // for csv 
  Serial.print("-");
  Serial.print(now.hour(),DEC);
  Serial.print(":");
  if(now.minute() < 10)
    Serial.print('0');
  Serial.print(now.minute(), DEC);
  Serial.print(":");
  if(now.second() < 10)
    Serial.print('0');
  Serial.print(now.second(), DEC);
  Serial.print("\"");
}

void writeIVSerial(float V, float I, float Ic){

  Serial.print(",");    
  Serial.print(V*1000);
  Serial.print(","); 
  Serial.print(I);
  Serial.print(","); 
  Serial.print(Ic);
}

//////////////////////////////////////////////
// Stamp data on SD
//////////////////////////////////////////////

void writeDateSD(File dataFile){
  now = rtc.now();
  dataFile.print("\"");
  dataFile.print(now.year(), DEC);
  dataFile.print("/");
  dataFile.print(now.month(), DEC);
  dataFile.print("/");
  dataFile.print(now.day(), DEC);
  //dataFile.print("\"");
  //dataFile.print(",\"");      // for csv
  dataFile.print("-");
  dataFile.print(now.hour(), DEC);
  dataFile.print(":");
  if(now.minute() < 10)
    dataFile.print('0');
  dataFile.print(now.minute(), DEC);
  dataFile.print(":");
  if(now.second() < 10)
    dataFile.print('0');
  dataFile.print(now.second(), DEC);
  dataFile.print("\"");   

}

void writeIVSD(File dataFile, float V, float I, float Ic){

  dataFile.print(",");       
  dataFile.print(V*1000);
  dataFile.print(","); 
  dataFile.print(I);
  dataFile.print(","); 
  dataFile.print(Ic);
}

///////////////////////////////////////////
// Stamp data headers on Serial  
///////////////////////////////////////////

void header(){  
  Serial.println();
  Serial.print("\"#\",");
  if(DST==0)
    {Serial.print("\"Date-Time (ST)\"");}
  else 
    {Serial.print("\"Date-Time (DST)\"");}
  for (int i=0; i<numCell; i++)
  {
    Serial.print(",\"V");
    Serial.print(i);
    Serial.print(" (mV)\",\"I");
    Serial.print(i);
    Serial.print(" (mA)\",\"Icorr");
    Serial.print(i);
    Serial.print(" (mA)\"");
  }    
  //Serial.print("\"T (C)\"");  

  Serial.println();
}


///////////////////////////////////////////
// Stamp data header on SD
///////////////////////////////////////////

void headerSD(File dataFile){

  //dataFile.println(nameProg);
  //dataFile.println(developer);
  dataFile.println(" ");
  dataFile.print("\"");
  dataFile.print(nameProg);
  dataFile.print("\",\"");
  dataFile.print(versProg);
  dataFile.println("\"");
  dataFile.print("\"Current offset:\",");
  dataFile.print(currentOffset);
  dataFile.println();
  dataFile.print("\"#\",");
  if(DST==0)
    {dataFile.print("\"Date-Time (ST)\"");}
  else 
    {dataFile.print("\"Date-Time (DST)\"");}
  for (int i=0; i<numCell; i++)
  {
    dataFile.print(",\"V");
    dataFile.print(i);
    dataFile.print(" (mV)\",\"I");
    dataFile.print(i);
    dataFile.print(" (mA)\",\"Icorr");
    dataFile.print(i);
    dataFile.print(" (mA)\"");
  } 

  //dataFile.print("\"T (C)\"");   

  dataFile.println(); 

} 

///////////////////////////////////////////
// Stamp analysis header on Serial
///////////////////////////////////////////

void analysisHeaderSerial() {
  Serial.println();
  Serial.print("\"Cell #\",");
  Serial.print("\"Voc (V)\",");
  Serial.print("\"Isc (mA)\",");
  Serial.print("\"Vmax (V)\",");
  Serial.print("\"Imax (mA)\",");
  Serial.print("\"Pmax (mW)\",");
  Serial.print("\"FF (%)\",");
  Serial.print("\"# Pmax\",");
  Serial.print("\"T (C)\"");
#ifdef TBAR
  Serial.print(",\"P (mbar)\"");
#endif
#ifdef IRR
  Serial.print(",\"VIrr. (mW/cm^2)\"");
  Serial.print(",\"HIrr. (mW/cm^2)\"");
#endif  
  Serial.print(",\"Sun Azimuth\"");
  Serial.print(",\"Sun Altitude\"");
  Serial.println();
}


///////////////////////////////////////////
// Stamp analysis data on Serial
///////////////////////////////////////////

void analysisSerial(int i, float V, float I, float V1, float I1, float P1, float FF, int j, float T, float P, float irra0, float irra1, DateTime now) {
  
  sunPos sPos = calcSunPos(now);
  
  Serial.print(i);
  Serial.print(",");
  Serial.print(V);
  Serial.print(",");
  Serial.print(I);
  Serial.print(",");
  Serial.print(V1);
  Serial.print(",");
  Serial.print(I1);
  Serial.print(",");
  Serial.print(P1);
  Serial.print(",");
  Serial.print(FF);
  Serial.print(",");
  Serial.print(j);
  Serial.print(",");
  Serial.print(T);
#ifdef TBAR
  Serial.print(",");
  Serial.print(P);
#endif
#ifdef IRR
  Serial.print(",");
  Serial.print(irra0/1000.00);
  Serial.print(",");
  Serial.print(irra1/1000.00);
#endif  
  Serial.print(",");
  Serial.print(sPos.azimuth);
  Serial.print(",");
  Serial.print(sPos.altitude);
  Serial.println();
  Serial.println();

}


///////////////////////////////////////////
// Stamp analysis header on SD
///////////////////////////////////////////

void analysisHeaderSD() {
  for (int i=0; i<numCell; i++) {
    File dataFile = SD.open(nameFileA[i], FILE_WRITE);
    dataFile.println();
    dataFile.print("\"");
    dataFile.print(nameProg);
    dataFile.print("\",\"");
    dataFile.print(versProg);
    dataFile.println("\"");
    dataFile.print("\"Current offset:\",");
    dataFile.print(currentOffset);
    dataFile.println();
    if(DST==0)
      dataFile.print("\"Date-Time (ST)\",");
    else  
      dataFile.print("\"Date-Time (DST)\",");
    dataFile.print("\"Voc (V)\",");
    dataFile.print("\"Isc (mA)\",");
    dataFile.print("\"Vmax (V)\",");
    dataFile.print("\"Imax (mA)\",");
    dataFile.print("\"Pmax (mW)\",");
    dataFile.print("\"FF (%)\",");
    dataFile.print("\"# Pmax\",");
    dataFile.print("\"T (C)\"");
#ifdef TBAR
    dataFile.print(",\"P (mbar)\"");
#endif
#ifdef IRR
    dataFile.print(",\"VIrr. (mW/cm^2)\"");
    dataFile.print(",\"HIrr. (mW/cm^2)\"");
#endif    
    dataFile.print(",\"Sun Azimuth\"");
    dataFile.print(",\"Sun Altitude\"");
    dataFile.println();
    dataFile.close();
  }
}


///////////////////////////////////////////
// Stamp analysis data on SD
///////////////////////////////////////////

void analysisSD(File dataFile, int i, float V, float I, float V1, float I1, float P1, float FF, int j, float T, float P, float irra0, float irra1, DateTime now) {

  sunPos sPos = calcSunPos(now);
  writeDateSD(dataFile);
  //dataFile.print(",");
  //dataFile.print(i);
  dataFile.print(",");
  dataFile.print(V);
  dataFile.print(",");
  dataFile.print(I);
  dataFile.print(",");
  dataFile.print(V1);
  dataFile.print(",");
  dataFile.print(I1);
  dataFile.print(",");
  dataFile.print(P1);
  dataFile.print(",");
  dataFile.print(FF);
  dataFile.print(",");
  dataFile.print(j);
  dataFile.print(",");
  dataFile.print(T);
#ifdef TBAR
  dataFile.print(",");
  dataFile.print(P);
#endif
#ifdef IRR
  dataFile.print(",");
  dataFile.print(irra0/1000.00);
  dataFile.print(",");
  dataFile.print(irra1/1000.00);
#endif  
  dataFile.print(",");
  dataFile.print(sPos.azimuth);
  dataFile.print(",");
  dataFile.print(sPos.altitude);
  dataFile.println();

}


///////////////////////////////////////////
// Stamp program info into Serial
///////////////////////////////////////////

void firstRunSerial()
{ 
  Serial.println();
  Serial.println("--------------------------------------");
  Serial.print(nameProg);
  Serial.print(" - v. ");
  Serial.println(versProg);
  Serial.println(developer);
  Serial.println("--------------------------------------");
  Serial.println();
}


///////////////////////////////////////////
// Stamp program info into SD
///////////////////////////////////////////

void firstRunSD(){
  File dataFile = SD.open(nameFile, FILE_WRITE);

  if (dataFile) {
    dataFile.println(nameProg);
    dataFile.println(developer);
  }  
  // if the file isn't open, pop up an error:
  else
  {
    Serial.print("error opening ");
    Serial.println(nameFile);
  }

} 


///////////////////////////////////////////
// Set filename with the format:
// yearmonthday.csv
///////////////////////////////////////////

String nameFile2(int a, int b) {
  String filename;

  now = rtc.now();
  //filename += now.year();
  if(a==0)
  {
    if(now.month() < 10)
      filename += 0;
    filename += (0 + now.month());
    if(now.day() < 10)
      filename += 0;
    filename += (0 + now.day());
  }
  if(a!=0)
  {
    filename += "SC";
    filename += b;
  }
  filename += ".csv";
  return filename;
}


///////////////////////////////////////////
// Preferences (read from file)
///////////////////////////////////////////


void Pref(){

  //SD.remove("test.txt");
  File myFile = SD.open(cfgFile, FILE_READ);
  if (myFile) {
    Serial.println("Configuration file found.");
    
    bootMode = value(myFile);       // set if automatic acquisition (0) or manual through serial commands (1)
    numCell = value(myFile);     // number of cells
    currentOffset = valuef(myFile);   // Offset in current measurement
    stopV = valuef(myFile);     // Max Voltage measured (stopV)
    restTime = value(myFile);    // time in between IV scans (msecs)
    restTimeSec = value(myFile);    // unit time in between IV scans (0: min; 1: sec)
    avNum = value(myFile);       // number of averages
    delayTime = value(myFile);   // generic delay time (msecs)
    latitude = valuef(myFile);   // location latitude
    longitude = valuef(myFile);  // location longitude
    timezone = valuef(myFile);   // location timezone
    DST = value(myFile);         // Daylight Saving Time
    
    for(int i=0; i<numCell; i++)
    {
      voltIN[i] = value(myFile);
    }  // Parameter to apply (0) or not (1) the voltage divider correction.

    myFile.close();
  }
  else 
  {
    File myFile = SD.open(cfgFile, FILE_WRITE);

    Serial.println("Missing configuration file on SD card");
    Serial.print("Creating configuration file: \"");
    Serial.print(cfgFile);
    Serial.println("\"");

    myFile.println(bootMode);
    myFile.println(numCell);    // number of cells
    myFile.println(currentOffset);  // Offset in current measurement   
    myFile.println(stopV);      // Max Voltage measured (stopV)
    myFile.println(restTime);   // time in between IV scans (minutes)
    myFile.println(restTimeSec); // Unit time in between IV scans (0: min; 1: sec)
    myFile.println(avNum);      // number of averages
    myFile.println(delayTime);  // generic delay time (msecs)
    myFile.println(latitude);   // location latitude
    myFile.println(longitude);  // location longitude
    myFile.println(timezone);   // location timezone
    myFile.println(DST);        // Daylight Saving Time
    
    for(int i=0; i<numCell; i++)
    {
      myFile.println(voltIN[i]);
    }  // Parameter to apply (0) or not (1) the voltage divider correction.

    myFile.close();

  }
}


///////////////////////////////////////////////////////
// Reads full integers or floats from a line in a file 
//////////////////////////////////////////////////////

int value(File myFile) {
  int t=0;
  int g=0;
  while (myFile.available()) {
    g = myFile.read();
    if(g==13)
      break;
    if(g!=10)
      t=t*10+(g - '0');
  }
  return t;
}

float valuef(File myFile) {
  float t=0.0;
  int g=0;
  int l=0;
  int q=0;
  int k=0;
  while (myFile.available()) {
    g = myFile.read();
    k++;
    if(g==13)
      break;
    if(g==45)  
      {q=1;}
    if(g==46)
      {l=k;}
    if(g!=10 && g!=45 && g!=46)
      {t=t*10.0+(g - '0');}
      }
  if(q==1)
     t=-t;
  if(l>0)
     t=t/pow(10,k-l-1);    
  return t;
}

///////////////////////////////////////////
// Blink a specific LED
///////////////////////////////////////////

void blinkLED(int pin, int times, int delays) {
  for(int g=0; g<times; g++){  
    digitalWrite(pin, HIGH); 
    delay(delays);
    digitalWrite(pin, LOW);
    delay(500);
  }
}



///////////////////////////////////////////
// Displays current time/date
///////////////////////////////////////////

void NowSerial(){
  // digital clock display of the time
#ifdef TBAR
  Wire.requestFrom(BMP085_ADDRESS, 1);
  if(Wire.available())
  {
    temperature = bmp085GetTemperature(bmp085ReadUT());
    pressure = bmp085GetPressure(bmp085ReadUP());
  }
#endif

#ifdef IRR
  getIrradiance(1, 0);
#endif

  DateTime now = rtc.now();
  
  Serial.println();
  Serial.println("---------------------------------------------");
  Serial.print("Right now it's: ");
  Serial.print(now.hour(),DEC);
  Serial.print(":");
  if(now.minute() < 10)
    Serial.print('0');
  Serial.print(now.minute(), DEC);
  Serial.print(":");
  if(now.second() < 10)
    Serial.print('0');
  Serial.print(now.second(), DEC);
  if(DST==0)
    {Serial.print(" (SDT) - (");}
  else
    {Serial.print(" (DST) - (");}  
  Serial.print(now.month(), DEC );
  Serial.print("-");
  Serial.print(now.day(),DEC );
  Serial.print("-");
  Serial.print(now.year(), DEC);
  Serial.println(")");
  
  Serial.print("Max Voltage: ");
  Serial.print(stopV);
  Serial.println(" V");
  Serial.print("Current offset: ");
  Serial.print(currentOffset);
  Serial.println(" mA");
  
  Serial.print("T = ");
#ifdef TBAR
  Serial.print(temperature*0.10);
#else
  Serial.print(TC());
#endif
  Serial.println(" C");
#ifdef TBAR
  Serial.print("P = ");
  Serial.print(pressure/100.00);
  Serial.print(" mbar");
#endif
  Serial.println();

#ifdef IRR
  Serial.print("Vertical irradiance: ");
  Serial.print(watt0/1000.0000);
  //Serial.print(watt0, DEC);
  Serial.println(" mW/cm^2");
  Serial.print("Horizontal irradiance: ");
  Serial.print(watt1/1000.0000);
  //Serial.print(watt1, DEC);
  Serial.println(" mW/cm^2"); 
#endif  
  Serial.print("Latitude: ");
  Serial.print(latitude);
  Serial.println(" deg");
  Serial.print("Longitude: ");
  Serial.print(longitude);
  Serial.println(" deg");
  Serial.print("Time zone: ");
  Serial.print(timezone);
  Serial.println(" GMT");
  
  sunPos sPos = calcSunPos(now);
  Serial.print("Sun azimuth: ");
  Serial.print(sPos.azimuth);
  Serial.println(" deg");
  Serial.print("Sun altitude: ");
  Serial.print(sPos.altitude);
  Serial.println(" deg");  
  Serial.println("---------------------------------------------");
  Serial.println();
}


/////////////////////////////////////////////////////
// Routines for Barometric and temperature pressure
// Using Bosch BMP085  
/////////////////////////////////////////////////////
#ifdef TBAR

// Stores all of the bmp085's calibration values into global variables
// Calibration values are required to calculate temp and pressure
// This function should be called at the beginning of the program
void bmp085Calibration()
{
  Wire.requestFrom(BMP085_ADDRESS, 1);
  if(!Wire.available())
  {
    Serial.println("No barometer found.");
  }
  else
  {
    Serial.println("Barometer found.");
    ac1 = bmp085ReadInt(0xAA);
    ac2 = bmp085ReadInt(0xAC);
    ac3 = bmp085ReadInt(0xAE);
    ac4 = bmp085ReadInt(0xB0);
    ac5 = bmp085ReadInt(0xB2);
    ac6 = bmp085ReadInt(0xB4);
    b1 = bmp085ReadInt(0xB6);
    b2 = bmp085ReadInt(0xB8);
    mb = bmp085ReadInt(0xBA);
    mc = bmp085ReadInt(0xBC);
    md = bmp085ReadInt(0xBE);
  }
  //Serial.println();
}

// Calculate temperature given ut.
// Value returned will be in units of 0.1 deg C
short bmp085GetTemperature(unsigned int ut)
{
  long x1, x2;

  x1 = (((long)ut - (long)ac6)*(long)ac5) >> 15;
  x2 = ((long)mc << 11)/(x1 + md);
  b5 = x1 + x2;

  return ((b5 + 8)>>4);  
}

// Calculate pressure given up
// calibration values must be known
// b5 is also required so bmp085GetTemperature(...) must be called first.
// Value returned will be pressure in units of Pa.
long bmp085GetPressure(unsigned long up)
{
  long x1, x2, x3, b3, b6, p;
  unsigned long b4, b7;

  b6 = b5 - 4000;
  // Calculate B3
  x1 = (b2 * (b6 * b6)>>12)>>11;
  x2 = (ac2 * b6)>>11;
  x3 = x1 + x2;
  b3 = (((((long)ac1)*4 + x3)<<OSS) + 2)>>2;

  // Calculate B4
  x1 = (ac3 * b6)>>13;
  x2 = (b1 * ((b6 * b6)>>12))>>16;
  x3 = ((x1 + x2) + 2)>>2;
  b4 = (ac4 * (unsigned long)(x3 + 32768))>>15;

  b7 = ((unsigned long)(up - b3) * (50000>>OSS));
  if (b7 < 0x80000000)
    p = (b7<<1)/b4;
  else
    p = (b7/b4)<<1;

  x1 = (p>>8) * (p>>8);
  x1 = (x1 * 3038)>>16;
  x2 = (-7357 * p)>>16;
  p += (x1 + x2 + 3791)>>4;

  return p;
}

// Read 1 byte from the BMP085 at 'address'
char bmp085Read(unsigned char address)
{
  unsigned char data;

  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();

  Wire.requestFrom(BMP085_ADDRESS, 1);
  while(!Wire.available());

  return Wire.read();
}

// Read 2 bytes from the BMP085
// First byte will be from 'address'
// Second byte will be from 'address'+1
int bmp085ReadInt(unsigned char address)
{
  unsigned char msb, lsb;

  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();

  Wire.requestFrom(BMP085_ADDRESS, 2);
  while(Wire.available()<2);
  msb = Wire.read();
  lsb = Wire.read();

  return (int) msb<<8 | lsb;
}

// Read the uncompensated temperature value
unsigned int bmp085ReadUT()
{
  unsigned int ut;

  // Write 0x2E into Register 0xF4
  // This requests a temperature reading
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF4);
  Wire.write(0x2E);
  Wire.endTransmission();

  // Wait at least 4.5ms
  delay(5);

  // Read two bytes from registers 0xF6 and 0xF7
  ut = bmp085ReadInt(0xF6);
  return ut;
}

// Read the uncompensated pressure value
unsigned long bmp085ReadUP()
{
  unsigned char msb, lsb, xlsb;
  unsigned long up = 0;

  // Write 0x34+(OSS<<6) into register 0xF4
  // Request a pressure reading w/ oversampling setting
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF4);
  Wire.write(0x34 + (OSS<<6));
  Wire.endTransmission();

  // Wait for conversion, delay time dependent on OSS
  delay(2 + (3<<OSS));

  // Read register 0xF6 (MSB), 0xF7 (LSB), and 0xF8 (XLSB)
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF6);
  Wire.endTransmission();
  Wire.requestFrom(BMP085_ADDRESS, 3);

  // Wait for data to become available
  while(Wire.available() < 3);
  msb = Wire.read();
  lsb = Wire.read();
  xlsb = Wire.read();

  up = (((unsigned long) msb << 16) | ((unsigned long) lsb << 8) | (unsigned long) xlsb) >> (8-OSS);

  return up;
}

#endif


/////////////////////////////////////////////////////
// Routines for Irradiance measurements 
// Using TAOS TSL230R  
/////////////////////////////////////////////////////
#ifdef IRR
void getIrradiance(int aver, int g)     // Get irradiance in uW/cm^2

{  // attach interrupt to pin2, send output pin of TSL230R to arduino 2
  // call handler on each rising pulse
  attachInterrupt(4, add_pulse, RISING);
  attachInterrupt(5, add_pulse1, RISING);

  watt0 = 0;
  watt1 = 0;
  freq0 = 0;
  freq1 = 0;

  if(g==0)
  {
    Serial.print("Measuring irradiance.");
  }
  if(g==1)
  {
    Serial.print("Calibrating light sensor.");
  }

  for(int j=0; j<aver; j++)
  {
    count++;   
    watt0 = (watt0 + getUwattCm2(0))/2;
    freq0 = (freq0 + frequency[0])/2;
    watt1 = (watt1 + getUwattCm2(1))/2;
    freq1 = (freq1 + frequency[1])/2;
    setSensitivity(0);
    setSensitivity(1);
    delay(500);
    if(g==0 || g==1)
    {
      Serial.print(".");
    }
  }
  if(g==0 || g==1)
  {
    Serial.println(" Done");
  }
}


void add_pulse() {
  // increase pulse count
  pulseCount[0]++;
  // DON'T calculate the frequency every READ_TM ms
  // just store the pulse count to be used outside of the interrupt
  currentTime0 = millis();
  if( currentTime0 - startTime0 >= READ_TM )
  {
    curPulseCount[0] = pulseCount[0];  // use curPulseCount for calculating freq/uW
    pulseCount[0] = 0;  
    startTime0 = millis();
  }
}

void add_pulse1() {
  // increase pulse count
  pulseCount[1]++;
  // DON'T calculate the frequency every READ_TM ms
  // just store the pulse count to be used outside of the interrupt
  currentTime1 = millis();
  if( currentTime1 - startTime1 >= READ_TM )
  {
    curPulseCount[1] = pulseCount[1];  // use curPulseCount for calculating freq/uW
    pulseCount[1] = 0;  
    startTime1 = millis();
  }
}

long getUwattCm2(int i) {
  // copy pulse counter and multiply.
  // the multiplication is necessary for the current
  // frequency scaling level. 

  frequency[i] = curPulseCount[i] * scale[i];

  // get uW observed - assume 640nm wavelength
  // calc_sensitivity is our divide-by to map to a given signal strength
  // for a given sensitivity (each level of greater sensitivity reduces the signal
  // (uW) by a factor of 10)

  float uw_cm2 = (float) frequency[i] / (float) calcSensitivity[i];

  // extrapolate into entire cm2 area
  uWattCm2[i]  = uw_cm2  * ( (float) 1 / (float) 0.0136 );


  return(uWattCm2[i]);
}


void setSensitivity(int i)
{ 

  getUwattCm2(i);
  if (uWattCm2[i] <  sensitivityHighThresh)
  {
    sensitivity(3, i);
    return;
  } 
  if (uWattCm2[i] > sensitivityLowThresh )
  {
    sensitivity(1, i);
    return;
  }
  sensitivity(2, i);
}


void sensitivity(uint8_t level, int i)
{
  switch (level)
  {
  case 1:
    if (calcSensitivity[i] != 10)
    {
      //Serial.print("Now at low sensitivity. ");
      //Serial.println(i);
    }
    if(i==0)
    {
      digitalWrite(TSL_S0, HIGH);  // S0 HIGH and S1 LOW = 1x sensitivity
      digitalWrite(TSL_S1, LOW);
    }
    if(i==1)
    {
      digitalWrite(TSL1_S0, HIGH);  // S0 HIGH and S1 LOW = 1x sensitivity
      digitalWrite(TSL1_S1, LOW);
    }
    calcSensitivity[i] = 10;
    break;
  case 2:
    if (calcSensitivity[i] != 100)
    {
      //Serial.print("Now at medim sensitivity. ");
      //Serial.println(i);
    }
    if(i==0)
    {
      digitalWrite(TSL_S0, LOW);  // S0 LOW and S1 HIGH = 10x sensitivity
      digitalWrite(TSL_S1, HIGH);
    }
    if(i==1)
    {
      digitalWrite(TSL1_S0, LOW);  // S0 LOW and S1 HIGH = 10x sensitivity
      digitalWrite(TSL1_S1, HIGH);
    }
    calcSensitivity[i] = 100;
    break;
  case 3:
    if (calcSensitivity[i] != 1000)
    {
      //Serial.print("Now at high sensitivity. ");
      //Serial.println(i);
    }
    if(i==0)
    {
      digitalWrite(TSL_S0, HIGH);  // S0 HIGH and S1 HIGH = 100x sensitivity
      digitalWrite(TSL_S1, HIGH);
    }
    if(i==1)
    {
      digitalWrite(TSL1_S0, HIGH);  // S0 HIGH and S1 HIGH = 100x sensitivity
      digitalWrite(TSL1_S1, HIGH);
    }
    calcSensitivity[i] = 1000;
    break;
  }
  return;
}
#endif


/////////////////////////////////////////////////////
// Routines for calculation of the sun position 
/////////////////////////////////////////////////////

//**************** Sun Position calculator **********************************
  sunPos calcSunPos(DateTime now)
  {
  float delta;
  float h;

  sunPos sPos;
  
  //now = rtc.now();
  float month2 = (float) now.month();
  float day = (float) now.day();
  float hour2 = (float) now.hour();
  float minute2 = (float) now.minute();
  
  
  float latitude1=latitude*pi/180;
   

  //START OF THE CODE THAT CALCULATES THE POSITION OF THE SUN
  float n = daynum(month2) + day;//NUMBER OF DAYS SINCE THE START OF THE YEAR. 
  delta = .409279 * sin(2 * pi * ((284 + n)/365.25));//SUN'S DECLINATION.
  day = dayToArrayNum(day);//TAKES THE CURRENT DAY OF THE MONTH AND CHANGES IT TO A LOOK UP VALUE ON THE HOUR ANGLE TABLE.
  h = (FindH(day,month2)) + longitude + (timezone * -1 * 15);//FINDS THE NOON HOUR ANGLE ON THE TABLE AND MODIFIES IT FOR THE USER'S OWN LOCATION AND TIME ZONE.
  h = ((((hour2 + minute2/60) - 12) * 15) + h)*pi/180;//FURTHER MODIFIES THE NOON HOUR ANGLE OF THE CURRENT DAY AND TURNS IT INTO THE HOUR ANGLE FOR THE CURRENT HOUR AND MINUTE.
  sPos.altitude = (asin(sin(latitude1) * sin(delta) + cos(latitude1) * cos(delta) * cos(h)))*180/pi;//FINDS THE SUN'S ALTITUDE.
  sPos.azimuth = ((atan2((sin(h)),((cos(h) * sin(latitude1)) - tan(delta) * cos(latitude1)))) + (northOrSouth*pi/180)) *180/pi;//FINDS THE SUN'S AZIMUTH.
  //END OF THE CODE THAT CALCULATES THE POSITION OF THE SUN

  return sPos;
}
  
//THIS CODE TURNS THE MONTH INTO THE NUMBER OF DAYS SINCE JANUARY 1ST.
//ITS ONLY PURPOSE IS FOR CALCULATING DELTA (DECLINATION), AND IS NOT USED IN THE HOUR ANGLE TABLE OR ANYWHERE ELSE.
      float daynum(float month){
       float day;
       if (month == 1){day=0;}
       if (month == 2){day=31;}       
       if (month == 3){day=59;}       
       if (month == 4){day=90;}
       if (month == 5){day=120;}
       if (month == 6){day=151;}
       if (month == 7){day=181;}
       if (month == 8){day=212;}
       if (month == 9){day=243;}
       if (month == 10){day=273;}
       if (month == 11){day=304;}
       if (month == 12){day=334;} 
       return day; 
      }

//THIS CODE TAKES THE DAY OF THE MONTH AND DOES ONE OF THREE THINGS: ADDS A DAY, SUBTRACTS A DAY, OR
//DOES NOTHING. THIS IS DONE SO THAT LESS VALUES ARE REQUIRED FOR THE NOON HOUR ANGLE TABLE BELOW.
       int dayToArrayNum(int day){
            if ((day == 1) || (day == 2) || (day == 3)){day = 0;}
            if ((day == 4) || (day == 5) || (day == 6)){day = 1;}  
            if ((day == 7) || (day == 8) || (day == 9)){day = 2;}
            if ((day == 10) || (day == 11) || (day == 12)){day = 3;}
            if ((day == 13) || (day == 14) || (day == 15)){day = 4;}
            if ((day == 16) || (day == 17) || (day == 18)){day = 5;}
            if ((day == 19) || (day == 20) || (day == 21)){day = 6;}
            if ((day == 22) || (day == 23) || (day == 24)){day = 7;}
            if ((day == 25) || (day == 26) || (day == 27)){day = 8;}
            if ((day == 28) || (day == 29) || (day == 30) || (day == 31)){day = 9;}
          return day;
       }

//////////////////////////////////////////////////////////////
//HERE IS THE TABLE OF NOON HOUR ANGLE VALUES. THESE VALUES GIVE THE HOUR ANGLE, IN DEGREES, OF THE SUN AT NOON (NOT SOLAR NOON)
//WHERE LONGITUDE = 0. DAYS ARE SKIPPED TO SAVE SPACE, WHICH IS WHY THERE ARE NOT 365 NUMBERS IN THIS TABLE.
      float FindH(int day, int month){
      float h;
      
      if (month == 1){
            float h_Array[10]={
            -1.038,-1.379,-1.703,-2.007,-2.289,-2.546,-2.776,-2.978,-3.151,-3.294,};
            h = h_Array[day];}

      if (month == 2){
            float h_Array[10]={
            -3.437,-3.508,-3.55,-3.561,-3.545,-3.501,-3.43,-3.336,-3.219,-3.081,};
            h = h_Array[day];}

      if (month == 3){
            float h_Array[10]={
            -2.924,-2.751,-2.563,-2.363,-2.153,-1.936,-1.713,-1.487,-1.26,-1.035,};
            h = h_Array[day];}

      if (month == 4){
            float h_Array[10]={
            -0.74,-0.527,-0.322,-0.127,0.055,0.224,0.376,0.512,0.63,0.728,};
            h = h_Array[day];}

      if (month == 5){
            float h_Array[10]={
            0.806,0.863,0.898,0.913,0.906,0.878,0.829,0.761,0.675,0.571,};
            h = h_Array[day];}

      if (month == 6){
            float h_Array[10]={
            0.41,0.275,0.128,-0.026,-0.186,-0.349,-0.512,-0.673,-0.829,-0.977,};
            h = h_Array[day];}
            
      if (month == 7){
            float h_Array[10]={
            -1.159,-1.281,-1.387,-1.477,-1.547,-1.598,-1.628,-1.636,-1.622,-1.585,};
            h = h_Array[day];}

      if (month == 8){
            float h_Array[10]={
            -1.525,-1.442,-1.338,-1.212,-1.065,-0.9,-0.716,-0.515,-0.299,-0.07,};
            h = h_Array[day];}

      if (month == 9){
            float h_Array[10]={
            0.253,0.506,0.766,1.03,1.298,1.565,1.831,2.092,2.347,2.593,};
            h = h_Array[day];}

      if (month == 10){
            float h_Array[10]={
            2.828,3.05,3.256,3.444,3.613,3.759,3.882,3.979,4.049,4.091,};
            h = h_Array[day];}

      if (month == 11){
            float h_Array[10]={
            4.1,4.071,4.01,3.918,3.794,3.638,3.452,3.236,2.992,2.722,};
            h = h_Array[day];}

      if (month == 12){
            float h_Array[10]={
            2.325,2.004,1.665,1.312,0.948,0.578,0.205,-0.167,-0.534,-0.893,};
            h = h_Array[day];}

return h;
      }
      
//////////////////////////////////////////////
// convert analog output into actual voltage.
//////////////////////////////////////////////

inline float voltage(int analogPin, float volt, float gain)
{
  
  int v = analogRead(analogPin); 
  float vf = gain*volt*((float)v/(float)1024.0);  //rescale channel with max voltage
  return vf;
}

//////////////////////////////////////////////////////////
// collect numAver times the analog output, average them, 
// and converts them into actual voltage.
/////////////////////////////////////////////////////////

float avoltage(int analogPin, float Volt, float gain, int numAver)
{
  float vt=0.0;
  for(int i = 0; i < numAver; ++i) 
  {
    float vf=voltage(analogPin,Volt, gain);
    vt += vf;
  }
  return vt/(float)numAver;
}      
     

////////////////////////////////////////////////////
// Routine for Transistor Control for amplification   
/////////////////////////////////////////////////////

#ifdef MULTIR

void TRselect(int t1, int t2, int t3) {

  if(t1==0)
    {analogWrite(TR1, LOW);}
  else
    {analogWrite(TR1, HIGH);
    RAmpI = RAi[0];}  
    
  if(t2==0)
    {analogWrite(TR2, LOW);}
  else
    {analogWrite(TR2, HIGH);
    RAmpI = RAi[1];} 

  if(t3==0)
    {analogWrite(TR3, LOW);}
  else
    {analogWrite(TR3, HIGH);
    RAmpI = RAi[2];}     
    
}

  ///////////////////////////////////////////
 // Check for correct amplification resistor
 ///////////////////////////////////////////
 
 void TRcheck(int seq){
 
 float Vi[numCell];
 
 for (int i=0; i<numCell; i++)
   Vi[i] = 0.0;
  
// create float variable for start and stop voltage DAC level:
  float startVLevelFloat = startV * 1000.0 + 0.5;
  float stopVLevelFloat = stopV * 1000.0 + 0.5;
  float stopVLevelFloatADC = stopV * 204.6;  // this is for ADC level
  
  // create increment to obtain numPoints number of data points
  float stepLevelFloat = (stopVLevelFloat - startVLevelFloat) / (numPoints - 1.0);
  
  // create int variables for start and stop voltage levels and stepLevel (truncates float):
  int startVLevel = (int)startVLevelFloat;
  boolean stepIsExact = (stepLevelFloat == float((int)stepLevelFloat)); // determines whether an extra last point should be taken
  
  //more bit math place holders
 
  byte highEight;
  byte lowEight;
  
  // define current and voltage variables:
  float current = 0.0;
  float deviceVoltage = 0.0;
  //int sweep = 0; //voltage sweep function
    
  highEight = channelV | (byte) ((3840 & ((int)startVLevelFloat)) >> 8);
  lowEight = (byte) 255 & startVLevel;
    
  dacWrite(highEight, lowEight);
    
  int ip=fp;
  for (int i=0; i<numCell; i++)
      {

      //V[i] = avoltage(ip, maxVolt, Vgain, avNum);
      //delay(2);
      
      // Current Measurement
      Vi[i] = currentRead(ip+1, maxVolt, polar, RAmpI);
      
      delay(2);
      
      if(Vi[i]>=maxVolt)
        {if(seq==0)
          {TRselect(0,1,0);
          Serial.println(" Using TR2 for amplification measurement");
          // Add new coefficient for the correct R value
          
          }
         if(seq==1)
          {TRselect(0,0,1);
          Serial.println(" Using TR3 for amplification measurement");
          // Add new coefficient for the correct R value
          }
        }
      ip+=2;
      }
 }

 //////////////////////////////////////////   
#endif


/////////////////////////////////////////////////
// Measure and save temperature 
// To be used with Vishay NTCLE100E3 thermistor
/////////////////////////////////////////////////

#ifdef TBAR
#else
float TC()
{ //Using the SteinhartHart formula
  // These values are specific for the thermistor, and it should customized for the thermistor used.
  // These values are for the

  float tc=0.0;
  float V = avoltage(Tpin, maxVolt, 1, avNum);
  float R = (Rtf*V)/(maxVolt-V);

  float A = 3.354016e-3;
  float B = 2.993410e-4;
  float C = 2.135133e-6;
  float D = -5.672000e-9;
  float K = 8.5; // mW/dec C – dissipation factor
  // calculate temperature
  float logR  = log(R/Rtt);
  float logR2 = logR * logR;
  float logR3 = logR * logR * logR;
  float devT = -1.5;  //deviation from formula. Value for T up to 35C
  //float devT = -2;  //deviation from formula. Value for T from 35 to 50C

  tc= (1.0/(A+B*logR+C*logR2+D*logR3))-V*V*1000/(K*R)-273.3+devT;   //in K
  return tc;
}
#endif


//////////////////////////////////////////////////////////////
//FUNCTIONS for DAC control (IVy)
//////////////////////////////////////////////////////////////

/*Define write function for DAC */
void dacWrite(byte highbyte, byte lowbyte) {
  // take the CS pin low to select the chip:
  digitalWrite(CS,LOW);
  delay(10);
  //  send in the address and value via SPI:
  SPI.transfer(highbyte);
  SPI.transfer(lowbyte);
  delay(10);
  // take the CS pin high to de-select the chip:
  digitalWrite(CS,HIGH);
}

float currentRead(int iPin, float Volt, int polar, float RAmpI){
  
  //float c1 = 20.066;   // this is now in the initial definitions. This is now called RAmp.  This is the value of (1+R22/R24)
  // R22=6040 ohms, R24=301 ohms
  float c2 = 3.01; // This is the value of R20 in Kohms
  float c3 = 15.01; //This is the value of R20+R21 in Kohms
  float c4 = 12.0; // This is the value of R21 in Kohms
  
#ifdef ArDUE  // The arduino DUE only accepts the standard 3.3V.
  analogReference(AR_DEFAULT);
#else  
  analogReference(DEFAULT);  //For any board other than the Arduino Due
#endif

  //float currentReadingmA = (-1*polar)*(((float)analogRead(iPin) * 5.0 / 1024.0) - refV * 334.33 * (3.01 / 15.01)) / (334.33 * (12.0 / 15.01));
  float currentReadingmA = (-1*polar)*(((float)analogRead(iPin) * Volt / 1024.0) - refV * RAmpI * (c2 / c3)) / (RAmpI * (c4 / c3));

#ifdef ArDUE  // The arduino DUE only accepts the standard 3.3V.
  analogReference(AR_DEFAULT);
#else  
  analogReference(DEFAULT);  //For any board other than the Arduino Due
#endif

  return currentReadingmA;
}


void resetVOpAmp() {

  dacWrite(B10000000, B00000000);  // reset output voltage
  
  // set op amp negative terminal voltage to 0.027 V
  // (+/-250mA output from 0V to 1V):
  //int vOpAmp = dacWrite (B10010000, B01011001);
 
  // for Vref=1000mV:
  dacWrite(B10010011, B11101000);
  
  //set op amp negative terminal voltage to 1V 
  //int vOpAmp = dacWrite(B10010011, B11101000);
  
  
  dacWrite(B00000000, B00000000);  // reset DACa voltage

}




