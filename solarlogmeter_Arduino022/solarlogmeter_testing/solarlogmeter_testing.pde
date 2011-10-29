/*
//*****************************************************************************
 
 SolarLogMeter (with weather measurements)						 
 		
 v. 0.10.7 - PV IV logging 
 
 2011 - Nicola Ferralis - ferralis@mit.edu					  
 
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
 1. Barometric pressure and temperature (via Bosch BMP085 Barometric sensor)
 2. 2 axis irradiation (mW/cm^2) via light sensors TAOS TSL230R.
 
 //**********************************************************************************
 
 User Notes for 
 
 1. The analog inputs for the (V,A) measurements for the indivual cells must be coupled: 
 (V0,I0) with pins (A8,A9)
 (V1,V2) with pins (A10, A11)
 etc...
 
 2. Resistors for voltage dividers are labeled Rv1 and Rv2, while shunts for current 
 measurements are labeled Ri. They are arrays of floats, with number defined by the
 variable numCell. Therefore the number of values in the arrays needs to be properly
 adjusted according to numCell. 
 
 3. Temperature measurement: The current code supports only one thermistor. The values
 need to be properly set according to the right thermistor.   
 
 4. SD Card
 If using the Arduino Mega, in the file:
 ~arduino/libraries/SD/utility/Sd2Card.h
 
 change the line: 
 #define MEGA_SOFT_SPI 0
 to 
 #define MEGA_SOFT_SPI 1
 
 Do not change the other pins!
 
 5. Temperature measurements:
 
 There are two ways temperature can be measured, using
 - thermistor (Vishay NTCLE100E3), pin 7- (comment the definition of TBAR)
 - Bosch BMP085 Barometric sensor - via I2C bus (uncomment the definition of TBAR)
 You must select at build time the type of sensor you want to use.
 
 6. Irradiance measurements:
 - Pins 2, 4, 5, 6, 7 for vertical irradiance.
 - Pins 3, 22, 23, 24, 25 for horizontal irradiance.
 
 
 //**************************************************************************************
 */

#include <SD.h> 
//#include <SdFat.h>
//#include <SdFatUtil.h>
#include <Wire.h>
#include <SPI.h>
#include "RTClib.h"

//-------------------------------------------------
// Type of temperature measurement system used.
//  Comment for thermistor, uncomment for barometer
//-------------------------------------------------
#define TBAR 

//--------------------------------------------------
// Activate debugging mode
// Comment for regular use, uncomment for debugging
//--------------------------------------------------


//#define DEBUG

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

//------------------
// Name and version 
//------------------

String nameProg = "SolarLogMeter";
String versProg = "0.10.7 - 20111028";
String developer = "Nicola Ferralis - ferralis@mit.edu";
char cfgFile[]="SLM.cfg";


//---------------
//Real time chip
//---------------

RTC_DS1307 RTC; // define the Real Time Clock object
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


//------------------
//Program variables
//------------------

void(* resetFunc) (void) = 0; //declare reset function @ address 0
int numCell = 4;        // Max number of cells to be measured

int voltIN[]={
  0,0,0,0};  // Applies the correction to the input voltage for a particular cell is the voltage divider is present (0), 
// or leave it with no correction (1).

float Rv1[] = {
  10000.0, 10000.0, 10000.0, 10000.0};         //resistor for voltage divider (top to Vin) 
float Rv2[] = {
  10000.0, 10000.0, 10000.0, 10000.0};         //resistor for voltage divider (bottom, to ground)

float Ri[] = {
  1.0, 1.0, 1.0, 1.0};     // resistor for current measurement A.

float Ri1[] = {
  1000.0, 1000.0, 1000.0, 1000.0};         //resistor for current amplification (fixed resistor 1K) 
float Ri2[] = {
  100000.0, 10000.0, 10000.0, 10000.0};     //resistor for current amplification (value of resistor determines the amplification factor) 

float Vcv[]= {                            // Multiplication factor for the voltage divider (it's calculated during setup).
  0.0, 0.0, 0.0, 0.0}; 
float Vci[]= {                            // Amplification factor for the current shunt (it's calculated during setup).
  0.0, 0.0, 0.0, 0.0}; 

// Because of the SD/RTC shield, the first available analog pin is #8
int fp=8;  

float maxVolt = 0.0; // Max voltage on scale. 
float lowV = 1.0;     // min voltage for LED warning

int avNum = 80;     // number of averages to be taken over an analog input
int ledPin = 13;       // on actual arduino boards this is pre-hooked up.
const int singleIVPin = 31;     // the number of the pushbutton pin
const int multiIVPin = 33;     // the number of the pushbutton pin
const int stopIVPin = 35;     // the number of the pushbutton pin
const int GLED = 7;     // Green LED
const int RLED = 6;    // Red LED
const int T1= 37;      // Transistor port for Voc1

/////////////////////////////////
//Definitions for debugging only 
////////////////////////////////
#ifdef DEBUG
//int potSteps[] = { 15, 15, 15, 15, 15, 15, 15, 15};  // Only for testing
//int potSteps[] = { 50, 50, 50, 50, 50, 50, 50, 100};
//int potSteps[] = {5, 5, 5, 5, 5, 5, 5, 5};
int potSteps[] = {10, 10, 10, 10, 10, 10, 10, 256};

int delaydeb = 1000;


#else
//Transistors for fixed resistors (to reduce resistance on dig. pots. at high load).
int potSteps[] = { 50, 50, 50, 50, 50, 50, 50, 240};  //steps for the potentiometer for each fixed resistor connected in parallel
#endif

int numFixedRes = 7;
const int Tr[]= {
  38, 39, 40, 41, 42, 43, 44};  //ransistor ports for fixed resistors

unsigned long restTime = 12;  //Time in between IV scans (minutes)
unsigned int delayTime = 1000; // Generic time delay (ms)

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

const int chipSelect = 10;
boolean sd = true;     // enable SD support
boolean sds = false;    //switch SD on/off
char nameFile[13];
char nameFileA[4][13];


//-----------------------------
// SPI specific (digital pot)
//-----------------------------
const int slaveSelectPin = 53;


//--------------------
// buttons definition
//--------------------

int singleIVbtn = 0;
int multiIVbtn = 0;
int stopIVbtn = 0;


//----------------------------------------
// Definitions for the Barometric sensor
//----------------------------------------

// setup the TLS230R to Arduion mapping
#define TSL_FREQ_PIN 19 // output use digital pin2 for interrupt 
#define TSL_S0       22 
#define TSL_S1       23   
#define TSL_S2       24   
#define TSL_S3       25
#define TSL1_FREQ_PIN 18 // output use digital pin2 for interrupt 
#define TSL1_S0       26 
#define TSL1_S1       27   
#define TSL1_S2       28   
#define TSL1_S3       29
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

  //----------------------------------------
  // set the transistor pin as output
  //----------------------------------------
  pinMode(T1, OUTPUT);
  for(int i=0; i<numFixedRes; i++)
  {
    pinMode(Tr[i], OUTPUT);
  }


  //----------------------------------------
  // set the slaveSelectPin as an output:
  //----------------------------------------
  pinMode (slaveSelectPin, OUTPUT);

  //----------------------------------------
  // initialize SPI:
  //----------------------------------------
  SPI.begin(); 

  //----------------------------------------
  //Initialize reference voltage
  //----------------------------------------
  analogReference(DEFAULT);  //0 to 5 V
  maxVolt = 5.0;
  //analogReference(INTERNAL2V56); //0-2.56V
  //maxVolt = 2.56;
  //analogReference(INTERNAL1V1); //0-1.1V
  //maxVolt = 1.1;

  //---------------------------------------------------
  //Stamp header with program details and data labels.
  //----------------------------------------
  delay(200);
  firstRunSerial();

  //----------------------------------------
  // get the time from the RTC
  //----------------------------------------
  Wire.begin();  
  RTC.begin();

  if (! RTC.isrunning()) {
    Serial.println("RTC is NOT running!");
  }

#ifdef TIMECAL
  else
  {
    RTC.adjust(DateTime(__DATE__, __TIME__));
    Serial.println("RTC is syncing!");
  }
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

  ////////////////////////////////////////////////////////////////////
  // Setup coeffcients for voltage dividers and amplification factors
  ////////////////////////////////////////////////////////////////////

  for (int i=0; i<numCell; i++) {

    if(voltIN[i]==0)
    {
      // set the coefficient to get the real voltage V before the divider  
      Vcv[i]=Rv2[i]/(Rv2[i]+Rv1[i]); 
    }
    else
    {
      Vcv[i]=1.0;
    }


    // set the amplification factor for voltage Vi of the shunt 
    Vci[i]=(Ri1[i] + Ri2[i])/Ri1[i]; 
  }


  //Run a few measurement to get the baseline (first measurements are skewed)
  getIrradiance(2, 1);


  //----------------------------------------
  // Stamp date, time, temperature
  //----------------------------------------  
  NowSerial();

  //-----------------------------------------------------------
  // Blink green LED for the number of cells that are active. 
  //----------------------------------------------------------- 
  blinkLED(GLED, numCell, 100);



  //----------------------------------------  
  // Print Serial menu 
  //----------------------------------------  
  Serial.println("---------------------------------------------------------------------------");
  Serial.println("SELECT FROM THE FOLLOWING OPTIONS:");
  Serial.println("1: Collect single IV - 2: Collect sequence IV - 3: Stop sequence IV");
  Serial.println("4: Start writing data on SD - 5: Stop writing data on SD");
  Serial.println("6: Stamp time/date/weather info - 0: Reset");
  Serial.println("---------------------------------------------------------------------------");

  Serial.println();  

  delay(100);

  if(sd==true)
  {
    analysisHeaderSD();
  }
}

///////////////////////////////////////////////////////////
// MAIN LOOP
///////////////////////////////////////////////////////////

void loop()
{// Accept input from serial 
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
        Serial.println(" minutes");
        delay(restTime*60*1000);
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
      Serial.println(" minutes");
      delay(restTime*60*1000);
    }


    blinkLED(GLED, 3, 200);
    stopIVbtn=0;
    sds=false;
    //headerSD();
    Serial.println("Stopping data logging into SD card...");

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

  float V[numCell], Vi[numCell];
  float Voc[numCell], Ioc[numCell], Isc[numCell], Pmax[numCell], Vmax[numCell], Imax[numCell];
  int jmax;
  int ip=fp;
  float T=0.0; 
  float P=0.0;
  float irra0 = 0.0;
  float irra1 = 0.0;

  // Close circuit with transistor to allow full measurement.

  digitalWrite(T1, HIGH);

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

  //Measure irradiance at the beginnig of the IV acquisition.
  getIrradiance(1, 2);
  irra0=watt0;
  irra1=watt1;


  //////////////////////////////
  //Measure IV
  //////////////////////////////

  //Setup for Isc, Pmax
  for (int i=0; i<numCell; i++)
  { 
    Isc[i]=0.0;
    Pmax[i] = 0.0;
    jmax=0;
  }

  int m1 = 0;

  //Turn all fixed resistors OFF 
  for(int g=0; g<numFixedRes; g++)
  {
    digitalWrite(Tr[g], HIGH);
  }

  delay(100);

  //Start the measurements
  for (int m=0; m<numFixedRes+1; m++)
  { 

    for (int j=0; j< potSteps[m]; j++)
    { 
      for (int g=0; g<6; g++) {
        digitalPotWrite(g, 255-j);
      }

      delay(100);

      Serial.print(j);
      Serial.print(",");

      writeDateSerial();  
      if(sds==true)
      {
        dataFile.print(j);
        dataFile.print(",");
        writeDateSD(dataFile);
      }

      ip=fp;


      for (int i=0; i<numCell; i++)
      { 
        // Acquire, average, and rescale with appropriate divider coefficient
        V[i]=avoltage(ip, maxVolt, avNum)/Vcv[i];  
        Vi[i]=avoltage(ip+1, maxVolt, avNum)/Vci[i];
        ip+=2;



        Isc[i]=max(Isc[i],Vi[i]/Ri[i]); 

        if(Pmax[i]<=V[i]*Vi[i]*1000/Ri[i])
        {
          Pmax[i]=V[i]*Vi[i]*1000/Ri[i];
          Vmax[i]=V[i];
          Imax[i]=Vi[i]*1000/Ri[i];
          jmax=j;
        }

        // write data on Serial and SD
        if(sds==true)
        {
          writeIVSD(dataFile,V[i],Vi[i],Ri[i]);
        }

        writeIVSerial(V[i],Vi[i],Ri[i]);  

      }
      dataFile.println();
      Serial.println();
      // Eventually insert T measurement here if T needs to be measured at every IV step 
      #ifdef DEBUG
      delay(delaydeb); //debugging only
      #endif
    }

    //Turn active transistor OFF.
    if(m!=numFixedRes+1)
    {
      digitalWrite(Tr[m], LOW);
      delay(100);
    }
    #ifdef DEBUG
    Serial.print("Switching OFF transistor: ");
    Serial.println(m+1);
    #endif
  }



  ////////////////////////
  // Measure and save Voc
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

  ip=fp;

  for (int i=0; i<numCell; i++) {

    digitalWrite(T1, LOW);
    delay(20);

    Voc[i]= avoltage(ip, maxVolt, avNum)/Vcv[i];
    Ioc[i]=avoltage(ip+1, maxVolt, avNum)/Vci[i];
    ip+=2;


    // write data on Serial and SD
    if(sds==true)
    {
      writeIVSD(dataFile,Voc[i],Ioc[i],Ri[i]);
      //delay(500); 
    }

    writeIVSerial(Voc[i],Ioc[i],Ri[i]);
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

  getIrradiance(1, 2);
  irra0=(irra0+watt0)/2;
  irra1=(irra1+watt1)/2;


  if(sds==true)
  {
    dataFile.println();
    dataFile.print("\"Average temperature (C)\",");
    dataFile.println(T);
#ifdef TBAR
    dataFile.print("\"Average pressure (mbar)\",");
    dataFile.println(P);    
#endif
    dataFile.print("\"Vertical Irradiance (mW/cm^2)\",");
    dataFile.println(irra0/1000.00);
    dataFile.print("\"Horizontal Irradiance (mW/cm^2)\",");
    dataFile.println(irra1/1000.00);
    dataFile.println();

  }
  Serial.println();
  Serial.print("\"Average temperature (C)\",");
  Serial.println(T);
#ifdef TBAR 
  Serial.print("\"Average pressure (mbar)\",");
  Serial.println(P); 
#endif  
  Serial.print("\"Vertical Irradiance (mW/cm^2)\",");
  Serial.println(irra0/1000.00);
  Serial.print("\"Horizontal Irradiance (mW/cm^2)\",");
  Serial.println(irra1/1000.00);
  Serial.println();


  if(sds==true)
  {
    dataFile.close();
  }

  ////////////////
  // Analyse data
  ////////////////


  analysisHeaderSerial();

  for (int i=0; i<numCell; i++) {
    File dataFile1 = SD.open(nameFileA[i], FILE_WRITE);

    analysisSerial(i, Voc[i], Isc[i]*1000, Vmax[i], Imax[i], Pmax[i], Vmax[i]*Imax[i]/(Voc[i]*Isc[i]*1000), jmax, T, P, irra0, irra1);

    if(sds==true)
    {
      analysisSD(dataFile1, i, Voc[i], Isc[i]*1000, Vmax[i], Imax[i], Pmax[i], Vmax[i]*Imax[i]/(Voc[i]*Isc[i]*1000), jmax, T, P, irra0, irra1);
    }

    if(sds==true)
    {
      dataFile1.close();
    }
  }

}



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
  float V = avoltage(Tpin, maxVolt, avNum);
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


//////////////////////////////////////////////
// convert analog output into actual voltage.
//////////////////////////////////////////////

inline float voltage(int analogPin, float volt)
{
  int v = analogRead(analogPin); 
  float vf = volt*((float)v/(float)1024);  //rescale channel with max voltage
  return vf;
}


//////////////////////////////////////////////////////////
// collect numAver times the analog output, average them, 
// and converts them into actual voltage.
/////////////////////////////////////////////////////////

float avoltage(int analogPin, float Volt, int numAver)
{
  float vt=0.0;
  for(int i = 0; i < numAver; ++i) 
  {
    float vf=voltage(analogPin,Volt);
    vt += vf;
  }
  return vt/(float)numAver;
}


//////////////////////////////////////////////
// set potentiometer
//////////////////////////////////////////////

inline int digitalPotWrite(int address, int value) {
  // take the SS pin low to select the chip:
  digitalWrite(slaveSelectPin,LOW);
  //  send in the address and value via SPI:
  SPI.transfer(address);
  SPI.transfer(value);
  // take the SS pin high to de-select the chip:
  digitalWrite(slaveSelectPin,HIGH);
}


//////////////////////////////////////////////
// Stamp data on Serial port
//////////////////////////////////////////////

void writeDateSerial(){
  // digital clock display of the time
  now = RTC.now();
  Serial.print("\"");
  Serial.print(now.month(), DEC );
  Serial.print("-");
  Serial.print(now.day(),DEC );
  Serial.print("-");
  Serial.print(now.year(), DEC);
  Serial.print("\"");
  //Serial.print(" ");     // for regular serial
  Serial.print(",\"");      // for csv 
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

void writeIVSerial(float V, float Vi, float Ri){

  Serial.print(",");    
  Serial.print(V*1000);
  Serial.print(","); 
  Serial.print(Vi*1000/Ri);
}

//////////////////////////////////////////////
// Stamp data on SD
//////////////////////////////////////////////

void writeDateSD(File dataFile){
  now = RTC.now();
  dataFile.print("\"");
  dataFile.print(now.month(), DEC);
  dataFile.print("-");
  dataFile.print(now.day(), DEC);
  dataFile.print("-");
  dataFile.print(now.year(), DEC);
  dataFile.print("\"");
  dataFile.print(",\"");      // for csv
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

void writeIVSD(File dataFile, float V, float Vi, float Ri){

  dataFile.print(",");       
  dataFile.print(V*1000);
  dataFile.print(","); 
  dataFile.print(Vi*1000/Ri);
}

///////////////////////////////////////////
// Stamp data headers on Serial  
///////////////////////////////////////////

void header(){  
  Serial.println();
  Serial.print("\"#\",");
  Serial.print("\"time\",\"date\"");
  for (int i=0; i<numCell; i++)
  {
    Serial.print(",\"V");
    //Serial.print(i);
    //Serial.print(" (V)\",\"V");
    Serial.print(i);
    Serial.print(" (mV)\",\"I");
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
  dataFile.print("\"#\",");
  dataFile.print("\"time\",\"date\"");
  for (int i=0; i<numCell; i++)
  {
    dataFile.print(",\"V");
    //dataFile.print(i);
    //dataFile.print(" (V)\",\"V");
    dataFile.print(i);
    dataFile.print(" (mV)\",\"I");
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
  Serial.print("\"j\",");
  Serial.print("\"T (C)\"");
#ifdef TBAR
  Serial.print(",\"P (mbar)\"");
#endif
  Serial.print(",\"VIrr. (mW/cm^2)\"");
  Serial.print(",\"HIrr. (mW/cm^2)\"");
  Serial.println();
}


///////////////////////////////////////////
// Stamp analysis data on Serial
///////////////////////////////////////////

void analysisSerial(int i, float V, float I, float V1, float I1, float P1, float FF, int j, float T, float P, float irra0, float irra1) {
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
  Serial.print(",");
  Serial.print(irra0/1000.00);
  Serial.print(",");
  Serial.print(irra1/1000.00);
  Serial.println();
  // Serial.println();

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
    dataFile.print("\"Date\",");
    dataFile.print("\"Time\",");
    dataFile.print("\"Voc (V)\",");
    dataFile.print("\"Isc (mA)\",");
    dataFile.print("\"Vmax (V)\",");
    dataFile.print("\"Imax (mA)\",");
    dataFile.print("\"Pmax (mW)\",");
    dataFile.print("\"FF (%)\",");
    dataFile.print("\"j\",");
    dataFile.print("\"T (C)\"");
#ifdef TBAR
    dataFile.print(",\"P (mbar)\"");
#endif
    dataFile.print(",\"VIrr. (mW/cm^2)\"");
    dataFile.print(",\"HIrr. (mW/cm^2)\"");
    dataFile.println();
    dataFile.close();
  }
}


///////////////////////////////////////////
// Stamp analysis data on SD
///////////////////////////////////////////

void analysisSD(File dataFile, int i, float V, float I, float V1, float I1, float P1, float FF, int j, float T, float P, float irra0, float irra1) {

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
  dataFile.print(",");
  dataFile.print(irra0/1000.00);
  dataFile.print(",");
  dataFile.print(irra1/1000.00);
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
    // dataFile.println(" ");
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

  now = RTC.now();
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
    numCell = value(myFile); // number of cells 
    avNum = value(myFile);     // number of averages
    restTime = value(myFile);  // time in between IV scans (msecs)
    delayTime = value(myFile);  // generic delay time (msecs)
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

    myFile.println(numCell);   // number of cells   
    myFile.println(avNum);      // number of averages
    myFile.println(restTime);   // time in between IV scans (minutes)
    myFile.println(delayTime);  // generic delay time (msecs)
    for(int i=0; i<numCell; i++)
    {
      myFile.println(voltIN[i]);
    }  // Parameter to apply (0) or not (1) the voltage divider correction.

    myFile.close();

  }
}


///////////////////////////////////////////
// Reads full integers from a line in a file 
///////////////////////////////////////////

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

  getIrradiance(1, 0);

  DateTime now = RTC.now();
  Serial.println();
  Serial.println("--------------------------------------");
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
  Serial.print(" (");
  Serial.print(now.month(), DEC );
  Serial.print("-");
  Serial.print(now.day(),DEC );
  Serial.print("-");
  Serial.print(now.year(), DEC);
  Serial.println(")");
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

  Serial.print("Vertical irradiance: ");
  Serial.print(watt0/1000.0000);
  //Serial.print(watt0, DEC);
  Serial.println(" mW/cm^2");
  Serial.print("Horizontal irradiance: ");
  Serial.print(watt1/1000.0000);
  //Serial.print(watt1, DEC);
  Serial.println(" mW/cm^2");  
  Serial.println("--------------------------------------");
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
  Serial.println();
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
  Wire.send(address);
  Wire.endTransmission();

  Wire.requestFrom(BMP085_ADDRESS, 1);
  while(!Wire.available())
    ;

  return Wire.receive();
}

// Read 2 bytes from the BMP085
// First byte will be from 'address'
// Second byte will be from 'address'+1
int bmp085ReadInt(unsigned char address)
{
  unsigned char msb, lsb;

  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.send(address);
  Wire.endTransmission();

  Wire.requestFrom(BMP085_ADDRESS, 2);
  while(Wire.available()<2)
    ;
  msb = Wire.receive();
  lsb = Wire.receive();

  return (int) msb<<8 | lsb;
}

// Read the uncompensated temperature value
unsigned int bmp085ReadUT()
{
  unsigned int ut;

  // Write 0x2E into Register 0xF4
  // This requests a temperature reading
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.send(0xF4);
  Wire.send(0x2E);
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
  Wire.send(0xF4);
  Wire.send(0x34 + (OSS<<6));
  Wire.endTransmission();

  // Wait for conversion, delay time dependent on OSS
  delay(2 + (3<<OSS));

  // Read register 0xF6 (MSB), 0xF7 (LSB), and 0xF8 (XLSB)
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.send(0xF6);
  Wire.endTransmission();
  Wire.requestFrom(BMP085_ADDRESS, 3);

  // Wait for data to become available
  while(Wire.available() < 3)
    ;
  msb = Wire.receive();
  lsb = Wire.receive();
  xlsb = Wire.receive();

  up = (((unsigned long) msb << 16) | ((unsigned long) lsb << 8) | (unsigned long) xlsb) >> (8-OSS);

  return up;
}

#endif


/////////////////////////////////////////////////////
// Routines for Irradiance measurements 
// Using TAOS TSL230R  
/////////////////////////////////////////////////////

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








