
//Note: the laser uses the I2C pins which are:
//SDA: 20
//SCL: 21
//I2C pins are labeled on the board

// For RAMPS 1.4
const int X_STEP_PIN = 54;
const int X_DIR_PIN = 55;
const int X_ENABLE_PIN = 38;
//#define X_MIN_PIN           3
//#define X_MAX_PIN           2
 
const int Y_STEP_PIN = 60;
const int Y_DIR_PIN = 61;
const int Y_ENABLE_PIN = 56;
//#define Y_MIN_PIN          14
//#define Y_MAX_PIN          15
 
const int Z_STEP_PIN = 46;
const int Z_DIR_PIN = 48;
const int Z_ENABLE_PIN = 62;
//#define Z_MIN_PIN          18
//#define Z_MAX_PIN          19

const int LightPin = 32;
//#define E_STEP_PIN         26
//#define E_DIR_PIN          28
//#define E_ENABLE_PIN       24
 
//#define SDPOWER            -1
//#define SDSS               53
//#define LED_PIN            13
 
//#define FAN_PIN            9
 
//#define PS_ON_PIN          12
//#define KILL_PIN           -1
 
//#define HEATER_0_PIN       10
//#define HEATER_1_PIN       8
//#define TEMP_0_PIN          13   // ANALOG NUMBERING
//#define TEMP_1_PIN          14   // ANALOG NUMBERING

#define HIGH_ACCURACY

//steps/mm ratio for this machine
const int ratio = 200;

//needed for Serial read functions
const byte numChars = 32;
char receivedChars[numChars];
boolean newData = false;

//some variable definitions
int const stepPins[] = {X_STEP_PIN,Y_STEP_PIN,Z_STEP_PIN};
int const dirPins[] = {X_DIR_PIN,Y_DIR_PIN,Z_DIR_PIN};
int const enablePins[] = {X_ENABLE_PIN,Y_ENABLE_PIN,Z_ENABLE_PIN};
double Z_read = 0; //in mm, 50mm is the minimum range
double OurPos[] = {0,0,0}; //in mm
String command = "";
double OurSpeed = 500; //initially set to 500mm/min, could be as high as 2000
double curSpeed = 1; //changed to global variable for debugging
double Y_incr = 1; //could be set as low as 0.005mm, but that would take forever
double ProfileZ = 50; //value between 0 and 150mm. should be 50-100mm above all target surfaces (50mm above tallest surface)
unsigned long programTime = 0;
bool motorsOn = false;
double defaultLength = 2; //mm, the length at which posXYorZ goes to
bool lightOn = false;

//some commands from python 
//define keyphrases; <> are the start and end markers
char beginCtrl = 'a';
char posX = 'b';
char negX = 'c';
char posY = 'd';
char negY = 'e';
char posZ = 'f';
char negZ = 'g';
char beginProfile = 'h';
char setZero = 'i';
char closeCtrl = 'l';
char setZ = 'm'; //sets the current z height to the profile height
char toggleLight = 'o'; //toggles light on and off
char getLocAndMeas = 'p'; //prints the current measurement and location

//define keyphrases that are included in data
char setOurSpeed = 'j'; //e.g. "<j500>" means set speed to 500mm/min (max chars is 7)
char setYincr = 'k'; //e.g. "<k1>" means set Yincr to 1mm
char setDefaultLength = 'n'; //e.g. "<n10>" means set length to 10mm

//Variable for if the sensor is working
bool sensorOn = false;

#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;


void setup(){
  //serial communication mega start
  Serial.begin(115200);
  Wire.begin();

  //sensor initialization
  sensorInit();

  //define pins for steppers
  pinMode(stepPins[0], OUTPUT);
  pinMode(dirPins[0], OUTPUT);
  pinMode(enablePins[0], OUTPUT);
  pinMode(stepPins[1], OUTPUT);
  pinMode(dirPins[1], OUTPUT);
  pinMode(enablePins[1], OUTPUT);
  pinMode(stepPins[2], OUTPUT);
  pinMode(dirPins[2], OUTPUT);
  pinMode(enablePins[2], OUTPUT);
  pinMode(LightPin, OUTPUT);

  //disable motor pins to start
  digitalWrite(enablePins[0], HIGH);
  digitalWrite(enablePins[1], HIGH);
  digitalWrite(enablePins[2], HIGH);
  digitalWrite(LightPin, LOW);

  
}

void loop(){
  int tempIndex;
  tempIndex = recvWithStartEndMarkers();

  //recievedChars;
  if(newData){
    Serial.println(receivedChars);
    if(receivedChars[0]==beginCtrl){
      //enable motor pins//we don't do this here anymore 
      motorsOn = true;     
      Serial.println("<Motors Online>");
    }else if(receivedChars[0]==posX&&motorsOn){
      gotoLinearly(0, 0, defaultLength);
    }else if(receivedChars[0]==negX&&motorsOn){
      gotoLinearly(0, 1, defaultLength);//go 2mm at OurSpeed in the negX direction
    }else if(receivedChars[0]==posY&&motorsOn){
      gotoLinearly(1, 0, defaultLength);
    }else if(receivedChars[0]==negY&&motorsOn){
      gotoLinearly(1, 1, defaultLength);
    }else if(receivedChars[0]==posZ&&motorsOn){
      gotoLinearly(2, 0, defaultLength);
    }else if(receivedChars[0]==negZ&&motorsOn){
      gotoLinearly(2, 1, defaultLength);
    }else if(receivedChars[0]==beginProfile&&motorsOn){
      performProfile();
    }else if(receivedChars[0]==setZero){
      OurPos[0]=0;
      OurPos[1]=0;
      OurPos[2]=0;
      Serial.println("<All axes Zeroed>");    
    }else if(receivedChars[0]==closeCtrl){
      //disable motor pins
      digitalWrite(enablePins[0], HIGH);
      digitalWrite(enablePins[1], HIGH);
      digitalWrite(enablePins[2], HIGH);
      motorsOn = false;
      Serial.println("<Motors Offline>");
    }else if(receivedChars[0]==setZ){
      ProfileZ = OurPos[2];
      Serial.println("<ProfileZ set to current Z>");
    }else if(receivedChars[0]==setOurSpeed){//e.g. "<j500>" means set speed to 500mm/min (max chars is 7)
      OurSpeed = getNumericCommand(tempIndex);
      String tempMess = "<New Speed = "+String(OurSpeed)+"mm/min>";
      Serial.println(tempMess);
    }else if(receivedChars[0]==setYincr){//e.g. "<k1>" means set Y_incr to 1mm
      Y_incr = getNumericCommand(tempIndex);
      String tempMess = "<New Y_incr = "+String(Y_incr)+"mm>";
      Serial.println(tempMess);
    }else if(receivedChars[0]==setDefaultLength){
      defaultLength = getNumericCommand(tempIndex);
      String tempMess = "<New defaultLength = "+String(defaultLength)+"mm>";
      Serial.println(tempMess);
    }else if(receivedChars[0]==toggleLight){
      if(lightOn){
        digitalWrite(LightPin, LOW);
        lightOn = false;
      }else{
        digitalWrite(LightPin, HIGH);
        lightOn = true; 
      }
      Serial.println("light toggled");
    }else if(receivedChars[0]==getLocAndMeas){
      SendAllVars();
    }else{
    }
  }
  newData = false;

}

void sensorInit(){
  //sensor initialization
  sensor.setTimeout(500);
  if (!sensor.init()){
    Serial.println("Failed to detect and initialize sensor!");
  }else{
    #if defined HIGH_ACCURACY
      // increase timing budget to 200 ms
      sensor.setMeasurementTimingBudget(200000);
    #endif
    sensorOn = true;    
  }  
}

double getNumericCommand(int tempIndex){
  //this program gets the numeric portion of the command string sent
  String strCommand="";
  double NumericCommand;
  for(int i=1; i<tempIndex; i++){
    strCommand = strCommand + receivedChars[i];
  }
  NumericCommand = strCommand.toDouble();
  return NumericCommand;  
}

void performProfile(){
  //performs a profile using previously obtained parameters
  //use: 
  //OurSpeed
  //Y_incr
  //ProfileZ
  //gotoLinearly(int XYorZ, int fororback, double OurDisp)
  double locStart[] = {0,0,ProfileZ};
  bool NotFinished = true;

  //go to ProfileZ before you start
  gotoLinearly_abs(2,locStart[2]);

  //go to ProfileX before you start
  gotoLinearly_abs(0,locStart[0]);

  //go to ProfileY before you start
  gotoLinearly_abs(1,locStart[1]);

  //main body of profile
  while(NotFinished){
    gotoLinearly_wSens(0,0,120);
    gotoLinearly_wSens(1,0,Y_incr);
    gotoLinearly_wSens(0,1,120);
    gotoLinearly_wSens(1,0,Y_incr);
    if(OurPos[1]>=130){
      NotFinished = false;
    }
  }

  //go to ProfileX before to end
  gotoLinearly_abs(0,locStart[0]);

  //go to ProfileY before to end
  gotoLinearly_abs(1,locStart[1]);
}

void gotoLinearly_abs_wSens(int XYorZ, double LocDesir){
  //changes an absolute command into relative to be used by gotoLinearly_wSens
  double tempDisp;
  int fororback;
  tempDisp = LocDesir-OurPos[XYorZ];
  if(tempDisp>=0){
    fororback = 0;
  }else{
    fororback = 1;
  }
  tempDisp = abs(tempDisp);
  gotoLinearly_wSens(XYorZ, fororback, tempDisp); 
}

void gotoLinearly_abs(int XYorZ, double LocDesir){
  //changes an absolute command into relative to be used by gotoLinearly
  double tempDisp;
  int fororback;
  tempDisp = LocDesir-OurPos[XYorZ];
  if(tempDisp>=0){
    fororback = 0;
  }else{
    fororback = 1;
  }
  tempDisp = abs(tempDisp);
  gotoLinearly(XYorZ, fororback, tempDisp);      
}

int recvWithStartEndMarkers() {
    //function which will recieve messages fully of style "<command>"
    //these recieved commands (without the <>) are stored in global receivedChars which needs to be converted to a string
    
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;
    int messageLength = 0;
 
    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                messageLength = ndx;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
    return messageLength;
}

void gotoLinearly_wSens(int XYorZ, int fororback, double OurDisp){
  //this function moves until you reach an X Y or Z only disp, velocity is not instantaneous 
  //will take a measurement every 0.5mm
  //XYorZ = 0,1,2
  //fororback = 0,1 forward and back respectively
  //OurDisp = the distance that needs to be moved

  //define some variables
  int OurSign = 1;
  double startPos = OurPos[XYorZ];

  //made specifically for this program
  double meas_incr = 1; //mm

  
  //determine direction
  if(fororback==0){
    digitalWrite(dirPins[XYorZ],HIGH);
    OurSign = 1;
  }else{
    digitalWrite(dirPins[XYorZ],LOW);
    OurSign = -1;
  }

  //determine where to end up
  double desPos = startPos+OurDisp*OurSign;
  
  bool Finished = false;
  while(Finished==false){

    //send data
    SendAllVarsAndMeas();

    //call gotoLinearly
    gotoLinearly(XYorZ, fororback, meas_incr);
    
    //determine if the loop should end
    if(OurSign>0){
      if(OurPos[XYorZ]>=desPos){
        Finished = true;
      }  
    }else{
      if(OurPos[XYorZ]<=desPos){
        Finished = true;
      }
    }
  }
  //disable pin
  digitalWrite(enablePins[XYorZ], HIGH);
    
}

void gotoLinearly(int XYorZ, int fororback, double OurDisp){
  //this function will take you to a X Y or Z position only, you start at zero speed and will end at zero speed
  //XYorZ = 0,1,2
  //fororback = 0,1 forward and back respectively
  //OurDisp = the distance that needs to be moved
  //OurSpeed = speed to go at

  //define some variables
  double DesSteps = round(OurDisp*ratio);
  double curSteps = 0;
  curSpeed = 1; //need to start at none zero value
  double nextSpeed = 0; 
  double accel = 60000; //mm/min^2 ; 7000 was too slow; 10800; corresponds to 6000steps/s^2 was too fast; //marlin recommends 1.8e+8 for x and y and 6e+6 for z
  double DesDelay = 150000/OurSpeed; //(0.5)*60s/1min*1000000micrS/s*1step*1mm/200steps*1/speed[mm/min]
  double curDelay = 150000; //microseconds; delay corresponding to 1mm/min
  double nextDelay = 150000;
  double curDisp = 0;
  int OurSign = 1;
  double startPos = OurPos[XYorZ];
  double accelSteps = 0;//used to tell if it needs to start decelerating

  //enable stepper
  digitalWrite(enablePins[XYorZ], LOW);
  
  //determine direction
  if(fororback==0){
    digitalWrite(dirPins[XYorZ],HIGH);
    OurSign = 1;
  }else{
    digitalWrite(dirPins[XYorZ],LOW);
    OurSign = -1;
  }
  bool Finished = false;
  while(Finished==false){

    //Update variables
    curDisp = curSteps/ratio;
    OurPos[XYorZ] = startPos+OurSign*curDisp;
    

    //acceleration branch                       
    if((OurSpeed>curSpeed)&&(DesSteps-curSteps>accelSteps)){
      nextSpeed = curSpeed+accel*curDelay*3.3*pow(10,(-8)); //convert from microseconds to mins
      nextDelay = 4.5*pow(10,12)/(3*pow(10,7)*curSpeed+accel*curDelay);
      accelSteps = accelSteps+1;
    }//constant velocity branch
    else if(DesSteps-curSteps>accelSteps){
      nextSpeed = curSpeed;
      nextDelay = 150000/nextSpeed;
    }//deceleration branch
    else if((DesSteps-curSteps<=accelSteps)&&(curSpeed>=1)){
      nextSpeed = curSpeed-accel*curDelay*3.3*pow(10,(-8)); //convert from microseconds to mins
      nextDelay = 4.5*pow(10,12)/(3*pow(10,7)*curSpeed-accel*curDelay);
    }

    if(curSpeed<1){
      curSpeed = 0;
    }else{
      //update variables accordingly
      curSpeed = nextSpeed;
      curDelay = nextDelay;
       

      //apply step
      digitalWrite(stepPins[XYorZ],HIGH);
      delayMicroseconds(curDelay);
      digitalWrite(stepPins[XYorZ],LOW);
      delayMicroseconds(curDelay);

      curSteps = curSteps+1;
    }
    
    //determine if the loop should end
    if(curSteps>=DesSteps||curSpeed==0){
      Finished = true;  
    } 
  }
  //disable stepper
  digitalWrite(enablePins[XYorZ], HIGH);
  //send current position out 
  //SendAllVars();
}

void SendAllVars(){
  MeasDist();
  programTime = millis();
  String message = "<"+String(OurPos[0])+","+String(OurPos[1])+","+String(OurPos[2])+","+String(Z_read)+","+String(programTime)+","+String(curSpeed)+">";
  Serial.println(message);  
}
void SendAllVarsAndMeas(){
  MeasDist();
  programTime = millis();
  String message = String(OurPos[0])+" "+String(OurPos[1])+" "+String(Z_read);
  Serial.println(message);  
}


void MeasDist(){
  if(sensorOn){
    Z_read = double(sensor.readRangeSingleMillimeters())/2;  
  }else{
    Z_read = 0;  
  }
}


//for later
//  Serial.print(double(sensor.readRangeSingleMillimeters())/2);
//  Serial.print("mm");
//  if (sensor.timeoutOccurred()) { 
//    Serial.print(" TIMEOUT"); 
//  }
