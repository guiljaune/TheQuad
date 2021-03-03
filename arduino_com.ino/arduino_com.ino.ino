#include <Servo.h>
#include <math.h>


#define MAX_GAMMA 50
///VARIABLES SERVOMOTORES
#define MAX_SERVOS 12
Servo Servos[MAX_SERVOS];


//VARIABLES PARA CONTROLAR EL TIEMPO
unsigned long previousMillis = 0;
const long interval = 20;
unsigned long loopTime;
unsigned long previousLooptime;
double t;

//VARIABLES PARA RECIVIR EL COMANDO
const byte numChars = 32;
char receivedChars[numChars];
int spaceCounter = 0;

boolean newData = false;

int pulse0 = 55;
int pulse1 = 0;
int pulse2 = 65;
int pulse3 = 55;
int pulse4 = 0;
int pulse5 = 65;
int pulse6 = 55;
int pulse7 = 0;
int pulse8 = 65;
int pulse9 = 1200;
int pulse10 = 1000;
int pulse11 = 1718;



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial1.begin(9600);
  //IMUSetup();
  connectServos();
  
  
}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      // save the last time you blinked the LED
      previousMillis = currentMillis;
      t = float(currentMillis) / 1000;
      /////////cuenta el tiempo que tarda el bucle en ejecutarse
      loopTime = currentMillis - previousLooptime;
      Serial1.print("<");
//      Serial1.print(pulse0);
//      Serial1.print("#");
//      Serial.println();      
//      delay(100);
//      if (Serial1.available()){
//          Serial1.println("ready");}
//      delay(20);
      recvWithStartEndMarkers();
//      Serial.print(pulse11);
//      Serial.println(); 
//      delay(20);     
        newData = false;
        moveServos(pulse0, pulse1, pulse2, pulse3, pulse4, pulse5, pulse6, pulse7, pulse8, pulse9, pulse10, pulse11);
//      Serial.print(pulse0);
//      Serial.print(pulse1);
//      Serial.print(pulse2);
//      Serial.println(); 
//      Serial1.print(pulse0);
//      Serial1.print(pulse1);
//      Serial1.print(pulse2);
      //Serial1.println(); 
      previousLooptime = currentMillis;
  }
}


void connectServos() {
  Servos[0].attach(0);//FR
  Servos[1].attach(1);//0=2500
  Servos[2].attach(2);

  Servos[3].attach(3);//FL
  Servos[4].attach(4);//0=550
  Servos[5].attach(5);

  Servos[6].attach(6);//BR
  Servos[7].attach(7);//0=2500
  Servos[8].attach(8);

  Servos[9].attach(9);//BL
  Servos[10].attach(10);//0=550
  Servos[11].attach(11);
}



void moveServos(int pulse0, int pulse1, int pulse2, int pulse3, int pulse4, int pulse5, int pulse6, int pulse7, int pulse8, int pulse9, int pulse10, int pulse11) { 

//  Servos[0].write(pulse0);
//  Servos[1].write(pulse1);
//  Servos[2].write(pulse2);
//
//  Servos[3].write(pulse3);
//  Servos[4].write(pulse4);
//  Servos[5].write(pulse5);
//
//  Servos[6].write(pulse6);
//  Servos[7].write(pulse7);
//  Servos[8].write(pulse8);

  Servos[9].writeMicroseconds(pulse9);
  Serial1.print(pulse9);
  Servos[10].writeMicroseconds(pulse10);
  Serial1.print(pulse10);
  Servos[11].writeMicroseconds(pulse11);
  Serial1.print(pulse11);
}


void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char spaceMarker = '#';
    
    char rc;
 
    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();
 //       Serial.print("serial In");
//        Serial1.println(rc);
//        Serial1.println(); 
        if (recvInProgress == true) {
            if (rc != endMarker && rc != spaceMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else if (rc == spaceMarker ){
              receivedChars[ndx] = '\0';
              if (spaceCounter==0){
                pulse0 = atoi(receivedChars);
                spaceCounter++;
                ndx=0;
              }
              else if (spaceCounter==1){
                //Serial.println(receivedChars);
                pulse1 = atoi(receivedChars);
                spaceCounter++;
                ndx=0;
              }
              else if (spaceCounter==2){
                //Serial.println(receivedChars);
                pulse2 = atoi(receivedChars);
                spaceCounter++;
                ndx=0;
              }
              else if (spaceCounter==3){
                //Serial.println(receivedChars);
                pulse3 = atoi(receivedChars);
                spaceCounter++;
                ndx=0;
              }
              else if (spaceCounter==4){
                //Serial.println(receivedChars);
                pulse4 = atoi(receivedChars);
                spaceCounter++;
                ndx=0;
              }
              else if (spaceCounter==5){
                //Serial.println(receivedChars);
                pulse5 = atoi(receivedChars);
                spaceCounter++;
                ndx=0;
              }
              else if (spaceCounter==6){
                //Seral.println(receivedChars);
                pulse6 = atoi(receivedChars);
                spaceCounter++;
                ndx=0;
              }
              else if (spaceCounter==7){
                //Serial.println(receivedChars);
                pulse7 = atoi(receivedChars);
                spaceCounter++;
                ndx=0;
              }
              else if (spaceCounter==8){
                //Serial.println(receivedChars);
                pulse8 = atoi(receivedChars);
                spaceCounter++;
                ndx=0;
              }
              else if (spaceCounter==9){
                //Serial.println(receivedChars);
                pulse9 = atoi(receivedChars);
//                Serial1.println(receivedChars);
                spaceCounter++;
                ndx=0;
              }
              else if (spaceCounter==10){
                //Serial.println(receivedChars);
                pulse10 = atoi(receivedChars);
                spaceCounter++;
                ndx=0;
              }
              else if (spaceCounter==11){
                //Serial.println(receivedChars);
                pulse11 = atoi(receivedChars);
                spaceCounter++;
                ndx=0;
              }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                //Serial.println(receivedChars);
                pulse11 = atoi(receivedChars);
                recvInProgress = false;
                ndx = 0;
                spaceCounter=0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}
