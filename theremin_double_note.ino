#include "Volume.h"

int pinBuzzer= 5;     // setted in library Volume.h, pinBuzzer not used

Volume vol;           // Volume.h object

int triggerPin = 10;  // HC-SR04 trigger
int echoPin = 8;      // echo

int pinLDR = A0;      // LDR photoresistor sensor

int pinPOT = A3;      // Potentiometer pin

int minL = 0;         // Minimum Light (changed on startup with calibration)
int maxL = 1023;      // Maximum Light

int minF = 1000;      // range for HC-SR04 distance
int maxF = 14000;

int ptr = 0;          // cursor for handling mean values

#define  siz  5       // size of arrays that store values to calculate mean values

float uss0[siz];      // values to calc mean of HC-SR04 measures
float uss1 = 0;       // mean value

float ldr0[siz];      // values to calc mean of LDR measures
float ldr1 = 0;       // mean value

float pot0[siz];      // values to calc mean of POT measures
float pot1 =0;        // mean value


// array of led pins
byte pins[7] = {12,11,7,6,4,3,2};


void setup() {

  pinMode(echoPin, INPUT);
  pinMode(triggerPin, OUTPUT);
  pinMode(pinLDR, INPUT);
  pinMode(pinPOT, INPUT_PULLUP);

  for(int i=0;i<7;i++) {pinMode(pins[i],OUTPUT); digitalWrite(pins[i],LOW);}

  pinMode(pinBuzzer,OUTPUT);
  
  Serial.begin(57600);
  Serial.println("Inizio");
  
  // zeros
  for(int j=0;j<siz;j++) {
    uss0[j]=0; pot0[j]=0; ldr0[j]=0;
  }

  // calibrate light sensor
  calibrate();

  // start vol library (note: timer change speed!)
  vol.begin();

}




// read "i" puleses and calculate the mean value
// (well... used to read jsut 1 pulse :-) )
float readPulse( int i ){
  float d = 0;
  for(int k=0;k<i;k++) {
    digitalWrite(triggerPin, HIGH);
    vol.delayMicroseconds(2);
    digitalWrite(triggerPin, LOW);  
    d += pulseIn(echoPin, HIGH,200000); // Read in times pulse
  }
  d = d/ (1.0*i);
  return d;
}


//
// calibrate max and min for LDR
void calibrate () {
  // calibrate min
  long m = millis() + 4000;
  long v = 0; int i=0;
  while(millis()< m) {
      v = v + analogRead(pinLDR);
      if((millis()/400)%2==0) digitalWrite(pins[0],HIGH); else digitalWrite(pins[0],LOW);
      delay(100);
      i=i+1;
  }
  v = v / i;
  minL = v;
  Serial.print("MINIMUS IS ");
  Serial.println(minL);
  Serial.println(i);
  digitalWrite(pins[0],LOW);
  // calibrate max
  m = millis() + 4000;
  v = 0; i=0;
  while(millis()< m) {
      v = v + analogRead(pinLDR);
      i=i+1;
      if((millis()/400)%2==0) digitalWrite(pins[6],HIGH); else digitalWrite(pins[6],LOW);
       delay(100);
  }
  v = v / i;
  maxL = v;
  Serial.print("MAX ");
  Serial.println(maxL);
  Serial.println(i);
  digitalWrite(pins[6],LOW);
}

int volFader = 0;

unsigned long timerPot = 0;

void showLed(float freq) {
  if (vol.millis() < timerPot) return;
  byte on = ( (freq - minF) / (maxF - minF)) * 7 ;
  for(int i =0;i<7;i++) {
    digitalWrite(pins[i], i == on ? HIGH : LOW); 
  }
}

int h = 0;

int c = 0;

int lastPitch = 0;
int lastVol = 0;

float olduss = 0;
float ussread = 0;
void loop() {

  //
  // read sensor LDR
  float ldr = analogRead(pinLDR);
  
  //
  // read sensor HC-SR04
  ussread = readPulse(1);
  float uss = ussread;
  if(ussread > 1500){  uss = olduss; c++;} else { olduss=ussread; c=0;}
 
  //
  // read POT value
  float potBase = analogRead(pinPOT);
  float pot = potBase / 207;

  //
  // store val mean for read HC-SR04
  uss1 += uss;
  uss0[ptr] = uss;

  //
  // store val for mean read LDR
  ldr1 += ldr;
  ldr0[ptr] = ldr;

  //
  // store val for mean read POT
  pot1 += pot;
  pot0[ptr] = pot;

  //
  // increase counter ptr for mean (both ldr and uss, and pot)
  ptr++;
  ptr = ptr % siz;

  //
  // calc mean
  float v0 = ldr1/siz;             // ldr ---> volume
  float f0 = 10 * ( uss1/siz );    // uss ---> pitch
  float p0 = pot1/siz;             // pot ---> sound type

  //
  // remove old value from mean arrays
  uss1 -= uss0[ptr];
  ldr1 -= ldr0[ptr];
  pot1 -= pot0[ptr];

  int pitchVal;
  int volVal;
  byte potVal;

  if (f0 < minF) { f0=minF;}
  if (f0 >= maxF ) { f0=maxF; } 
    
  if (v0 < minL) v0=minL;
  if (v0 > maxL) v0=maxL;
  volVal = map(v0,minL,maxL,0,255);
  
  potVal = floor(p0) + 1;

  showLed(f0);

  if (c > 2 ) {
    volFader =  volVal;
  } else {
    volFader = 0;  
  }

  int v = volVal - volFader;

  //
  // classic theremin
  if (potVal == 1) {
    float ff = ( (f0 - minF) / (maxF - minF)) * ( 21.0 );
    pitchVal = 440  * pow( 1.05946, ff  );
    vol.tone(pitchVal, v);
    vol.delay(5);
  }

  //
  // incremental divider
  if (potVal == 2) {
    float ff = ( (f0 - minF) / (maxF - minF)) * ( 11.0 );
    h++;
    pitchVal = 1140.0 * pow( 1.05946, ff  ) / (1.0 * h);
    if (h> 10) h = 0;
    vol.tone(pitchVal, v);
    vol.delay(5);    
  }

  //
  // tang blips
  if (potVal == 3) {
    float ff = ( (f0 - minF) / (maxF - minF)) * ( 11.0 );
    pitchVal = 440.0 * pow( 1.05946, ff  ) + 100.0 * tan(vol.millis()/200.0);
    if(pitchVal < 0) pitchVal = 0;
    vol.tone(pitchVal, v);
    vol.delay(5);
  }

  //
  // noisy
  if (potVal == 4) {
    float ff = ( (f0 - minF) / (maxF - minF)) * ( 1.5 );
    pitchVal = ff * random(8000,12000);
    vol.tone(pitchVal, v);
    vol.delay(3);    
    vol.noTone();
    vol.delay(2);
  }

  //
  // bell notes
  if (potVal == 5) {
    float ff = ( (f0 - minF) / (maxF - minF)) * ( 7.0 );
    int remap[13] = {262,294,330,349,392,440,493,523,587,659,698,783,880};
    pitchVal = remap[byte(ff)];
    int flag = false;
    if (lastPitch == pitchVal) {
      lastVol = lastVol / 1.05;
      v = lastVol;
      flag = true;
    } else {
      lastVol = v;
    }
    lastPitch = pitchVal;
    vol.tone(pitchVal, v);
    vol.delay( flag ? 50 : 5);    
  }

  //
  // debug, monitor/plot
  Serial.print("pot:");
  Serial.print(potVal);
  Serial.print(",");
  Serial.print("vol:");
  Serial.print(volVal);
  Serial.print(",");
  Serial.print("pitch:");
  Serial.print(pitchVal);
  Serial.println();

}