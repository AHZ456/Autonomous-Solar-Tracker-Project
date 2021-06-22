#include <Servo.h>
#include <avr/sleep.h>
#define SLEEP_TIMER 60
#define UPR_LMT 45
#define LWR_LMT 100
int ULp=A2,DLp=A6,URp=A0,DRp=A4;
int UL=0,DL=0,UR=0,DR=0;
int pinsb=3,pinsp=6,pint=2,posb=0,posp=UPR_LMT,suml=0,sumr=0,sumd=0,sumu=0,INTR=13,threshold=16,b=0,previous=0,off=10000,WIRES=10000;
int timer=0,ML=0;
//MOTORS SHOULD BE CONNECTED TO THE DRAIN!!!!
//FREE WHEELING DIODE!!!!!+ capacitor
//servob 85 stopping speed
//NOISE PROBLEM!!!!!!!!!!!!!!!!!!!!!! (reduce resistance)
//REMOVE THE LEDS!
Servo servob;
Servo servop;
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GoingToSleep(){
sleep_enable();
attachInterrupt(0,wakeUp,LOW);
set_sleep_mode(SLEEP_MODE_PWR_DOWN);
sleep_cpu(); 
}
void wakeUp(){
  sleep_disable();
  detachInterrupt(0);
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(9600);
servob.attach(pinsb);
servop.attach(pinsp);
pinMode(INTR,INPUT_PULLUP);
pinMode(ULp,INPUT);
pinMode(DLp,INPUT);
pinMode(URp,INPUT);
pinMode(DRp,INPUT);
servop.write(posp);
 
}
//counter+
//clock-
void loop() {
digitalWrite(pint,1);  
b=0;
UL=analogRead(ULp);
DL=analogRead(DLp);
UR=analogRead(URp);
DR=analogRead(DRp);  
sumr=UR+DR;
suml=UL+DL;
sumu=UL+UR;
sumd=DL+DR;


if(sumr > suml+threshold){
  posb=88;
  b=1;
  previous=millis();
  
}
else if(sumr+threshold < suml){
posb=82;
b=1;
previous=millis();
}
else if(sumr <= suml+threshold && sumr+threshold >= suml){
posb=85;
timer=millis();
}
if(sumu > sumd+threshold){
if(posp >UPR_LMT){
posp--;
  delay(100);
  b=1;
}
}
else if(sumu+threshold < sumd){
if(posp < LWR_LMT){
posp++;
  delay(100);
  b=1;

}
}

ML=millis();
if((posb==85 || ML-timer>=WIRES ) && (sumu<= sumd+threshold && sumu+threshold >= sumd)){
 //sleep mode
 //transistor
if(posb==85){
 if(b==1){
previous=millis();
}
}
ML=millis();
if( (ML-previous >= off) || (ML-timer>=WIRES)){
  digitalWrite(pint,0);
  Serial.println("IDEAL////////////////////////////////////////////////////////////////////////////////");
  Serial.println("IDEAL////////////////////////////////////////////////////////////////////////////////");
  Serial.println("IDEAL////////////////////////////////////////////////////////////////////////////////");
  Serial.println("IDEAL////////////////////////////////////////////////////////////////////////////////");
  Serial.println("IDEAL////////////////////////////////////////////////////////////////////////////////");
  Serial.println("IDEAL////////////////////////////////////////////////////////////////////////////////");
  Serial.println("IDEAL////////////////////////////////////////////////////////////////////////////////");
  Serial.println("IDEAL////////////////////////////////////////////////////////////////////////////////");
  Serial.println("IDEAL////////////////////////////////////////////////////////////////////////////////");
  Serial.println("IDEAL////////////////////////////////////////////////////////////////////////////////");
  ML=millis();
  if(ML-previous >= off){
  Serial.println("PREVIOUS");
  }
  else if(ML-timer >= WIRES){
  Serial.println("WIRES");
  }
 delay(10000);
  previous=millis();
  timer=millis();
  }
 


}
  
servob.write(posb);
servop.write(posp);


}
