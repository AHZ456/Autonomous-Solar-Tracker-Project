#include <Servo.h>
#include <avr/sleep.h>
#define SLEEP_TIMER 60
#define UPR_LMT 25
#define LWR_LMT 70
#define STP 90
#define RGT 80
#define LFT 105
int ULp=A0,DLp=A4,URp=A6,DRp=A2;
int UL=0,DL=0,UR=0,DR=0;
const int pinsb=6,pinsp=3,pint=2,INTR=13,threshold=20,off=1000,WIRES=10000,JITTER=25000;
int timer=0,ML=0,previous=0,jit=0,b=0,posb=0,posp=UPR_LMT,suml=0,sumr=0,sumd=0,sumu=0;
//servob STP stopping speed
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
  //IR SENSORS CURRENT DRAW!!!!
  Serial.begin(9600);
servob.attach(pinsb);
servop.attach(pinsp);
pinMode(INTR,INPUT_PULLUP);
pinMode(ULp,INPUT);
pinMode(DLp,INPUT);
pinMode(URp,INPUT);
pinMode(DRp,INPUT);
servop.write(UPR_LMT);
 
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
  posb=RGT;
  previous=millis();
//previous=millis();
  
}
else if(sumr+threshold < suml){
posb=LFT;
previous=millis();
//previous=millis();
}
else if(sumr <= suml+threshold && sumr+threshold >= suml){
posb=STP;
timer=millis();
}
if(sumu > sumd+threshold){
if(posp >UPR_LMT){
posp--;
  delay(100);
  previous=millis();
  timer=millis();
}
}
else if(sumu+threshold < sumd){
if(posp < LWR_LMT){
posp++;
  delay(100);
  previous=millis();
  timer=millis();

}
}

ML=millis();
if( (posb==STP) || (ML-timer>=WIRES) || (ML-jit>=JITTER) ){
 //sleep mode
 //transistor
ML=millis();
if( (ML-previous >= off) || (ML-timer>=WIRES) || (ML-jit>=JITTER) ){
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
  else if(ML-jit >= JITTER){
    Serial.println("JITTER");
  }
 delay(10000);
  jit=millis();
  previous=millis();
  timer=millis();
  }
 


}
  Serial.println(posb);
servob.write(posb);
servop.write(posp);


}
