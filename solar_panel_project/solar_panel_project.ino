#include <Servo.h>
#include <avr/sleep.h>
#define SLEEP_TIMER 60
#define UPR_LMT 27
#define LWR_LMT 60
int ULp=A6,DLp=A4,URp=A2,DRp=A0;
int UL=0,DL=0,UR=0,DR=0;
int pinsb=6,pinsp=3,pint=7,posb=0,posp=LWR_LMT,suml=0,sumr=0,sumd=0,sumu=0,INTR=13,threshold=16,a=0,b=0,previous=0,off=2500;
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
}
else if(sumr+threshold < suml){
posb=82;
b=1;
}
else if(sumr <= suml+threshold && sumr+threshold >= suml){
posb=85;

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


else if(posb==85 && (sumu<= sumd+threshold && sumu+threshold >= sumd)){
 //sleep mode
 //transistor
 if(b==1){
previous=millis();
}
if(a==0){ 
previous=millis();
}
a++;
if( millis()-previous == off){
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
  delay(30000);
  a=0;
 
  }
 


}
  
servob.write(posb);
servop.write(posp);
 Serial.println(posp);

}
