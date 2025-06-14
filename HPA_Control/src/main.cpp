#include <IcsHardSerialClass.h>

const byte EN_PIN = 2;
const long BAUDRATE = 115200;
const int TIMEOUT = 10;		//通信できてないか確認用にわざと遅めに設定


int off0;
int off1;
float a0 = 0.0; // rudder
float a1 = 0.0; // elevater
float rudMax = 833.3;
float eleMax = 555.6;
float trim_min = 13.9;
int b0 = 0;
int b1 = 0;

int trim;
int trimMax = 10;
long previous;
long interval = 1000;


IcsHardSerialClass krs(&Serial1,EN_PIN,BAUDRATE,TIMEOUT);  //インスタンス＋ENピン(2番ピン)およびUARTの指定


void setup() {
  // put your setup code here, to run once:
  //Serial.begin(115200);
  //delay(1000);

  krs.begin();  //サーボモータの通信初期設定

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(6, INPUT_PULLUP);
  pinMode(7, INPUT_PULLUP);

  off0 = analogRead(A0);
  off1 = analogRead(A1);

  trim = 0;
  previous = millis();
}


void loop() {
  // put your main code here, to run repeatedly:
  if(digitalRead(6) == LOW && millis() - previous > interval){
    trim += 1;
    previous = millis();
  }
  if(digitalRead(7) == LOW && millis() - previous > interval){
    trim -= 1;
    previous = millis();
  }

  if(trim > trimMax) trim = trimMax;
  if(trim < -trimMax) trim = -trimMax;

  a0 = 0.15*(float(analogRead(A0) - off0)/512.0) + 0.85*a0;
  a1 = 0.15*(float(analogRead(A1) - off1)/512.0) + 0.85*a1;

/*
  a0 = float(analogRead(A0) - 512)/512.0;
  a1 = float(analogRead(A1) - 512)/512.0;
*/

  if(-1 < a0 && a0 < -0.02){
    a0 = a0;
  }else if(-0.02 < a0 && a0 < 0.02){
    a0 = 0.0;
  }else if(0.02 < a0 && a0 < 1.0){
    a0 = a0;
  }
  
  if(a0 < -1){
    a0 = -1.0;
  }else if(1.0 < a0){
    a0 = 1.0;
  }


  if(-1.0 < a1 && a1 < -0.02){
    a1 = a1;
  }else if(-0.02 < a1 && a1 < 0.02){
    a1 = 0.0;
  }else if(0.02 < a1 && a1 < 1.0){
    a1 = a1;
  }
  
  if(a1 < -1.0){
    a1 = -1.0;
  }else if(1.0 < a1){
    a1 = 1.0;
  }

/*
  Serial.print(a0);
  Serial.print("\t");
  Serial.println(a1);
*/

  b0 = (int)(7500 + 1381.3 + rudMax*a0);
  b1 = (int)(7500 + 1547.3 - eleMax*a1 + trim_min*trim);

/*
  Serial.print(b0);
  Serial.print("\t");
  Serial.println(b1);
*/



  krs.setPos(0,b0);    
  krs.setPos(1,b1);

  //delay(50); 

  
}
