#include <Arduino.h>

#define R_EN 11
#define IN1 12
#define L_EN 9
#define IN2 8 
#define SW 12

int speed;
int pot;
// put function declarations here:
int myFunction(int, int);

void setup() {
  Serial.begin(9600);
  pinMode(R_EN, OUTPUT);
  pinMode(L_EN, OUTPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  //pinMode(SW, INPUT_PULLUP);
  analogWrite(L_EN,255);
  analogWrite(R_EN,0);

  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,HIGH);
  
}

void loop() {
  pot = analogRead(A0);
  
  speed = map(pot,0,1023,0,255);

  
  
  analogWrite(L_EN,speed);
  
  Serial.print(pot);
  Serial.print(" speed: ");
  Serial.print(speed);
  Serial.println();
}

