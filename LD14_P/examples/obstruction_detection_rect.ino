#include "LD14_P.h"

#define RXD2 47
#define TXD2 48

// #define TRIGGER_PIN 12
#define TRIGGER_PIN LED_BUILTIN

LD14_P lidar;

void setup() {
  pinMode(TRIGGER_PIN, OUTPUT);
  digitalWrite(TRIGGER_PIN, HIGH);

  Serial.begin(115200);
  Serial2.begin(230400, SERIAL_8N1, RXD2, TXD2);

}

unsigned long triggerTime = 0;

bool triggered = false;
void loop() {
  
  if(lidar.update()){
    if(lidar.checkCollisionRect(550, 500, 2)){
      triggerTime = millis();
      triggered = true;
    }else{
      if(millis() - triggerTime > 400){
        triggered = false;
      }else{
        triggered = true;
      }
    }
  }

  digitalWrite(TRIGGER_PIN, !triggered);

}