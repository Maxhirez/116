//  Prior Art:
//    https://playground.arduino.cc/Code/CapacitiveSensor

// * This program is free software; you can redistribute it and/or modify
// * it under the terms of the GNU General Public License as published by
// * the Free Software Foundation; either version 3 of the License, or
// * (at your option) any later version.



//  0=record, 1=play
#include <Wire.h>
#include "pins_arduino.h" // Arduino pre-1.0 needs this



#define gatePin A5
#define tempoPin A4
#define recordPin 0
#define playStopPin 1
static const uint8_t output_pins[]={A3,A2,A1,A0,13,10,11,12};
static const uint8_t gate_pin=A5;

int detect, lastRead, minCap, maxCap, tempo, minTempo, maxTempo, tempoDelay;
bool play=false;
void setup(){
  //note keys
  for (int thisPin = 2; thisPin<=9;thisPin++){
    pinMode(thisPin,INPUT);
  }
  //v/oct out
  for (int i=0;i<8;i++){
    pinMode(output_pins[i],OUTPUT);
   }
  pinMode(gate_pin,OUTPUT);
  lastRead = 0;
  minCap = 3;
  maxCap = 12;
  minTempo = 30;
  maxTempo = 300;
  digitalWrite(gatePin, LOW);
  pinMode(playStopPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(playStopPin), togglePlay, FALLING);
}

void loop(){
  //kill last note if released
  detect = readCapacitivePin(lastRead);
  if (detect<minCap) {
    digitalWrite(gatePin, LOW);//turn off if nothing is happening
    digitalWrite(output_pins[lastRead], LOW);//turn off last pin.  This probably isn't right.
   }

  //main controller play loop
  for (int key = 9; key >=2; key--){//mono with high-note priority
    detect = readCapacitivePin (key);
    if (detect >= minCap){
      digitalWrite(output_pins[key-2], HIGH);//Again probably not right .  Yet.
      digitalWrite(gatePin, HIGH);
      lastRead = key;
      break;
    }
    digitalWrite(output_pins[key-2],LOW);//?Right?
  }

  //sequence loop
  while(play){
    for(int playNote=0;playNote<8;playNote++){
      tempo = map(analogRead(tempoPin),0,1024,minTempo,maxTempo);
      tempoDelay = 30000/tempo;//ms for 50% duty cycle
      digitalWrite(output_pins[playNote], HIGH);
      digitalWrite(gatePin, HIGH);
      delay(tempoDelay);
      digitalWrite(gatePin, LOW);
      digitalWrite(output_pins[playNote], LOW);
      delay(tempoDelay);
    }
}
}


uint8_t readCapacitivePin(int pinToMeasure) {
  // Variables used to translate from Arduino to AVR pin naming
  volatile uint8_t* port;
  volatile uint8_t* ddr;
  volatile uint8_t* pin;
  // Here we translate the input pin number from
  //  Arduino pin number to the AVR PORT, PIN, DDR,
  //  and which bit of those registers we care about.
  byte bitmask;
  port = portOutputRegister(digitalPinToPort(pinToMeasure));
  ddr = portModeRegister(digitalPinToPort(pinToMeasure));
  bitmask = digitalPinToBitMask(pinToMeasure);
  pin = portInputRegister(digitalPinToPort(pinToMeasure));
  // Discharge the pin first by setting it low and output
  *port &= ~(bitmask);
  *ddr  |= bitmask;
  delay(1);
  uint8_t SREG_old = SREG; //back up the AVR Status Register
  // Prevent the timer IRQ from disturbing our measurement
  noInterrupts();
  // Make the pin an input with the internal pull-up on
  *ddr &= ~(bitmask);
  *port |= bitmask;

  // Now see how long the pin to get pulled up. This manual unrolling of the loop
  // decreases the number of hardware cycles between each read of the pin,
  // thus increasing sensitivity.
  uint8_t cycles = 17;
  if (*pin & bitmask) { cycles =  0;}
  else if (*pin & bitmask) { cycles =  1;}
  else if (*pin & bitmask) { cycles =  2;}
  else if (*pin & bitmask) { cycles =  3;}
  else if (*pin & bitmask) { cycles =  4;}
  else if (*pin & bitmask) { cycles =  5;}
  else if (*pin & bitmask) { cycles =  6;}
  else if (*pin & bitmask) { cycles =  7;}
  else if (*pin & bitmask) { cycles =  8;}
  else if (*pin & bitmask) { cycles =  9;}
  else if (*pin & bitmask) { cycles = 10;}
  else if (*pin & bitmask) { cycles = 11;}
  else if (*pin & bitmask) { cycles = 12;}
  else if (*pin & bitmask) { cycles = 13;}
  else if (*pin & bitmask) { cycles = 14;}
  else if (*pin & bitmask) { cycles = 15;}
  else if (*pin & bitmask) { cycles = 16;}

  // End of timing-critical section; turn interrupts back on if they were on before, or leave them off if they were off before
  SREG = SREG_old;

  // Discharge the pin again by setting it low and output
  //  It's important to leave the pins low if you want to 
  //  be able to touch more than 1 sensor at a time - if
  //  the sensor is left pulled high, when you touch
  //  two sensors, your body will transfer the charge between
  //  sensors.
  *port &= ~(bitmask);
  *ddr  |= bitmask;

  return cycles;
}

int togglePlay(){
  //do stuff
  play=!play; 
}
