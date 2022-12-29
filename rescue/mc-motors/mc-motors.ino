#include <Wire.h>
#define SLAVE_ADDRESS 0x10

#include "DualMC33926MotorShield.h"

DualMC33926MotorShield md;  

int n_motore = 0;
int value= 0;

void loop() {}
void setup() 
{
  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveData);
  DualMC33926MotorShield();
  md.init();
}

void decodeString()
{
  if(n_motore == 0 || n_motore == 1)
  {  
    if(value > 127)
    {
      value = value - 256;
    }

    value *= 4;
  }
  if(n_motore == 0)
  {
    md.setM1Speed(value);
  }
  else
  {
    md.setM2Speed(value);
  }
  
}

void receiveData(int bytecount)
{
  for (int i = 0; Wire.available() > 0; i++)
  {
    if(i == 0){
      n_motore = Wire.read();
    }
    if(i == 1){
      value = Wire.read();
    }
  }
  decodeString();
}

