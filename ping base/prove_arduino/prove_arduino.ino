#include <Wire.h>
#define SLAVE_ADDRESS 0x10
byte data = 0;
void setup() 
{
  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveData);
}
void loop() { }
void receiveData(int bytecount)
{
  for (int i = 0; i < bytecount; i++) {
    data = Wire.read();
  }
  Serial.println(data);
}