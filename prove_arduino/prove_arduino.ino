#include <Wire.h>
#define SLAVE_ADDRESS 0x10
byte data = 0;
void setup() 
{
  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveData);
}
void loop() { }
void decodeString(String s){
  int dato = 0;   // dato 0 mode //dato 1 linea // dato 2 angolo
  char charArr[15];
  s.toCharArray(charArr, 15); //strtok() lavora con gli array di stringhe
  
  char* token = strtok(charArr, ";");
  while(token != NULL){
    int n = atoi(token);
    if(dato == 0){inMode = n;}else if(dato == 1){linea = n;} else if(dato == 2){angolo = n;}
    token = strtok(NULL, ";");
    dato++;
  }
}
void receiveData(int bytecount)
{
  for (int i = 0; i < bytecount; i++) {
    data = Wire.read();
  }
  Serial.println(data);
}