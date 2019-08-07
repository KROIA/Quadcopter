// ask_receiver.pde
// -*- mode: C++ -*-
// Simple example of how to use RadioHead to receive messages
// with a simple ASK transmitter in a very simple way.
// Implements a simplex (one-way) receiver with an Rx-B1 module
// Tested on Arduino Mega, Duemilanova, Uno, Due, Teensy, ESP-12
#include "Timer.h"
#include "button.h"
#include <RH_ASK.h>
#ifdef RH_HAVE_HARDWARE_SPI
#include <SPI.h> // Not actually used but needed to compile
#endif

//RH_ASK driver;
// RH_ASK driver(2000, 4, 5, 0); // ESP8266 or ESP32: do not use pin 11 or 2
 RH_ASK driver(2000, 3, 4, 0); // ATTiny, RX on D3 (pin 2 on attiny85) TX on D4 (pin 3 on attiny85), 
Timer timer;
Timer sendTimer;
const char *msg = "from mega";
int SPEED = 100;
int poti1 = A0;
int poti2 = A1;
int poti3 = A2;

unsigned int p_RAWvalue = 0;
unsigned int i_RAWvalue = 0;
unsigned int d_RAWvalue = 0;

unsigned int last_p_RAWvalue = 0;
unsigned int last_i_RAWvalue = 0;
unsigned int last_d_RAWvalue = 0;

Button resetButton(5);
Button saveButton(6);
bool _saveToggle = false;
unsigned int logCounter = 0;
void sendReset();
void saveToggle();

void setup()
{
    Serial.begin(115200);	  // Debugging only
    if (!driver.init())
         Serial.println("init failed");
    pinMode(poti1,INPUT);
    pinMode(poti2,INPUT);
    pinMode(poti3,INPUT);
    resetButton.OnPressedEdge(sendReset);
    saveButton.OnPressedEdge(saveToggle);
}
bool pottiEnable = true;
bool messageToSend = false;
void loop()
{
  resetButton.update();
  saveButton.update();
  /*  uint8_t buf[RH_ASK_MAX_MESSAGE_LEN];
    uint8_t buflen = sizeof(buf);
if(!speektoggle){
  if(timeoutTimer.start(5000))
    {
      speektoggle = true;
    }
    if (driver.recv(buf, &buflen)) // Non-blocking
    {
      String message;
      for(unsigned int a=0; a<buflen; a++)
      {
        message += (char)buf[a];
      }
      speektoggle = true;
      Serial.println(message);
  // Message with a good checksum received, dump it.
  //driver.printBuffer("Got:", buf, buflen);
    }
    }
*/

if(pottiEnable && timer.start(100))
{
  p_RAWvalue = map(analogRead(poti1),0,1023,0,1000);
  i_RAWvalue = map(analogRead(poti2),0,1023,0,1000);
  d_RAWvalue = map(analogRead(poti3),0,1023,0,1000);



  
  const long delta = 3;
  //if(abs((signed long)p_RAWvalue - (signed long)last_p_RAWvalue) > delta || abs((signed long)i_RAWvalue - (signed long)last_i_RAWvalue) > delta || abs((signed long)d_RAWvalue - (signed long)last_d_RAWvalue) > delta)
  {
    messageToSend = true;
  }
  last_p_RAWvalue = p_RAWvalue;
  last_i_RAWvalue = i_RAWvalue;
  last_d_RAWvalue = d_RAWvalue;
}
if(messageToSend)
{
  if(sendTimer.start(200))
  {
    float divisor = 50;
    unsigned int p_value = pow(p_RAWvalue ,2)/divisor;
    unsigned int i_value = pow(i_RAWvalue ,2)/divisor;
    unsigned int d_value = pow(d_RAWvalue ,2)/divisor;
    String message = "p"+String(p_value)+"|"+String(checksum("p"+String(p_value)))+"]i"+String(i_value)+"|"+String(checksum("i"+String(i_value)))+"]d"+String(d_value)+"|"+String(checksum("d"+String(d_value)))+"]";

    Serial.println("----------------------");
    Serial.println("kp: "+String((float)p_value/100));
    Serial.println("ki: "+String((float)i_value/10000,4));
    Serial.println("kd: "+String((float)d_value/100));
    
    sendMessage(message);
    messageToSend = false;
  }
}
String message = "";
while(Serial.available())
{
  delay(5);
  message+=(char)Serial.read();
}
if(message.indexOf("poti") != -1)
{
  pottiEnable = !pottiEnable;
  Serial.println("potienable: "+String(pottiEnable));
}else if(message != "")
{
    driver.send((uint8_t *)message.c_str(), strlen(message.c_str()));
    driver.waitPacketSent();
}
}

int checksum(String text)
{
  int sum = 0;
  //Serial.println("chgecksum: \"");
  for(unsigned int a=0; a<text.length(); a++)
  {
    sum += text.charAt(a);
   // Serial.print(text.charAt(a));
    //s++;
  }
  //Serial.println("\" is: "+String(sum));
  return sum;
}

void sendReset()
{
  String message = "rstI|"+String(checksum("rstI"))+"]";
  sendMessage(message);
}
void saveToggle()
{
  _saveToggle = !_saveToggle;
  
  String message;
  if(_saveToggle)
  {
    message = "lon"+String(logCounter)+"|"+String(checksum("lon"+String(logCounter)))+"]";
  }
  else
  {
    logCounter++;
    message = "loff|"+String(checksum("loff"))+"]";
  }
  sendMessage(message);
}
void sendMessage(String message)
{
  Serial.println("sending message: "+message);
  driver.send((uint8_t *)message.c_str(), strlen(message.c_str()));
  driver.waitPacketSent();
}
