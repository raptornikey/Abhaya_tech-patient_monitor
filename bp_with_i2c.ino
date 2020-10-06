#include <LiquidCrystal_I2C.h>

//#include<LiquidCrystal.h>
#include<Wire.h>
//LiquidCrystal lcd(8,9,4,5,6,7);
LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 16, 2);
float pressureMin =-15, pressureMax=15, Vsupply=5;
int anpin =A0;
float volta=0;
int i ;
float maxvolt=0, volt=0, pressure =0, MAP=0, maxv =0;

void setup()
{
 // lcd.begin(16,2);
  lcd.init();
  lcd.backlight(); 
  pinMode(3,OUTPUT);
}
void loop()
{
  digitalWrite(3,HIGH);

  for(i=0;i<40;i++)
  {
    volta = analogRead(anpin);
    volt=(volta*Vsupply)/(pow(2,10)-1);
    maxv = max(abs(volt-2.5), maxvolt);
    maxvolt =abs(maxv-2.5);
    delay(250);
  }
  pressure= (((maxvolt)-.1*Vsupply)/((.8*Vsupply)/(pressureMax-pressureMin)))+pressureMin;
  MAP= -1*(14.7-pressure*-1)*51.7-3.16/maxvolt;
  digitalWrite(3,LOW);

  lcd.setCursor(0, 0);
  lcd.print(" MAP = ");
  lcd.print(MAP);
  lcd.setCursor(0,1);
  Serial.print(" ");
  Serial.print(MAP*1.1);
  lcd.print(" / ");
  lcd.print(MAP*0.8);

  while(1){}
  
}
