int av=A0;
float val=0;
float val1=0;
float Dia=0,Sys=0,pressureMax=15,pressureMin=-15,Vsupply=5,volta=0,maxvolt=0,volt=0,pressure=0,MAP=0,maxv=0;
int i;
void setup() {
  pinMode(3,OUTPUT);
  Serial.begin(9600);
  // put your setup code here, to run once:

}

void loop() {
  digitalWrite(3,HIGH);
  Serial.println("Valve locked");
  for(i=0;i<40;i++)
  {
  volta=analogRead(av);
  volt=(volta*5)/(1023.00);
  maxv=max(abs(volt-2.5),maxvolt);
  maxvolt=abs(maxv-2.5);
  delay(250);
  }
  pressure=(((maxvolt)-(0.1*Vsupply))/((0.8*Vsupply)/(pressureMax-pressureMin)))+pressureMin;
  //MAP=5;
  MAP=(((-1*(14.7-pressure*-1))*51.7)-(3.16/maxvolt));
  digitalWrite(3,LOW);
  MAP*=-1;
  Sys=MAP*1.1;
  Dia=MAP*0.8;
  Serial.print("MAP =");
  Serial.print(MAP);
    Serial.print("  Systolic =");
  Serial.print(Sys);
    Serial.print("  Diastolic =");
  Serial.println(Dia);
   Serial.println("Valve opened");
  // put your main code here, to run repeatedly:
while(1){}
}
