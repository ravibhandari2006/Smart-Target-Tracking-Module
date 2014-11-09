#include<Servo.h>
Servo myservo1,myservo2;
int val = 0;   
char response[20];
char x[5],y[5];
int flag=0;
int dataCounter = 0;
void setup() {
  Serial.begin(115200);
  myservo1.attach(9);
  myservo2.attach(10);
  //pinMode(13, OUTPUT);
}

void loop () {

  if (Serial.available() ) {
    val=Serial.read();
    if(val==':')
    {
      flag=2;
      y[dataCounter]='\0';
    }
    if(val=='-')
    {
      flag=1;
      x[dataCounter]='\0';
      dataCounter=0;
    }
    if(flag==0)
    {
      x[dataCounter]=val;
      dataCounter++; 
    }
    if((flag==1) && (val!='-'))
    {
      y[dataCounter]=val;
      dataCounter++; 
    }   
    if(flag==2) 
    {
        int valx=atoi(x);
        int valy=atoi(y);
        //int x1=map(valx,0,640,63,109)-1;
        //int y1=map(valy,0,480,92,55)+2;
        int x1=map(valx,0,640,72,121)-4;
        int y1=map(valy,0,480,99,61)-3;
        myservo1.write(x1);
        myservo2.write(y1);
        /*Serial.print(x1);
        Serial.print(':');
        Serial.print(y1);
        Serial.print('-');
        Serial.print(valx);
        Serial.print(':');
        Serial.print(valy);
        Serial.println();
        */
        flag=0;
        dataCounter=0;
    }
  }
}

float mapfloat(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}
