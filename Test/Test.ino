/*
 * Author Name: P Sumantha Aithal
 * Functions: setup(), loop(), EM1_ON(),  EM2_ON(),  EM1_OFF(),  EM2_OFF(),
 *            motorForwardL(), motorForwardR(), motorBackwardL(), motorBackwardR(),
 *            motorStop()
 * GLobal Variables: Mag1,Mag2,InL1,InL2,PWML,PWMR,InR1,InR2,fcut,pi,compa,alpha,tau,la,acc,gyro,roll
 *                   al,temp,digital,Yaxis,Xaxis,i,PWM,a,elapsedtime,time,previoustime
 * 
 */
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

MPU6050 accelgyro;

#define Mag1 50                    //Electromagnet-1 Pin 
#define Mag2 51                    //Electromagnet-2 Pin
#define InL1 10                    //Left Motor Pin
#define PWML 13                    //Left Motor PWM Pin
#define InL2 9                     //Left Motor Pin
#define InR1 6                     //Right Motor Pin
#define PWMR 4                     //Right Motor PWM Pin  
#define InR2 7                     //Right Motor Pin  
#define fcut 5.0
#define pi 3.1415926
#define compa 0.03

double alpha,tau;
double la[5],acc,gyro,roll,a1[5],temp[5];
int digital,Yaxis,Xaxis;           //Digital,X-axis,Y-axis of Joystick
int i,PWM;
int16_t a[5];
double elapsedtime,time,previoustime;

void setup() {                     //Setup
 
 Serial.begin(115200);             //Terminal Serial
 Serial1.begin(9600);              //Xbee Serial
 pinMode(Mag1, OUTPUT);
 pinMode(Mag2, OUTPUT);
 pinMode(InL1, OUTPUT);
 pinMode(InL2, OUTPUT);
 pinMode(PWML, OUTPUT);   
 pinMode(InR1, OUTPUT);
 pinMode(InR2, OUTPUT);
 pinMode(PWMR, OUTPUT);
     Wire.begin();
    accelgyro.initialize();
}

void loop() {                      //Loop
  previoustime=time;
time=micros();
elapsedtime=(time-previoustime)/1000000;
if (Serial1.available()>=20)
{
  if (Serial1.read()== 0x7E)
  {
 for (i=0;i<11;i++){
  byte discard=Serial1.read();     //Discarding unwanted data
 }
 digital=Serial1.read();           //Storing required data from Xbee
 int analogMSB1=Serial1.read();
 int analogLSB1=Serial1.read();
 Yaxis=analogLSB1+(analogMSB1*256); //Combining the data from the Xbee
 int analogMSB2=Serial1.read();
 int analogLSB2=Serial1.read();
 Xaxis=analogLSB2+(analogMSB2*256); //Combining the data from the Xbee
 } 
}
tau=1/(2*pi*fcut);
alpha=tau/(tau+elapsedtime);
    for (i=3;i<=5;i++)
   {
    temp[i]=a1[i];
   }
accelgyro.getMotion6(&a[0], &a[1], &a[2], &a[3], &a[4], &a[5]);
   for(i=0;i<=2;i++)
   {
    a1[i]=a[i]/16384.0;
    la[i]=(1-alpha)*a1[i]+alpha*la[i];
   }
   for (i=3;i<=5;i++)
   {
    a1[i]=a[i]/131.0;
    la[i]=(1-alpha)*la[i]+(1-alpha)*(a1[i]-temp[i]);
    
   }
   acc=(180*atan(la[0]/abs(la[2])))/pi;
   roll=(1-compa)*(roll+la[4]*elapsedtime)+(compa*acc);
   Serial.println(roll);
//The Nested-if statements to control the Motors
if (Yaxis==1023 && Xaxis==1023)
{
  motorStop();
}
else if (Xaxis==1023 && (Yaxis<1023 && Yaxis>520))
{
  PWM=map(Yaxis,1023,520,0,255);
  motorForwardR(PWM);
  motorForwardL(PWM);
}
else if (Xaxis==1023 && (Yaxis<520 && Yaxis>=0))
{
  PWM=map(Yaxis,520,0,0,255);
  motorBackwardR(PWM);
  motorBackwardL(PWM);
}
else if (Yaxis==1023 && (Xaxis<1023 && Xaxis>520))
{
  PWM=map(Xaxis,1023,520,0,255);
  motorForwardL(PWM);
  motorBackwardR(PWM);
}
else if (Yaxis==1023 && (Xaxis<520 && Xaxis>=0))
{
  PWM=map(Xaxis,520,0,0,255);
  motorForwardR(PWM);
  motorBackwardL(PWM);
}
else
{
  motorStop();
}
//The Nested-if statements to ON and OFF the electromagnets
if (digital==8)
{
  EM1_ON();
  EM2_OFF();
}
else if (digital==16)
{
  EM2_ON();
  EM1_OFF();
}
else if (digital==0)
{
  EM1_ON();
  EM2_ON();
}
else if (digital==24)
{
  EM1_OFF();
  EM2_OFF();
}
}
//Function for right Motor forward 
void motorForwardR(int PWM_val)
{
    analogWrite(PWMR, PWM_val);
    digitalWrite(InR2, LOW);
    digitalWrite(InR1, HIGH);
}
//Function for right Motor backward
void motorBackwardR(int PWM_val)
{
    analogWrite(PWMR, PWM_val);
    digitalWrite(InR1, LOW);
    digitalWrite(InR2, HIGH);
}
//Function for left Motor forward 
void motorForwardL(int PWM_val)
{
    analogWrite(PWML, PWM_val);
    digitalWrite(InL1, LOW);
    digitalWrite(InL2, HIGH);
}
//Function for left Motor backward 
void motorBackwardL(int PWM_val)
{
    analogWrite(PWML, PWM_val);
    digitalWrite(InL2, LOW);
    digitalWrite(InL1, HIGH);
}
//Function to stop the motors
void motorStop(void)
{
    digitalWrite(InL1, HIGH);
    digitalWrite(InL2, HIGH);
    digitalWrite(InR1, HIGH);
    digitalWrite(InR2, HIGH);
}
//Function to ON Electromagnet-1
void EM1_ON(void)
{
  digitalWrite(Mag1, HIGH);
}
//Function to OFF Electromagnet-1
void EM1_OFF(void)
{
  digitalWrite(Mag1, LOW);
}
//Function to ON Electromagnet-2
void EM2_ON(void)
{
  digitalWrite(Mag2, HIGH);
}
//Function to OFF Electromagnet-2
void EM2_OFF(void)
{
  digitalWrite(Mag2, LOW);
}
