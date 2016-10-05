#include<Servo.h>

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;

float L1=11.8 , L12=139.24 , L14=19387.77;
float L2=9.8 , L22=96.04 , L24=9223.68;
float L3=0;

float x,y,z;
double q0,q1,q2,q3,q4;
double h,h2,h4;
double z2,z4;

double th;
int r;

double X0 , X1;//vertical limits of writing
double Y0 , Y1 ; //orizhontal limits 
double s; //motion step
double d1 , d2; //delay small and long
double zH , zC , xC; //keep up 
//------------------------------------------------------------------------------------------

void IKFW() // this function computes the inverse kinematics of the robot and applies the complience correction algorithms 
{
   h=sqrt(pow(x,2)+pow(y,2));
   zC = z + h/10 - 2 - 0.017*y; // Compliance compensation of the arm's base as it move away from the center 
   if( y > 0 )  {  xC = x - 0.082*y;  } //{  xC = x - 0.0074*pow(y,2);  } 
   else  {  xC=x;  } 
   
   if ( h=sqrt(pow(xC,2)+pow(y,2)) < 20 )
   {
     h=sqrt(pow(xC,2)+pow(y,2)); 
   }
   else h = 19.5;
   
   
   h2=pow(h,2);
   h4=pow(h2,2);
   z2=pow(zC,2);
   z4=pow(z2,2);
   q1=(180*atan2(y,xC))/3.14;
   q2=(180*2*(atan2(2*L1*zC+sqrt(-h4+2*h2*L12+2*h2*L22-2*h2*z2-L14+2*L12*L22+2*L12*z2-L24+2*L22*z2-z4),h2+2*h*L1+L12-L22+z2)))/3.14;
   q3=(180*2*(atan2(-sqrt(-h4+2*h2*L12+2*h2*L22-2*h2*z2-L14+2*L12*L22+2*L12*z2-L24+2*L22*z2-z4),h2-L12+2*L1*L2-L22+z2)))/3.14;   
   q4 = - (q2 + q3) + 97;
   
   servo1.write(90+q1);
   servo2.write(157-q2);
   servo3.write(177+q3);
   servo4.write(195-q4);
   servo5.write(q4);  
}

//------------------------------------------------------------------------------------------
void setup() 
{ 
  servo1.attach(3);  // attaches the servo on pin 9 to the servo object 
  servo2.attach(5);
  servo3.attach(6);
  servo4.attach(9);
  servo5.attach(10);

  s=0.1;
  X0=9;
  X1=15;
  Y0=-12;
  Y1=-8;
  zH=1.5;
  
  th=-90;
  r=2;
  
  d1=20;
  d2=500;
}

//--------------------------------------------------------------------

void loop()
{
  x=13; y=0; z=8;
  IKFW();
  delay(1000);
  
 
  for(y = 0 ; y > Y0 ; y-=s)   { IKFW();  delay(d1); } 

  z=6;  IKFW();  delay(80);
 
  for(x = 13 ; x < X1 ; x+=s)  { IKFW();  delay(d1); } 
  
  delay(1000);
  
  z=5.5; IKFW();  delay(1000);
  
 
  
  x=X1; y=Y0;  
  IKFW();   delay(d2);
  
  z=2.8;//<<<-------------------<<<-----------Z POSITION!!!
  IKFW();   delay(d2);
  
//---------------------------------------------H------------------(X0,Y0)----start
  
  for (x = X1 ; x > X0 ; x-=s) 
  {
    IKFW();   delay(d1); 
  }
    
  delay(d2);
  z = z + zH;//___
  IKFW();   delay(d2);
  
   for (x = X0 ; x < (X0 + X1)/2 ; x+=s) //---
  {
    y = Y0 + 0.3;
    IKFW();   delay(d1); 
  }
  
  z = z - zH;//___
  IKFW();   delay(d2);
  
  for (y = Y0 + 0.3 ; y < Y1 ; y+=s) //---
  {
    IKFW();  delay(d1); 
  }
  
  delay(d2);
  z = z + zH;
  IKFW();   delay(d2);
  
  for (x = (X0 + X1)/2 ; x > X0 ; x-=s) //---
  {
    IKFW();   delay(d1); 
  }
  
  
  z = z - zH;
  IKFW();   delay(d2);

  for (x = X0 ; x < X1 ; x+=s) 
  {
    IKFW();   delay(d1); 
  }
  
  delay(d2);
//--------------------------------------------H-------------------(X1,Y1)---------finish

z = z + zH;
IKFW();   delay(d2);

 for (y = Y1 ; y < Y1 + 1 + 4 ; y+=s) //---
{
  IKFW();   delay(d1); 
}

z = z - zH;
IKFW();   delay(d2);

Y0=-7; 
Y1=-3;

 //----------------------------------------E------------------(X1,Y1)----start
  for(y = Y1 ; y > Y0 ; y-=s)
    {
      IKFW();  delay(d1);
    } 
   
  for(x = X1 ; x > X0 ; x-=s)
    {
      IKFW();  delay(d1);
    }
 
  for(y = Y0 ; y < Y1 ; y+=s)
    {
      IKFW();  delay(d1);
    }
  
  delay(d2);
  z = z + 1.5;
  IKFW();   delay(d2);
  
  for(y = Y1 ; y > Y0 ; y-=s)
  { 
    x += ((X1-X0)/(2*(Y1-Y0)))*s;
    IKFW();  delay(d1);
  }
  
  delay(d2);
  z = z - 1.5;
  IKFW();   delay(d2);
 
  for(y = Y0 ; y < Y1 ; y+=s)
    {
      IKFW();  delay(d1);
    }
  delay(d2);
 //----------------------------------------E----------finish-----------------------


z = z + zH;
IKFW();   delay(d2);

 for (y = Y1 ; y < Y1 + 1 ; y+=s) //---
{
  x+=(X1-X0)*s/2;
  IKFW();   delay(d1); 
}

z = z - zH;
IKFW();   delay(d2);

Y0=-2; 
Y1=2;

//---------------------------------L1------------X1,Y0--------------------start-------------

 for(x = X1 ; x > X0 ; x-=s)
  {
    IKFW();  delay(d1);
  }
  
  for(y = Y0 ; y < Y1 ; y+=s)
  {
    IKFW();  delay(d1);
  }
  delay(d2);

//-----------------------------------L1------------X0,Y1-----------------finish----------

z = z + zH;
IKFW();   delay(d2);

 for (y = Y1 ; y < Y1 + 1 ; y+=s) //---
{
   IKFW();   delay(d1); 
}
 for (x = X0 ; x < X1 ; x+=s) //---
{
   IKFW();   delay(d1); 
}

z = z - zH;
IKFW();   delay(d2);

Y0=3; 
Y1=7;

//---------------------------------L2------------X1,Y0--------------------start-------------

for(x = X1 ; x > X0 ; x-=s)
  {
    IKFW();  delay(d1);
  }
  
  for(y = Y0 ; y < Y1 ; y+=s)
  {
    IKFW();  delay(d1);
  }
delay(d2);
//-----------------------------------L2------------X0,Y1-----------------finish----------

z = z + zH; 
IKFW();   delay(d2);

for (y = Y1 ; y < Y1 + 1 ; y+=s) 
{
   IKFW();   delay(d1);
}
for (x = X0 ; x < X0 + 1.8 ; x+=s) 
{
   IKFW();   delay(d1); 
}
z = z - zH;
IKFW();   delay(d2);

Y0=8; 
Y1=12;

//-----------------------------------O------------X0,(Y1+Y0)/2-----------------start----------


for( x = X0 + 1.8 ; x < X0 + 4 ; x+=s )
{
  IKFW();  delay(d1);
}
for( th = -90 ; th < 90 ; th+=2 )
{
  x = r*cos(th*3.14/180)+X0+4;
  y = r*sin(th*3.14/180)+Y0+2;
  IKFW();  delay(d1);
}

for( x = X0 + 4 ; x > X0 + 2 ; x-=s )
{
  IKFW();  delay(d1);
}
for( th = 90 ; th < 270 ; th+=2 )
{
  x = r*cos(th*3.14/180)+X0+2;
  y = r*sin(th*3.14/180)+Y0+2;
  IKFW();  delay(d1);
}
delay(d2);
//-----------------------------------O------------(X0+2,Y0)-----------------finish----------
z = 6;
IKFW();   delay(d2);

x=13;
y=0;
IKFW();  delay(d2);

Y0=-12;
Y1=-8;
delay(7000);


//--------------------------------GOOD BYE-----------------------SEE YOU------------------------

}


