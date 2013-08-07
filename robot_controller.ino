#include <Servo.h>
#include <Wire.h> 
#include "IOpins.h" 
#include "Constants.h" 
#include "Adafruit_BMP085.h"

//-------------------------------------------------------------- define global variables --------------------------------------------

unsigned int Volts;
unsigned int LeftAmps;
unsigned int RightAmps;
unsigned long chargeTimer;
unsigned long leftoverload;
unsigned long rightoverload;
int  highVolts;
int  startVolts;
int  Leftspeed=0;
int  Rightspeed=0;
byte Charged=1;                                               // 0=Flat battery  1=Charged battery
int  Leftmode=1;                                              // 0=reverse, 1=brake, 2=forward
int  Rightmode=1;                                             // 0=reverse, 1=brake, 2=forward
byte Leftmodechange=0;                                        // Left input must be 1500 before brake or reverse can occur
byte Rightmodechange=0;                                       // Right input must be 1500 before brake or reverse can occur
int  LeftPWM;                                                 // PWM value for left  motor speed / brake
int  RightPWM;                                                // PWM value for right motor speed / brake

//желаемая скорость
int  t_LeftPWM;                                                 // PWM value for left  motor speed 
int  t_RightPWM;                                                // PWM value for right motor speed 
long t_Time;

//делитель для определения скорости двигателей в зависимости от входного напряжения
int  scale    = 20;
long Time     = 0;                                            //счетчик времени для таймаута
int  TimeOut  = 1000;		                              //таймаут
String Command = String("");                                  //буфер комманд
int  mSpeed   = 90;					      //скорость двигателей


//-------------------------------------------------------------- BMP085 -------------------------------------------------------------
//барометр и др. вкусности
Adafruit_BMP085 bmp;   


//-------------------------------------------------------------- define servos ------------------------------------------------------
Servo Servo[7];                                               // define servos
int ServoPos[7];

  
int16_t GetTemperature(void)
{
  if (bmp.begin()) 
    return bmp.readTemperature();
  else //not found
    return 0;
}

int32_t GetPressure(void)
{
  if (bmp.begin()) 
    return bmp.readPressure();
  else //not found
    return 0;
}

void SerPrint(int32_t value)
{
  int v;
  for (int i=0;i<8;i++)
  {
     v = (value & 0xF0000000) >> 28;
     if (v > 9)
       Serial.write(0x41+v-10);
     else Serial.write(0x30+v);  
     value = value << 4;
  }
}

//послать ответ
void sendAns(String s, bool sendEnd = 1)
{
    Serial.print(s);
    if(sendEnd) Serial.write(13);
}

void setTimeOut(int t)
{
    Time    = millis();
    t_Time  = Time;
    TimeOut = t;
}

//Комманды для двигателей
void move_stop()
{
    setTimeOut(1000);  
    t_LeftPWM   = 0;
    t_RightPWM  = 0;
    LeftPWM     = 0;
    RightPWM    = 0;
}
void move_forward()
{
    setTimeOut(1000);
    //forward
    Leftmode  = 2;
    Rightmode = 2;
    t_LeftPWM   = mSpeed;
    t_RightPWM  = mSpeed;
}
void move_backward()
{
    setTimeOut(1000);
    //reverse
    Leftmode  = 0;
    Rightmode = 0;
    t_LeftPWM   = mSpeed;
    t_RightPWM  = mSpeed;
}
void move_left()
{
    setTimeOut(1000);
    Leftmode  = 0;
    Rightmode = 2;
    t_LeftPWM   = mSpeed;
    t_RightPWM  = mSpeed;
}
void move_right()
{
    setTimeOut(1000);
    Leftmode  = 2;
    Rightmode = 0;
    t_LeftPWM   = mSpeed;
    t_RightPWM  = mSpeed;
}
//комманды сервам
void cam_up()
{
    ServoPos[1] += 5;
    if(ServoPos[1] > 140) ServoPos[1] = 140;
}
void cam_down()
{
    ServoPos[1] -= 5;
    if(ServoPos[1] < 30) ServoPos[1] = 30;
}
void cam_left()
{
    ServoPos[0] += 5;
    if(ServoPos[0] > 180) ServoPos[0] = 180;
}
void cam_right()
{
    ServoPos[0] -= 5;
    if(ServoPos[0] < 0) ServoPos[0] = 0;
}


void SCmode()
{// ------------------------------------------------------------ Code for Serial Communications --------------------------------------
  int16_t Temperature;
  int32_t Pressure;
  int     data;


  if (millis() - t_Time > 10)
  {
//    LeftPWM  = t_LeftPWM;
//    RightPWM = t_RightPWM; 
     t_Time = millis();
     if (LeftPWM < t_LeftPWM )
         LeftPWM  = t_LeftPWM  * 2 * (t_Time - Time)/TimeOut; 
     if (RightPWM < t_RightPWM )
         RightPWM = t_RightPWM * 2 * (t_Time - Time)/TimeOut;
  }
  

  //не было комманд, по тормозам
  if (millis() - Time > TimeOut)
  {
    // стоп
    move_stop();   
  }
  
  //сервы в позиции
  for(int i=0;i<7;i++)
  {
    Servo[i].write(ServoPos[i]); 
  }

  //что-то пришло
  if (Serial.available() > 0)
  {
    data = Serial.read();
//    Serial.write(data);

    if (data != 13)
    {
      Command += char(data);                                         // пришел байт
    }
    if(Command.length() >= 10)                                       // переполнение, не было символа конца
    {
      Command = String("");
    }
    //конец комманды
    if(data == 13)
    {
//motors      
      if(Command.equals(String("MU"))) //forward
      {
        move_forward();
        sendAns(Command);
      }
      if(Command.equals(String("MD"))) //reverse
      {
        move_backward();
        sendAns(Command);
      }
      if(Command.equals(String("ML"))) //Left
      {
        move_left();
        sendAns(Command);
      }
      if(Command.equals(String("MR"))) //Right
      {
        move_right();
        sendAns(Command);
      }
//camera
      if(Command.equals(String("CU"))) //up
      {
        cam_up();
        sendAns(Command);
      }
      if(Command.equals(String("CD"))) //down
      {
        cam_down();
        sendAns(Command);
      }
      if(Command.equals(String("CL"))) //Left
      {
        cam_left();
        sendAns(Command);
      }
      if(Command.equals(String("CR"))) //Right
      {
        cam_right();
        sendAns(Command);
      }
//Temperature
      if(Command.equals(String("TG")))
      {
         Temperature = GetTemperature();
         sendAns(Command+String(Temperature));
      } 
//Pressure
      if(Command.equals(String("PG")))
      {
         Pressure = GetPressure();
         sendAns(Command+String(Pressure));
      } 
      Command = String("");                                                     //чистим буфер
    } // if(data == 13)                                                    
  } //  if (Serial.available() > 0 )
}

float getRealVolts(int Volts)
{
  return Volts / 1023.0 * 5.0 * 3.0;
}

void setup()
{
  //------------------------------------------------------------ Initialize Servos ----------------------------------------------------
  Servo[0].attach(S0);                                          // attach servo to I/O pin
  Servo[1].attach(S1);                                          // attach servo to I/O pin
  Servo[2].attach(S2);                                          // attach servo to I/O pin
  Servo[3].attach(S3);                                          // attach servo to I/O pin
  Servo[4].attach(S4);                                          // attach servo to I/O pin
  Servo[5].attach(S5);                                          // attach servo to I/O pin
  Servo[6].attach(S6);                                          // attach servo to I/O pin
  //------------------------------------------------------------ Set servos to default position ---------------------------------------

  Servo[0].writeMicroseconds(DServo0);                          // set servo to default position
  Servo[1].writeMicroseconds(DServo1);                          // set servo to default position
  Servo[2].writeMicroseconds(DServo2);                          // set servo to default position
  Servo[3].writeMicroseconds(DServo3);                          // set servo to default position
  Servo[4].writeMicroseconds(DServo4);                          // set servo to default position
  Servo[5].writeMicroseconds(DServo5);                          // set servo to default position
  Servo[6].writeMicroseconds(DServo6);                          // set servo to default position

  //------------------------------------------------------------ Initialize I/O pins --------------------------------------------------

  pinMode (Charger,OUTPUT);                                   // change Charger pin to output
  digitalWrite (Charger,1);                                   // disable current regulator to charge battery

  Serial.begin(Brate);                                      // enable serial communications if Cmode=1
  Serial.flush();                                           // flush buffer

  //сервы в позицию 90 градусов
  for(int i=0;i<7;i++)
  {
    ServoPos[i] = 90;
    Servo[i].write(ServoPos[i]);
  }

  //#todo написать применение
  //вычисляем scale в зависимости от входного напряжения
  Volts    = analogRead(Battery);                                  // read the battery voltage
  //пока scale ни к чему
  //#todo написать объяснение
  scale = 15 * (Volts / 1023.0 * 5.0 * 3.0) / 6.0 * (500.0 / 255);
  // считываем скорость для двигателей
  mSpeed = 3.0 / getRealVolts(Volts) * 255; //надо на двигателях X.X Вольт

}

void loop()
{
  //------------------------------------------------------------ Check battery voltage and current draw of motors ---------------------

  Volts    = analogRead(Battery);                             // read the battery voltage
  LeftAmps = analogRead(LmotorC);                             // read left motor current draw
  RightAmps= analogRead(RmotorC);                             // read right motor current draw

  if (LeftAmps>Leftmaxamps)                                   // is motor current draw exceeding safe limit
  {
    analogWrite (LmotorA,0);                                  // turn off motors
    analogWrite (LmotorB,0);                                  // turn off motors
    leftoverload=millis();                                    // record time of overload
  }

  if (RightAmps>Rightmaxamps)                                 // is motor current draw exceeding safe limit
  {
    analogWrite (RmotorA,0);                                  // turn off motors
    analogWrite (RmotorB,0);                                  // turn off motors
    rightoverload=millis();                                   // record time of overload
  }

/* убрано, иначе из-за старта двигателей напряжение проседает
  if ((Volts<lowvolt) && (Charged==1))                        // check condition of the battery
  {                                                           // change battery status from charged to flat

    //---------------------------------------------------------- FLAT BATTERY speed controller shuts down until battery is recharged ----
    //---------------------------------------------------------- This is a safety feature to prevent malfunction at low voltages!! ------

    Charged=0;                                                // battery is flat
    highVolts=Volts;                                          // record the voltage
    startVolts=Volts;
    chargeTimer=millis();                                     // record the time

    if(lipoBatt==0)                                           // checks if LiPo is being used, if not enable the charge cir$
    {
          digitalWrite (Charger,0);                           // enable current regulator to charge battery
    }
  }
*/
//------------------------------------------------------------ CHARGE BATTERY -------------------------------------------------------
  if ((Charged==0) && (Volts-startVolts>67) && (lipoBatt == 0)) // if battery is flat and charger has been connected (voltage has increased by at least 1V) and there i$
  {
    if (Volts>highVolts)                                      // has battery voltage increased?
    {
      highVolts=Volts;                                        // record the highest voltage. Used to detect peak charging.
      chargeTimer=millis();                                   // when voltage increases record the time
    }

    if (Volts>batvolt)                                        // battery voltage must be higher than this before peak charging can occur.
    {
      if ((highVolts-Volts)>5 || (millis()-chargeTimer)>chargetimeout) // has voltage begun to drop or levelled out?
      {
        Charged=1;                                            // battery voltage has peaked
        digitalWrite (Charger,1);                             // turn off current regulator
      }
    }
  }
  else
  {//----------------------------------------------------------- GOOD BATTERY speed controller opperates normally ----------------------

    SCmode();                                                 // Serial mode via D0(RX) and D1(TX)

    // --------------------------------------------------------- Code to drive dual "H" bridges --------------------------------------

    if (Charged==1)                                           // Only power motors if battery voltage is good
    {
      if ((millis()-leftoverload)>overloadtime)
      {
        switch (Leftmode)                                     // if left motor has not overloaded recently
        {
        case 2:                                               // left motor forward
          analogWrite(LmotorA,0);
          analogWrite(LmotorB,LeftPWM);
          break;

        case 1:                                               // left motor brake
          analogWrite(LmotorA,LeftPWM);
          analogWrite(LmotorB,LeftPWM);
          break;

        case 0:                                               // left motor reverse
          analogWrite(LmotorA,LeftPWM);
          analogWrite(LmotorB,0);
          break;
        }
      }
      if ((millis()-rightoverload)>overloadtime)
      {
        switch (Rightmode)                                    // if right motor has not overloaded recently
        {
        case 2:                                               // right motor forward
          analogWrite(RmotorA,0);
          analogWrite(RmotorB,RightPWM);
          break;

        case 1:                                               // right motor brake
          analogWrite(RmotorA,RightPWM);
          analogWrite(RmotorB,RightPWM);
          break;

        case 0:                                               // right motor reverse
          analogWrite(RmotorA,RightPWM);
          analogWrite(RmotorB,0);
          break;
        }
      }
    }
    else                                                      // Battery is flat
    {
      analogWrite (LmotorA,0);                                // turn off motors
      analogWrite (LmotorB,0);                                // turn off motors
      analogWrite (RmotorA,0);                                // turn off motors
      analogWrite (RmotorB,0);                                // turn off motors
    }
  }
}

