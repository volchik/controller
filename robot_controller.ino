#include <EEPROM.h>
#include <Servo.h>
#include <Wire.h> 
#include "IOpins.h" 
#include "Constants.h" 
#include "Adafruit_BMP085.h"

//-------------------------------------------------------------- define global variables --------------------------------------------

uint16_t Volts;
uint16_t LeftAmps;
uint16_t RightAmps;
unsigned long chargeTimer;
unsigned long leftoverload;
unsigned long rightoverload;
uint16_t highVolts;
uint16_t startVolts;
byte Charged=1;                                               // 0=Flat battery  1=Charged battery
int  Leftmode=1;                                              // 0=reverse, 1=brake, 2=forward
int  Rightmode=1;                                             // 0=reverse, 1=brake, 2=forward
byte Leftmodechange=0;                                        // Left input must be 1500 before brake or reverse can occur
byte Rightmodechange=0;                                       // Right input must be 1500 before brake or reverse can occur
uint16_t LeftPWM;                                             // PWM value for left  motor speed / brake
uint16_t RightPWM;                                            // PWM value for right motor speed / brake

//желаемая скорость
uint16_t t_LeftPWM;                                           // PWM value for left  motor speed 
uint16_t t_RightPWM;                                          // PWM value for right motor speed 
unsigned long t_Time;

//делитель для определения скорости двигателей в зависимости от входного напряжения
unsigned long StartTime= 0;                                   //счетчик времени для таймаута
uint16_t TimeOut  = 1000;		                      //таймаут
String Command = String("");                                  //буфер комманд
double RealVolts;
uint16_t LowMotorSpeed;                                       //мин. значение для ШИМ двигателей 
uint16_t HighMotorSpeed;                                      //макс. значение для ШИМ двигателей

uint16_t lightOn = 0;                                         //состояние освещения

//-------------------------------------------------------------- BMP085 -------------------------------------------------------------
//барометр и др. вкусности
Adafruit_BMP085 bmp;   


//-------------------------------------------------------------- define servos ------------------------------------------------------
Servo Servo_Turn;                                             // серва поворота
Servo Servo_Tilt;                                             // серва наклона
uint16_t Pos_Turn;                                            // позиция сервы поворота
uint16_t Pos_Tilt;                                            // позиция сервы наклона

  
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

void setTimeOut(uint16_t timeOut)
{
    StartTime = millis();
    t_Time    = StartTime;
    TimeOut   = timeOut;
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
    t_LeftPWM  = HighMotorSpeed;
    t_RightPWM = HighMotorSpeed;
    //начальное значение чтобы двигатели не гудели
    LeftPWM   = LowMotorSpeed;
    RightPWM  = LowMotorSpeed;
}
void move_backward()
{
    setTimeOut(1000);
    //reverse
    Leftmode  = 0;
    Rightmode = 0;
    t_LeftPWM  = HighMotorSpeed;
    t_RightPWM = HighMotorSpeed;
    //начальное значение чтобы двигатели не гудели
    LeftPWM   = LowMotorSpeed;
    RightPWM  = LowMotorSpeed;
}
void move_left()
{
    setTimeOut(1000);
    Leftmode  = 0;
    Rightmode = 2;
    t_LeftPWM  = HighMotorSpeed;
    t_RightPWM = HighMotorSpeed;
    //начальное значение чтобы двигатели не гудели
    LeftPWM   = LowMotorSpeed;
    RightPWM  = LowMotorSpeed;
}
void move_right()
{
    setTimeOut(1000);
    Leftmode  = 2;
    Rightmode = 0;
    t_LeftPWM  = HighMotorSpeed;
    t_RightPWM = HighMotorSpeed;
    //начальное значение чтобы двигатели не гудели
    LeftPWM   = LowMotorSpeed;
    RightPWM  = LowMotorSpeed;
}
//комманды сервам
void cam_up()
{
    Pos_Tilt += Step_Pos_Tilt;
    if(Pos_Tilt > Max_Pos_Tilt) 
        Pos_Tilt = Max_Pos_Tilt;
    else 
       EEPROM.write(Adr_Tilt,Pos_Tilt);
}
void cam_down()
{
    Pos_Tilt -= Step_Pos_Tilt;
    if(Pos_Tilt < Min_Pos_Tilt) 
       Pos_Tilt = Min_Pos_Tilt;
    else 
       EEPROM.write(Adr_Tilt,Pos_Tilt);
}
void cam_left()
{
    Pos_Turn += Step_Pos_Turn;
    if(Pos_Turn > Max_Pos_Turn) 
       Pos_Turn = Max_Pos_Turn;
    else 
       EEPROM.write(Adr_Turn,Pos_Turn);
}
void cam_right()
{
    Pos_Turn -= Step_Pos_Turn;
    if(Pos_Turn < Min_Pos_Turn) 
       Pos_Turn = Min_Pos_Turn;
    else 
       EEPROM.write(Adr_Turn,Pos_Turn);
}

void LightOn(uint16_t On)
{
    if (Light_Invert)
      digitalWrite(Light,~On);                                  // установить значение освещения
    else digitalWrite(Light,On);                                // установить значение освещения
    if (lightOn != On)
    {   
       lightOn = On;
       EEPROM.write(Adr_Light,On);
    }
}

//Преобразование считанного напряжения с АЦП в вольты
double getRealVolts(uint16_t Volts)
{
  return Volts / 1023.0 * 5.0 * 3.0;
}

String FloatToString(double Value)
{
  String str = String("");
  uint16_t val = Value;
  str += String(val);
  str += ".";
  val = 100*(Value - int(Value));
  str += String(val);
  return str;
}

void SCmode()
{// ------------------------------------------------------------ Code for Serial Communications --------------------------------------
  int16_t Temperature;
  int32_t Pressure;
  uint16_t data;

  //наращиваем напряжение на двигателях с xxxPWM до t_xxx_PWM за 0.3с
  if (millis() - t_Time > 10)
  {
     t_Time = millis();
     if (LeftPWM < t_LeftPWM )
         LeftPWM  = LeftPWM  + (t_Time - StartTime)*(t_LeftPWM-LeftPWM)/300; 
     if (RightPWM < t_RightPWM )
         RightPWM = RightPWM + (t_Time - StartTime)*(t_RightPWM-RightPWM)/300;

     
     if (LeftPWM > t_LeftPWM )
         LeftPWM = t_LeftPWM;
     if (RightPWM > t_RightPWM )
         RightPWM = t_RightPWM;
  }
  

  //не было комманд, по тормозам
  if (millis() - StartTime > TimeOut)
  {
    // стоп
    move_stop();   
  }
  
  //сервы в позиции
  Servo_Turn.write(Pos_Turn); 
  Servo_Tilt.write(Pos_Tilt); 

  //что-то пришло
  if (Serial.available() > 0)
  {
    data = Serial.read();

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
//LightOn
      if(Command.equals(String("LO")))
      {
         LightOn(255);
         sendAns(Command);
      }
//LightOff
      if(Command.equals(String("LF")))
      {
         LightOn(0);
         sendAns(Command);
      }
//LightGet
      if(Command.equals(String("LG")))
      {
         if (lightOn)
            sendAns(Command+String("1"));
         else
            sendAns(Command+String("0"));
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
//Volts
      if(Command.equals(String("VG")))
      {
         RealVolts = getRealVolts(Volts);
         sendAns(Command+FloatToString(RealVolts));
      }
      Command = String("");                                                     //чистим буфер
    } // if(data == 13)                                                    
  } //  if (Serial.available() > 0 )
}

void setup()
{
  //------------------------------------------------------------ Initialize Servos ----------------------------------------------------
  Servo_Turn.attach(Pin_Turn);                                  // подключить серву поворота
  Servo_Tilt.attach(Pin_Tilt);                                  // подключить серву наклона


  //------------------------------------------------------------ Чтение значений из EEPROM --------------------------------------------
  Pos_Turn = EEPROM.read(Adr_Turn);
  Pos_Tilt = EEPROM.read(Adr_Tilt);
  lightOn  = EEPROM.read(Adr_Light);
  pinMode(Light, OUTPUT);
  if (Light_Invert)
    digitalWrite(Light,~lightOn);                               // установить значение освещения
  else digitalWrite(Light,lightOn);                             // установить значение освещения


  //------------------------------------------------------------ Initialize I/O pins --------------------------------------------------

  pinMode (Charger,OUTPUT);                                   // change Charger pin to output
  digitalWrite (Charger,1);                                   // disable current regulator to charge battery

  Serial.begin(Brate);                                      // enable serial communications if Cmode=1
  Serial.flush();                                           // flush buffer

  Volts    = analogRead(Battery);                                  // read the battery voltage
  // считываем скорость для двигателей
  LowMotorSpeed  = LowMotorVolts  / getRealVolts(Volts) * 255; //надо на двигателях LowMotorVolts вольт
  HighMotorSpeed = HighMotorVolts / getRealVolts(Volts) * 255; //надо на двигателях HighMotorVolts вольт
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

