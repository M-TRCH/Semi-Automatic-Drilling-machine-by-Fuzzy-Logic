
////////////////////////////////////////////////////////////////////////////////
/* current sensor */
#include <PZEM004Tv30.h>
#define rxPin 10 // tx
#define txPin 11 // rx
#define timeAlign 3000
float amp, ampOffset; 
PZEM004Tv30 sense(rxPin, txPin);

float getCurrent()
{
  float current = sense.current();
  if(current == NAN)
  {
    current = 0;
  }
  return current;
}
float getCurrentOffset()
{
  return getCurrent() - ampOffset; 
}
void alignCurrent()
{
  unsigned long time = millis();
  while(millis()-time <= timeAlign)
  {
    ampOffset = getCurrent();  
  }
}

////////////////////////////////////////////////////////////////////////////////
/* loadcell */
#include "HX711.h"
#define outPin_1 A1  
#define clkPin_1 A0 
#define outPin_2 A3  
#define clkPin_2 A2 
#define wOffset_1 9045912
#define wOffset_2 8429178
#define refWeight 7.13
#define wAlignTime 500
const float wfactor_1 = 115281 / refWeight;
const float wfactor_2 =  89189 / refWeight;
float kilo, kiloOffset = 0;
HX711 load_2(outPin_2, clkPin_2);
HX711 load_1(outPin_1, clkPin_1);

void loadcellInit()
{
  load_1.set_scale(wfactor_1); 
  load_1.set_offset(wOffset_1);
  load_2.set_scale(wfactor_2); 
  load_2.set_offset(wOffset_2);
}
float getWeight_1()
{
  float w = load_1.get_units();
  return w;
}
float getWeight_2()
{
  float w = load_2.get_units();
  return w;
}
float getWeight()
{
  return (getWeight_1() + getWeight_2()) / 2.0;
}
float getWeightOffset()
{
  float w = getWeight() - kiloOffset;
  return w;
}
void weightAlign()
{
  delay(wAlignTime);
  kiloOffset = getWeight();
}

////////////////////////////////////////////////////////////////////////////////
/* single phase motor */
#define minFreq 0
#define maxFreq 60
#define minPwm  0
#define maxPwm  128
#define minSpd  0
#define maxSpd  2820
#define pwmPin  3
#define spdFactor 1.073 
#define limitPwm 112
uint16_t pwmVal;
uint16_t rpmCal, freqSet; 

void setSpd(uint16_t freqIn)
{
  freqSet = constrain(freqIn * spdFactor, minFreq, maxFreq);
  pwmVal = map(freqSet, minFreq, maxFreq, minPwm, maxPwm);
  if(pwmVal > limitPwm) pwmVal = limitPwm;
  
  rpmCal = map(pwmVal, minPwm, maxPwm, minSpd, maxSpd); 
  analogWrite(pwmPin, pwmVal);
}

////////////////////////////////////////////////////////////////////////////////
/* switch */
#define startPin 52
#define debounceTime 500
#define indicatorTime 3000
void switchInit()
{
  pinMode(startPin, INPUT_PULLUP);
}
void(*resetFunc) (void) = 0;

////////////////////////////////////////////////////////////////////////////////
/* procress */
#include "Fuzzy.h"
#define gPilotLampPin 8
#define rPilotLampPin 9 
#define delayStart    3000
#define initSpdStart  55
#define spdStart      28
#define detectTimeFuzzy 2000
#define detectTimeStop  3000
#define startTimeFuzzy  2000
#define detectHighWeight 4.0 // for start
#define detectLowWeight  1.5 // for stop
boolean highSpdCase = false; 
#define minWeightPilotLamp -7 // for green lamp
#define maxWeightPilotLamp 7  // 

void pilotLampInit()
{
  pinMode(gPilotLampPin, OUTPUT);
  pinMode(rPilotLampPin, OUTPUT);  
}
void readyIndicator()
{
  digitalWrite(gPilotLampPin, HIGH);
  digitalWrite(rPilotLampPin,  LOW);
}
void unreadyIndicator()
{
  digitalWrite(rPilotLampPin, HIGH);
  digitalWrite(gPilotLampPin,  LOW);  
}
////////////////////////////////////////////////////////////////////////////////
/* step motor */
#define pwm1 4
#define pwm2 5
#define pwm3 6
#define pwm4 7
#define ena1 40
#define ena2 42
#define ena3 44
#define ena4 46
#define stepMotorPwm 255
#define stepTopSpdPeriod 30 // เพื่มเเล้วช้าลงท๊อกมากขึ้น
#define stepBotSpdPeriod 30
#define DIR_CW 1
#define DIR_CCW 0
#define topLimitSwPin 53
#define botLimitSwPin 51
#define dbTime 5
boolean table[4][4] =
{
  {0, 1, 1, 0},
  {0, 1, 0, 1},
  {1, 0, 0, 1},
  {1, 0, 1, 0}
};
void stepMotorInit()
{
  pinMode(ena1, OUTPUT);
  pinMode(ena2, OUTPUT);
  pinMode(ena3, OUTPUT);
  pinMode(ena4, OUTPUT);
  pinMode(pwm1, OUTPUT);
  pinMode(pwm2, OUTPUT);
  pinMode(pwm3, OUTPUT);
  pinMode(pwm4, OUTPUT);
  pinMode(topLimitSwPin, INPUT_PULLUP);
  pinMode(botLimitSwPin, INPUT_PULLUP);
}
void stepMotorStop()
{
  digitalWrite(ena1, LOW);
  digitalWrite(ena2, LOW);
  digitalWrite(ena3, LOW);
  digitalWrite(ena4, LOW);
}
void stepMotorStart()
{
  digitalWrite(ena1, HIGH);
  digitalWrite(ena2, HIGH);
  digitalWrite(ena3, HIGH);
  digitalWrite(ena4, HIGH);
}
void stepMotorDriveCCW(int period)
{
  for(int i=0; i<4; i++)
  {
    analogWrite(pwm1, table[i][0] * stepMotorPwm);
    analogWrite(pwm2, table[i][1] * stepMotorPwm);
    analogWrite(pwm3, table[i][2] * stepMotorPwm);
    analogWrite(pwm4, table[i][3] * stepMotorPwm);
    delay(period);
  }
}
void stepMotorDriveCW(int period)
{
  for(int i=3; i>=0; i--)
  {
    analogWrite(pwm1, table[i][0] * stepMotorPwm);
    analogWrite(pwm2, table[i][1] * stepMotorPwm);
    analogWrite(pwm3, table[i][2] * stepMotorPwm);
    analogWrite(pwm4, table[i][3] * stepMotorPwm);
    delay(period);
  }
}
void goToTop()
{
  stepMotorStart();
  while(true)
  {
    if(!digitalRead(topLimitSwPin))  break;
    delay(dbTime);
    {  
      if(!digitalRead(topLimitSwPin))  
      { 
        break;
      }
    }
    stepMotorDriveCW(stepTopSpdPeriod); 
  }
  stepMotorStop();
  Serial.println("\nTop Finished");
}
void goToBottom()
{
  stepMotorStart();
  while(true)
  {
    if(!digitalRead(botLimitSwPin))  
    { 
      delay(dbTime);
      if(!digitalRead(botLimitSwPin))  
      { 
        break;
      }
    }
    stepMotorDriveCCW(stepBotSpdPeriod); 
  }
  stepMotorStop();
  Serial.println("\nBottom Finished");
} 
////////////////////////////////////////////////////////////////////////////////

void setup() 
{ 
  Serial.begin(9600); 
  pilotLampInit();
  unreadyIndicator();
  
  setSpd(0);
  
  stepMotorInit();
  goToTop();
      
  /* loadcell init */
  loadcellInit();
  /* wait to start */
  switchInit();
  unsigned long timerPrint = millis(); 
  Serial.print("\nwaiting");
  for(int count=0; count<1; count++)     /////////////ปุ่มเปิดเครื่องเช็ค 3 ครั้ง
  {
    while(true)
    {
      if(!digitalRead(startPin))
      {
        delay(debounceTime);
        if(!digitalRead(startPin))
        {
          delay(debounceTime);
          if(!digitalRead(startPin))
          {
            delay(debounceTime);
            if(!digitalRead(startPin))
            {
             
              break;
            }
          }
        }
      }
      if(millis() - timerPrint > indicatorTime)
      {
        Serial.print(".");
        timerPrint = millis();
      }
    }
    Serial.println();
  }
  
  /* fisrt motor start */
  Serial.println("motor start");
  // high speed run
  setSpd(initSpdStart); delay(delayStart); 
  // default speed run 
  setSpd(spdStart);
  // current sensor alignment                                 
  alignCurrent();  
  // loadcell alignment
  weightAlign();
  
  #define timeStop 10000 // #######################
  unsigned long timerStop = millis();
  while(true)
  {
    if(millis() - timerStop > timeStop)
    {
      resetFunc();
    }
    
    float w = getWeightOffset(); 
    float a = getCurrentOffset();
    if(w > detectHighWeight)
    {
      if(w > minWeightPilotLamp && w < maxWeightPilotLamp)
      {
        readyIndicator();
      }
      break;
    }
    
    
    Serial.print(a); Serial.print("\t");
    Serial.println(w);
    delay(detectTimeFuzzy);
  }
  Serial.println("fuzzy start");
  delay(startTimeFuzzy); 

  // for(;;);
}


void loop() 
{
  float a  = getCurrentOffset();
  float kg = getWeightOffset();
    
  g_fisInput[0] = kg;
  g_fisInput[1] = a;
  g_fisOutput[0] = 0;
  fis_evaluate();
  
  float spd = g_fisOutput[0];
  float freq = 0;
//////////////////////////////////////////////////////// 
  //
  //
  //
  //
  // LOW  SPEED :  700-1400 rpm : 28, 55 Hz : ratio 2:1
  // HIGH SPEED : 1400-2800 rpm : 28, 55 Hz : ratio 1:1
  //
  //
  //
  //
  if(spd < 1400)
  {
    // case -> low speed
    freq = map(spd, 0, 1400, minFreq, maxFreq); 
  }
  else
  {
    // case -> high speed
    freq = map(spd, 0, 2800, minFreq, maxFreq); 
    highSpdCase = true;
    stepMotorStart();
  }
  setSpd(freq);
  //
  //
  //
  //
////////////////////////////////////////////////////////


 
//  setSpd(freq);
  
//  Serial.print(getWeight_1()); Serial.print("\t");
//  Serial.println(getWeight_2());

  const float f_spd = spd;
  const float f_freq = freq;
  const float f_a = a;
  const float f_kg = kg;
  
  Serial.print(f_spd);  Serial.print("\t");
  Serial.print(f_freq);  Serial.print("\t"); 
  Serial.print(f_a);  Serial.print("\t"); 
  Serial.println(f_kg);
  delay(1000); // delay for read

  unsigned long noLoadTimer = 0;
  boolean stopState = false;
  while(true)
  {
    if(highSpdCase)
    {
      if(!digitalRead(botLimitSwPin))  
      { 
        delay(dbTime);
        if(!digitalRead(botLimitSwPin))  
        { 
          stepMotorStop();
          highSpdCase = false;
        }
      }
      stepMotorDriveCCW(stepBotSpdPeriod); 
    }
  
    float w = getWeightOffset();
    float a = getCurrentOffset(); 
    
    if(w < detectLowWeight && !stopState)
    {
      stopState = true;
      noLoadTimer = millis();
    }
    if(w >= detectLowWeight)
    {
      stopState = false;
      noLoadTimer = 0;
    }
    if(millis()-noLoadTimer > detectTimeStop && noLoadTimer != 0)
    {
      break;
    }

    Serial.print(a); Serial.print("\t");
    Serial.println(w);
  }
  
  unreadyIndicator();
  setSpd(0); // stop
  
  Serial.print("\nSpeed = "); Serial.println(f_spd);  
  Serial.print("Frequency = "); Serial.println(f_freq); 
  Serial.print("Current = "); Serial.println(f_a);   
  Serial.print("Weight = "); Serial.println(f_kg);
  delay(2000); // delay for read
  
  resetFunc();
  
//  Serial.println(a);
//  delay(3000);
  
}
