/**
 *  Modbus RTU slave Device
 * Device can Modbus RTU cominication over RS-485 hardware
 * 150w Dc motor direction and speed can controll
 * for feedback use 4-20ma incliometer , as5600 or mpu 6050 imu chip over i2c
 * Desing By MOB
 */

#include <ModbusRtu.h>
#include <DipSwitch.h>

#include <avr/wdt.h>
#include <Wire.h>

// Slave Device Specification
//=======================================================
#define ID 7 // if Dipswitch sellected , Doesnt matter this line
// node id = 0 for master, = 1..247 for slave
#define BAUNDRATE 4800
//=======================================================
//*************Tracker Dinamic Parameters***********
#define motor_max_speed 125 // 0-255  //to test value 30
#define pos_tolerance 5     // 3 =  tolerans +/-  % 0.025

const short int min_angle_limit = -70;         // minimal panel angle input value -70   155
const short int max_angle_limit = 50;          // maximum panel angleinput  value 50   190
const short int direction_relieve_count = 300; // cycle
int fadeValue = 2;
//**************************************************

//*****************************************
const int MPU_addr = 0x68;
int16_t AcX, AcY, AcZ, GyX, GyY, GyZ, temp, y;

// const unsigned long ONE_WEEK = 7ul * 24ul * 60ul * 60ul * 1000ul;
//_____________________________week__day____hour___min____second
const unsigned long ONE_HOURS = 60ul * 60ul * 1000ul;

const short int mid_value = 180; // (max_angle_limit-min_angle_limit)/2; // middle input value
int minVal = 265;
int maxVal = 402;
//*****************************************

// PCB pin Configration
#define MAX485_En 2
#define led_stat 13
#define led_err 11
#define motor_en 3
#define motor_dir 4

#define limit_L 8
#define limit_R 9

#define add_1 14 // 23
#define add_2 15 // 24
#define add_3 16 // 25
#define add_4 17 // 26

#define add_5 10 // 14
#define add_6 7  // 11
#define add_7 6  // 10
#define add_8 5  // 9

int16_t au16data[4] = {0, 0, 0, 0}; // data array for modbus network sharing

boolean direction = 0;
boolean enable = 0;
boolean relieve = 0;

boolean enable_permit_L = 1;
boolean enable_permit_R = 1;
boolean refresh = 0;
boolean current = 0;

int8_t state = 0;
int16_t relieve_count = 0;

int16_t real_angle = 0;
int16_t show_angle = 0;

int16_t convert_from_angle;

int16_t target_pos, target_input;
int16_t abs_result;

int16_t last_limit_R_value = 0, last_limit_L_value = 0;

unsigned long tempus;

int16_t direction_count = direction_relieve_count;

bool LED_STATE = true;

int dipSwitchPins[] = {14, 15, 16, 17, 5, 6, 7, 10}; // 0 1 2 3 4 5 6 7 8
DipSwitch myDipSwitch(8, dipSwitchPins);
Modbus slave(myDipSwitch.read(), Serial, MAX485_En); // this is slave id
// Modbus slave(ID, Serial, MAX485_En); // For Manual id Define

void setup()
{

  //==========interrupt Settings==========================
  //---------50Hz setting timer 1 interrupts------------
  cli(); // stop interrupts for till we make the settings
  // 1. First we reset the control register to amke sure we start with everything disabled.
  TCCR1A = 0; // Reset entire TCCR1A to 0
  TCCR1B = 0; // Reset entire TCCR1B to 0
  // 2. We set the prescalar to the desired value by changing the CS10 CS12 and CS12 bits.
  TCCR1B |= B00000100; // Set CS12 to 1 so we get prescalar 256
  // 3. We enable compare match mode on register A
  TIMSK1 |= B00000010; // Set OCIE1A to 1 so we enable compare match A
  // 4. Set the value of register A to 31250
  OCR1A = 250; // Finally we set compare register A to this value

  //-------Limit Switch interrupt define
  PCICR |= B00000001;  // Enable interrupts on PB port
  PCMSK0 |= B00000011; // Trigger interrupts on pins D8 and D9

  sei(); // Enable back the interrupts
  //=================Watchdog Self Reset Setup===========
  wdt_enable(WDTO_8S);
  //=====================================================

  pinMode(MAX485_En, OUTPUT);
  pinMode(motor_en, OUTPUT);
  pinMode(motor_dir, OUTPUT);
  pinMode(led_stat, OUTPUT);
  pinMode(led_err, OUTPUT);
  pinMode(limit_L, INPUT);
  pinMode(limit_R, INPUT);
  digitalWrite(motor_dir, LOW); // Motor Direction pin start value is 0 mean is => Motor will rotate at start Right
  analogWrite(motor_en, 0);     // Motor speed is zero

  Serial.begin(BAUNDRATE, SERIAL_8N1); // baud-rate at 19200
  slave.start();
  tempus = millis() + 100;
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Wire.begin();
  //////////////////////////////////
}

void loop()
{

  target_pos = au16data[1]; //-----------All Data input
  show_angle = data_imu();  //---------- IMU read
  au16data[0] = target_pos; //-----------Look Back Controll
  // au16data[3] = temp;

  if (0 > show_angle)
  {
    show_angle = 360 + show_angle;
    au16data[2] = show_angle;
  }
  else
  {
    au16data[2] = show_angle;
  }

  motor_drive(target_pos);

  if (state > 4) // if state above 2-> do it
  {
    tempus = millis() + 3;
    digitalWrite(led_stat, HIGH);

    //=================Watchdog Self Reset===========
    // if loop() doesn't reset the timer at least every 8 seconds. If code locks up, the watchdog timer will reset

    //   if (millis() < ONE_HOURS)
    wdt_reset();
    //=====================================================
  }
  if (millis() > tempus)
    digitalWrite(led_stat, LOW);
}

void limit_check()
{

  if (((PINB & B00000001) == 0) && ((PINB & B00000010) == 0))
  {
    digitalWrite(led_err, LOW);
    return 1;
  }

  if ((PINB & B00000001))
  {
    limit_left();
    return 2;
  }
  else if (PINB & B00000010)
  {
    limit_right();
    return 3;
  }
}

void soft_start()
{
  int k;

  /* for (int k = 0; k <= motor_max_speed; k += (fadeValue / 2))
   {

     delay(1);
     /*
     if (k > motor_max_speed-10)
     {
       k = motor_max_speed;
       break;
     }
------------------
     motor_speed(k);
   }
 */
  motor_speed(motor_max_speed);
  refresh = 1;
}

void soft_stop()
{
  int k;
  /*
    for (int k = motor_max_speed; k >= 0; k -= fadeValue)
    {

      delay(1);
      if (k < 5)
      {
        k = 0;
        break;
      }
      motor_speed(k);
    }
    */

  motor_speed(0);
  refresh = 0;
}

void motor_speed(int speed) { analogWrite(motor_en, speed); } // set motor speed mean is PWM

void motor_pros()
{
  if (enable)
  {

    if ((enable == 1) && (refresh == 0))
    {

      soft_start();
      refresh = current;
    }
    enable = 0;
    motor_speed(motor_max_speed);
    digitalWrite(motor_dir, direction); //////////////////__Opsional
  }

  else if (enable == 0)
  {

    if ((enable == 0) && (refresh == 1))
    {

      soft_stop();
      refresh = current;
    }

    enable = 1;
    motor_speed(0);
    digitalWrite(motor_dir, direction); //////////////////__Opsional
  }
}

void motor_position(int position) // for direction
{

  int16_t position_read = data_imu();
  real_angle = position_read;

  /////////// Tolerance ////////////////////
  // final position error tolerans +/- 0.025
  abs_result = abs(position - position_read);

  if (abs_result < pos_tolerance)
  {
    enable = 0;
    digitalWrite(motor_dir, direction);

    //-------------------Relieve Function ------------------ // relay coil non energize
    if (relieve == 1)
    {
      if (direction_count > 0)
      {
        direction_count--;
      }
      else
      {
        direction = 0;
        motor_speed(0);
        digitalWrite(motor_dir, direction);
        delay(500); //  dalay function use only relay stop. and after its gonna none use again
        relieve = 0;
        return 0;
      }
    }
    //-------------------Relieve Function ------------------
    return 0;
  }

  else if (abs_result > pos_tolerance)
  {
    enable = 1;
    digitalWrite(motor_dir, direction);

    direction_count = direction_relieve_count;
    relieve = 1;
  }

  //***********************up to zero valuable******************************
  if ((position - position_read >= 0))
  {
    if ((abs_result < mid_value))

    {

      direction = 1;
    }
    else
    {
      direction = 0;
    }
  }

  //**********************down to zero valuable****************************
  if ((position - position_read <= 0))
  {
    if ((abs_result < mid_value))

    {
      direction = 0;
    }
    else
    {
      direction = 1;
    }
  }
  //********************************************************************

  motor_pros();
  return 1;
}

int target_limits(int input_target)
{
  if (((0 >= input_target) && (input_target >= min_angle_limit)) || ((0 <= input_target) && (input_target <= max_angle_limit)))
  {
    return input_target;
  }

  else
    return 9999; //-----Out of working space value
}

int motor_drive(int target_pos)
{
  convert_from_angle = target_pos; //   map(target_pos, 0, 360, 0, 4096); // to as5600

  if (0 >= convert_from_angle)
  {
    if (((last_limit_R_value <= convert_from_angle)) || (last_limit_R_value == 0))

    {
      enable_permit_L = 1;
      limit_check();
    }
    else
    {
      enable_permit_L = 0;
    }
  }

  else if (0 <= convert_from_angle)
  {
    if ((last_limit_L_value >= convert_from_angle) || (last_limit_L_value == 0))
    {
      enable_permit_R = 1;
      limit_check();
    }
    else
    {
      enable_permit_R = 0;
    }
  }

  /*
    if ((convert_from_angle < 0) && (enable_permit_L))
    {
      convert_from_angle = 180 + convert_from_angle;
      // digitalWrite(led_err, HIGH); //Negative values
      // enable_permit_R = 1;
    }

    else if ((convert_from_angle > 0) && (enable_permit_R))
    {
      // au16data[3] = convert_from_angle;/////////////////////////////////////////////////*****************************
      //   digitalWrite(led_err, LOW);  // positive values
      //  enable_permit_L = 1;
    }
  */

  if ((target_limits(convert_from_angle) != 9999) && (enable_permit_R && enable_permit_L))
  {
    motor_position(convert_from_angle);
    return 1;
  }
  else
  {
    // target_pos=0;
    // digitalWrite(led_err, HIGH);

    digitalWrite(led_err, 1);
    delay(120);
    digitalWrite(led_err, 0);
    delay(120);

    motor_speed(0);
    digitalWrite(motor_dir, 0);

    return 0;
  }
}

int16_t data_imu()
{
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true);
  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  temp = Wire.read() << 8 | Wire.read();

  int xAng = map(AcX, minVal, maxVal, -45, 45);
  int yAng = map(AcY, minVal, maxVal, -45, 45);
  int zAng = map(AcZ, minVal, maxVal, -45, 45);
  
  const float TEMP_MUL = 1.0 / 340.0;
  temp = ((temp * TEMP_MUL) + 36.53);

  y = RAD_TO_DEG * (atan2(-xAng, -zAng) + PI);

  if (180 < y)
  {
    y = y - 360;
  }

  return y;
}

void limit_left() // white cable
{
  motor_speed(0);
  last_limit_L_value = au16data[2];
  // au16data[6] = last_limit_L_value;/////////////////////////////////////////////////*****************************
  enable_permit_R = 0;
  digitalWrite(led_err, HIGH);
}

void limit_right() // yellow cable
{
  motor_speed(0);

  last_limit_R_value = au16data[2];
  last_limit_R_value = last_limit_R_value - 360;

  // au16data[5] = last_limit_R_value;/////////////////////////////////////////////////*****************************
  enable_permit_L = 0;

  digitalWrite(led_err, HIGH);
}

ISR(PCINT0_vect) // Settings for limit switch interrupts routine
{
  _delay_us(800);
  if (PINB & B00000001)
  {
    limit_left();
  }
  else if (PINB & B00000010)
  {
    limit_right();
  }
}

// Timer 1 Interrupt routine duty (network sharing and limit porsses)
ISR(TIMER1_COMPA_vect)

{

  // bitWrite(au16data[4], 7, 1); // busy stuation flag
  /*
    if ((enable_permit_R || enable_permit_L) == 0)
    {
      motor_speed(0);
    }
  */
  state = slave.poll(au16data, 4);

  // LED_STATE = !LED_STATE;           // Invert LED state
  // digitalWrite(led_err, LED_STATE); // Write new state to the LED on pin 11

  OCR1A = 1250;
  TCNT1 = 0; // First, set the timer back to 0 so it resets for next interrupt
}
