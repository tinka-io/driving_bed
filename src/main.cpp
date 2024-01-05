#include <Arduino.h>
#include "DCmotor.hpp"

typedef uint8_t u8;
typedef uint32_t u32;

const u8 led_pin = LED_BUILTIN;

const u8 lift_dir_pin = 16;
const u8 lift_pwm_pin = 17;
const u8 lift_break = 18;
const u8 lift_sw_pin = 11;
const u8 lift_motor_relais = 13;

const u8 couch_dir_pin = 20;
const u8 couch_pwm_pin = 21;
const u8 couch_up_sw_pin = 1;
const u8 couch_down_sw_pin = 2;

const u8 led_red_pin = 10;
const u8 led_green_pin = 12;
const u8 led_blue_pin = 14; // 13;

DCmotor motor_lift(lift_dir_pin, lift_pwm_pin);
DCmotor motor_couch(couch_dir_pin, couch_pwm_pin);

const bool ON = true;
const bool OFF = false;
void led_red(bool state)
{
  digitalWrite(led_red_pin, state);
}
void led_green(bool state)
{
  digitalWrite(led_green_pin, state);
}
void led_blue(bool state)
{
  digitalWrite(led_blue_pin, state);
}
void setup()
{
  // LED internal
  pinMode(led_pin, OUTPUT);
  digitalWrite(led_pin, HIGH);

  // LEDs
  pinMode(led_red_pin, OUTPUT);
  pinMode(led_green_pin, OUTPUT);
  pinMode(led_blue_pin, OUTPUT);
  led_red(ON);
  led_green(ON);
  led_blue(ON);

  // Lift
  pinMode(lift_break, OUTPUT);
  pinMode(lift_sw_pin, INPUT_PULLUP);
  pinMode(lift_motor_relais, OUTPUT);
  digitalWrite(lift_break, OFF);
  digitalWrite(lift_motor_relais, OFF);

  // Couch
  pinMode(couch_up_sw_pin, INPUT_PULLUP);
  pinMode(couch_down_sw_pin, INPUT_PULLUP);

  motor_lift.set_accel_time(500);
  motor_lift.set_dir(DIR_CW);
  motor_lift.set_speed(255);
  motor_lift.stop();

  motor_couch.set_accel_time(500);
  motor_couch.set_dir(DIR_CW);
  motor_couch.set_speed(255);
  motor_couch.stop();

  Serial.begin(115200);
  delay(1000); // wait for Monitor after Upload
  Serial.println("Hello from BETT");
}

void status_led(ut dt)
{
  static ut last_blink = 0;

  if (millis() > last_blink + dt)
  {
    last_blink = millis();
    digitalWrite(led_pin, !digitalRead(led_pin));
  }
}

void lift_unlock_break(bool status)
{
  if (digitalRead(lift_break) != status)
  {
    Serial.println("Motor Break: " + String(status));
    digitalWrite(lift_break, status);
    delay(300);
  }
}

void lift_drive_motor_relais(bool status)
{
  if (digitalRead(lift_motor_relais) != status)
  {
    Serial.println("Motor Relais: " + String(status));
    digitalWrite(lift_motor_relais, status);
    delay(300);
  }
}

void show_led(u8 dir)
{
  switch (dir)
  {
  case 0:
    led_green(ON);
    led_red(OFF);
    break;
  case 1:
    led_green(OFF);
    led_red(ON);
    break;
  case 2:
    led_green(OFF);
    led_red(OFF);
    break;
  }
}

void lift_loop()
{
  static u32 last = 0;
  static u8 stage = 0;
  static bool dir = 0;
  // dir means: 0 = up, 1 = down

  switch (stage)
  {
  case 0:
    // Sleep and wait for SW
    if (!digitalRead(lift_sw_pin))
    {
      last = millis();
      stage++;
      show_led(dir);
    }
    // LEDs off after some time
    if (millis() > last + 3000)
    {
      show_led(2);
    }
    break;
  case 1:
    // Change Direction
    if (digitalRead(lift_sw_pin))
    {
      stage--;
      dir = !dir;
      show_led(dir);
      last = millis();
      if (dir)
        Serial.println("Change direction: down");
      else
        Serial.println("Change direction: up");
    }
    // when still active after some time, drive lift
    if (millis() > last + 1000 &&
        motor_lift.get_current_speed() == 0)
    {
      stage++;
      last = millis();
      motor_lift.start();
      motor_lift.set_dir(dir);
      lift_drive_motor_relais(!dir);
      lift_unlock_break(true);
      if (dir)
        Serial.println("Drive down");
      else
        Serial.println("Drive up");
    }
    break;
  case 2:
    // drive as long as sw is on
    if (digitalRead(lift_sw_pin))
    {
      stage++;
    }
    break;
  case 3:
    // break and go to sleep
    Serial.println("Drive Stop");
    stage = 0;
    last = millis();
    motor_lift.stop();
    break;
  }

  motor_lift.drive();
  if (motor_lift.get_current_speed() == 0)
  {
    lift_unlock_break(false);
    lift_drive_motor_relais(false);
  }
}

void couch_loop()
{
  static bool drive = false;
  bool up = !digitalRead(couch_up_sw_pin);
  bool down = !digitalRead(couch_down_sw_pin);

  if (up && !drive)
  {
    up = true;
    drive = true;
    motor_couch.set_dir(DIR_CW);
    motor_couch.start();
  }

  if (down && !drive)
  {
    down = true;
    drive = true;
    motor_couch.set_dir(DIR_CCW);
    motor_couch.start();
  }

  if (!up && !down)
  {
    motor_couch.stop();
  }

  if (motor_couch.get_current_speed() == 0)
  {
    drive = false;
  }

  motor_couch.drive();
}

void loop()
{
  lift_loop();
  couch_loop();
}