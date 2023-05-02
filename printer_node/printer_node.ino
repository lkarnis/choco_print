#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

#include <OneWire.h>
#include <DallasTemperature.h>
#include <U8glib.h>
#include <AccelStepper.h>

// ROS
ros::NodeHandle nh;


// Senzori
#define ONE_WIRE_BUS_1 2 // Pin za senzor 1
#define ONE_WIRE_BUS_2 3 // Pin za senzor 2
OneWire oneWire_1(ONE_WIRE_BUS_1);
OneWire oneWire_2(ONE_WIRE_BUS_2);
DallasTemperature sensors_1(&oneWire_1);
DallasTemperature sensors_2(&oneWire_2);
float temp_set = 29.0;
float temp_1 = 0.0;
float temp_2 = 0.0;
bool flag = false;

// Releji
#define RELAY_PIN_1 4 // Pin za relej 1
#define RELAY_PIN_2 5 // Pin za relej 2
bool relay_state_1 = true;
bool relay_state_2 = true;
unsigned long relay_off_time_1 = 0;
unsigned long relay_off_time_2 = 0;
unsigned long relay_start_time_1 = 0;
unsigned long relay_start_time_2 = 0;
unsigned long relay_duration = 1000;
const unsigned long relay_duration_off = 3000; // 3 seconds off time for the relay

// Motor
#define MOTOR_STEP_PIN 6 // Pin za korak motora
#define MOTOR_DIR_PIN 7 // Pin za smjer motora
#define MOTOR_ENABLE_PIN 8 // Pin za omogućavanje motora
#define MOTOR_MAX_SPEED 800.0 // Maksimalna brzina motora
float motor_speed = 1.0;
AccelStepper stepper(AccelStepper::DRIVER, MOTOR_STEP_PIN, MOTOR_DIR_PIN);
int motor_step_delay = 2000;

// Display
U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NO_ACK);

//Publisher "flag"
std_msgs::Bool flag_msg;
ros::Publisher flag_pub("flag", &flag_msg);

void setup() {
  // ROS
  nh.initNode();
  // Subscriber "temp_set"
  ros::Subscriber<std_msgs::Float32> temp_sub("temp_set", &tempSetCallback);
  nh.subscribe(temp_sub);

  // Subscriber "motor_speed"
  ros::Subscriber<std_msgs::Float32> motor_sub("motor_speed", &motorSpeedCallback);
  nh.subscribe(motor_sub);

  // Publish
  nh.advertise(flag_pub);

  sensors_1.begin();
  sensors_2.begin();
  pinMode(RELAY_PIN_1, OUTPUT);
  pinMode(RELAY_PIN_2, OUTPUT);

  // Motor
  stepper.setMaxSpeed(MOTOR_MAX_SPEED);
  stepper.setSpeed(motor_speed * MOTOR_MAX_SPEED);
  pinMode(MOTOR_ENABLE_PIN, OUTPUT);
  digitalWrite(MOTOR_ENABLE_PIN, LOW);

  u8g.setColorIndex(1);
}

void loop() {
  // ROS
  nh.spinOnce();

    // Mjerenje temperature
    static unsigned long last_temp_check_time = 0;
    if (millis() - last_temp_check_time > 5000 || temp_set == 0) {
      last_temp_check_time = millis();
      sensors_1.requestTemperatures();
      temp_1 = sensors_1.getTempCByIndex(0);
      sensors_2.requestTemperatures();
      temp_2 = sensors_2.getTempCByIndex(0);
    }

  if (temp_set != 0) {
    float temp_diff_1 = temp_set - temp_1;
    float temp_diff_2 = temp_set - temp_2;

    if (relay_state_1) {
      if (millis() - relay_start_time_1 >= relay_duration) {
        digitalWrite(RELAY_PIN_1, LOW);
        relay_state_1 = false;
        relay_off_time_1 = millis(); // postavi vrijeme gasenja
      }
    } else if (millis() - relay_off_time_1 >= relay_duration_off) { // jel proslo dovoljno vremena od gasenja
      if (temp_diff_1 >= 0.50) {
        digitalWrite(RELAY_PIN_1, HIGH);
        relay_state_1 = true;
        relay_start_time_1 = millis();
      }
    }

    if (relay_state_2) {
      if (millis() - relay_start_time_2 >= relay_duration) {
        digitalWrite(RELAY_PIN_2, LOW);
        relay_state_2 = false;
        relay_off_time_2 = millis(); //  postavi vrijeme gasenja
      }
    } else if (millis() - relay_off_time_2 >= relay_duration_off) { // jel proslo dovoljno vremena od gasenja
      if (temp_diff_2 >= 0.50) {
        digitalWrite(RELAY_PIN_2, HIGH);
        relay_state_2 = true;
        relay_start_time_2 = millis();
      }
      if (temp_1 > temp_set && temp_2 > temp_set){
        flag_msg.data = true;
        flag_pub.publish(&flag_msg);
      }
    }
  }

  // Motor
  stepper.setSpeed(motor_speed * MOTOR_MAX_SPEED);
  stepper.runSpeed();
  static unsigned long last_step_time = 0;
  if (millis() - last_step_time >= motor_step_delay) {
    stepper.runSpeed();
    last_step_time = millis();
  }

  // Display
  u8g.firstPage();
  do {
    u8g.setFont(u8g_font_unifont);
    u8g.drawStr(0, 10, "Temp set: ");
    u8g.setPrintPos(75, 10);
    u8g.print(temp_set);
    u8g.drawStr(0, 22, "Temp 1: ");
    u8g.setPrintPos(60, 22);
    u8g.print(temp_1);
    u8g.drawStr(0, 34, "Temp 2: ");
    u8g.setPrintPos(60, 34);
    u8g.print(temp_2);
    u8g.drawStr(0, 46, "Motor speed: ");
    u8g.setPrintPos(95, 46);
    u8g.print(motor_speed);
    u8g.drawStr(0, 58, "Flag:");
    u8g.setPrintPos(40, 58);
    u8g.print(flag);
  } while (u8g.nextPage());

}

// Callback funkcije
void tempSetCallback(const std_msgs::Float32& msg) {
temp_set = msg.data;
}

void motorSpeedCallback(const std_msgs::Float32& msg) {
motor_speed = msg.data;
}