#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <OneWire.h>
#include <DS18B20.h>
#include <U8glib.h>
#include <AccelStepper.h>

// ROS
ros::NodeHandle nh;

// Serial Communication
const int serial_interval = 500; // Interval između serijskog ispisivanja u ms
unsigned long last_serial_time = 0;

// Senzori
#define ONE_WIRE_BUS_1 2
#define ONE_WIRE_BUS_2 3
OneWire oneWire_1(ONE_WIRE_BUS_1);
OneWire oneWire_2(ONE_WIRE_BUS_2);
DS18B20 sen1(&oneWire_1);
DS18B20 sen2(&oneWire_2);
float temp_set1 = 44.0;
float temp_set2 = 44.0;
float temp_1 = 0.0;
float temp_2 = 0.0;
bool flag = false;

// Releji
#define RELAY_PIN_1 4 // Pin za relej 1
#define RELAY_PIN_2 5 // Pin za relej 2
bool relay_state_1 = false;
bool relay_state_2 = false;
unsigned long relay_off_time_1 = 0;
unsigned long relay_off_time_2 = 0;
unsigned long relay_start_time_1 = 0;
unsigned long relay_start_time_2 = 0;
unsigned long relay_duration = 500;
const unsigned long relay_duration_off = 3000; // 3 seconds off time for the relay

// Motor
#define MOTOR_STEP_PIN 6 // Pin za korak motora
#define MOTOR_DIR_PIN 7 // Pin za smjer motora
#define MOTOR_MICROSTEP_PIN 8//mikrostep
AccelStepper stepper(AccelStepper::DRIVER, MOTOR_STEP_PIN, MOTOR_DIR_PIN);
float motor_speed = 0.0; // Brzina motora

// Display
U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NO_ACK);

// Publisher "flag"
std_msgs::Bool flag_msg;
ros::Publisher flag_pub("flag", &flag_msg);

// Varijabla za praćenje proteklog vremena od uključenja
unsigned long start_time = 0;
const unsigned long flag_duration = 7 * 60 * 1000; // 7 minuta u milisekundama

void setup() {
  // ROS
  nh.initNode();
  // Subscriber "temp_set1"
  ros::Subscriber<std_msgs::Float32> temp_sub1("temp_set1", &tempSet1Callback);
  nh.subscribe(temp_sub1);
  // Subscriber "temp_set2"
  ros::Subscriber<std_msgs::Float32> temp_sub2("temp_set2", &tempSet2Callback);
  nh.subscribe(temp_sub2);
  // Subscriber "motor_speed"
  ros::Subscriber<std_msgs::Float32> motor_sub("motor_speed", &motorSpeedCallback);
  nh.subscribe(motor_sub);

  // Initialize publisher
  nh.advertise(flag_pub);

  sen1.begin();
  sen2.begin();
  sen1.setResolution(12);
  sen2.setResolution(12);

  sen1.requestTemperatures();
  sen2.requestTemperatures();

  pinMode(RELAY_PIN_1, OUTPUT);
  pinMode(RELAY_PIN_2, OUTPUT);

  // Motor
  stepper.setMaxSpeed(40*16);
  pinMode(MOTOR_MICROSTEP_PIN, OUTPUT);
  digitalWrite(MOTOR_MICROSTEP_PIN, HIGH);

  u8g.setColorIndex(1);
  Serial.begin(9600);

  start_time = millis(); // Pohrani trenutno vrijeme kao vrijeme uključenja
}

void loop() {
  // ROS
  nh.spinOnce();

  // Mjerenje temperature
  static unsigned long last_temp_check_time = 0;
  if (millis() - last_temp_check_time > 2000 || temp_set1 == 0 || temp_set2 == 0) {
    last_temp_check_time = millis();
    if (sen1.isConversionComplete()) {
      temp_1 = sen1.getTempC();
      sen1.requestTemperatures();
    }
    if (sen2.isConversionComplete()) {
      temp_2 = sen2.getTempC();
      sen2.requestTemperatures();
    }
  }

  if (temp_set1 != 0) {
    float temp_diff_1 = temp_set1 - temp_1;

    if (relay_state_1) {
      if (millis() - relay_start_time_1 >= relay_duration) {
        digitalWrite(RELAY_PIN_1, LOW);
        relay_state_1 = false;
        relay_off_time_1 = millis();
      } else {
        digitalWrite(RELAY_PIN_1, HIGH);
      }
    } else if (millis() - relay_off_time_1 >= relay_duration_off) {
      if (temp_diff_1 >= 0.50) {
        digitalWrite(RELAY_PIN_1, HIGH);
        relay_state_1 = true;
        relay_start_time_1 = millis();
      } else {
        digitalWrite(RELAY_PIN_1, LOW);
      }
    }
  }

  if (temp_set2 != 0) {
    float temp_diff_2 = temp_set2 - temp_2;

    if (relay_state_2) {
      if (millis() - relay_start_time_2 >= relay_duration) {
        digitalWrite(RELAY_PIN_2, LOW);
        relay_state_2 = false;
        relay_off_time_2 = millis();
      } else {
        digitalWrite(RELAY_PIN_2, HIGH);
      }
    } else if (millis() - relay_off_time_2 >= relay_duration_off) {
      if (temp_diff_2 >= 0.50) {
        digitalWrite(RELAY_PIN_2, HIGH);
        relay_state_2 = true;
        relay_start_time_2 = millis();
      } else {
        digitalWrite(RELAY_PIN_2, LOW);
      }
    }
  }

  // Motor
  if (motor_speed > 0.0) {
    stepper.setSpeed(-motor_speed*16);  // broj okreta u minuti*16
    stepper.runSpeed();
  } else {
    stepper.stop();
  }

  // Serijsko printanje za grafove
  //if (millis() - last_serial_time >= serial_interval) {
  //  last_serial_time = millis();
  //  Serial.print(temp_1);
  //  Serial.print('\t');
  //  Serial.print(temp_2);
  //  Serial.print('\t');
  //  Serial.print(temp_set1);
  //  Serial.print('\t');
  //  Serial.print(temp_set2);
  //  Serial.print('\t');
  //  Serial.print(relay_state_1 ? "1" : "0");
  //  Serial.print('\t');
  //  Serial.println(relay_state_2 ? "1" : "0");
  //}
  // Provjera vremena za postavljanje flaga
  if (millis() - start_time >= flag_duration && !flag) {
    flag = true; // Postavi flag na 1
    flag_msg.data = flag;
    flag_pub.publish(&flag_msg);
  }
}

// Callback funkcije
void tempSet1Callback(const std_msgs::Float32& msg) {
  temp_set1 = msg.data;
}

void tempSet2Callback(const std_msgs::Float32& msg) {
  temp_set2 = msg.data;
}

void motorSpeedCallback(const std_msgs::Float32& msg) {
  if (msg.data > 0.0) {
    motor_speed = msg.data;
  } else {
    motor_speed = 0.0;
  }
}
