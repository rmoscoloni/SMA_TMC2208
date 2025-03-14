#include <TMCstepper.h>

// PINS
// 10uF CAP BETWEEN ESP32 PINS EN AND GND 
// POT GND 3V3 CENTER TO ESP32 PIN 32
// TMC2209 ENABLE TO ESP32 PIN 4 
// TMC2209 ENABLE 10K RESISTOR TO 3V3 (HARDWARE DISABLE)
// TMC2209 UART TO ESP32 PIN 16
// TMC2209 UART TO 1K RESISTOR OTHER SIDE TO ESP32 PIN 17
// TMC2209 LOGIC 3v3 AND GND FROM ESP32
// TMC2209 MOTOR POWER SUPPLY TO PINS GND AND SMD POT SIDE
// 100uF ELECTROLYTIC AND 100nF CERAMIC CAP ON MOTOR POWER SUPPLY
// TMC2209 TO MOTOR COILS

// This example will not work on Arduino boards without HardwareSerial ports,
// such as the Uno, Nano, and Mini.
//
// See this reference for more details:
// https://www.arduino.cc/reference/en/language/functions/communication/serial/

#include "BluetoothSerial.h"
BluetoothSerial SerialBT;

/*------------------------- DRIVER     ------------------------*/
const int ENABLE_PIN = 4;
const long SERIAL_BAUD_RATE = 115200;
const int DELAY = 2000;
int32_t VELOCITY = 0;
#define SERIAL_PORT Serial2 //RX pin 16 y TX pin 17
#define R_SENSE 0.11f
TMC2208Stepper driver(&SERIAL_PORT, R_SENSE);
unsigned long millis_last_driver;
int interval_driver = 50;  //100ms velocity service loop

/*------------------------- MOVING AVG ------------------------*/
unsigned int movingAverage(unsigned int value) {
const byte nvalues = 30;  // Moving average window size
static byte current = 0;         // Index for current value
static byte values_counter = 0;  // Count of values read (<= nvalues)
static long sum = 0;             // Rolling sum
static int values[nvalues];

sum += value;
// If the window is full, adjust the sum by deleting the oldest value
if (values_counter == nvalues)
  sum -= values[current];
values[current] = value;  // Replace the oldest with the latest
if (++current >= nvalues)
  current = 0;
if (values_counter < nvalues)
  values_counter += 1;
return sum / values_counter;
}

/*------------------------- POTE      -------------------------*/
const int potPin = 32; // center pin
//variable for storing the potentiometer value
int pot_value = 0;

/*------------------------- TIME CONTROL ----------------------*/
unsigned int remaining_t(int update_t) {
  return millis() - update_t;
}

/*------------------------- CHECK VOLATILE ----------------------*/
uint8_t interfase_counter = false;
void CHECK_VOLATILE() {
  interfase_counter = driver.IFCNT();
}

/*------------------------- WRITE VOLATILE ----------------------*/
void WRITE_VOLATILE() {
  driver.pdn_disable(false);
  driver.toff(6);                 // Higher TOFF = More drive strength
  driver.rms_current(1000);       // Set motor RMS current
  driver.microsteps(0b010);       // RM:doesnt work for 1 microstep min 2 microstp
  driver.freewheel(true);
  driver.en_spreadCycle(true);    // Enable spreadCycle (better torque at high speed)
  driver.pwm_autoscale(false);    // Disable auto-scaling (forces strong PWM output)
  driver.ihold(0);
  driver.irun(31);
  driver.intpol(false); // disable uart interpolation 256 microsteps
}

/*------------------------- SERIAL    -------------------------*/
unsigned long millis_last_serial;
int interval_serial = 2000;  //2000 milliseconds

void SERIAL_PRINT() {
  if (remaining_t(millis_last_serial) > interval_serial) {
    millis_last_serial = millis();

    Serial.print("velocity_cmd = ");
    Serial.println(VELOCITY);
    Serial.print("driver_isEnabled = ");
    Serial.println(driver.isEnabled());
    Serial.print("microsteps = ");
    u_int8_t microsteps = driver.microsteps();
    Serial.println(microsteps);
    Serial.print("i_rms = ");
    Serial.println(driver.rms_current());
    Serial.print("i_hold = ");
    Serial.println(driver.ihold());
    Serial.print("interfase_counter = ");
    Serial.println(interfase_counter);
    Serial.println("-----------------------------------");
    // RPM = VELOCITY*60*12000000/2^24*2*200
    uint16_t scaled_rpm = VELOCITY*0.107288;
    SerialBT.print(scaled_rpm);
    SerialBT.println("RPM");
    CHECK_VOLATILE();
    // if (!interfase_counter){
    //   Serial.println("WRITE VOLATILE");
    //   Serial.println("-----------------------------------");
    //   WRITE_VOLATILE();
    // }
    if (microsteps != 2){
      Serial.println("WRITE VOLATILE");
      Serial.println("-----------------------------------");
      WRITE_VOLATILE();
    }
  }
}

/*------------------------- SETUP     -------------------------*/
void setup()
{
  pinMode(potPin, INPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  Serial.begin(SERIAL_BAUD_RATE);
  delay(DELAY);
  driver.begin(); 
  SERIAL_PORT.begin(115200);
  WRITE_VOLATILE();
  if (analogRead(potPin <= 10)){
    digitalWrite(ENABLE_PIN, LOW);
    Serial.println("P4 ENABLE");
  }
  SerialBT.begin("SMA_TMC2208");
}

/*------------------------- LOOP      -------------------------*/
void loop()
{
if (remaining_t(millis_last_driver) > interval_driver) {
  millis_last_driver = millis();
  pot_value = (analogRead(potPin));
  int32_t avg_pot = movingAverage(pot_value);
  int32_t pot_maped = map(avg_pot, 0, 4095, 0, 9321); // 9321 gives aprox. 1000RPM 
  VELOCITY  = pot_maped;
  // see motor_speed picture on the main.cpp folder
  driver.VACTUAL(VELOCITY);
  }
  SERIAL_PRINT();
}