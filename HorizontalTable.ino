// 水平な台

// MP6050から値を取得する
// 接続  Pico               MP6050
//       Pin6 GP4        -> SDA
//       Pin7 GP5        -> SCL
//       Pin36 3V3(OUT)  -> VCC
//       Pin3 GND        -> GND
//
//       Pin4 GP2        -> サーボ1
//       Pin5 GP3        -> サーボ2

#include <Servo.h>		// サーボライブラリ
#include <Wire.h>
#include <MadgwickAHRS.h>
#include "RPi_Pico_TimerInterrupt.h"

Madgwick MadgwickFilter;

#define TIMER_INTERVAL_US  2500L // 400Hz
#define SRV_PIN1 2
#define SRV_PIN2 3
#define SRV_CENTER1 85
#define SRV_CENTER2 105
#define SRV_MAX1 60
#define SRV_MAX2 60

// Init RPI_PICO_Timer
RPI_PICO_Timer ITimer(1);

Servo servo1;
Servo servo2;

float roll  = 0;
float pitch = 0;
float yaw   = 0;

bool TimerHandler(struct repeating_timer *t)
{
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 14);
  while (Wire.available() < 14);
  
  int16_t axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw, temperature;
  axRaw = Wire.read() << 8 | Wire.read();
  ayRaw = Wire.read() << 8 | Wire.read();
  azRaw = Wire.read() << 8 | Wire.read();
  temperature = Wire.read() << 8 | Wire.read();
  gxRaw = Wire.read() << 8 | Wire.read();
  gyRaw = Wire.read() << 8 | Wire.read();
  gzRaw = Wire.read() << 8 | Wire.read();

/*
  Serial.print(axRaw); Serial.print(",");
  Serial.print(ayRaw); Serial.print(",");
  Serial.print(azRaw); Serial.print(",");
  Serial.print(gxRaw); Serial.print(",");
  Serial.print(gyRaw); Serial.print(",");
  Serial.println(gzRaw);
*/

  // 加速度値を分解能で割って加速度(G)に変換する
  float acc_x = axRaw / 16384.0;  //FS_SEL_0 16,384 LSB / g
  float acc_y = ayRaw / 16384.0;
  float acc_z = azRaw / 16384.0;

  // 角速度値を分解能で割って角速度(degrees per sec)に変換する
  float gyro_x = gxRaw / 131.0;  // (度/s)
  float gyro_y = gyRaw / 131.0;
  float gyro_z = gzRaw / 131.0;

  //Madgwickフィルターを用いて、PRY（pitch, roll, yaw）を計算
  MadgwickFilter.updateIMU(gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z);

  //PRYの計算結果を取得する
  roll  = MadgwickFilter.getRoll();
  pitch = MadgwickFilter.getPitch();
  yaw   = MadgwickFilter.getYaw();

  return true;
}


void setup() {
  servo1.attach(SRV_PIN1);
  servo2.attach(SRV_PIN2);
  servo1.write(SRV_CENTER1);
  servo2.write(SRV_CENTER2);

  //Serial.begin(9600);
  Serial.begin(115200);

  Wire.setSDA(4);       //I2C0 SDA GP4
  Wire.setSCL(5);       //I2C0 SCL GP5
  Wire.setClock(400000);
  Wire.begin();

  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();

  MadgwickFilter.begin(1000000 / TIMER_INTERVAL_US); //Hz
  MadgwickFilter.setGain(5.0);

  ITimer.attachInterruptInterval(TIMER_INTERVAL_US, TimerHandler);

  delay(1000);  // 1秒待つ
}

void loop() {

  //Serial.print(roll); Serial.print(",");
  //Serial.print(pitch); Serial.print(",");
  //Serial.println(yaw);
  
  float r = roll;
  float p = pitch;
  
  if (r > SRV_MAX1) r = SRV_MAX1;
  if (r < -SRV_MAX1) r = -SRV_MAX1;

  if (p > SRV_MAX2) p = SRV_MAX2;
  if (p < -SRV_MAX2) p = -SRV_MAX2;

  servo1.write(SRV_CENTER1 + r);
  servo2.write(SRV_CENTER2 - p);

  delay(10);
}
