#include <Wire.h>
#include "ClosedCube_SHT31D.h"
#include <PID_v1_bc.h>
/*
 * I2C  GPIO8 SDA
 * I2C  GPIO9 SCL
 */


ClosedCube_SHT31D sht3xd;
#define RELAY_PIN 19
double Setpoint = 50;  // 設定目標溫度
double Input, Output;  // PID控制器的輸入、輸出值
double Kp = 2, Ki = 5, Kd = 1;  // PID控制器的係數
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  pinMode(RELAY_PIN, OUTPUT); // 設定GPIO 19為輸出模式
  digitalWrite(RELAY_PIN, LOW); // 初始化狀態為關閉

  Wire.begin();
  Serial.begin(9600);
  Serial.println("ClosedCube SHT3X-D Single Shot Mode Example");
  Serial.println("supports SHT30-D, SHT31-D and SHT35-D");

  sht3xd.begin(0x44); // I2C address: 0x44 or 0x45

  Serial.print("Serial #");
  Serial.println(sht3xd.readSerialNumber());

  // 初始化PID控制器
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255);

  // 開啟串口
  Serial.begin(115200);
}

void loop() {
  // 讀取溫度數據
  SHT31D result = sht3xd.readTempAndHumidity(SHT3XD_REPEATABILITY_LOW, SHT3XD_MODE_CLOCK_STRETCH, 50);

  // 如果讀取成功
  if (result.error == SHT3XD_NO_ERROR) {
    // 將溫度值設定為PID控制器的輸入值
    Input = result.t;

    // 更新PID控制器
    myPID.Compute();

    // 根據PID控制器的輸出值控制GPIO 19的狀態
    if (Output > 0) {
      digitalWrite(19, HIGH);
    } else {
      digitalWrite(19, LOW);
    }

    // 打印結果
    Serial.print("Temperature: ");
    Serial.print(result.t);
    Serial.print("C, Setpoint: ");
    Serial.print(Setpoint);
    Serial.print("C, Output: ");
    Serial.print(Output);
    Serial.println("");

    // 將溫度和PID控制的溫度輸出到串口
    Serial.print(result.t);
    Serial.print(",");
    Serial.println(Output);
  } else {
    // 打印錯誤信息
    Serial.print("Error reading temperature: [ERROR] Code #");
    Serial.println(result.error);
  }

  // 等待一段時間
  delay(1000);
}
