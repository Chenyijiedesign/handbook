#include <SoftwareSerial.h>

#include "MPU6050.h"
#include "I2Cdev.h"
#include "Adafruit_NeoPixel.h"

// Master
#define PIN 7
Adafruit_NeoPixel strip 
= Adafruit_NeoPixel(64, PIN, NEO_GRB + NEO_KHZ800);

#define BUTTON 11

MPU6050 accelgyro;
int16_t ax,ay,az; //储存IMU角度信息，并通过蓝牙传出
int16_t gx,gy,gz;
float Gyro_x;
float Gyro_y;
float Gyro_z;
float angleAx;
float angle6;
float K1=0.05;
float Angle;
float accelz=0;
int sign;
int sign_pre;
int test;
int buttonState=0;

SoftwareSerial BlueTooth(8,9); //创建虚拟串口 8 Rx 9 Tx

// Kalman_Filter

float P[2][2] = {{ 1, 0 },
  { 0, 1 }
};
float Pdot[4] = { 0, 0, 0, 0};
float Q_angle = 0.001, Q_gyro = 0.005; //角度数据置信度,角速度数据置信度
float R_angle = 0.5 , C_0 = 1;
float q_bias, angle_err, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;
float timeChange = 5; //滤波法采样时间间隔毫秒
float dt = timeChange * 0.001; //注意：dt的取值为滤波器采样时间

void Angletest()
{
  //平衡参数
  Angle = atan2(ay , az) * 57.3;           //角度计算公式
  Gyro_x = (gx - 128.1) / 131;              //角度转换
  Kalman_Filter(Angle, Gyro_x);            //卡曼滤波
  //旋转角度Z轴参数
  if (gz > 32768) gz -= 65536;              //强制转换2g  1g
  Gyro_z = -gz / 131;                      //Z轴参数转换
  accelz = az / 16.4; 
  angleAx = atan2(ax, az) * 180 / PI; //计算与x轴夹角
  Gyro_y = -gy / 131.00; //计算角速度
  //一阶互补滤波
  angle6 = K1 * angleAx + (1 - K1) * (angle6 + Gyro_y * dt);
}


float angle, angle_dot;                                //平衡角度值
void Kalman_Filter(double angle_m, double gyro_m)
{
  angle += (gyro_m - q_bias) * dt;
  angle_err = angle_m - angle;
  Pdot[0] = Q_angle - P[0][1] - P[1][0];
  Pdot[1] = - P[1][1];
  Pdot[2] = - P[1][1];
  Pdot[3] = Q_gyro;
  P[0][0] += Pdot[0] * dt;
  P[0][1] += Pdot[1] * dt;
  P[1][0] += Pdot[2] * dt;
  P[1][1] += Pdot[3] * dt;
  PCt_0 = C_0 * P[0][0];
  PCt_1 = C_0 * P[1][0];
  E = R_angle + C_0 * PCt_0;
  K_0 = PCt_0 / E;
  K_1 = PCt_1 / E;
  t_0 = PCt_0;
  t_1 = C_0 * P[0][1];
  P[0][0] -= K_0 * t_0;
  P[0][1] -= K_0 * t_1;
  P[1][0] -= K_1 * t_0;
  P[1][1] -= K_1 * t_1;
  angle += K_0 * angle_err; //角度
  q_bias += K_1 * angle_err;
  angle_dot = gyro_m - q_bias; //角速度
}

void setup() {
  sign = 0;
  sign_pre = 0;
  test = 0;
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
  Serial.begin(9600);
  BlueTooth.begin(9600);
  accelgyro.initialize();
  pinMode(BUTTON,INPUT_PULLUP);
}

void loop() {
  for (uint16_t i = 0; i < 100; i++)
  {    
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    Angletest();                                      //获取angle 角度和卡曼滤波

      if(angle6 > 60)
      {
        sign = 1;
      }
      else if(angle6 < 60)
      {
        sign = 0;
      }
      if(sign == 1 && sign_pre == 0)
      {
          if(test == 0)
          {
              test = 1;
              Serial.println(sign, test);
              colorWipe(strip.Color(100, 0, 0), 1); // Red
          }
          else if(test == 1)
          {
              test = 2;
              Serial.println(sign, test);
              colorWipe(strip.Color(0, 100, 0), 1);// Green
              
          }
          else if(test == 2)
          {
              test = 0;
              Serial.println(sign, test);
              colorWipe(strip.Color(0, 0, 100), 1); // Blue
          }
       }
      sign_pre = sign;
      
     buttonState = digitalRead(BUTTON);
      if(buttonState == 0){
      test = 3;
      if(Serial.available()>0)
      {
          Serial.println(test);
          BlueTooth.write(byte(test));      
      }
      rainbowCycle(1);
    }
      
    if(Serial.available()>0)
    {
      if(i == 99)
      {
        Serial.println(test);
        BlueTooth.write(byte(test));
      }      
    }     
  }     
}

// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, c);
      strip.show();
      delay(wait);
  }
}

void rainbowCycle(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< strip.numPixels(); i++) {
    strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

uint32_t Wheel(byte WheelPos) {
  if(WheelPos < 85) {
   return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } else if(WheelPos < 170) {
   WheelPos -= 85;
   return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else {
   WheelPos -= 170;
   return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}
