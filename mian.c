
#include <SimpleFOC.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <MPU6050_tockn.h>

//------------------IIC编码器注释---------------
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);
TwoWire I2Cone = TwoWire(0);
TwoWire I2Ctwo = TwoWire(1);
//SoftwareSerial BT(25,26);
//unsigned char comdata[60];
signed short int mpu_data[12];  //mpu6050的数据，陀螺仪，加速度，姿态角
signed short int ax_encoder_delta[2]; //编码器累加值
signed short int ax_encoder_delta_target[2] = {0}; //编码器变化值
signed short int robot_odom[6] = {0}; //里程计数据，绝对值和变化值
unsigned short int ax_bat_vol = 1100; //电池电压
//unsigned char comdata[60];
signed short int robot_target_speed[3];
int res;
int count = 0;
int rev_buf[12];
int num;
unsigned char check_sum_r;
float v1, v0;
long timer = 0;
float dis = 0.2;//两轮之间的距离，单位是m，这里暂设为0.15，后续要进行修改的
float r = 0.032;
//电机参数
BLDCMotor motor = BLDCMotor(14);
BLDCDriver3PWM driver = BLDCDriver3PWM(32, 33, 25, 22);

BLDCMotor motor1 = BLDCMotor(14);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(26, 27, 14, 12);

MPU6050 mpu6050(Wire);
//命令设置
float target_velocity = 0;
Commander command = Commander(Serial);
void doTarget(char* cmd) {
  command.scalar(&target_velocity, cmd);
}

//-------------使用BT这个映射的串口发送数据，左轮目标速度、右轮目标速度  rad/s
//例子：AA 55 2D 06 F5 80 0E 1C 36 18 FF 07 00 1D FF E0 05 81 04 43 00 11 01 92 00 00 00 00 00 00 00 00 00 00 00 00 00 00 04 07 03 C8 04 4C 2C
//例子：AA 55 2D 06 F8 1C 0F F4 35 D0 FE FF 00 1A FF DA 06 56 03 12 04 33 01 92 00 00 00 00 00 00 00 00 00 00 00 00 00 00 72 C7 0D E0 04 4C 63
void send_data()
{
  static unsigned char comdata[60];
  unsigned char check_num ;
  check_num = 0x00;
  //陀螺仪数据= (ax_gyro/32768) * 2000 ?s
  comdata[0] = (unsigned char)( mpu_data[0] >> 8 );
  comdata[1] = (unsigned char)( mpu_data[0] );
  comdata[2] = (unsigned char)( mpu_data[1] >> 8 );
  comdata[3] = (unsigned char)( mpu_data[1] );
  comdata[4] = (unsigned char)( mpu_data[2] >> 8 );
  comdata[5] = (unsigned char)( mpu_data[2] );
  //加速度 = (ax_acc/32768) * 2G
  comdata[6] = (unsigned char)( mpu_data[3] >> 8 );
  comdata[7] = (unsigned char)( mpu_data[3] );
  comdata[8] = (unsigned char)( mpu_data[4] >> 8 );
  comdata[9] = (unsigned char)( mpu_data[4] );
  comdata[10] = (unsigned char)( mpu_data[5] >> 8 );
  comdata[11] = (unsigned char)( mpu_data[5] );

  //姿态角度 = (ax_angle/100)
  comdata[12] = (unsigned char)( mpu_data[6] >> 8 );
  comdata[13] = (unsigned char)( mpu_data[6] );
  comdata[14] = (unsigned char)( mpu_data[7] >> 8 );
  comdata[15] = (unsigned char)( mpu_data[7] );
  comdata[16] = (unsigned char)( mpu_data[8] >> 8 );
  comdata[17] = (unsigned char)( mpu_data[8] );
  //里程计坐标 x(m) y(m) yaw(rad)  odom_frame
  comdata[18] = (unsigned char)( robot_odom[0] >> 8 );
  comdata[19] = (unsigned char)( robot_odom[0] );
  comdata[20] = (unsigned char)( robot_odom[1] >> 8 );
  comdata[21] = (unsigned char)( robot_odom[1] );
  comdata[22] = (unsigned char)( robot_odom[2] >> 8 );
  comdata[23] = (unsigned char)( robot_odom[2] );

  //里程计坐标变化量  d_x(m) d_y(m) d_yaw(rad)  base_frame
  comdata[24] = (unsigned char)( robot_odom[3] >> 8 );
  comdata[25] = (unsigned char)( robot_odom[3] );
  comdata[26] = (unsigned char)( robot_odom[4] >> 8 );
  comdata[27] = (unsigned char)( robot_odom[4] );
  comdata[28] = (unsigned char)( robot_odom[5] >> 8 );
  comdata[29] = (unsigned char)( robot_odom[5] );
  //编码器当前值和目标值
  comdata[30] = (unsigned char)( ax_encoder_delta[0] >> 8 );
  comdata[31] = (unsigned char)( ax_encoder_delta[0] );
  comdata[32] = (unsigned char)( ax_encoder_delta[1] >> 8 );
  comdata[33] = (unsigned char)( ax_encoder_delta[1] );


  comdata[34] = (unsigned char)( ax_encoder_delta_target[0] >> 8 );
  comdata[35] = (unsigned char)( ax_encoder_delta_target[0] );
  comdata[36] = (unsigned char)( ax_encoder_delta_target[1] >> 8 );
  comdata[37] = (unsigned char)( ax_encoder_delta_target[1] );

  //电池电压
  comdata[38] = (unsigned char)( ax_bat_vol >> 8 );
  comdata[39] = (unsigned char)( ax_bat_vol );

  Serial.write(0xaa);
  Serial.write(0x55);
  Serial.write(0x2D);  //共45个字节
  Serial.write(0x06);
  Serial.write(comdata, 40);
  //测试 AA 55 2D 06 00 9B 36 D1 6C 07 A2 3D D8 73 0E A9 44 DF 7A 15 B0 4B E6 81 1C B7 52 ED 88 23 BE 59 F4 8F 2A C5 60 FB 96 31 CC 67 02 9D F6
  //测试 AA 55 2D 06 00 87 0E 95 1C A3 2A B1 38 BF 46 CD 54 DB 62 E9 70 F7 7E 05 8C 13 9A 21 A8 2F B6 3D C4 4B D2 59 E0 67 EE 75 FC 83 0A 91 06
  //测试 AA 55 2D 06 01 01 01 01 01 01 01 01 01 01 01 01 01 01 01 01 01 01 01 01 01 01 01 01 01 01 01 01 01 01 01 01 01 01 01 01 01 01 01 01 5A
  for (int i = 0; i < 40; i++)
  {
    //comdata[i] = 0x05; //测试所用
    check_num = comdata[i] + check_num;
  }
  check_num = check_num + 0xaa + 0x55 + 0x2D + 0x06;
  Serial.write(check_num);
}
//
//-------------使用BT这个映射的串口接收数据 ，左轮目标速度、右轮目标速度  rad/s
void revi_data(signed short int *robot_target_speed)
{
  if (Serial.available() > 0)
  {
    res = Serial.read();
    switch (count)
    {
      case 0:
        if (res == 0xaa)
        {
          count++;
        }
        else
        {
          count = 0;
        }
        break;
      case 1:
        if (res == 0x55)
        {
          count++;
        }
        else
        {
          count = 0;
        }
        break;
      case 2:
        if (res == 0x0b)
        {
          count++;
        }
        else
        {
          count = 0;
        }
        break;
      case 3:
        if (res == 0x11)
        {
          count++;
        }
        else
        {
          count = 0;
        }
        break;
      case 4:
        rev_buf[0] = res;
        count++;
        break;
      case 5:
        rev_buf[1] = res;
        count++;
        break;
      case 6:
        rev_buf[2] = res;
        count++;
        break;
      case 7:
        rev_buf[3] = res;
        count++;
        break;
      case 8:
        rev_buf[4] = res;
        count++;
        break;
      case 9:
        rev_buf[5] = res;
        count++;
        break;
      case 10:
        rev_buf[6] = res;
        check_sum_r = (0xaa + 0x55 + 0x0b + 0x11 + rev_buf[0] + rev_buf[1] + rev_buf[2] + rev_buf[3] + rev_buf[4] + rev_buf[5]);
        if (rev_buf[6] == check_sum_r)
        {
          robot_target_speed[0] = (int16_t)((rev_buf[0] << 8) | rev_buf[1]);
          robot_target_speed[1] = (int16_t)((rev_buf[2] << 8) | rev_buf[3]);
          robot_target_speed[2] = (int16_t)((rev_buf[4] << 8) | rev_buf[5]);
          //              BT.print(robot_target_speed[0]);
          //              BT.print(" ");
          //              BT.print(robot_target_speed[1]);
          //              BT.print(" ");
          //              BT.println(robot_target_speed[2]);
          Serial.flush(); //清除缓冲
        }
        count = 0;
        break;
    }
  }
}
//
//-----函数由x,y,yaw解算出左右两轮的速度,
//-----输入数据：x,y,yaw是单位为m/s的里程计数据,含有小数的,dis是两轮之间的距离，单位是m
//-----输出数据：v0,v1 是解算后的传入两个无刷电机的速度，单位是rad/s，这里要单位的解算
//vr,vl是轮子的线速度m/s，v0,v1是电机的角速度，rad/s,两者的转换关系是vr = v0 * R,R是电机轮子的半径
void motor_vel_control(signed short int *robot_target_speed, float v0, float v1, float dis, float r, signed short *ax_encoder_delta_target)
{
  float x, y, yaw;
  x = (float)(robot_target_speed[0]) / 1000;
  y = float(robot_target_speed[1]) / 1000;
  yaw = (float)(robot_target_speed[2]) / 1000;
  //    Serial.print(x);
  //    Serial.print(" ");
  //    Serial.print(y);
  //    Serial.print(" ");
  //    Serial.println(yaw);

  v0 = (x + dis * yaw / 2) / r; //0.032是电机轮子的半径，在线速度和角速度之间转化的，/s
  v1 = (x - dis * yaw / 2) / r; //直接可以传入simplefoc的控制器中。
  ax_encoder_delta_target[0] = (signed short int)(v0 * 1000); //注意ax_encoder_delta_target的数据类型
  ax_encoder_delta_target[1] = (signed short int)(v1 * 1000);
  //    Serial.print(ax_encoder_delta_target[0]);
  //    Serial.print(" ");
  //    Serial.print(ax_encoder_delta_target[1]);
  motor.loopFOC();
  motor1.loopFOC();
  motor.move(v0);
  motor1.move(v1);
  //  Serial.print(" ");
  //  Serial.print(v0);
  //  Serial.print("  ");
  //  Serial.println(v1);
}


//----------------MPU6050的初始化---
void MPU6050_init()
{
  Wire.begin(15, 13);   //这个的的端口需要改变一下，不一定是这个io口，SDA SDL for esp32
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
}
//------用于获取MPU6050的数据，输出就是mpu_data这个数组-----
void Get_MPU_data(signed short *mpu_data)
{
  mpu6050.update();
  //Serial.print("temp : ");Serial.println(mpu6050.getTemp()); //读取温度，用处不大。
  //Serial.print("accX : ");Serial.println(mpu6050.getAccX());
  mpu_data[0] = (mpu6050.getRawAccX());
  mpu_data[1] = (mpu6050.getRawAccY());
  mpu_data[2] = (mpu6050.getRawAccZ());
  mpu_data[3] = ((mpu6050.getRawGyroX ()));
  mpu_data[4] = ((mpu6050.getRawGyroY ()));
  mpu_data[5] = ((mpu6050.getRawGyroZ ()));
  mpu_data[6] = (mpu6050.getAngleX() * 100);
  mpu_data[7] = (mpu6050.getAngleY() * 100);
  mpu_data[8] = (mpu6050.getAngleZ() * 100);
}

//-----------用于获得电机的里程计的数据并控制电机-----------
//输出就是robot_odom[6]这个数组，x,y,yaw和dx，dy,dyaw
void get_odom_data(signed short *robot_odom, float r, float dis)
{
  robot_odom[0] = (sensor.getAngle() + sensor1.getAngle()) * r / 2 * 1000;
  robot_odom[1] = 0;
  robot_odom[2] = (sensor.getAngle() - sensor1.getAngle()) * r / dis * 1000;
  robot_odom[3] = (sensor.getVelocity() + sensor1.getVelocity()) * r / 2 * 1000;
  robot_odom[4] = 0;
  robot_odom[5] = (sensor.getVelocity() - sensor1.getVelocity()) * r / dis * 1000;
}
//获取电机编码器的现在状态的速度。 m/s
void get_encoder_data(signed short *ax_encoder_delta, float r)
{
  ax_encoder_delta[0] = sensor.getVelocity() * r * 1000;
  ax_encoder_delta[1] = sensor1.getVelocity() * r * 1000;
}
void setup() {

 
  MPU6050_init();
  delay(1000);
  //------------------IIC编码器注释---------------
  I2Cone.begin(19, 18, 400000);
  I2Ctwo.begin(23, 5, 400000);
  sensor.init(&I2Cone);
  sensor1.init(&I2Ctwo);

  //  // 初始化磁性传感器硬件
  //  sensor.init();
  ////  // 以阻塞（非中断）方式使用传感器，请注释掉此行
  //  sensor.enableInterrupt(doPWM);
  //连接motor对象与传感器对象
  motor.linkSensor(&sensor);
  motor1.linkSensor(&sensor1);

  //供电电压设置 [V]
  driver.voltage_power_supply = 12;
  driver.init();

  driver1.voltage_power_supply = 12;
  driver1.init();
  //连接电机和driver对象
  motor.linkDriver(&driver);
  motor1.linkDriver(&driver1);

  //FOC模型选择
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor1.foc_modulation = FOCModulationType::SpaceVectorPWM;
  //运动控制模式设置
  motor.controller = MotionControlType::velocity;
  motor1.controller = MotionControlType::velocity;


  //速度PI环设置
  motor.PID_velocity.P = 0.2;
  motor1.PID_velocity.P = 0.2;
  motor.PID_velocity.I = 20;
  motor1.PID_velocity.I = 20;
  //最大电机限制电机
  motor.voltage_limit =  16.8;
  motor1.voltage_limit = 16.8;

  //速度低通滤波时间常数
  motor.LPF_velocity.Tf = 0.01;
  motor1.LPF_velocity.Tf = 0.01;

  //设置最大速度限制
  motor.velocity_limit = 40;
  motor1.velocity_limit = 40;

  Serial.begin(115200);
  //BT.begin(9600);
  motor.useMonitoring(Serial);
  motor1.useMonitoring(Serial);

  //初始化电机
  motor.init();
  motor1.init();
  //初始化 FOC
  motor.initFOC();
  motor1.initFOC();
  command.add('T', doTarget, "target velocity");

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target velocity using serial terminal:"));
 
}



void loop() {

  revi_data(robot_target_speed);   //接收数据
  //              BT.print(robot_target_speed[0]);
  //              BT.print(" ");
  //              BT.print(robot_target_speed[1]);
  //              BT.print(" ");
  //              BT.println(robot_target_speed[2]);
    if(millis() - timer > 20)  //20个计量单位更新一次
    {
      Get_MPU_data(mpu_data);  //更新IMU数据
      get_odom_data(robot_odom,r,dis);  //更新odom数据
      get_encoder_data(ax_encoder_delta,r);  //编码器数据
      send_data();   //发送数据包。
      timer = millis();
      }
  motor_vel_control(robot_target_speed, v0, v1, dis, r, ax_encoder_delta_target);
  //  motor.loopFOC();
  //  motor1.loopFOC();
  //
  //  motor.move(0.67);
  //  motor1.move(4);
  //  motor.loopFOC();
  //  motor1.loopFOC();
  //  revi_data(x,y,yaw);
  //  calculate_v(x,y,yaw,v0,v1,dis);


  //  motor.move(target_velocity);  //这里的target_velcocity单位是rad/s，前面里程计x,y,yaw的单位是mm/s ,m/s扩大了1000倍的。
  //  motor1.move(target_velocity);
  //  Serial.print(sensor.getAngle());
  //  Serial.print("\t");
  //  Serial.println(sensor.getVelocity());
  //  command.run();//换种方式传输数据的话，这句command.add应该是不需要的，后续可以测试一下的。
}
