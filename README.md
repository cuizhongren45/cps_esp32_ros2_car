# cps_esp32_ros2_car

esp32做主控，控制simplefoc电机，采集mpu6050，as5600编码器，和ros2-foxy通信

bilibilib视频地址：https://www.bilibili.com/video/BV1CM4y1c7eE?spm_id_from=333.999.0.0

谢谢，欢迎交流。

### **一、开发环境：**

基于灯哥开源的simplefoc的电机闭环程序

所以需要先安装灯哥simplefoc的开发环境

arduino+simplefoc+esp32

参考链接如下：

https://github.com/cuizhongren45/Deng-s-foc-controller

### 二、MPU6050的库

将MPU6050_tockn文件夹放在aduino目录下/libraries中，算是引入库文件

### 三、main

 将main.c的代码复制到自己的arduino中，编译烧录即可。

### 四、接线

代码里都有备注，两个as5600就按照灯哥v3的版本里接线方式接线

MPU6050接线SDA---I_0       SCL---I_1
