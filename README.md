# TIVA_FlightController
基于德州仪器(TI)的TM4C123G单片机的飞控，目前已实现定高模式和姿态模式，姿态解算和控制移植的匿名的领航者，匿名的地面站全兼容可在线调参。
## 硬件
IMU       ====>> ICM20602 IST8310 
Barometer ====>> MS5611
Sonar     ====>> US100 & KS103
## 供电
5vBuck电路====>> TPS62163
3.3vLDO   ====>> TPS79333
3.0vLDO   ====>> TPS79330
飞控可由电池直接供电或者由电调5V输入供电
## 无线数传&遥控器链接
可用大疆DR16接收机，固件中有D-BUS解析功能，或者可焊接板载NRF24L01做遥控器通信，板载一路串口接无线数传，默认波特率115200，地面端链接PC打开匿名地面站就可以调参
## 接插件规格
接插件均是SMT1.25规格的贴片母座
PWM输出以及D-BUS捕获接口采用侧卧直插2.52排针(电调接口标准接插件)
## PCB
![FC](https://github.com/ZhiyangZhou24/TIVA_FlightController/blob/master/Picture/FC.png "FC")
![PCB_F](https://github.com/ZhiyangZhou24/TIVA_FlightController/blob/master/Picture/PCB_2.jpg "PCBF")
![PCB_B](https://github.com/ZhiyangZhou24/TIVA_FlightController/blob/master/Picture/PCB_1.jpg "PCB_B")
![ASSM](https://github.com/ZhiyangZhou24/TIVA_FlightController/blob/master/Picture/assembled.jpg "ASSM")
![PCB_B](https://github.com/ZhiyangZhou24/TIVA_FlightController/blob/master/Picture/PCB_1.jpg "PCB_B")
## 最后
如有疑问，企鹅号778733609

