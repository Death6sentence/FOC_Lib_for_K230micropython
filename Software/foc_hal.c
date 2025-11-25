#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>

#include "drv_fpioa.h"
#include "drv_gpio.h"
#include "drv_i2c.h"
#include "drv_pwm.h"
#include "drv_timer.h"

#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define _3PI_2 4.71238898038f
#define PI 3.14159265359f

#define AS5600_I2C_ADDR 0x36
#define AS5600_ANGLE_REG 0x0C

#define PWM0_PIN_1     42
#define PWM1_PIN_1     46
#define PWM2_PIN_1     52
#define PWM0_CHANNEL_1 PWM0
#define PWM1_CHANNEL_1 PWM2
#define PWM2_CHANNEL_1 PWM4

#define PWM0_PIN_2     61
#define PWM1_PIN_2     47
#define PWM2_PIN_2     53
#define PWM0_CHANNEL_2 PWM1
#define PWM1_CHANNEL_2 PWM3
#define PWM2_CHANNEL_2 PWM5

drv_i2c_inst_t* AS5600_i2c_1 = NULL;
drv_i2c_inst_t* AS5600_i2c_2 = NULL;
drv_hard_timer_inst_t* timer = NULL;
drv_gpio_inst_t* led_G = NULL;
drv_gpio_inst_t* led_R = NULL;
drv_gpio_inst_t* foc_en_1 = NULL;
drv_gpio_inst_t* foc_en_2 = NULL;

float voltage_power_supply;//电源电压
float Ualpha_1,Ubeta_1=0,Ua_1=0,Ub_1=0,Uc_1=0;
float Ualpha_2,Ubeta_2=0,Ua_2=0,Ub_2=0,Uc_2=0;

float zero_electric_angle_1=1.0f;
float zero_electric_angle_2=1.0f;
int PP=7,DIR=-1;

int32_t full_rotations_1=0, full_rotations_2=0; // full rotation tracking;
float angle_prev_1=0, angle_prev_2=0; 

void hard_timer_callback(void* args);
float _normalizeAngle_180(float angle);

// 定义PID结构体
typedef struct {
    float Kp;  // 比例系数
    float Ki;  // 积分系数
    float Kd;  // 微分系数
    float setpoint;  // 设定值
    float integral;  // 积分项
    float prev_error;  // 上一次的误差
} PID;

// 初始化PID控制器
void PID_Init(PID *pid, float Kp, float Ki, float Kd, float setpoint) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->setpoint = setpoint;
    pid->integral = 0.0;
    pid->prev_error = 0.0;
}

// 计算PID输出
float PID_Compute(PID *pid, float current_value) {
    // 计算当前误差
    float Sensor_Angle_Error = (current_value - pid->setpoint)*180/PI;
    float error = _normalizeAngle_180(Sensor_Angle_Error);
    if(pid->prev_error > 0 && error < 0 || pid->prev_error < 0 && error > 0)
    {
        pid->integral = 0;
    }
    // 计算积分项
    pid->integral += error;

    // 计算微分项
    float derivative = error - pid->prev_error;

    // 计算PID输出
    float output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;

    // 更新上一次的误差
    pid->prev_error = error;

    if(output > voltage_power_supply/2) 
    {
        output = voltage_power_supply/2;
    }
    else if(output < -voltage_power_supply/2) 
    {
        output = -voltage_power_supply/2;
    }

    return output;
}

int Timer_Init(int timer_num, drv_hard_timer_inst_t** timer)
{
    rt_hwtimer_info_t info;
    uint32_t freq;
    if(drv_hard_timer_inst_create(timer_num, timer) == -1)
    {
        printf("Failed to create timer instance\n");
        return -1;
    }
    drv_hard_timer_stop(*timer);
    drv_hard_timer_set_mode(*timer, HWTIMER_MODE_PERIOD);
    drv_hard_timer_get_info(*timer, &info);
    uint32_t valid_freq = (info.minfreq + info.maxfreq) / 2;
    // uint32_t valid_freq = 20000000; // 20MHz
    drv_hard_timer_set_freq(*timer, valid_freq);
    drv_hard_timer_get_freq(*timer, &freq);
    printf("Timer frequency: %d Hz\n", freq);
    int ret = drv_hard_timer_set_period(*timer, 1);
    printf("Timer period: %d ms\n", 10);
    printf("Set period ret: %d\n", ret);
    drv_hard_timer_register_irq(*timer, hard_timer_callback, NULL);
    drv_hard_timer_start(*timer);
    printf("Timer initialized\n");
    return 0;
}

int AS5600_Init(drv_i2c_inst_t** i2c, fpioa_func_t scl_func, int scl_pin, fpioa_func_t sda_func, int sda_pin, int i2c_clock)
{
    int num = 0;
    switch(scl_func)
    {
    case IIC0_SCL:
        num = 0;
    break;
    case IIC1_SCL:
        num = 1;
    break;
    case IIC2_SCL:
        num = 2;
    break;
    case IIC3_SCL:
        num = 3;
    break;
    case IIC4_SCL:
        num = 4;
    break;
    default:
        printf("Invalid I2C number\n");
    return -1;
    }
    if(drv_fpioa_set_pin_func(scl_pin, scl_func) == -1 || drv_fpioa_set_pin_func(sda_pin, sda_func) == -1)
    {
        printf("Failed to set fpioa pin function\n");
        return -1;
    }
    if(drv_i2c_inst_create(num, i2c_clock, 1000, 0xff, 0xff, i2c) == -1)
    {
        printf("Failed to create i2c instance\n");
        return -1;
    }
    return 0;
}
uint16_t AS5600_Get_Angle(drv_i2c_inst_t* i2c)
{
    uint8_t write_buf[1] = {AS5600_ANGLE_REG};  // 要写入的寄存器地址
    uint8_t read_buf[2];  // 读取数据的缓冲区，最大 256 字节
    // 构造写消息，发送要读取的寄存器地址
    i2c_msg_t write_msg = {
        .addr = AS5600_I2C_ADDR,
        .flags = DRV_I2C_WR,
        .len = 1,
        .buf = write_buf
    };
    // 构造读消息，读取寄存器数据
    i2c_msg_t read_msg = {
        .addr = AS5600_I2C_ADDR,
        .flags = DRV_I2C_RD,
        .len = 2,
        .buf = read_buf
    };
    i2c_msg_t msgs[2] = {write_msg, read_msg};  // 消息数组
    drv_i2c_transfer(i2c, msgs, 2);  // 发送 I2C 消息
    uint16_t raw_angle = (read_buf[0] << 8) | read_buf[1];
    return raw_angle;
}

float getAngle_Without_track(drv_i2c_inst_t* i2c)
{
    float Angle = AS5600_Get_Angle(i2c)*0.08789* PI / 180;
    // printf("Angle: %f\n", Angle);
    return Angle;
}

int FOC_PWM_Init(fpioa_func_t pwm0_func, int pwm0_pin,
                 fpioa_func_t pwm1_func, int pwm1_pin,
                 fpioa_func_t pwm2_func, int pwm2_pin)
{
    int ret = 0;
    ret |= drv_fpioa_set_pin_func(pwm0_pin, pwm0_func);
    ret |= drv_fpioa_set_pin_func(pwm1_pin, pwm1_func);
    ret |= drv_fpioa_set_pin_func(pwm2_pin, pwm2_func);
    if(ret != 0)
    {
        printf("Failed to set pwm fpioa pin function\n");
        return -1;
    }
    drv_pwm_init();
    drv_pwm_set_freq(pwm0_func - PWM0, 100000);
    drv_pwm_set_freq(pwm1_func - PWM0, 100000);
    drv_pwm_set_freq(pwm2_func - PWM0, 100000);

    drv_pwm_set_duty(pwm0_func - PWM0, 0);
    drv_pwm_set_duty(pwm1_func - PWM0, 0);
    drv_pwm_set_duty(pwm2_func - PWM0, 0);

    drv_pwm_enable(pwm0_func - PWM0);
    drv_pwm_enable(pwm1_func - PWM0);
    drv_pwm_enable(pwm2_func - PWM0);
    return 0;
}

// 归一化角度到 [0,2PI]
float _normalizeAngle(float angle)
{
  float a = fmod(angle, 2*PI);   //取余运算可以用于归一化，列出特殊值例子算便知
  return a >= 0 ? a : (a + 2*PI);  
}

// 将角度转换到 -180 到 +180 度的函数
float _normalizeAngle_180(float angle) 
{
    while (angle > 180.0) {
        angle -= 360.0;
    }
    while (angle < -180.0) {
        angle += 360.0;
    }
    return angle;
}

// 设置PWM到控制器输出
void setPwm(float Ua, float Ub, float Uc, int PWM0_CHANNEL, int PWM1_CHANNEL, int PWM2_CHANNEL) 
{

    // 限制占空比从0到1
    float dc_a = _constrain(Ua / voltage_power_supply, 0.0f , 1.0f );
    float dc_b = _constrain(Ub / voltage_power_supply, 0.0f , 1.0f );
    float dc_c = _constrain(Uc / voltage_power_supply, 0.0f , 1.0f );

    //写入PWM到PWM 0 1 2 通道
    drv_pwm_set_duty(PWM0_CHANNEL - PWM0, (int)(dc_a*100));
    drv_pwm_set_duty(PWM1_CHANNEL - PWM0, (int)(dc_b*100));
    drv_pwm_set_duty(PWM2_CHANNEL - PWM0, (int)(dc_c*100));
}

void setTorque(float Uq,float angle_el, float *Ualpha, float *Ubeta, float *Ua, float *Ub, float *Uc, int PWM0_CHANNEL, int PWM1_CHANNEL, int PWM2_CHANNEL) 
{
  Uq=_constrain(Uq,-voltage_power_supply/2,voltage_power_supply/2);
//   float Ud=0;
  angle_el = _normalizeAngle(angle_el);
  // 帕克逆变换
  *Ualpha =  -Uq*sin(angle_el); 
  *Ubeta =   Uq*cos(angle_el); 

  // 克拉克逆变换
  *Ua = *Ualpha + voltage_power_supply/2;
  *Ub = (sqrt(3)*(*Ubeta)-(*Ualpha))/2 + voltage_power_supply/2;
  *Uc = (-(*Ualpha)-sqrt(3)*(*Ubeta))/2 + voltage_power_supply/2;
  setPwm(*Ua,*Ub,*Uc, PWM0_CHANNEL, PWM1_CHANNEL, PWM2_CHANNEL);
}

void DFOC_Vbus(float power_supply)
{
  voltage_power_supply=power_supply;
}

float _electricalAngle(float zero_electric_angle, drv_i2c_inst_t* i2c)
{
  return  _normalizeAngle((float)(DIR *  PP) * getAngle_Without_track(i2c)-zero_electric_angle);
}

void DFOC_alignSensor(drv_i2c_inst_t* i2c, int _PP, int _DIR, float *zero_electric_angle, float *Ualpha, float *Ubeta, float *Ua, float *Ub, float *Uc, int PWM0_CHANNEL, int PWM1_CHANNEL, int PWM2_CHANNEL)
{ 
  PP=_PP;
  DIR=_DIR;
  setTorque(3, _3PI_2, Ualpha, Ubeta, Ua, Ub, Uc, PWM0_CHANNEL, PWM1_CHANNEL, PWM2_CHANNEL);
  sleep(1);
  *zero_electric_angle=_electricalAngle(0.0f, i2c);
  setTorque(0, _3PI_2, Ualpha, Ubeta, Ua, Ub, Uc, PWM0_CHANNEL, PWM1_CHANNEL, PWM2_CHANNEL);
  printf("0电角度：%f\n", *zero_electric_angle);
}

float getAngle(drv_i2c_inst_t* i2c, int32_t *full_rotations,float *angle_prev)
{
    float val = getAngle_Without_track(i2c);
    float d_angle = val - *angle_prev;
    //计算旋转的总圈数
    //通过判断角度变化是否大于80%的一圈(0.8f*6.28318530718f)来判断是否发生了溢出，如果发生了，则将full_rotations增加1（如果d_angle小于0）或减少1（如果d_angle大于0）。
    if(abs(d_angle) > (0.8f*6.28318530718f) ) *full_rotations += ( d_angle > 0 ) ? -1 : 1; 
    *angle_prev = val;
    return (float)*full_rotations * 6.28318530718f + *angle_prev;
    
}

// float DFOC_M0_Angle(drv_i2c_inst_t* i2c)
// {
//   return getAngle(i2c);
// }

int num[2];
float Kp=0.05;//位置环比例系数
int Sensor_DIR=1;    //传感器方向
int Motor_PP=7;    //电机极对数
float motor_target_1, motor_target_2; //目标位置
PID pid1, pid2;
int main(int argc, char *argv[])
{
    
    if (argc < 2) 
    {
        // 检查是否有命令行参数传入，如果没有则提示用户并退出程序
        printf("请至少传入一个整数参数。\n");
        return 1;
    }

    for (int i = 1; i < argc; i++) 
    {
        // 使用 atoi 函数将字符串参数转换为整数
        num[i - 1] = atoi(argv[i]);
        printf("第 %d 个参数转换后的整数是: %d\n", i, num);
    }
    motor_target_1 = (float)num[0];
    motor_target_2 = (float)num[1];
    
    //led test
    drv_fpioa_set_pin_func(20, GPIO20);
    drv_gpio_inst_create(20, &led_G);
    drv_gpio_mode_set(led_G, GPIO_DM_OUTPUT);

    drv_fpioa_set_pin_func(62, GPIO62);
    drv_gpio_inst_create(62, &led_R);
    drv_gpio_mode_set(led_R, GPIO_DM_OUTPUT);


    drv_fpioa_set_pin_func(40, GPIO40);
    drv_gpio_inst_create(40, &foc_en_1);
    drv_gpio_mode_set(foc_en_1, GPIO_DM_OUTPUT);
    drv_gpio_value_set(foc_en_1, GPIO_PV_HIGH);

    drv_fpioa_set_pin_func(04, GPIO4);
    drv_gpio_inst_create(04, &foc_en_2);
    drv_gpio_mode_set(foc_en_2, GPIO_DM_OUTPUT);
    drv_gpio_value_set(foc_en_2, GPIO_PV_HIGH);
    //led test

    
    AS5600_Init(&AS5600_i2c_1, IIC1_SCL, 34, IIC1_SDA, 35, 4000000);
    AS5600_Init(&AS5600_i2c_2, IIC0_SCL, 48, IIC0_SDA, 49, 4000000);
    FOC_PWM_Init(PWM0_CHANNEL_1, PWM0_PIN_1, PWM1_CHANNEL_1, PWM1_PIN_1, PWM2_CHANNEL_1, PWM2_PIN_1);
    FOC_PWM_Init(PWM0_CHANNEL_2, PWM0_PIN_2, PWM1_CHANNEL_2, PWM1_PIN_2, PWM2_CHANNEL_2, PWM2_PIN_2);

    DFOC_Vbus(12.6);   //设定驱动器供电电压
    DFOC_alignSensor(AS5600_i2c_1, 7, -1, &zero_electric_angle_1, &Ualpha_1, &Ubeta_1, &Ua_1, &Ub_1, &Uc_1, PWM0_CHANNEL_1, PWM1_CHANNEL_1, PWM2_CHANNEL_1);
    DFOC_alignSensor(AS5600_i2c_2, 7, -1, &zero_electric_angle_2, &Ualpha_2, &Ubeta_2, &Ua_2, &Ub_2, &Uc_2, PWM0_CHANNEL_2, PWM1_CHANNEL_2, PWM2_CHANNEL_2);
    PID_Init(&pid1, 0.133, 0.0000, 0.0000, motor_target_1);
    PID_Init(&pid2, 0.133, 0.0000, 0, motor_target_2);
    printf("FOC_Init\n");
    while(1)
    {
        hard_timer_callback(NULL);
    }
}



void hard_timer_callback(void* args) 
{
    static long int count = 0;
    // Timer中断回调函数，可以在这里添加需要定时执行的代码
    float Sensor_Angle_1=getAngle_Without_track(AS5600_i2c_1);
    float Sensor_Angle_2=getAngle_Without_track(AS5600_i2c_2);
    float Sensor_Angle_Error_1 = (Sensor_Angle_1 - motor_target_1)*180/PI;
    float Sensor_Angle_Error_2 = (Sensor_Angle_2 - motor_target_2)*180/PI;
    float Motor_Output_1;
    float Motor_Output_2;
    if(Sensor_Angle_Error_1 <= 20 && Sensor_Angle_Error_1 >= -20)
    {
      drv_gpio_value_set(led_G, GPIO_PV_HIGH);
    }
    else
    {
        drv_gpio_value_set(led_G, GPIO_PV_LOW);
    }
    
    Motor_Output_1 = PID_Compute(&pid1, Sensor_Angle_1);
    setTorque(Motor_Output_1,_electricalAngle(zero_electric_angle_1, AS5600_i2c_1), &Ualpha_1, &Ubeta_1, &Ua_1, &Ub_1, &Uc_1, PWM0_CHANNEL_1, PWM1_CHANNEL_1, PWM2_CHANNEL_1);   //位置闭环
      

    if(Sensor_Angle_Error_2 <= 20 && Sensor_Angle_Error_2 >= -20)
    {

      drv_gpio_value_set(led_R, GPIO_PV_HIGH);
    }
    else
    {
        drv_gpio_value_set(led_R, GPIO_PV_LOW);
    }
    
    Motor_Output_2 = PID_Compute(&pid2, Sensor_Angle_2);
    setTorque(Motor_Output_2,_electricalAngle(zero_electric_angle_2, AS5600_i2c_2), &Ualpha_2, &Ubeta_2, &Ua_2, &Ub_2, &Uc_2, PWM0_CHANNEL_2, PWM1_CHANNEL_2, PWM2_CHANNEL_2);   //位置闭环
    
        
}