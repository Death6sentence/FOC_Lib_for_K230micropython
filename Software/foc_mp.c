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

#include"py/obj.h"
#include"py/runtime.h"

#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define _3PI_2 4.71238898038f
#define PI 3.14159265359f

#define AS5600_I2C_ADDR 0x36
#define AS5600_ANGLE_REG 0x0C

#define PWM0_PIN     42
#define PWM1_PIN     46
#define PWM2_PIN     52
#define PWM0_CHANNEL PWM0
#define PWM1_CHANNEL PWM2
#define PWM2_CHANNEL PWM4

drv_i2c_inst_t* AS5600_i2c1 = NULL;
drv_hard_timer_inst_t* foc_timer_1 = NULL;
drv_gpio_inst_t* led_G = NULL;
drv_gpio_inst_t* foc_en_1 = NULL;

float voltage_power_supply;//电源电压
float Ualpha_1,Ubeta_1=0,Ua_1=0,Ub_1=0,Uc_1=0;

float zero_electric_angle_1=1.0f;
int PP=7,DIR=-1;

int32_t full_rotations=0; // full rotation tracking;
float angle_prev=0; 

int num;
float Kp=0.083;//位置环比例系数
int Sensor_DIR=-1;    //传感器方向
int Motor_PP=7;    //电机极对数
float motor_target = 0; //目标位置

void hard_timer_callback(void* args);

STATIC int AS5600_Init(drv_i2c_inst_t** i2c, fpioa_func_t scl_func, int scl_pin, fpioa_func_t sda_func, int sda_pin, int i2c_clock)
{
    if(drv_fpioa_set_pin_func(scl_pin, scl_func) == -1 || drv_fpioa_set_pin_func(sda_pin, sda_func) == -1)
    {
        printf("Failed to set fpioa pin function\n");
        return -1;
    }
    if(drv_i2c_inst_create(1, i2c_clock, 1000, 0xff, 0xff, i2c) == -1)
    {
        printf("Failed to create i2c instance\n");
        return -1;
    }
    return 0;
}

STATIC mp_obj_t mp_AS5600_Init(void)
{
    AS5600_Init(&AS5600_i2c1, IIC1_SCL, 34, IIC1_SDA, 35, 4000000);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(foc_AS5600_Init_obj, mp_AS5600_Init);

STATIC uint16_t AS5600_Get_Angle(drv_i2c_inst_t* i2c)
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

STATIC mp_obj_t mp_AS5600_Get_Angle(void)
{
    uint16_t angle = AS5600_Get_Angle(AS5600_i2c1);
    return mp_obj_new_int_from_uint(angle);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(foc_AS5600_Get_Angle_obj, mp_AS5600_Get_Angle);

// 归一化角度到 [0,2PI]
float _normalizeAngle(float angle)
{
  float a = fmod(angle, 2*PI);   //取余运算可以用于归一化，列出特殊值例子算便知
  return a >= 0 ? a : (a + 2*PI);  
  //三目运算符。格式：condition ? expr1 : expr2 
  //其中，condition 是要求值的条件表达式，如果条件成立，则返回 expr1 的值，否则返回 expr2 的值。可以将三目运算符视为 if-else 语句的简化形式。
  //fmod 函数的余数的符号与除数相同。因此，当 angle 的值为负数时，余数的符号将与 _2PI 的符号相反。也就是说，如果 angle 的值小于 0 且 _2PI 的值为正数，则 fmod(angle, _2PI) 的余数将为负数。
  //例如，当 angle 的值为 -PI/2，_2PI 的值为 2PI 时，fmod(angle, _2PI) 将返回一个负数。在这种情况下，可以通过将负数的余数加上 _2PI 来将角度归一化到 [0, 2PI] 的范围内，以确保角度的值始终为正数。
}

STATIC float getAngle_Without_track(drv_i2c_inst_t* i2c)
{
    float Angle = AS5600_Get_Angle(i2c)*0.08789* PI / 180;
    // printf("Angle: %f\n", Angle);
    return Angle;
}

float getAngle(drv_i2c_inst_t* i2c)
{
    float val = getAngle_Without_track(i2c);
    float d_angle = val - angle_prev;
    //计算旋转的总圈数
    //通过判断角度变化是否大于80%的一圈(0.8f*6.28318530718f)来判断是否发生了溢出，如果发生了，则将full_rotations增加1（如果d_angle小于0）或减少1（如果d_angle大于0）。
    if(abs(d_angle) > (0.8f*6.28318530718f) ) full_rotations += ( d_angle > 0 ) ? -1 : 1; 
    angle_prev = val;
    return (float)full_rotations * 6.28318530718f + angle_prev;
    
}

STATIC float _electricalAngle(float zero_electric_angle, drv_i2c_inst_t* i2c)
{
  return  _normalizeAngle((float)(DIR *  PP) * getAngle_Without_track(i2c)-zero_electric_angle);
}

// 设置PWM到控制器输出
STATIC void setPwm(float Ua, float Ub, float Uc) 
{

  // 计算占空比
  // 限制占空比从0到1
  float dc_a = _constrain(Ua / voltage_power_supply, 0.0f , 1.0f );
  float dc_b = _constrain(Ub / voltage_power_supply, 0.0f , 1.0f );
  float dc_c = _constrain(Uc / voltage_power_supply, 0.0f , 1.0f );

  // printf("占空比：%d %d %d\n", (int)(dc_a*100), (int)(dc_b*100), (int)(dc_c*100));
  //写入PWM到PWM 0 1 2 通道
    drv_pwm_set_duty(PWM0_CHANNEL - PWM0, (int)(dc_a*100));
    drv_pwm_set_duty(PWM1_CHANNEL - PWM0, (int)(dc_b*100));
    drv_pwm_set_duty(PWM2_CHANNEL - PWM0, (int)(dc_c*100));
}

STATIC void setTorque(float Uq,float angle_el, float *Ualpha, float *Ubeta, float *Ua, float *Ub, float *Uc) 
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
  setPwm(*Ua,*Ub,*Uc);
}

STATIC int Timer_Init(int timer_num, drv_hard_timer_inst_t** timer)
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

STATIC int GPIO_Init(void)
{
    drv_fpioa_set_pin_func(20, GPIO20);
    drv_gpio_inst_create(20, &led_G);
    drv_gpio_mode_set(led_G, GPIO_DM_OUTPUT);

    drv_fpioa_set_pin_func(40, GPIO40);
    drv_gpio_inst_create(40, &foc_en_1);
    drv_gpio_mode_set(foc_en_1, GPIO_DM_OUTPUT);
    drv_gpio_value_set(foc_en_1, GPIO_PV_LOW);
    return 0;
}

STATIC int FOC_PWM_Init(fpioa_func_t pwm0_func, int pwm0_pin,
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

STATIC void DFOC_Vbus(float power_supply)
{
  voltage_power_supply=power_supply;

}

STATIC mp_obj_t mp_DFOC_Init(void)
{
    DFOC_Vbus(12.6);   //设定驱动器供电电压
    GPIO_Init();
    AS5600_Init(&AS5600_i2c1, IIC1_SCL, 34, IIC1_SDA, 35, 4000000);
    FOC_PWM_Init(PWM0_CHANNEL, PWM0_PIN, PWM1_CHANNEL, PWM1_PIN, PWM2_CHANNEL, PWM2_PIN);
    Timer_Init(0, &foc_timer_1);
    mp_printf(&mp_plat_print, "FOC Module Initialized\n");
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(foc_DFOC_Init_obj, mp_DFOC_Init);

STATIC mp_obj_t mp_DFOC_Set_Target(mp_obj_t target_obj)
{
    motor_target = mp_obj_get_float(target_obj);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(foc_DFOC_Set_Target_obj, mp_DFOC_Set_Target);

STATIC const mp_rom_map_elem_t foc_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_foc) },
    { MP_ROM_QSTR(MP_QSTR_AS5600Init), MP_ROM_PTR(&foc_AS5600_Init_obj) },
    { MP_ROM_QSTR(MP_QSTR_AS5600GetAngle), MP_ROM_PTR(&foc_AS5600_Get_Angle_obj) },
    { MP_ROM_QSTR(MP_QSTR_DFOCInit), MP_ROM_PTR(&foc_DFOC_Init_obj) },
    { MP_ROM_QSTR(MP_QSTR_DFOCSetTarget), MP_ROM_PTR(&foc_DFOC_Set_Target_obj) },
};

STATIC MP_DEFINE_CONST_DICT(foc_globals_table, foc_module_globals_table);

const mp_obj_module_t mp_module_foc = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&foc_globals_table,
};

MP_REGISTER_MODULE(MP_QSTR_foc, mp_module_foc);

void hard_timer_callback(void* args) 
{
    static long int count = 0;
    count++;
    if(count >= 1000) // 每100次中断切换一次LED状态（1秒钟）
    {
        count = 0;
    }
    // Timer中断回调函数，可以在这里添加需要定时执行的代码
    float Sensor_Angle=getAngle(AS5600_i2c1);
    if(motor_target-Sensor_DIR*Sensor_Angle <= 0.01 && motor_target-Sensor_DIR*Sensor_Angle >= -0.01)
    {
      setPwm(0,0,0);
      drv_gpio_value_set(foc_en_1, GPIO_PV_LOW);
      drv_gpio_value_set(led_G, GPIO_PV_HIGH);       
    }
    else
    {
      setTorque(Kp*(motor_target-Sensor_DIR*Sensor_Angle)*180/PI,_electricalAngle(zero_electric_angle_1, AS5600_i2c1), &Ualpha_1, &Ubeta_1, &Ua_1, &Ub_1, &Uc_1);   //位置闭环
      drv_gpio_value_set(foc_en_1, GPIO_PV_HIGH);
      drv_gpio_value_set(led_G, GPIO_PV_LOW);
    }    
        
}