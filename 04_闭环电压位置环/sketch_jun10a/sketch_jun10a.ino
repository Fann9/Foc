#include "AS5600.h"

// 定义三相输出
int pwmA = 32;
int pwmB = 33; 
int pwmC = 25;

// 编码器对象
Sensor_AS5600 sensor(0);

/**
 * @brief 输入限幅
 * @param amt - 输入
 * @param low - 最小值
 * @param high - 最大值
*/
#define _constrain(amt, low, high) ((amt)<(low) ? (low) : ((amt) > (high) ? (high) : (amt)))

float voltage_limit = 10;
float voltage_power_supply = 12.3;  // 输入电压 
float shaft_angle = 0, open_loop_timestamp = 0;

float zero_electric_angle = 0;
float Ualpha = 0, Ubeta = 0;
float Ua = 0, Ub = 0, Uc = 0;
float dc_a = 0, dc_b = 0, dc_c = 0;

int PR = 7;
int DIR = -1;

/**
 * @brief 角度归一化
 * @note 限制在[0, 2π]
*/
float _normalizeAngle(float angle)
{
    float a = fmod(angle, 2*PI);    // 取余
    return a >= 0 ? a : (a + 2*PI);
}

/**
 * @brief 电角度求解函数
 * @param _sensor - 编码器对象
 * @return 电角度
 * @note _electricalAngle(sensor);
*/
float _electricalAngle(Sensor_AS5600 &_sensor)
{
    return _normalizeAngle((float)(DIR * PR) * _sensor.getMechanicalAngle() - zero_electric_angle);  // 电角度 = 机械角度 * 极对数
}

/**
 * @brief 设置pwm到控制器输出
*/
void setPWM(float Ua, float Ub, float Uc)
{
    // 调用限幅
    Ua = _constrain(Ua, 0.0f, voltage_limit);
    Ub = _constrain(Ub, 0.0f, voltage_limit);
    Uc = _constrain(Uc, 0.0f, voltage_limit);

    // 归一计算占空比
    dc_a = _constrain(Ua / voltage_power_supply, 0.0f, 1.0f);
    dc_b = _constrain(Ub / voltage_power_supply, 0.0f, 1.0f);
    dc_c = _constrain(Uc / voltage_power_supply, 0.0f, 1.0f);

    // 写入占空比
    ledcWrite(pwmA, dc_a * 255);
    ledcWrite(pwmB, dc_b * 255);
    ledcWrite(pwmC, dc_c * 255);
}

/**
 * @brief 将控制量转为三相pwm电压驱动电机
 * @param Uq - 力矩环(电压)
 * @param Ud - D轴垂直于磁场, 这里默认为0传入
 * @param angle_el - 电角度
*/
void setPhaseVoltage(float Uq, float Ud, float angle_el)
{
    angle_el = _normalizeAngle(angle_el);

    // 帕克逆变换
    Ualpha = -Uq*sin(angle_el);
    Ubeta = Uq*cos(angle_el);

    // 克拉克逆变换
    Ua = Ualpha + voltage_power_supply/2;
    Ub = (sqrt(3)*Ubeta-Ualpha)/2 + voltage_power_supply/2;
    Uc = (-Ualpha-sqrt(3)*Ubeta)/2 + voltage_power_supply/2;
    setPWM(Ua, Ub, Uc);
}

void setup() {
    // put your setup code here, to run once:
    Serial.begin(115200);

    // 初始化引脚
    pinMode(pwmA, OUTPUT);
    pinMode(pwmB, OUTPUT);
    pinMode(pwmC, OUTPUT);

    // 配置PWM
    ledcAttach(pwmA, 30000, 8);
    ledcAttach(pwmB, 30000, 8);
    ledcAttach(pwmC, 30000, 8);

    // 启动电机
    pinMode(12, OUTPUT);
    digitalWrite(12, HIGH);

    // 初始化编码器 (SDA=19, SCL=18)
    Wire.begin(19, 18);
    sensor.Sensor_init(&Wire);
    
    setPhaseVoltage(3, 0, 3*PI/2);
    delay(3000);
    zero_electric_angle =_electricalAngle(sensor);
    setPhaseVoltage(0, 0, 3*PI/2);

    Serial.print("0电角度:");Serial.println(zero_electric_angle);
}

float motor_target = PI;
float Kp = 0.133;

void loop() {
    // put your main code here, to run repeatedly:
    sensor.Sensor_update();
    Serial.println(sensor.getAngle());
    float Sensor_Angle = sensor.getAngle();
    
    setPhaseVoltage(_constrain(Kp * (motor_target - DIR*Sensor_Angle)*180/PI, -6, 6), 0, _electricalAngle(sensor));
}
