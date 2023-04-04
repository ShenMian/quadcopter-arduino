#include <SPI.h>
#include <Servo.h>
#include <RF24.h>
#include <nRF24L01.h>
#include <JY901.h>
#include <limits.h>

#define QUAD_PLUS 0x01 ///< 十字形模式, QUAD +
#define QUAD_X    0x02 ///< X型模式

#define QUAD_TYPE QUAD_X
#if QUAD_TYPE != QUAD_PLUS && QUAD_TYPE != QUAD_X
#error Unsupported QUAD type
#endif

/**
 * 机体坐标系:
 *
 *        z
 *      x ↑
 *       \|
 * y ←----+
 * 
 * 坐标系原点为飞行器重心, 飞行器正前方为 X 轴正方向, 正左方为 Y 轴正方向, 正上方为 Z 轴正方向
 */

enum Pin : uint8_t
{
  /**
  * (FL) (FR)
  *    \ /
  *     X
  *    / \
  * (BL) (BR)
  *
  * 电机 FL & BR 顺时针旋转.
  * 电机 FR & BL 逆顺时针旋转.
  */
  Motor_FR = 5,
  Motor_FL = 6,
  Motor_BR = 9,
  Motor_BL = 3,

  /**
  *      (F)
  *       |
  * (L)---o---(R)
  *       |
  *      (B)
  *
  * 电机 F & B 顺时针旋转.
  * 电机 L & R 逆顺时针旋转.
  */
  Motor_F = 5,
  Motor_B = 6,
  Motor_R = 9,
  Motor_L = 3,
  
  RF24_CE = 8,
  RF24_CS = 7,

  NavLight_FR = 10,
  NavLight_FL = 11,
  NavLight_BR = 12,
  NavLight_BL = 13,
};

/**
 * @brief 电机数组下标别名
 */
enum Motor : uint8_t
{
  FR = 0, ///< 右前
  FL,     ///< 左前
  BR,     ///< 右后
  BL,     ///< 左后

  F = 0,  ///< 前
  B,      ///< 后
  R,      ///< 右
  L,      ///< 左
};

constexpr float   dt              = 0.002;   ///< 时间变化量
constexpr uint8_t radio_address[] = "00004";

constexpr float deg_to_rad = PI / 180.f; ///< 角度制转弧度制
constexpr float rad_to_deg = 180.f / PI; ///< 弧度制转角度制

constexpr float max_angle         = 20.f * deg_to_rad; ///< 最大倾角, 弧度制
constexpr float max_takeoff_angle = 5.f * deg_to_rad;  ///< 起飞时最大倾角, 弧度制

/**
 * @brief 三维向量
 */
union Vector3
{
  struct
  {
    float x;
    float y;
    float z;
  };
  float v[3];
};

/**
 * @brief 欧拉角
 */
union EulerAngles
{
  struct
  {
    float yaw;
    float pitch;
    float roll;
  };
  float v[3];
};

/**
 * @brief 错误代码
 */
enum class ErrorCode
{
  Normal,             ///< 正常
  RadioDisconnected,  ///< 无线电信号中断
  LowBattery,         ///< 低电量
  CriticalLowBattery, ///< 严重低电量
  Unknown,            ///< 未知错误
};

/**
 * @brief 指示器状态
 */
struct IndicatorState
{
  uint8_t timer;
  uint8_t interval; ///< 闪烁间隔
  int8_t count;     ///< 闪烁次数
};

/**
 * @brief 无线电数据包
 */
struct RadioPackage
{
  EulerAngles target_angles;
  Vector3     target_position;
  float       throttle;
};

/**
 * @brief PID 状态
 *
 * 存储计算 PID 所需的参数.
 */
struct PIDState
{
  PIDState(float kP, float kI, float kD)
    : kP(kP), kI(kI), kD(kD)
  {}

  float kP, kI, kD;     ///< PID 参数
  float integral = 0.f; ///< 积分
  float prev_error;     ///< 上一次误差
};

PIDState angle_states[3] = {
  {4.0, 0.02, 0},  // yaw
  {1.3, 0.04, 18}, // pitch
  {1.3, 0.04, 18}, // roll
};

PIDState position_states[3] = {
  {0.1, 0.1, 20}, // x
  {0.1, 0.1, 20}, // y
  {0.1, 0.1, 20}, // z
};

// ZYX 欧拉角, 弧度制
EulerAngles target_angles = {}; // 目标姿态

// 以起飞点为坐标系原点
Vector3 target_position = {}; // 目标坐标
Vector3 actual_position = {}; // 当前坐标
Vector3 actual_velocity = {}; // 当前速度

float throttle = 0.0f;  // 节流阀
long  takeoff_altitude; // 起飞时海拔, 用于计算相对海拔

Servo motors[4];
RF24  radio(RF24_CE, RF24_CS);

IndicatorState indicator_state = {};

/**
 * @brief PID 控制器
 *
 * @param target 设定点(setpoint)
 * @param actual 测量的过程值(process variable)
 * @param dt     时间变化量
 * @param state  PID 状态
 */
float pid(float target, float actual, float dt, PIDState& state)
{
  // TODO: 可能要对积分项进行一个阈值的限定, 因为不能无限累积(可能会累积过度)

  const float error = target - actual;
  state.prev_error  = error;

  const float p   = state.kP * error;
  state.integral += state.kI * error * dt;
  const float d   = state.kD * (error - state.prev_error) / dt;

  return p + state.integral + d;
}

void print_actual_angles()
{
  const EulerAngles actual_angles = get_actual_angles();
  Serial.print(F("Actual angles: "));
  for (int i = 0; i < 3; i++) {
    Serial.print(actual_angles.v[i] * rad_to_deg);
    Serial.print(F(", "));
  }
  Serial.print(F("\n"));
}

void print_actual_position()
{
  Serial.print(F("Actual position: "));
  for (int i = 0; i < 3; i++) {
    Serial.print(actual_position.v[i]);
    Serial.print(F(", "));
  }
  Serial.print(F("\n"));

  Serial.print(abs(get_actual_altitude() - actual_position.z));
  Serial.print(F("\n"));
}

void update_indicator_light()
{
  if(indicator_state.count == 0)
    return;
  indicator_state.timer += micros();
  if(indicator_state.timer < indicator_state.interval)
    return;
  indicator_state.timer = 0;

  const uint8_t value = indicator_state.count % 2;
  digitalWrite(NavLight_FR, value);
  digitalWrite(NavLight_FL, value);
  digitalWrite(NavLight_BR, value);
  digitalWrite(NavLight_BL, value);
  if(indicator_state.count != -1)
    indicator_state.count++;
}

void set_indicator_light(ErrorCode code)
{
  switch(code)
  {
    case ErrorCode::Normal:
      indicator_state.count    = -1;
      indicator_state.interval = 1000;
      break;
      
    case ErrorCode::RadioDisconnected:
      indicator_state.count    = 3 * 2;
      indicator_state.interval = 200;
      break;
      
    case ErrorCode::LowBattery:
      indicator_state.count    = 5 * 2;
      indicator_state.interval = 500;
      break;
      
    case ErrorCode::CriticalLowBattery:
      indicator_state.count    = 5 * 2;
      indicator_state.interval = 200;
      break;

    case ErrorCode::Unknown:
      indicator_state.count    = 5 * 2;
      indicator_state.interval = 500;
      break;
  }
}

void setup_nav_lights()
{
  pinMode(NavLight_FR, OUTPUT);
  pinMode(NavLight_FL, OUTPUT);
  pinMode(NavLight_BR, OUTPUT);
  pinMode(NavLight_BL, OUTPUT);
}

/**
 * @brief 初始化无线电
 */
void setup_radio(uint8_t channel)
{
  while(!radio.begin());
  radio.setChannel(channel);
  radio.openReadingPipe(1, radio_address);
  radio.setPALevel(RF24_PA_MIN);
  radio.setDataRate(RF24_1MBPS);
  radio.startListening();
}

/**
 * @brief 初始化电机
 */
void setup_motors()
{
#if QUAD_TYPE == QUAD_X
  if(motors[FR].attach(Motor_FR) == INVALID_SERVO)
    while(true)
    {
      set_indicator_light(ErrorCode::Unknown);
      delay(1000);
    }
  if(motors[FL].attach(Motor_FL) == INVALID_SERVO)
    while(true)
    {
      set_indicator_light(ErrorCode::Unknown);
      delay(1000);
    }
  if(motors[BR].attach(Motor_BR) == INVALID_SERVO)
    while(true)
    {
      set_indicator_light(ErrorCode::Unknown);
      delay(1000);
    }
  if(motors[BL].attach(Motor_BL) == INVALID_SERVO)
    while(true)
    {
      set_indicator_light(ErrorCode::Unknown);
      delay(1000);
    }
#else
  if(motors[F].attach(Motor_F) == INVALID_SERVO)
    while(true)
    {
      set_indicator_light(ErrorCode::Unknown);
      delay(1000);
    }
  if(motors[B].attach(Motor_B) == INVALID_SERVO)
    while(true)
    {
      set_indicator_light(ErrorCode::Unknown);
      delay(1000);
    }
  if(motors[R].attach(Motor_R) == INVALID_SERVO)
    while(true)
    {
      set_indicator_light(ErrorCode::Unknown);
      delay(1000);
    }
  if(motors[L].attach(Motor_L) == INVALID_SERVO)
    while(true)
    {
      set_indicator_light(ErrorCode::Unknown);
      delay(1000);
    }
#endif
}

/**
 * @brief 更新 IMU 数据
 */
void update_jy901()
{
  while (Serial.available()) {
    JY901.CopeSerialData(Serial.read()); // Call JY901 data cope function
  }
}

void update_actual_position_and_velocity()
{
  // TODO: 加速度计在震动环境下误差较大, 结合加速度计和陀螺仪的数据来获得更加准确的姿态
  actual_velocity.x = JY901.stcAcc.a[1];
  actual_velocity.y = -JY901.stcAcc.a[0];
  actual_velocity.z = JY901.stcAcc.a[2];
  for(int i = 0; i < 3; i++)
    actual_position.v[i] += actual_velocity.v[i] * dt;
}

void check_battery_level()
{
  const auto voltage       = analogRead(A0) * 5.f / 1023.f;
  const auto battery_level = map(voltage, 3.6f, 4.2f, 0.f, 100.f);

  if(battery_level < 20.f)
    set_indicator_light(ErrorCode::CriticalLowBattery);
  else if(battery_level < 30.f)
    set_indicator_light(ErrorCode::LowBattery);
}

void update_radio()
{
  RadioPackage package;
  while (radio.available()) {
    radio.read(&package, sizeof(package));
    target_angles   = package.target_angles;
    target_position = package.target_position;
    throttle        = package.throttle;
  }
}

void update_motors(const EulerAngles& angles, float throttle)
{
  // FIXME:
  // 1. writeMicroseconds 理论上可以让电机反转, 但实际上不行, 桨叶的气动结构基本都是设计为单向转的. 反向转的效果只用减小电机力, 让重力实现就行
  // 2. 为了适应不同的电机需要加一些东西, writeMicroseconds 是针对无刷电调的 PWM 控制的, 如果是有刷电调(或普通 MOS 管)可能需要用 analogWrite, 因为 writeMicroseconds 貌似不能做到输出满占空比

  // 假设 1000-1499 为逆时针旋转, 1501-2000 为顺时针旋转, 1500 为停止
#if QUAD_TYPE == QUAD_X
  motors[FR].writeMicroseconds(constrain(1500 - (throttle * 400 - angles.pitch + angles.roll - angles.yaw), 1000, 1500));
  motors[FL].writeMicroseconds(constrain(1500 - (throttle * 400 - angles.pitch - angles.roll + angles.yaw), 1000, 1500));
  
  motors[BR].writeMicroseconds(constrain(1500 - (throttle * 400 + angles.pitch + angles.roll + angles.yaw), 1000, 1500));
  motors[BL].writeMicroseconds(constrain(1500 - (throttle * 400 + angles.pitch - angles.roll - angles.yaw), 1000, 1500));
#else
  motors[F].writeMicroseconds(constrain(1500 + (throttle * 400 - angles.pitch + angles.yaw), 1500, 2000));
  motors[B].writeMicroseconds(constrain(1500 + (throttle * 400 + angles.pitch + angles.yaw), 1500, 2000));
  
  motors[R].writeMicroseconds(constrain(1500 - (throttle * 400 + angles.roll - angles.yaw), 1000, 1500));
  motors[L].writeMicroseconds(constrain(1500 - (throttle * 400 - angles.roll - angles.yaw), 1000, 1500));
#endif
}

/**
 * @brief 获取实际姿态角
 *
 * MPU 6050 坐标系:
 *
 *   z
 * y ↑
 *  \|
 *   +----→ x
 */
EulerAngles get_actual_angles()
{
  return { (float)JY901.stcAngle.Angle[2] / (SHRT_MAX + 1),   // yaw
           (float)JY901.stcAngle.Angle[0] / (SHRT_MAX + 1),   // pitch
           (float)JY901.stcAngle.Angle[1] / (SHRT_MAX + 1) }; // roll
}

/**
 * @brief 获取气压计测量海拔
 *
 * 精度: ±1m
 */
long get_actual_altitude()
{
  return JY901.stcPress.lAltitude;
}

void setup()
{
  Serial.begin(115200);
  while(!Serial);

  setup_nav_lights();
  setup_radio(100);
  setup_motors();

  EulerAngles actual_angles;
  // 等待倾角小于起飞时最大倾角
  do {
    update_jy901();
    actual_angles = get_actual_angles();
  } while(abs(actual_angles.pitch) > max_takeoff_angle || abs(actual_angles.pitch) > max_takeoff_angle);

  target_angles.yaw   = actual_angles.yaw;
  target_angles.pitch = 0;
  target_angles.roll  = 0;

  target_position.x = 0;
  target_position.y = 0;
  target_position.z = 1;

  takeoff_altitude = get_actual_altitude();

  set_indicator_light(ErrorCode::Normal);
}

void loop()
{
  check_battery_level();
  update_radio();
  update_jy901();
  update_actual_position_and_velocity();

  update_indicator_light();

  // 调整到目标姿态
  const EulerAngles actual_angles = get_actual_angles(); // 实际姿态, 弧度制
  if(abs(actual_angles.pitch) > max_angle || abs(actual_angles.roll) > max_angle)
  {
    // TODO: 倾角过大
  }

  EulerAngles actuator_angles; // 姿态控制量
  for(int i = 0; i < 3; i++)
    actuator_angles.v[i] = pid(target_angles.v[i], actual_angles.v[i], dt, angle_states[i]);
    
  Vector3 actuator_position;

  // 调整到指定高度(Z)
  actual_position.z   = get_actual_altitude() - takeoff_altitude;
  actuator_position.z = pid(target_position.z, actual_position.z, dt, position_states[2]);
  throttle += actuator_position.z > 0.f ? 0.1f : -0.1f;

  // 调整姿态来到达指定 X, Y
  actuator_position.x  = pid(target_position.x, actual_position.x, dt, position_states[0]);
  actuator_position.y  = pid(target_position.y, actual_position.y, dt, position_states[1]);
  target_angles.pitch -= actuator_position.x;
  target_angles.roll  -= actuator_position.y;

  // 限定目标角度最大倾角
  target_angles.pitch = constrain(target_angles.pitch, -max_angle, max_angle);
  target_angles.roll  = constrain(target_angles.roll, -max_angle, max_angle);

  // TODO: 感觉需要乘以一个系数, 具体多少不太清楚
  const float factor = 3.f;
  for(int i = 0; i < 3; i++)
    actuator_angles.v[i] *= factor;

  update_motors(actuator_angles, throttle);
}
