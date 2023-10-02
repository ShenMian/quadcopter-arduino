# Quadcopter

> [!WARNING]  
> 该项目没有经过任何测试.

## 测试

由于 Arduino 可能不支持通过相对路径导入 Sketch 以外的头文件, 所以进行测试需要先用 `tests` 中对应的 ino 文件替代 `src` 中的 `src.ino` 文件.  

## 飞行控制器(Flight controller, FC)

```cpp
motor_speed flight_controller(sensor_reading) { ... }
```

```txt
Sensors -> Flight controller -> Actuators
```

## 传感器(Sensor)

由于传感器组件可能包含一个或多个不同类型的传感器, 例如 MPU6050 包含加速度计/陀螺仪和温度传感器, 因此需要一种方法能够统一地从各种传感器组件中获取数据.  

使用多继承是一个简单的解决方法:  

```cpp
class Accelerometer
{
public:
  virtual Vector3 get_acceleration() const = 0;
};

class Gyroscope { ... };
class Temperature { ... };

class MPU6050 : public Accelerometer, public Gyroscope, public Temperature
{ ... };
```

这样便可以通过基类指针访问对应传感器的数据. 比如获取 MPU6050 的加速度计的数据:  

```cpp
MPU6050 mpu;
const auto acc = static_cast<const Accelerometer&>(mpu).get_acceleration();
```

传感器和它们的使用目的有:  

- 加速度计: 加速度(积分得到速度, 二次积分得到位移), 俯仰角, 滚转角(变化量为角速度).
- 陀螺仪: 角速度(积分得到姿态角).
- 磁力计: 偏航角.
- 气压计: 气压(通过压高公式得到高度).
- 超声波传感器: 测距.
- 光流传感器: 位移.

## 评估器(Estimator)

评估器负责进行姿态解算. 封装具体使用的传感器, 并对传感器数据进行融合. 传统的叫法可能是 `SensorManager`.  

```cpp
class Estimator
{
public:
  void update(float dt) { ... };

  virtual Vector3     get_acceleration() const { return accelerometer_->get_acceleration(); };
  virtual EulerAngles get_angular_velocity() const { return gyroscope_->get_angular_velocity(); };
  ...

private:
  Accelerometer* accelerometer_;
  Gyroscope*     gyroscope_;
  ...
};
```

能进行的操作有:  

- 对加速度进行二次积分得到位置, 并通过气压计/光流传感器/超声波测距和 GPS 进行校准.
- 对角速度进行积分得到姿态角, 并通过磁力计和加速度计进行校准.

## 控制器(Controller)

控制器负责控制分配(control allocation). 通过调整电机的转速进而调整机体的姿态.  

### 控制分配

受 `PX4 Autopilot` 启发, 添加了转子(rotor):  

```cpp
struct Rotor
{
  Motor* motor;
  float  yaw_scale;
  float  pitch_scale;
  float  roll_scale;
  float  scale = 1.f;
};
```

除了包含电机, 转子附加了更多信息:  

- `yaw_scale`/`pitch_scale`/`roll_scale` 影响电机在调整姿态时贡献的比例, 由电机的位置等因素决定.
- `scale` 是总体的缩放比例, 由电机的转速和桨叶的效率等因素决定.

具体参数可以通过 [Motor mixing calculator] 计算, 需要注意坐标系转换.  

[Motor mixing calculator]: https://www.iforce2d.net/mixercalc/

### 节流阀上限

电机的功能不仅包括控制无人机的姿态, 还包括提供无人机的总升力, 使其能够垂直飞行.  
然而, 电机的输出力是有限的, 当无人机的实际姿态与期望姿态相差较大时, 需要更多的力来调节姿态, 减少总升力.  

为了简化计算和提高兼容性, 电机转速/节流阀和 PID 控制器的输出将被归一化到 [0, 1] 区间. 当节流阀达到 1.0 时, 电机将以最大转速运行, 无人机将完全失去姿态调节的能力. 因此, 需要给节流阀设定一个上限, 以保证无人机的姿态稳定, 避免失控.  

节流阀应该为姿态控制留出充足的余量:  

$$ 节流阀_{max} = 1.0 - 姿态控制的控制量_{max} $$

```cpp
constexpr uint8_t n = 4;
float control_speed[n];

... // 计算姿态调节的控制量

float control_speed_max = 0.f;
for(uint8_t i = 0; i < n; i++)
  control_speed_max = 1.f - max(control_speed[i], control_speed_max);
throttle_limited = min(throttle, 1.f - control_speed_max);

for(uint8_t i = 0; i < n; i++)
  rotors[i].motor->set_speed(throttle_limited + control_speed[i]);
```

**可能的改进**: 滚转/偏航和节流阀对机体稳定性的贡献可能比偏航大, 因此可以牺牲偏航的调节量.  

### 故障保护(Fault protection)

当无人机处于异常状态时采取保护措施. 例如:  

- 俯仰角和滚转角大于设定阈值时紧急停机, 防止螺旋桨伤人, 同时保护机体.  
- 小范围试飞测试时, 当相对坐标 x, y, z 超过设定阈值时尝试返回指定区域, 若无效则紧急停机.  

## 依赖项

- [Adafruit BMP280 Library](https://github.com/adafruit/Adafruit_BMP280_Library)
- [Adafruit MPU6050](https://github.com/adafruit/Adafruit_MPU6050)
- [Kalman Filter Library](https://github.com/TKJElectronics/KalmanFilter)
- [RF24](https://github.com/nRF24/RF24)
- [Servo](https://github.com/arduino-libraries/Servo)
