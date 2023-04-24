# Quadcopter

## 测试

由于 Arduino 可能不支持通过相对路径导入 Sketch 以外的头文件, 所以进行测试需要先用 `tests` 中对应的 ino 文件替代 `src` 中的 ino 文件.  

## 传感器

由于传感器组件可能包含一个或多个不同类型的传感器, 例如 MPU6050 包含加速度计/陀螺仪和温度传感器, 因此需要一种方法能够统一地从各种传感器组件中获取数据.  

使用多继承是一个简单的解决方法:  

```cpp
class Accelerometer
{
public:
  virtual Vector3 get_acceleration() const = 0;
};

class Gyroscope
{
public:
  virtual EulerAngles get_angular_velocity() const = 0;
};

class Temperature
{
public:
  virtual float get_temperature() const = 0;
};

class MPU6050 : public Accelerometer, public Gyroscope
{
  ...
};
```

这样便可以通过基类指针访问对应传感器的数据. 比如获取 MPU6050 的加速度计的数据:  

```cpp
MPU6050 mpu;
const auto acc = static_cast<const Accelerometer&>(mpu).get_acceleration();
```

## 评估器

评估器负责进行姿态解算. 封装具体使用的传感器, 并对传感器数据进行融合. 传统的叫法可能是 `SensorManager`.  

```cpp
class Evaluator
{
public:
  void update(float dt) { ... };

  virtual Vector3     get_acceleration() const { return accelerometer->get_acceleration(); };
  virtual EulerAngles get_angular_velocity() const { return gyroscope->get_angular_velocity(); };
  ...

private:
  Accelerometer* accelerometer;
  Gyroscope*     gyroscope;
  ...
};
```
