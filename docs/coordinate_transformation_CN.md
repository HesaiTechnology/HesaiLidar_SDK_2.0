# 坐标转换
坐标变换通常用于外参标定等类似应用。

## 代码结构
在 `lib_hesai/driver_param.h` 中定义了坐标转换相关代码。
```cpp
typedef struct TransformParam  
{
  ///< unit, m
  float x = 0.0f; 
  ///< unit, m     
  float y = 0.0f;
  ///< unit, m      
  float z = 0.0f;  
  ///< unit, radian    
  float roll = 0.0f;  
  ///< unit, radian 
  float pitch = 0.0f;  
  ///< unit, radian
  float yaw = 0.0f;    
} TransformParam;
```
## 相关配置
该转换的默认值设置为0
- `x,y,z` 的单位是 `meter`
- `Roll, Pitch, Yaw` 角度单位是 `radian`
- 应用与激光雷达的转换顺序是 `Yaw - Pitch - Roll`

请参考源码以获得更多信息。
