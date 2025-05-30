# Coordinate Transformation
Coordinate transformation is commonly applied in extrinsic calibration and similar operations.

## Code Structure
The coordinate transformation is defined in `lib_hesai/driver_param.h` 
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
## Configuration
The transformation default value is given as zero.
- `x,y,z` are given in `meter`
- `Roll, Pitch, Yaw` angles are given in `radian`
- The order of angles applied to the sensor is `Yaw - Pitch - Roll`

Please refer to the source code for more details.
