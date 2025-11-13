#pragma once
#include <iostream>
#include <string>
#include <string.h>
#include <cstdint>
namespace hesai
{
namespace lidar
{

#define DEFINE_MEMBER_CHECKER(member)                                                                                  \
  template <typename T, typename V = bool>                                                                             \
  struct has_##member : std::false_type                                                                                \
  {                                                                                                                    \
  };                                                                                                                   \
  template <typename T>                                                                                                \
  struct has_##member<                                                                                                 \
      T, typename std::enable_if<!std::is_same<decltype(std::declval<T>().member), void>::value, bool>::type>          \
      : std::true_type                                                                                                 \
  {                                                                                                                    \
  };
#define PANDAR_HAS_MEMBER(C, member) has_##member<C>::value

#define DEFINE_SET_GET(member, Type)                                                                                   \
  template <typename T_Point>                                                                                          \
  inline typename std::enable_if<!PANDAR_HAS_MEMBER(T_Point, member)>::type set_##member(T_Point& point, const Type& value) \
  {                                                                                                                    \
  }                                                                                                                    \
  template <typename T_Point>                                                                                          \
  inline typename std::enable_if<PANDAR_HAS_MEMBER(T_Point, member)>::type set_##member(T_Point& point, const Type& value) \
  {                                                                                                                    \
      point.member = value;                                                                                            \
  }                                                                                                                    \
  template <typename T_Point>                                                                                          \
  inline typename std::enable_if<!PANDAR_HAS_MEMBER(T_Point, member)>::type get_##member(T_Point& point, Type& value)  \
  {                                                                                                                    \
  }                                                                                                                    \
  template <typename T_Point>                                                                                          \
  inline typename std::enable_if<PANDAR_HAS_MEMBER(T_Point, member)>::type get_##member(T_Point& point, Type& value)  \
  {                                                                                                                    \
      value = point.member;                                                                                            \
  }  

DEFINE_MEMBER_CHECKER(x)
DEFINE_MEMBER_CHECKER(y)
DEFINE_MEMBER_CHECKER(z)

DEFINE_SET_GET(x, float)  
DEFINE_SET_GET(y, float)  
DEFINE_SET_GET(z, float)  

#define HS_MIN(x, y) ((x) > (y) ? (y) : (x))
#define HS_MAX(x, y) ((x) > (y) ? (x) : (y))

typedef struct TransformParam  
{
  bool use_flag = false;
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

struct RemakeConfig {
  bool flag = false;
  float min_azi = -1;
  float max_azi = -1;
  float ring_azi_resolution = -1;
  int max_azi_scan = -1;   // (max_azi - min_azi) / ring_azi_resolution
  float min_elev = -1;
  float max_elev = -1;
  float ring_elev_resolution = -1;
  int max_elev_scan = -1;   // (max_elev - min_elev) / ring_elev_resolution
};


#define DEFAULT_MAX_MULTI_FRAME_NUM 10.0


}
}