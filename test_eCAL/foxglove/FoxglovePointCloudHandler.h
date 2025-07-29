#pragma once

#include "FoxglovePointCloudDefinition.hpp"
#include "FoxglovePointCloudStructs.hpp"

#include <vector>
#include <string>

inline foxglove::PackedElementField_NumericType getFoxgloveFieldNumericType(const int intType)
{
  foxglove::PackedElementField_NumericType type;

  switch (intType)
  {
  case 1:
    type = foxglove::PackedElementField_NumericType::PackedElementField_NumericType_UINT8;
    break;
  case 2:
    type = foxglove::PackedElementField_NumericType::PackedElementField_NumericType_INT8;
    break;
  case 3:
    type = foxglove::PackedElementField_NumericType::PackedElementField_NumericType_UINT16;
    break;
  case 4:
    type = foxglove::PackedElementField_NumericType::PackedElementField_NumericType_INT16;
    break;
  case 5:
    type = foxglove::PackedElementField_NumericType::PackedElementField_NumericType_UINT32;
    break;
  case 6:
    type = foxglove::PackedElementField_NumericType::PackedElementField_NumericType_INT32;
    break;
  case 7:
    type = foxglove::PackedElementField_NumericType::PackedElementField_NumericType_FLOAT32;
    break;
  case 8:
    type = foxglove::PackedElementField_NumericType::PackedElementField_NumericType_FLOAT64;
    break;
  case 0:
  default:
    type = foxglove::PackedElementField_NumericType::PackedElementField_NumericType_UNKNOWN;
    break;
  }
  return type;
}

/// <summary>
/// Converts Foxglove PointCloud x, y and z points to a vector. The vector will contain the values as follow: [x0, y0, z0, x1, y1, z1, x2, y2, z2, ... xn, yn, zn]
/// for a pointcloud with n points.
/// </summary>
/// <typeparam name="T">Type of the x, y and z values to be saved in the vector.</typeparam>
/// <param name="pointcloud">Foxglove PointCloud object to extract the points.</param>
/// <param name="points">Vector in which to write the points.</param>
template<typename T>
void getPoints(const foxglove::PointCloud& pointcloud, std::vector<T>& points);

template <typename T>
void fillPointsVector(const foxglove::PointCloud& pointcloud, std::vector<T>& points);

/// <summary>
/// Converts some selected point fields of a Foxglove PointCloud object to a vector.
/// </summary>
/// <typeparam name="T">Type of the point fields to be saved in the vector.</typeparam>
/// <param name="pointcloud">Foxglove PointCloud object to extract the point fields.</param>
/// <param name="points">Vector in which to write the point fields.</param>
/// <param name="pointFieldNames">Vector of strings containing the names of the point fields to extract.</param>
template<typename T>
void getPointFields(const foxglove::PointCloud& pointcloud, std::vector<T>& points, std::vector<std::string> pointFieldNames);

/// <summary>
/// Retrieves the value of a point from the data buffer at a specified index, given a specific numeric type.
/// </summary>
/// <typeparam name="T">Type of the point value to be retrieved.</typeparam>
/// <param name="data">Pointer to the data buffer containing point values.</param>
/// <param name="dataType">Numeric type of the point value to be retrieved.</param>
/// <param name="index">Index of the point value to be retrieved.</param>
/// <returns>The value of the point at the specified index.</returns>
template<typename T>
T getPointValue(const char* data, foxglove::PackedElementField_NumericType dataType, int index);
