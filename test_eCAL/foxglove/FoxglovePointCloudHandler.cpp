#include "FoxglovePointCloudHandler.h"

#include <vector>
#include <string>

using namespace std;

template <typename T>
T getPointValue(const char* data, foxglove::PackedElementField_NumericType dataType, int index)
{
  T retVal = 0;

  switch (dataType)
  {
  case foxglove::PackedElementField_NumericType_INT8:
  {
    int8_t value;
    std::memcpy(&value, data + index, sizeof(int8_t));
    retVal = (T)value;
    break;
  }
  case foxglove::PackedElementField_NumericType_UINT8:
  {
    uint8_t value;
    std::memcpy(&value, data + index, sizeof(uint8_t));
    retVal = (T)value;
    break;
  }
  case foxglove::PackedElementField_NumericType_INT16:
  {
    int16_t value;
    std::memcpy(&value, data + index, sizeof(int16_t));
    retVal = (T)value;
    break;
  }
  case foxglove::PackedElementField_NumericType_UINT16:
  {
    uint16_t value;
    std::memcpy(&value, data + index, sizeof(uint16_t));
    retVal = (T)value;
    break;
  }
  case foxglove::PackedElementField_NumericType_INT32:
  {
    int32_t value;
    std::memcpy(&value, data + index, sizeof(int32_t));
    retVal = (T)value;
    break;
  }
  case foxglove::PackedElementField_NumericType_UINT32:
  {
    uint32_t value;
    std::memcpy(&value, data + index, sizeof(uint32_t));
    retVal = (T)value;
    break;
  }
  case foxglove::PackedElementField_NumericType_FLOAT32:
  {
    float value;
    std::memcpy(&value, data + index, sizeof(float));
    retVal = value;
    break;
  }
  case foxglove::PackedElementField_NumericType_FLOAT64:
  {
    double value;
    std::memcpy(&value, data + index, sizeof(double));
    retVal = (T)value;
    break;
  }
  }

  return retVal;
}
template int8_t getPointValue(const char* data, foxglove::PackedElementField_NumericType dataType, int index);
template uint8_t getPointValue(const char* data, foxglove::PackedElementField_NumericType dataType, int index);
template int16_t getPointValue(const char* data, foxglove::PackedElementField_NumericType dataType, int index);
template uint16_t getPointValue(const char* data, foxglove::PackedElementField_NumericType dataType, int index);
template int32_t getPointValue(const char* data, foxglove::PackedElementField_NumericType dataType, int index);
template uint32_t getPointValue(const char* data, foxglove::PackedElementField_NumericType dataType, int index);
template float getPointValue(const char* data, foxglove::PackedElementField_NumericType dataType, int index);
template double getPointValue(const char* data, foxglove::PackedElementField_NumericType dataType, int index);


/// <summary>
/// Fills a vector with points from a Foxglove PointCloud message.
/// </summary>
/// <typeparam name="T">Type of the points to be saved in the vector.</typeparam>
/// <param name="pointcloud">The Foxglove PointCloud message to extract the points from.</param>
/// <param name="points">Vector in which to write the points.</param>
template <typename T>
void fillPointsVector(const foxglove::PointCloud& pointcloud, std::vector<T>& points)
{
  if (pointcloud.point_stride() != sizeof(T))
  {
    std::cerr << "Point Stride and Point Size don't match!" << std::endl;
    return;
  }

  auto width = static_cast<int>(pointcloud.data().size() / pointcloud.point_stride());
  auto numPoints = width;
  points.clear();
  points.resize(numPoints);

  // Pointer to the beginning of the data in the point cloud message
  const T* pointPtr = reinterpret_cast<const T*>(pointcloud.data().data());

  // Use std::copy to copy the points
  std::copy(pointPtr, pointPtr + numPoints, points.begin());
}


/// <summary>
/// Converts Foxglove PointCloud x, y and z points to a vector. The vector will contain the values as follow: [x0, y0, z0, x1, y1, z1, x2, y2, z2, ... xn, yn, zn]
/// for a pointcloud with n points.
/// </summary>
/// <typeparam name="T">Type of the x, y and z values to be saved in the vector.</typeparam>
/// <param name="pointcloud">Foxglove PointCloud object to extract the points.</param>
/// <param name="points">Vector in which to write the points.</param>
template <typename T>
void getPoints(const foxglove::PointCloud& pointcloud, std::vector<T>& points)
{
  getPointFields(pointcloud, points, { "x", "y", "z" });
}
template void getPoints(const foxglove::PointCloud& pointcloud, std::vector<int8_t>& points);
template void getPoints(const foxglove::PointCloud& pointcloud, std::vector<uint8_t>& points);
template void getPoints(const foxglove::PointCloud& pointcloud, std::vector<int16_t>& points);
template void getPoints(const foxglove::PointCloud& pointcloud, std::vector<uint16_t>& points);
template void getPoints(const foxglove::PointCloud& pointcloud, std::vector<int32_t>& points);
template void getPoints(const foxglove::PointCloud& pointcloud, std::vector<uint32_t>& points);
template void getPoints(const foxglove::PointCloud& pointcloud, std::vector<float>& points);
template void getPoints(const foxglove::PointCloud& pointcloud, std::vector<double>& points);


/// <summary>
/// Converts some selected point fields of a Foxglove Pointcloud object to a vector.
/// </summary>
/// <typeparam name="T">Type of the point fields to be saved in the vector.</typeparam>
/// <param name="pointcloud">Foxglove Pointcloud object to extract the point fields.</param>
/// <param name="points">Vector in which to write the point fields.</param>
/// <param name="pointFieldNames">Vector of strings containing the names of the point fields to extract.</param>
template <typename T>
void getPointFields(const foxglove::PointCloud& pointcloud, std::vector<T>& points, vector<string> pointFieldNames)
{
  vector<foxglove::PackedElementField> fieldsToParse;
  auto width = static_cast<int>(pointcloud.data().size() / pointcloud.point_stride());
  const size_t pointCount = width;
  const size_t pointStep = pointcloud.point_stride();
  const auto& data = pointcloud.data().c_str();

  for (const auto& fieldName : pointFieldNames)
  {
    for (const auto& field : pointcloud.fields())
    {
      if (field.name() == fieldName)
        fieldsToParse.emplace_back(field);
    }
  }

  if (fieldsToParse.size() != pointFieldNames.size())
    return;

  const size_t pointFieldCount = fieldsToParse.size();

  points.resize(pointCount * pointFieldCount);

  for (size_t i = 0; i < pointCount; ++i)
  {
    for (size_t j = 0; j < pointFieldCount; ++j)
    {
      const auto& field = fieldsToParse[j];

      // Points are extracted from the data using offset.
      const size_t index = (i * pointStep) + field.offset();
      const T value = getPointValue<T>(data, field.type(), index);

      points[i * pointFieldCount + j] = value;
    }
  }
}
template void getPointFields(const foxglove::PointCloud& pointcloud, std::vector<int8_t>& points, std::vector<std::string> pointFieldNames);
template void getPointFields(const foxglove::PointCloud& pointcloud, std::vector<uint8_t>& points, std::vector<std::string> pointFieldNames);
template void getPointFields(const foxglove::PointCloud& pointcloud, std::vector<int16_t>& points, std::vector<std::string> pointFieldNames);
template void getPointFields(const foxglove::PointCloud& pointcloud, std::vector<uint16_t>& points, std::vector<std::string> pointFieldNames);
template void getPointFields(const foxglove::PointCloud& pointcloud, std::vector<int32_t>& points, std::vector<std::string> pointFieldNames);
template void getPointFields(const foxglove::PointCloud& pointcloud, std::vector<uint32_t>& points, std::vector<std::string> pointFieldNames);
template void getPointFields(const foxglove::PointCloud& pointcloud, std::vector<float>& points, std::vector<std::string> pointFieldNames);
template void getPointFields(const foxglove::PointCloud& pointcloud, std::vector<double>& points, std::vector<std::string> pointFieldNames);
