#pragma once

#include <vector>
#include <string>
#include <json.hpp>
#include <fstream>
#include <sstream>
#include <iostream>

#include "FoxglovePointCloudDefinition.hpp"

namespace pcl
{
  struct PointFieldData
  {
    int datatype;
    std::string name;
    int size;
    int offset;
    int count;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(PointFieldData, datatype, name, size, offset, count);
  };

  enum DatatypesEnum
  {
    boolean,
    int8,
    uint8,
    int16,
    uint16,
    int32,
    uint32,
    float32,
    float64,
  };

  const std::map<foxglove::PackedElementField_NumericType, int> FoxgloveDatatypeSizesMap = {
    {foxglove::PackedElementField_NumericType::PackedElementField_NumericType_INT8, 1},
    {foxglove::PackedElementField_NumericType::PackedElementField_NumericType_UINT8, 1},
    {foxglove::PackedElementField_NumericType::PackedElementField_NumericType_INT16, 2},
    {foxglove::PackedElementField_NumericType::PackedElementField_NumericType_UINT16, 2},
    {foxglove::PackedElementField_NumericType::PackedElementField_NumericType_INT32, 4},
    {foxglove::PackedElementField_NumericType::PackedElementField_NumericType_UINT32, 4},
    {foxglove::PackedElementField_NumericType::PackedElementField_NumericType_FLOAT32, 4},
    {foxglove::PackedElementField_NumericType::PackedElementField_NumericType_FLOAT64, 8},
  };

  template<typename T>
  struct LabeledPoint {
    float x;
    float y;
    float z;
    T label;
  };
}