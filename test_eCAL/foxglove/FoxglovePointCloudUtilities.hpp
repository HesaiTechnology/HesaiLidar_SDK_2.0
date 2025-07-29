#pragma once

#include <FoxglovePointCloudStructs.hpp>
#include <FoxglovePointCloudHandler.h>

#ifndef _WIN32 // required for linux
template< class T, class U >
constexpr bool is_same_v = std::is_same<T, U>::value;
#else
template< class T, class U >
constexpr bool is_same_v = std::is_same_v<T, U>::value;
#endif // _WIN32


/// <summary>
/// Adds a point field to a RepeatedPtrField array of PackedElementField.
/// </summary>
/// <param name="pointFieldArray">Pointer to the RepeatedPtrField array to which the point field will be added.</param>
/// <param name="name">The name of the point field.</param>
/// <param name="offset">The offset of the point field.</param>
/// <param name="type">The numeric type of the point field.</param>
void addPointFieldToFieldArray(
  google::protobuf::RepeatedPtrField<foxglove::PackedElementField>* pointFieldArray,
  const std::string& name,
  int& offset,
  foxglove::PackedElementField_NumericType type);

/// <summary>
/// Creates a Foxglove PointCloud message from a vector of data.
/// </summary>
/// <typeparam name="T">Type of the data elements in the vector.</typeparam>
/// <param name="dataVector">The vector containing the point cloud data.</param>
/// <param name="frame_id">The frame ID for the PointCloud message. Defaults to "base_link".</param>
/// <returns>A Foxglove PointCloud message containing the point cloud data.</returns>
template<typename T>
foxglove::PointCloud createFoxglovePointCloudMessage(const std::vector<T>& dataVector, std::string frame_id = "base_link")
{
  auto bytesInMessage = sizeof(T) * dataVector.size();

  foxglove::PointCloud pointcloud_msg;

  pointcloud_msg.set_frame_id(frame_id);  // Set the appropriate frame ID

  // Get the current time
  std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
  std::chrono::seconds seconds_since_epoch = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch());
  std::chrono::nanoseconds nanoseconds_since_epoch = now.time_since_epoch();
  auto timestamp = pointcloud_msg.mutable_timestamp();
  timestamp->set_seconds(static_cast<google::protobuf::uint32>(seconds_since_epoch.count()));
  timestamp->set_nanos(static_cast<google::protobuf::uint32>(nanoseconds_since_epoch.count()));

  pointcloud_msg.set_point_stride(sizeof(T));

  // Copy the T vector into the point cloud data buffer
  pointcloud_msg.clear_data();
  pointcloud_msg.mutable_data()->resize(dataVector.size() * sizeof(T));

  // Use std::memcpy to copy the data from gsmPoint_vector to pointcloud_msg
  std::memcpy((void*)pointcloud_msg.mutable_data()->data(), dataVector.data(), bytesInMessage);

  return pointcloud_msg;
}

/// <summary>
/// Creates a Foxglove PointCloud message from a vector of data and field information.
/// </summary>
/// <typeparam name="T">Type of the data elements in the vector.</typeparam>
/// <param name="dataVector">The vector containing the point cloud data.</param>
/// <param name="fieldsPairVector">Vector of pairs containing field names and their numeric types.</param>
/// <param name="seconds">The seconds component of the timestamp.</param>
/// <param name="nanos">The nanoseconds component of the timestamp.</param>
/// <param name="frame_id">The frame ID for the PointCloud message. Defaults to "base_link".</param>
/// <returns>A Foxglove PointCloud message containing the point cloud data and field information.</returns>
template<typename T>
foxglove::PointCloud createFoxglovePointCloudMessage(
  const std::vector<T>& dataVector,
  std::vector<std::pair<std::string, foxglove::PackedElementField_NumericType>> fieldsPairVector,
  google::protobuf::uint32 seconds,
  google::protobuf::uint32 nanos,
  std::string frame_id = "base_link"
)
{
  auto bytesInMessage = sizeof(T) * dataVector.size();

  foxglove::PointCloud pointcloud_msg;

  // Fill fields
  auto fields = pointcloud_msg.mutable_fields();
  int offset = 0U;
  for (const auto& pair : fieldsPairVector)
  {
    addPointFieldToFieldArray(
      fields, pair.first, offset, pair.second
    );
  }

  // Set the appropriate frame ID
  pointcloud_msg.set_frame_id(frame_id);

  // Set the timestamp
  auto timestamp = pointcloud_msg.mutable_timestamp();
  timestamp->set_seconds(seconds);
  timestamp->set_nanos(nanos);

  // Set point stride
  pointcloud_msg.set_point_stride(sizeof(T));

  // Copy the T vector into the point cloud data buffer
  pointcloud_msg.clear_data();
  pointcloud_msg.mutable_data()->resize(dataVector.size() * sizeof(T));

  // Use std::memcpy to copy the data from gsmPoint_vector to pointcloud_msg
  std::memcpy((void*)pointcloud_msg.mutable_data()->data(), dataVector.data(), bytesInMessage);

  return pointcloud_msg;
}
