#include "PointCloudHandler.h"

#include "pcl_header.pb.h"
#include "pcl_pfield.pb.h"

//#include <vector>
//#include <string>

using namespace std;

template<typename T> T getPointValue(const char* data, int dataType, int index)
{
	T retVal = 0;

	switch(dataType)
	{
	case 1:
	{
		int8_t value;
		std::memcpy(&value, data + index, sizeof(int8_t));
		retVal = (T) value;
		break;
	}
	case 2:
	{
		uint8_t value;
		std::memcpy(&value, data + index, sizeof(uint8_t));
		retVal = (T) value;
		break;
	}

	case 3:
	{
		int16_t value;
		std::memcpy(&value, data + index, sizeof(int16_t));
		retVal = (T) value;
		break;
	}

	case 4:
	{
		uint16_t value;
		std::memcpy(&value, data + index, sizeof(uint16_t));
		retVal = (T) value;
		break;
	}
	case 5:
	{
		int32_t value;
		std::memcpy(&value, data + index, sizeof(int32_t));
		retVal = (T) value;
		break;
	}
	case 6:
	{
		uint32_t value;
		std::memcpy(&value, data + index, sizeof(uint32_t));
		retVal = (T) value;
		break;
	}

	case 7:
	{
		float value;
		std::memcpy(&value, data + index, sizeof(float));
		retVal = value;
		break;
	}

	case 8:
	{
		double value;
		std::memcpy(&value, data + index, sizeof(double));
		retVal = (T) value;
		break;
	}
	}

	return retVal;
}
template int8_t getPointValue(const char* data, int dataType, int index);
template uint8_t getPointValue(const char* data, int dataType, int index);
template int16_t getPointValue(const char* data, int dataType, int index);
template uint16_t getPointValue(const char* data, int dataType, int index);
template int32_t getPointValue(const char* data, int dataType, int index);
template uint32_t getPointValue(const char* data, int dataType, int index);
template float getPointValue(const char* data, int dataType, int index);
template double getPointValue(const char* data, int dataType, int index);

/// <summary>
/// Converts PointCloud2 x, y and z points to a vector. The vector will contain the values as follow: [x0, y0, z0, x1, y1, z1, x2, y2, z2, ... xn, yn, zn] 
/// for a pointcloud with n points.
/// </summary>
/// <typeparam name="T">Type of the x, y and z values to be saved in the vector.</typeparam>
/// <param name="pointcloud">PointCloud2 object to extract the points.</param>
/// <param name="points">Vector in which to write the points.</param>
template<typename T> void getPoints(const POINTCLOUD_CLASS& pointcloud, vector<T>& points)
{
	getPointFields(pointcloud, points, { "x", "y", "z" });
}
template void getPoints(const POINTCLOUD_CLASS& pointcloud, std::vector<int8_t>& points);
template void getPoints(const POINTCLOUD_CLASS& pointcloud, std::vector<uint8_t>& points);
template void getPoints(const POINTCLOUD_CLASS& pointcloud, std::vector<int16_t>& points);
template void getPoints(const POINTCLOUD_CLASS& pointcloud, std::vector<uint16_t>& points);
template void getPoints(const POINTCLOUD_CLASS& pointcloud, std::vector<int32_t>& points);
template void getPoints(const POINTCLOUD_CLASS& pointcloud, std::vector<uint32_t>& points);
template void getPoints(const POINTCLOUD_CLASS& pointcloud, std::vector<float>& points);
template void getPoints(const POINTCLOUD_CLASS& pointcloud, std::vector<double>& points);

/// <summary>
/// Converts some selected point fields of a PointCloud2 object to a vector.
/// </summary>
/// <typeparam name="T">Type of the point fields to be saved in the vector.</typeparam>
/// <param name="pointcloud">PointCloud2 object to extract the point fields.</param>
/// <param name="points">Vector in which to write the point fields.</param>
/// <param name="pointFieldNames">Vector of strings containing the names of the point fields to extract.</param>
template<typename T> void getPointFields(const POINTCLOUD_CLASS& pointcloud, std::vector<T>& points, vector<string> pointFieldNames)
{
	vector<pcl_pfield::PointField> fieldsToParse;
	int pointCount = pointcloud.width() * pointcloud.height();
	int pointFieldCount = pointcloud.fields().size();
	int pointStep = pointcloud.point_step();
	const char* data = pointcloud.data().c_str();

	for(const auto& fieldName : pointFieldNames)
	{
		for(const auto& field : pointcloud.fields())
		{
			if(field.name() == fieldName)
				fieldsToParse.push_back(field);
		}
	}

	if(fieldsToParse.size() != pointFieldNames.size())
	{
		return;
	}

	for(int i = 0; i < pointCount; ++i)
	{
		for(const auto& field : fieldsToParse)
		{
			int index = (i * pointStep) + field.offset();        // Points are extracted from the data using offset.
			T value = getPointValue<T>(data, field.datatype(), index);
			points.push_back(value);
		}
	}
}
template void getPointFields(const POINTCLOUD_CLASS& pointcloud, std::vector<int8_t>& points, std::vector<std::string> pointFieldNames);
template void getPointFields(const POINTCLOUD_CLASS& pointcloud, std::vector<uint8_t>& points, std::vector<std::string> pointFieldNames);
template void getPointFields(const POINTCLOUD_CLASS& pointcloud, std::vector<int16_t>& points, std::vector<std::string> pointFieldNames);
template void getPointFields(const POINTCLOUD_CLASS& pointcloud, std::vector<uint16_t>& points, std::vector<std::string> pointFieldNames);
template void getPointFields(const POINTCLOUD_CLASS& pointcloud, std::vector<int32_t>& points, std::vector<std::string> pointFieldNames);
template void getPointFields(const POINTCLOUD_CLASS& pointcloud, std::vector<uint32_t>& points, std::vector<std::string> pointFieldNames);
template void getPointFields(const POINTCLOUD_CLASS& pointcloud, std::vector<float>& points, std::vector<std::string> pointFieldNames);
template void getPointFields(const POINTCLOUD_CLASS& pointcloud, std::vector<double>& points, std::vector<std::string> pointFieldNames);

/// <summary>
/// Sets the points from a vector to a PointCloud2 object. The fields from the data vector have to be precised in the pointFieldNames parameter and should 
/// be written in a 1D vector where the data for each point are concatenated to each other: [x0, y0, z0, feature1_0, feature2_0, x1, y1, z1, 
/// feature1_1, feature2_1, ... xn, yn, zn, feature1_n, feature2_n].
/// </summary>
/// <typeparam name="T">Type of the points to write to the PointCloud2 object.</typeparam>
/// <param name="pointcloud">PointCloud2 object in which to write the points from the vector.</param>
/// <param name="pointFieldNames">Vector containing the point fields names (ex: ["x", "y", "z", "feature1", "feature2", ...])</param>
/// <param name="points">Vector containing the data to write to the PointCloud2 object.</param>
template<typename T> void setPointCloud(POINTCLOUD_CLASS* pointcloud, vector<string> pointFieldNames, vector<T> points)
{
	int pointCount = points.size();
	int dataSize = sizeof(T);
	char* bytes = (char*) malloc(pointCount * dataSize * sizeof(char));

	if(bytes != 0)
	{
		memcpy(bytes, points.data(), pointCount * sizeof(T));
	}

	for(int i = 0; i < pointFieldNames.size(); i++)
	{
		pcl_pfield::PointField* field = pointcloud->add_fields();
		field->set_name(pointFieldNames[i]);
		field->set_offset(dataSize * i);
		field->set_datatype(7/* hack!!*/);// dataSize);
		field->set_count(1);
	}

	pcl_header::Header* header = new pcl_header::Header();
	header->set_seq(0);
	header->set_frame_id("returned_data");

	pointcloud->set_allocated_header(header);
	pointcloud->set_data(bytes, pointCount * dataSize);

	pointcloud->set_height(1);
	pointcloud->set_width(points.size() / pointFieldNames.size());

	pointcloud->set_is_bigendian(false);
	pointcloud->set_point_step(sizeof(T) * pointFieldNames.size());
	pointcloud->set_row_step((sizeof(T) * pointFieldNames.size()) * (points.size() / pointFieldNames.size()));
	pointcloud->set_is_dense(true);

	free(bytes);
}

template void setPointCloud(POINTCLOUD_CLASS* pointcloud, std::vector<std::string> pointFieldNames, std::vector<int> points);
template void setPointCloud(POINTCLOUD_CLASS* pointcloud, std::vector<std::string> pointFieldNames, std::vector<float> points);
template void setPointCloud(POINTCLOUD_CLASS* pointcloud, std::vector<std::string> pointFieldNames, std::vector<double> points);

template<typename T>
T getPointValue(const char* data, int index)
{
	T value;
	std::memcpy(&value, data + index, sizeof(T));
	return value;
}
template int8_t getPointValue(const char* data, int index);
template uint8_t getPointValue(const char* data, int index);
template int16_t getPointValue(const char* data, int index);
template uint16_t getPointValue(const char* data, int index);
template int32_t getPointValue(const char* data, int index);
template uint32_t getPointValue(const char* data, int index);
template float getPointValue(const char* data, int index);
template double getPointValue(const char* data, int index);

void copyPointCloudWithNewXYZ(const POINTCLOUD_CLASS pointcloudIn, POINTCLOUD_CLASS& pointcloudOut, std::vector<float> xyz)
{
	//Copy input pointcloud to the output pointcloud
	pcl_header::Header* header = pointcloudOut.mutable_header();
	header->mutable_stamp()->set_secs(pointcloudIn.header().stamp().secs());
	header->mutable_stamp()->set_nsecs(pointcloudIn.header().stamp().nsecs());
	header->set_frame_id(pointcloudIn.header().frame_id());

	pointcloudOut.set_sensorip(pointcloudIn.sensorip());
	pointcloudOut.set_scan_pattern(pointcloudIn.scan_pattern());
	pointcloudOut.set_num_multiple_echoes(pointcloudIn.num_multiple_echoes());
	pointcloudOut.set_multi_echo_mode(pointcloudIn.multi_echo_mode());
	pointcloudOut.set_height(pointcloudIn.height());
	pointcloudOut.set_width(pointcloudIn.width());
	pointcloudOut.set_is_bigendian(pointcloudIn.is_bigendian());
	pointcloudOut.set_point_step(pointcloudIn.point_step());
	pointcloudOut.set_row_step(pointcloudIn.row_step());

	pointcloudOut.set_telemetry_revision(pointcloudIn.telemetry_revision());
	pointcloudOut.set_board_temperature_celsius(pointcloudIn.board_temperature_celsius());
	pointcloudOut.set_laser_temperature_celsius(pointcloudIn.laser_temperature_celsius());
	pointcloudOut.set_scanhead_temperature_celsius(pointcloudIn.scanhead_temperature_celsius());
	pointcloudOut.set_scanhead_relative_humidity_percent(pointcloudIn.scanhead_relative_humidity_percent());
	pointcloudOut.set_scanhead_window_temperature_celsius(pointcloudIn.scanhead_window_temperature_celsius());
	pointcloudOut.set_scanhead_window_relative_humidity(pointcloudIn.scanhead_window_relative_humidity());
	pointcloudOut.set_board_temperature_near_adc_celsius(pointcloudIn.board_temperature_near_adc_celsius());
	pointcloudOut.set_adc_temperature_in_celsius(pointcloudIn.adc_temperature_in_celsius());

	pointcloudOut.set_skirt_thresh_far_high_gain(pointcloudIn.skirt_thresh_far_high_gain());
	pointcloudOut.set_skirt_thresh_far_low_gain(pointcloudIn.skirt_thresh_far_low_gain());
	pointcloudOut.set_skirt_thresh_near_high_gain(pointcloudIn.skirt_thresh_near_high_gain());
	pointcloudOut.set_skirt_thresh_near_low_gain(pointcloudIn.skirt_thresh_near_low_gain());

	pointcloudOut.set_pose_x(pointcloudIn.pose_x());
	pointcloudOut.set_pose_y(pointcloudIn.pose_y());
	pointcloudOut.set_pose_z(pointcloudIn.pose_z());
	pointcloudOut.set_pose_yaw(pointcloudIn.pose_yaw());
	pointcloudOut.set_pose_pitch(pointcloudIn.pose_pitch());
	pointcloudOut.set_pose_roll(pointcloudIn.pose_roll());

	pointcloudOut.set_num_multiple_echoes(pointcloudIn.num_multiple_echoes());

	for(const auto field : pointcloudIn.fields())
	{
		pcl_pfield::PointField* outField = pointcloudOut.mutable_fields()->Add();
		outField->set_name(field.name());
		outField->set_count(field.count());
		outField->set_datatype(field.datatype());
		outField->set_offset(field.offset());
	}

	pointcloudOut.set_data(pointcloudIn.data());

	modifyXYZ(pointcloudOut, xyz);
}

void modifyXYZ(POINTCLOUD_CLASS& pointcloud, std::vector<float> xyz)
{
	int nbPoints = pointcloud.width() * pointcloud.height();
	int pointstep = pointcloud.point_step();

	if(nbPoints != xyz.size() / 3)
	{
		std::cerr << "Unable to modify Pointcloud2 message due to errneous input sizes" << std::endl;
		return;
	}

	std::string data = pointcloud.data();

	//Get xyz field info
	uint32_t xOffset = 0, yOffset = 0, zOffset = 0;
	for(size_t j = 0; j < pointcloud.fields_size(); j++)
	{
		pcl_pfield::PointField field = pointcloud.fields()[j];
		if(field.name() == "x")
		{
			xOffset = field.offset();
		}
		if(field.name() == "y")
		{
			yOffset = field.offset();
		}
		if(field.name() == "z")
		{
			zOffset = field.offset();
		}
	}

	for(size_t i = 0; i < nbPoints; i++)
	{
		float xValue = xyz[i * 3];		
		float yValue = xyz[i * 3 + 1];
		float zValue = xyz[i * 3 + 2];

		char xValueChar[sizeof(float)];
		char yValueChar[sizeof(float)];
		char zValueChar[sizeof(float)];

		std::memcpy(xValueChar, &xyz[i * 3], sizeof(float));
		std::memcpy(yValueChar, &xyz[i * 3 + 1], sizeof(float));
		std::memcpy(zValueChar, &xyz[i * 3 + 2], sizeof(float));

		data.replace(pointstep * i + xOffset, sizeof(float), std::string(xValueChar, sizeof(float)));
		data.replace(pointstep * i + yOffset, sizeof(float), std::string(yValueChar, sizeof(float)));
		data.replace(pointstep * i + zOffset, sizeof(float), std::string(zValueChar, sizeof(float)));
	}

	std::copy(data.begin(), data.end(), pointcloud.mutable_data()->begin());
}
