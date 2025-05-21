#pragma once

#include "PointCloudDefinition.hpp"

#include <vector>
#include <string>

#ifdef _WIN32
#else
#include <cstring>	// Linux - std::memcpy
#endif

/// <summary>
/// Converts PointCloud2 x, y and z points to a vector. The vector will contain the values as follow: [x0, y0, z0, x1, y1, z1, x2, y2, z2, ... xn, yn, zn] 
/// for a pointcloud with n points.
/// </summary>
/// <typeparam name="T">Type of the x, y and z values to be saved in the vector.</typeparam>
/// <param name="pointcloud">PointCloud2 object to extract the points.</param>
/// <param name="points">Vector in which to write the points.</param>
template<typename T> void getPoints(const POINTCLOUD_CLASS& pointcloud, std::vector<T>& points);


/// <summary>
/// Converts some selected point fields of a PointCloud2 object to a vector.
/// </summary>
/// <typeparam name="T">Type of the point fields to be saved in the vector.</typeparam>
/// <param name="pointcloud">PointCloud2 object to extract the point fields.</param>
/// <param name="points">Vector in which to write the point fields.</param>
/// <param name="pointFieldNames">Vector of strings containing the names of the point fields to extract.</param>
template<typename T> void getPointFields(const POINTCLOUD_CLASS& pointcloud, std::vector<T>& points, std::vector<std::string> pointFieldNames);

/// <summary>
/// Sets the points from a vector to a PointCloud2 object. The fields from the data vector have to be precised in the pointFieldNames parameter and should 
/// be written in a 1D vector where the data for each point are concatenated to each other: [x0, y0, z0, feature1_0, feature2_0, x1, y1, z1, 
/// feature1_1, feature2_1, ... xn, yn, zn, feature1_n, feature2_n].
/// </summary>
/// <typeparam name="T">Type of the points to write to the PointCloud2 object.</typeparam>
/// <param name="pointcloud">PointCloud2 object in which to write the points from the vector.</param>
/// <param name="pointFieldNames">Vector containing the point fields names (ex: ["x", "y", "z", "feature1", "feature2", ...])</param>
/// <param name="points">Vector containing the data to write to the PointCloud2 object.</param>
template<typename T> void setPointCloud(POINTCLOUD_CLASS* pointcloud, std::vector<std::string> pointFieldNames, std::vector<T> points);

template<typename T>
T getPointValue(const char* data, int index);

void copyPointCloudWithNewXYZ(const POINTCLOUD_CLASS pointcloud, POINTCLOUD_CLASS& pointcloudOut, std::vector<float> xyz);

void modifyXYZ(POINTCLOUD_CLASS& pointcloud, std::vector<float> xyz);
