// Decompression for clouds without color
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "xyz_decompression.h"

#include <ros/node_handle.h>

#include <cloud_compression/CompressedCloud.h>

#include <pcl_ros/point_cloud.h>

#include <pluginlib/class_list_macros.h>

#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/filter.h>

namespace cloud_compression
{

XYZDecompression::XYZDecompression()
{
}

XYZDecompression::~XYZDecompression()
{
}

void XYZDecompression::onInit()
{
	ros::NodeHandle& nh = getPrivateNodeHandle();

	m_compression.reset(
		new Compression(pcl::io::HIGH_RES_ONLINE_COMPRESSION_WITHOUT_COLOR)
	);

	m_sub = nh.subscribe("input", 1, &XYZDecompression::handleMsg, this);
	m_pub = nh.advertise<PointCloud>("output", 1);
}

class VectorStreamBuf : public std::streambuf
{
public:
	explicit VectorStreamBuf(const std::vector<uint8_t>* in)
	{
		setg(
			reinterpret_cast<char*>(const_cast<uint8_t*>(in->data())),
			reinterpret_cast<char*>(const_cast<uint8_t*>(in->data())),
			reinterpret_cast<char*>(const_cast<uint8_t*>(in->data() + in->size()))
		);
	}
};

class VectorStream : public std::istream
{
public:
	explicit VectorStream(const std::vector<uint8_t>* in)
	 : m_buf(in)
	{
		rdbuf(&m_buf);
	}
private:
	VectorStreamBuf m_buf;
};

void XYZDecompression::handleMsg(const CompressedCloud::ConstPtr& msg)
{
	PointCloud::Ptr cloud(new PointCloud);

	VectorStream stream(&msg->data);

	m_compression->decodePointCloud(stream, cloud);

	pcl_conversions::toPCL(msg->header, cloud->header);

// 	ROS_INFO("Decompression: got cloud with %lux%lu points (compressed size: %lu)", (long unsigned int)cloud->width, (long unsigned int)cloud->height, msg->data.size());

	if (cloud->points.size() > 0)
		m_pub.publish(cloud);
}

}

PLUGINLIB_EXPORT_CLASS(cloud_compression::XYZDecompression, nodelet::Nodelet)

