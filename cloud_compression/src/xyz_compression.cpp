// Compression for clouds without color
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "xyz_compression.h"

#include <ros/node_handle.h>

#include <cloud_compression/CompressedCloud.h>

#include <pcl_ros/point_cloud.h>

#include <pluginlib/class_list_macros.h>

#include <pcl_conversions/pcl_conversions.h>

namespace cloud_compression
{

XYZCompression::XYZCompression()
{
}

XYZCompression::~XYZCompression()
{
}

void XYZCompression::onInit()
{
	ros::NodeHandle& nh = getPrivateNodeHandle();

	m_compression.reset(
		new Compression(pcl::io::HIGH_RES_ONLINE_COMPRESSION_WITHOUT_COLOR)
	);

	m_sub = nh.subscribe("input", 1, &XYZCompression::handleCloud, this);
	m_pub = nh.advertise<CompressedCloud>("output", 1);
}

class VectorStreamBuf : public std::streambuf
{
public:
	explicit VectorStreamBuf(std::vector<uint8_t>* out)
	 : m_out(out)
	{}

	virtual int overflow(int c)
	{
		m_out->push_back(c);
		return 0;
	}
private:
	std::vector<uint8_t>* m_out;
};

class VectorStream : public std::ostream
{
public:
	explicit VectorStream(std::vector<uint8_t>* out)
	 : m_buf(out)
	{
		rdbuf(&m_buf);
	}
private:
	VectorStreamBuf m_buf;
};

void XYZCompression::handleCloud(const PointCloud::ConstPtr& cloud)
{
	CompressedCloudPtr msg(new CompressedCloud);

	VectorStream stream(&msg->data);

	m_compression.reset(
		new Compression(pcl::io::HIGH_RES_ONLINE_COMPRESSION_WITHOUT_COLOR)
	);
	m_compression->encodePointCloud(cloud, stream);

	pcl_conversions::fromPCL(cloud->header, msg->header);

// 	ROS_INFO("Compressed %lu points to %lu bytes", cloud->size(), msg->data.size());

	m_pub.publish(msg);
}

}

PLUGINLIB_EXPORT_CLASS(cloud_compression::XYZCompression, nodelet::Nodelet)
