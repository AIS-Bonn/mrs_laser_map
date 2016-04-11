// Decompression for clouds without color
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef XYZ_DECOMPRESSION_H
#define XYZ_DECOMPRESSION_H

#include <nodelet/nodelet.h>

#include <cloud_compression/octree_pointcloud_compression.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <ros/subscriber.h>
#include <ros/publisher.h>

#include <cloud_compression/CompressedCloud.h>

namespace cloud_compression
{

class XYZDecompression : public nodelet::Nodelet
{
public:
	XYZDecompression();
	virtual ~XYZDecompression();

	virtual void onInit() override;
private:
	typedef pcl::PointXYZ Point;
	typedef pcl::PointCloud<Point> PointCloud;
	typedef cloud_compression::OctreePointCloudCompression<Point> Compression;

	void handleMsg(const CompressedCloud::ConstPtr& msg);

	ros::Subscriber m_sub;
	ros::Publisher m_pub;
	Compression::Ptr m_compression;
};

}

#endif
