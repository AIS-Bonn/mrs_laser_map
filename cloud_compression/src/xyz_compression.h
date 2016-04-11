// Compression for clouds without color
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef XYZ_COMPRESSION_H
#define XYZ_COMPRESSION_H

#include <nodelet/nodelet.h>

#include <cloud_compression/octree_pointcloud_compression.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <ros/subscriber.h>
#include <ros/publisher.h>

namespace cloud_compression
{

class XYZCompression : public nodelet::Nodelet
{
public:
	XYZCompression();
	virtual ~XYZCompression();

	virtual void onInit() override;
private:
	typedef pcl::PointXYZ Point;
	typedef pcl::PointCloud<Point> PointCloud;
	typedef cloud_compression::OctreePointCloudCompression<Point> Compression;

	void handleCloud(const PointCloud::ConstPtr& cloud);

	ros::Subscriber m_sub;
	ros::Publisher m_pub;
	Compression::Ptr m_compression;
};

}

#endif
