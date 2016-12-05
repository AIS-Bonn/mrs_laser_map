

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <float.h>

// TF
#include <tf/transform_listener.h>
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"

//Filters
#include "filters/filter_chain.h"

/** @b CloudToCloudFilterChain is a simple node that filters points in a point cloud and publishes the results.
 */
class CloudToCloudFilterChain
{
public:

  double laser_max_range_;           // Used in laser scan projection
  int window_;
    
  bool high_fidelity_;                    // High fidelity (interpolating time across scan)
  std::string target_frame_;                   // Target frame for high fidelity result
  std::string scan_topic_, cloud_topic_;

  ros::NodeHandle nh;
  ros::NodeHandle private_nh;
    
  // TF
  tf::TransformListener tf_;

  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_;
  tf::MessageFilter<sensor_msgs::PointCloud2> filter_;

  double tf_tolerance_;
  filters::FilterChain<sensor_msgs::PointCloud2> cloud_filter_chain_;
  ros::Publisher cloud_pub_;

  ////////////////////////////////////////////////////////////////////////////////
  CloudToCloudFilterChain () : private_nh("~"), filter_(tf_, "", 50),
                   cloud_filter_chain_("sensor_msgs::PointCloud2")
  {
    private_nh.param("high_fidelity", high_fidelity_, false);
    private_nh.param("notifier_tolerance", tf_tolerance_, 0.03);
    private_nh.param("target_frame", target_frame_, std::string ("base_link"));

    filter_.setTargetFrame(target_frame_);
    filter_.registerCallback(boost::bind(&CloudToCloudFilterChain::cloudCallback, this, _1));
    filter_.setTolerance(ros::Duration(tf_tolerance_));

    sub_.subscribe(private_nh, "input", 50);

    filter_.connectInput(sub_);

    cloud_pub_ = private_nh.advertise<sensor_msgs::PointCloud2> ("output", 10);

    std::string cloud_filter_xml;

    cloud_filter_chain_.configure("cloud_filter_chain", private_nh);
  }

  ////////////////////////////////////////////////////////////////////////////////
  void
  cloudCallback (const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
  {
    sensor_msgs::PointCloud2 filtered_cloud;
    cloud_filter_chain_.update (*cloud_msg, filtered_cloud);

    cloud_pub_.publish(filtered_cloud);
  }

} ;



int
main (int argc, char** argv)
{
  
//   if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
//    ros::console::notifyLoggerLevelsChanged();
//   }

  ros::init (argc, argv, "cloud_to_cloud_filter_chain");
  ros::NodeHandle nh;
  CloudToCloudFilterChain f;

  ros::spin();

  return (0);
}


