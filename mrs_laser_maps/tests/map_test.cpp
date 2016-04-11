#include <gtest/gtest.h>
#include <boost/concept_check.hpp>

#include <mrs_laser_maps/map_level.h>
#include <mrs_laser_maps/map_multiresolution.h>
#include <ros/console.h>

template <typename PointType>
bool comparePoint(PointType p1, PointType p2)
{
  if (p1.x != p2.x)
    return p1.x > p2.x;
  else if (p1.y != p2.y)
    return p1.y > p2.y;
  else
    return p1.z > p2.z;
}

template <typename PointType>
bool equalPoint(PointType p1, PointType p2)
{
  return (p1.getVector3fMap().isApprox(p2.getVector3fMap(), 1e-3));
}

template <typename PointType>
void createRandomCloud(pcl::PointCloud<PointType>& cloud, unsigned int numPoints)
{
  cloud.points.clear();

  cloud.width = numPoints;
  cloud.height = 1;
  cloud.is_dense = false;
  cloud.points.resize(cloud.width * cloud.height);

  for (size_t i = 0; i < cloud.points.size(); ++i)
  {
    cloud.points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
    cloud.points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
    cloud.points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
  }
}

template <typename PointType>
bool equalXYZ(pcl::PointCloud<PointType>& cloud1, pcl::PointCloud<PointType>& cloud2)
{
  std::sort(cloud1.begin(), cloud1.end(), comparePoint<PointType>);
  std::sort(cloud2.begin(), cloud2.end(), comparePoint<PointType>);
  cloud1.erase(std::unique(cloud1.begin(), cloud1.end(), equalPoint<PointType>), cloud1.end());
  cloud2.erase(std::unique(cloud2.begin(), cloud2.end(), equalPoint<PointType>), cloud2.end());

  //  	for (auto& p: cloud1.points)
  //  		std::cout << std::setprecision(6) << p.x << " " << p.y << " " << p.z << std::endl;
  //
  //  	std::cout << "cloud2: " << std::endl;
  //  	for (auto& p: cloud2.points)
  //  		std::cout << std::setprecision(6) << p.x << " " << p.y << " " << p.z << std::endl;

  EXPECT_EQ(cloud1.size(), cloud2.size());

  if (cloud1.size() != cloud2.size())
  {
    for (auto& p : cloud1.points)
      std::cout << std::setprecision(6) << p.x << " " << p.y << " " << p.z << std::endl;

    std::cout << "cloud2: " << std::endl;
    for (auto& p : cloud2.points)
      std::cout << std::setprecision(6) << p.x << " " << p.y << " " << p.z << std::endl;

    return false;
  }

  for (size_t i = 0; i < cloud1.points.size(); ++i)
  {
    EXPECT_TRUE(equalPoint<PointType>(cloud1.points[i], cloud2.points[i]));
    if (!equalPoint<PointType>(cloud1.points[i], cloud2.points[i]))
      std::cout << "point " << cloud1.points[i] << " vs. " << cloud2.points[i] << std::endl;
  }
  return true;
}

TEST(MapTests, testMapPointXYZRGB)
{
  typedef pcl::PointXYZRGB PointType;
  typedef pcl::PointCloud<PointType> PointCloudType;
  typedef pcl::PointCloud<PointType>::Ptr PointCloudPtrType;

  PointCloudPtrType cloud(new PointCloudType());
  createRandomCloud<PointType>(*cloud, 10000);

  mrs_laser_maps::MultiResolutionalMap<PointType>::MapPtr map_ptr(
      new mrs_laser_maps::MultiResolutionalMap<PointType>(16, 4.f, 4, 100, "base_link"));
  map_ptr->addCloud(cloud);

  PointCloudPtrType cloudMap(new PointCloudType());
  map_ptr->getCellPoints(cloudMap);

  EXPECT_TRUE(equalXYZ<PointType>(*cloud, *cloudMap));
}

TEST(MapTests, testAddWithRaytracingPointXYZ)
{
  typedef pcl::PointXYZ PointType;
  typedef pcl::PointCloud<PointType> PointCloudType;
  typedef pcl::PointCloud<PointType>::Ptr PointCloudPtrType;

  PointCloudPtrType cloud(new PointCloudType());
  createRandomCloud<PointType>(*cloud, 10000);

  mrs_laser_maps::MultiResolutionalMap<PointType>::MapPtr map_ptr(
      new mrs_laser_maps::MultiResolutionalMap<PointType>(16, 4.f, 4, 100, "base_link"));
  map_ptr->addCloud(cloud, true);

  PointCloudPtrType cloudMap(new PointCloudType());
  map_ptr->getCellPoints(cloudMap);

  EXPECT_TRUE(equalXYZ<PointType>(*cloud, *cloudMap));
}

/*
 * test adding pcl::PointXYZ points to the map, translate it back and forth and compare input vs. output
 */
TEST(MapTests, testAddPointsToMapAndTranslateAndComparePointXYZ)
{
  typedef pcl::PointXYZ PointType;
  typedef pcl::PointCloud<PointType> PointCloudType;
  typedef pcl::PointCloud<PointType>::Ptr PointCloudPtrType;

  PointCloudPtrType cloud(new PointCloudType());
  // 	createRandomCloud<PointType>(*cloud, 10);
  PointType pTest;
  pTest.x = 0.569423f;
  pTest.y = 0.143187f;
  pTest.z = 0.0145769f;
  pTest.x = 3.569423f;
  pTest.y = 0.143187f;
  pTest.z = 0.0145769f;
  cloud->points.push_back(pTest);

  mrs_laser_maps::MultiResolutionalMap<PointType>::MapPtr map_ptr(
      new mrs_laser_maps::MultiResolutionalMap<PointType>(64, 8.0, 4, 2500, "base_link"));
  map_ptr->addCloud(cloud);

  Eigen::Vector3f translation;
  translation(0) = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
  translation(1) = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
  translation(2) = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);

  translation(0) = 0.969382f;
  translation(1) = 0.633011f;
  translation(2) = 0.832335f;

  std::cout << "translation: " << translation << std::endl;
  for (unsigned int i = 0; i < 1; ++i)
  {
    std::cout << " -------" << std::endl;
    PointCloudPtrType cloudMap(new PointCloudType());
    map_ptr->getCellPoints(cloudMap);
    for (auto& p : cloudMap->points)
      std::cout << std::setprecision(6) << p.x << " " << p.y << " " << p.z << std::endl;
    std::cout << "size: " << cloudMap->points.size() << std::endl;
    std::cout << "num: " << map_ptr->getNumCellPoints() << std::endl;
    std::cout << " |||||||" << std::endl;
    map_ptr->translateMap(-translation);
    map_ptr->getCellPoints(cloudMap);
    for (auto& p : cloudMap->points)
      std::cout << std::setprecision(6) << p.x << " " << p.y << " " << p.z << std::endl;
    std::cout << "size: " << cloudMap->points.size() << std::endl;
    std::cout << "num: " << map_ptr->getNumCellPoints() << std::endl;
    std::cout << " +++++++" << std::endl;
  }
  // 	for (unsigned int i = 0; i < 10; ++i)
  // 	{
  // 		std::cout << " -------" << std::endl;
  // 		PointCloudPtrType cloudMap(new PointCloudType());
  // 		map_ptr->getCellPoints(cloudMap);
  //  		for (auto& p: cloudMap->points)
  //  			std::cout << std::setprecision(6) << p.x << " " << p.y << " " << p.z << std::endl;
  // 		std::cout << "size: " << cloudMap->points.size() << std::endl;
  // 		std::cout << "num: " << map_ptr->getNumCellPoints() << std::endl;
  // 		std::cout << " |||||||" << std::endl;
  // 		map_ptr->translateMap(translation);
  // 		map_ptr->getCellPoints(cloudMap);
  //  		for (auto& p: cloudMap->points)
  //  			std::cout << std::setprecision(6) << p.x << " " << p.y << " " << p.z << std::endl;
  // 		std::cout << "size: " << cloudMap->points.size() << std::endl;
  // 		std::cout << "num: " << map_ptr->getNumCellPoints() << std::endl;
  // 		std::cout << " +++++++" << std::endl;
  // 	}
  PointCloudPtrType cloudMap(new PointCloudType());
  map_ptr->getCellPoints(cloudMap);

  EXPECT_TRUE(equalXYZ<PointType>(*cloud, *cloudMap));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  srand(static_cast<unsigned>(time(0)));

  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  {
    ros::console::notifyLoggerLevelsChanged();
  }

  return RUN_ALL_TESTS();
}
