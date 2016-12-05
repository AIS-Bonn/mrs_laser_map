#include <gtest/gtest.h>
#include <boost/concept_check.hpp>

#include <ros/console.h>

#include <mrs_laser_maps/map_point_types.h>
 #include <mrs_laser_maps/map_level.h>
// #include <mrs_laser_maps/map_multiresolution.h>
#include <mrs_laser_maps/grid_cell_with_statistics.h>

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
  typedef pcl::PointXYZ PointType;
  typedef pcl::PointCloud<PointType> PointCloudType;
  typedef pcl::PointCloud<PointType>::Ptr PointCloudPtrType;

  PointCloudPtrType cloud(new PointCloudType());
  createRandomCloud<PointType>(*cloud, 10000);

  mrs_laser_maps::MultiResolutionalMap<PointType>::MapPtr map_ptr(
      new mrs_laser_maps::MultiResolutionalMap<PointType>(16, 4.f, 4, 100, "base_link"));
  map_ptr->addCloud(cloud);

  PointCloudPtrType cloudMap(new PointCloudType());
file:///home/droeschel/workspace/laser_mapping_release/src/mrs_laser_map/mrs_laser_maps/tests/map_test.cpp  map_ptr->getCellPoints(cloudMap);

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


struct PointerPoint
{
  PointerPoint()
  : ptr_(boost::make_shared<PointXYZRGBScanLabel>())
   , x(ptr_->x)
  {
  }
  
  PointerPoint(PointXYZRGBScanLabel point)
  : ptr_(boost::make_shared<PointXYZRGBScanLabel>(point))
   , x(ptr_->x)
  {
  }
  
  PointerPoint& operator=(const PointerPoint& point ) {
        ptr_=point.ptr_;
//         x=ptr_->x;
        return *this;
    }

  
    boost::shared_ptr<PointXYZRGBScanLabel> ptr_;

    float& x;
};

// struct PointerPoint
// {
//   PointerPoint(PointXYZRGBScanLabel point)
//   : ptr_(new PointXYZRGBScanLabel(point))
//   {
//   }
//   
//     PointXYZRGBScanLabel* ptr_;
// };

// struct PointerInt
// {
//   PointerPoint(int v)
//   : ptr_(boost::make_shared<int>(v))
//   {
//   }
//   
//     boost::shared_ptr<int> ptr_;
// };

TEST(MapTests, testPointerStruct)
{
  typedef PointXYZRGBScanLabel PointType;
  typedef pcl::PointCloud<PointType> PointCloudType;
  typedef pcl::PointCloud<PointType>::Ptr PointCloudPtrType;

//   mrs_laser_maps::MultiResolutionalMap<PointType>::MapPtr map_ptr(
//      new mrs_laser_maps::MultiResolutionalMap<PointType>(64, 8.0, 4, 2500, "base_link"));
//     map_ptr->addCloud(cloud);

  mrs_laser_maps::MapLevel<PointType> map_level(64, 8.0, 2500);
//     map_ptr->addCloud(cloud);

  
  PointCloudPtrType cloud(new PointCloudType());
  createRandomCloud<PointType>(*cloud, /*1024*1024*10*/ 1000000);

//    mrs_laser_maps::MultiResolutionalMap<PointType>::MapPtr map_ptr(
//        new mrs_laser_maps::MultiResolutionalMap<PointType>(16, 4.f, 4, 100, "base_link"));
//   map_ptr->addCloud(cloud);

  
//    boost::circular_buffer<PointerPoint> points(5);
//    
// //    boost::shared_ptr<PointerPoint> ptr(boost::make_shared<PointerPoint>(42));
// //    points.push_front(ptr);
// //    
//    boost::circular_buffer<PointerPoint>::iterator iter = points.begin();
// 
// //    int* int_ptr = &(*iter);
// //    std::cout << "point: " << *iter << " " << *int_ptr << " " << int_ptr << std::endl;
//    
// //    points.push_front(2);
// //    points.push_front(3);
// 
//    PointerPoint p(cloud->points[42]);
//    points.push_front(p);
//    
//    std::cout << "point p: " << p.x << std::endl;
//    
//    std::cout << "use count: " << p.ptr_.use_count() << std::endl;
//    
//    for (int i = 0; i < 10; ++i)
//    {
// //      PointerPoint p(cloud->points[i]);
// //      points.push_front(p);
//     points.push_front(PointerPoint(cloud->points[i]));
//      
// //      std::cout << "point: " << *iter << " " << *int_ptr << " " << int_ptr << std::endl;
//    }
//    std::cout << "use count: " << p.ptr_.use_count() << std::endl;
//    
// //   boost::shared_ptr<int> int_ptr(points.front()); 
// 
// //    *ptr = 22;
//    
//    //    points[0] = 11;
//    
//    
// //    points.push_front(2);
// //    points.push_front(3);
// //          std::cout << "pointa: " << *int_ptr.get() << std::endl;
// 
//    for (unsigned int i = 0; i < points.size(); ++i)
//    {
//       std::cout << "point: " << points[i].x << " " << p.x << std::endl;
//    }
//    
// //    float f = 0.42f;
//    
//     p.x = 0.42f;
// //    p.ptr_->x = 0.42f;
//    
//    for (unsigned int i = 0; i < points.size(); ++i)
//    {
//       std::cout << "point: " << points[i].x << " and " << points[i].ptr_->x << "  " << p.x << std::endl;
//    }
//    
   
   
   
//    boost::circular_buffer_space_optimized<int>::allocator_type alloc = points.get_allocator();
   
  
//   mrs_laser_maps::MultiResolutionalMap<PointType>::MapPtr map_ptr(
//       new mrs_laser_maps::MultiResolutionalMap<PointType>(16, 4.f, 4, 100, "base_link"));
//   map_ptr->addCloud(cloud);
//   


//   mrs_laser_maps::MultiResolutionalMap<PointType>::MapPtr map_ptr(
//       new mrs_laser_maps::MultiResolutionalMap<PointType>(64, 8.0, 4, 5000, "base_link"));
   
  
//   mrs_laser_maps::MultiResolutionalMap<PointType>::MapPtr map_ptr(
//       new mrs_laser_maps::MultiResolutionalMap<PointType>(16, 4.f, 4, 100, "base_link"));
//   map_ptr->addCloud(cloud);
  
   pcl::StopWatch watch;
   boost::circular_buffer_space_optimized<PointType> points_2(5000);
   std::cout << " create: " << watch.getTime() << std::endl;
   for (unsigned int i = 0; i < cloud->points.size(); ++i)
   {
     points_2.push_front(cloud->points[i]);
   }
   std::cout << " adding: " << watch.getTime() << std::endl;
   
   
    for (typename boost::circular_buffer_space_optimized<PointType>::iterator it_cell = points_2.begin(); it_cell != points_2.end();
         it_cell++)
    {
      Eigen::Matrix<double, 3, 1> pos;
      pos(0) = (*it_cell).x;
      pos(1) = (*it_cell).y;
      pos(2) = (*it_cell).z;
    }
    std::cout << " accessing: " << watch.getTime() << std::endl;
    watch.reset();
   
   std::vector<PointType> vec;
   vec.reserve(1024*1024*10);
   for (unsigned int i = 0; i < cloud->points.size(); ++i)
   {
     vec.push_back(cloud->points[i]);
   }
   std::cout << " adding vec: " << watch.getTime() << std::endl;
    for (typename std::vector<PointType>::iterator it_cell = vec.begin(); it_cell != vec.end();
         it_cell++)
    {
      Eigen::Matrix<double, 3, 1> pos;
      pos(0) = (*it_cell).x;
      pos(1) = (*it_cell).y;
      pos(2) = (*it_cell).z;
    }
    std::cout << " accessing: " << watch.getTime() << std::endl;
    watch.reset();
   
   
    
//    boost::circular_buffer<PointerPoint> points_3(1024*1024*10);
//    std::cout << " create: " << watch.getTime() << std::endl;
//    for (unsigned int i = 0; i < cloud->points.size(); ++i)
//    {
//      points_3.push_front(cloud->points[i]);
//    }
//    std::cout << " adding: " << watch.getTime() << std::endl;
//    
//    
//     for (typename boost::circular_buffer<PointerPoint>::iterator it_cell = points_3.begin(); it_cell != points_3.end();
//          it_cell++)
//     {
//       Eigen::Matrix<double, 3, 1> pos;
//       pos(0) = (*it_cell).ptr_->x;
//       pos(1) = (*it_cell).ptr_->y;
//       pos(2) = (*it_cell).ptr_->z;
//     }
//     std::cout << " accessing: " << watch.getTime() << std::endl;
//     watch.reset();
   
   
   
   std::cout << " size: " << sizeof(PointType) << " " << sizeof(PointerPoint) <<  std::endl;
   
   mrs_laser_maps::GridCellWithStatistics<PointType, boost::circular_buffer_space_optimized<PointType>> cell(1000000);
   
   for (unsigned int i = 0; i < cloud->points.size(); ++i)
   {
     cell.addPoint(cloud->points[i]);
   }
      
   std::cout << " adding to grid cell with stadnard buffer: " << watch.getTime() << std::endl;
      
   watch.reset();   
      
   for (typename mrs_laser_maps::GridCellWithStatistics<PointType>::CircularBufferIterator it_cell = cell.getPoints()->begin(); it_cell != cell.getPoints()->end();
         it_cell++)
    {
      Eigen::Matrix<double, 3, 1> pos;
      pos(0) = (*it_cell).x;
      pos(1) = (*it_cell).y;
      pos(2) = (*it_cell).z;
    }

    std::cout << " accessing standard grid cell: " << watch.getTime() << std::endl;
    watch.reset();

   boost::shared_ptr<PointType> point_ptr = boost::make_shared<PointType>(cloud->points[0]);
   
   typedef mrs_laser_maps::GridCellWithStatistics<PointType, mrs_laser_maps::cell_buffer<PointType>> GridCellTestType;
   GridCellTestType cell_ptr(1000000);

   for (unsigned int i = 0; i < cloud->points.size(); ++i)
   {
//      cell_ptr.addPoint(cloud->points[i]);
      cell_ptr.addPoint(point_ptr);
   }
   std::cout << " adding to grid cell with pointers: " << watch.getTime() << std::endl;
   watch.reset();  
      
   int c = 0;
   for (typename GridCellTestType::CircularBufferIterator it_cell = cell_ptr.getPoints()->begin(); it_cell != cell_ptr.getPoints()->end();
         it_cell++)
    {
      Eigen::Matrix<double, 3, 1> pos;
      pos(0) = (*it_cell).x;
      pos(1) = (*it_cell).y;
      pos(2) = (*it_cell).z;
      c++;
    }
    
     std::cout << " accessing pointer grid cell: " << watch.getTime() << " with " << cell_ptr.getPoints()->size() << " and " << c << std::endl;
    watch.reset();
	 
    
    mrs_laser_maps::cell_buffer<PointType> buffer(5);
     
      for (unsigned int i = 0; i < cloud->points.size(); ++i)
   {
     buffer.push_front(cloud->points[i]);
   }

   mrs_laser_maps::cell_buffer<PointType>::iterator it_wrapper =  buffer.begin() ;

     
   for (; it_wrapper != buffer.end();
         it_wrapper++)
    {
       Eigen::Matrix<double, 3, 1> pos;
        pos(0) = (*it_wrapper).x;
        pos(1) = (*it_wrapper).y;
	pos(2) = (*it_wrapper).z;
//        
       std::cout << " x: " << pos(0) << std::endl;
    }

    
    std::cout << " ref count " << point_ptr.use_count() << std::endl;
    buffer.push_front(point_ptr);
    
    boost::shared_ptr<PointType> point_ptr2 = buffer.front_ptr();
    
    buffer.push_front(cloud->points[0]);
    
    
    std::cout << " ref count " << point_ptr.use_count() << std::endl;
    
    
      
//   sleep(100);
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
