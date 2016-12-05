#include <gtest/gtest.h>
#include <boost/concept_check.hpp>

#include <ros/console.h>

#include <mrs_laser_maps/map_point_types.h>

#include <mrs_laser_maps/map_multiresolution.h>
#include <mrs_laser_maps/map_multiresolution_refineable.h>
#include <mrs_laser_maps/grid_cell_with_statistics.h>
 
 
 
typedef PointXYZRGBScanLabel PointType;
typedef pcl::PointCloud<PointXYZRGBScanLabel> PointCloudType;
typedef pcl::PointCloud<PointXYZRGBScanLabel>::Ptr PointCloudPtrType;

PointCloudPtrType g_cloud(new PointCloudType());

PointCloudPtrType g_cloud_sorted(new PointCloudType());



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

bool equalPointByLabel(PointXYZRGBScanLabel p1, PointXYZRGBScanLabel p2)
{
//   ROS_INFO_STREAM_THROTTLE(0.1, "equal point with label");
  return ( (p1.pointNr == p2.pointNr) && (p1.getVector3fMap().isApprox(p2.getVector3fMap(), 1e-3)) );
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

bool equalByLabel(pcl::PointCloud<PointXYZRGBScanLabel> cloud1, pcl::PointCloud<PointXYZRGBScanLabel> cloud2)
{
//   pcl::PointCloud<PointXYZRGBScanLabel> cloud1(cloud_a);
//   pcl::PointCloud<PointXYZRGBScanLabel> cloud2(cloud_b);
  
  std::sort(cloud1.begin(), cloud1.end(), [] ( const PointXYZRGBScanLabel a, const PointXYZRGBScanLabel b ) {return ( a.pointNr < b.pointNr );});
  std::sort(cloud2.begin(), cloud2.end(), [] ( const PointXYZRGBScanLabel a, const PointXYZRGBScanLabel b ) {return ( a.pointNr < b.pointNr );});
  cloud1.erase(std::unique(cloud1.begin(), cloud1.end(), equalPointByLabel), cloud1.end());
  cloud2.erase(std::unique(cloud2.begin(), cloud2.end(), equalPointByLabel), cloud2.end());
  
  if ( cloud1.size() == 0 ) 
    return false;
  
  
  for (size_t i = 0; i < cloud1.points.size(); ++i)
  {
    if (!equalPointByLabel(cloud1.points[i], cloud2.points[i]))
    {
      return false;
    }
  }
  
  
//   EXPECT_EQ(cloud1.size(), cloud2.size());
// 
//   for (size_t i = 0; i < cloud1.points.size(); ++i)
//   {
//     EXPECT_TRUE(equalPointByLabel(cloud1.points[i], cloud2.points[i]));
//     if (!equalPointByLabel(cloud1.points[i], cloud2.points[i]))
//       ROS_INFO_STREAM_THROTTLE(0.1, "point " << cloud1.points[i].pointNr << " x: " <<  cloud1.points[i].x << " vs. " << cloud2.points[i].pointNr << " x: " <<  cloud2.points[i].x << std::endl);
//   }
  return true;
  
}


bool isSortedAsc(pcl::PointCloud<PointXYZRGBScanLabel>& cloud)
{
  bool sorted = true;
  for (size_t i = 1; i < cloud.points.size(); ++i)
  {
    if ((cloud.points[i].pointNr <= cloud.points[i-1].pointNr ))
    {
      ROS_ERROR_STREAM( "point " << cloud.points[i].pointNr << " vs. " << cloud.points[i-1].pointNr << " for id: " << i);
      sorted = false;
      break;
    }
  }
  return sorted; 
}

bool isSortedDesc(pcl::PointCloud<PointXYZRGBScanLabel>& cloud)
{
  bool sorted = true;
  for (int i = 1; i < cloud.points.size(); ++i)
  {
    if ((cloud.points[i].pointNr >= cloud.points[i-1].pointNr ))
    {
      ROS_ERROR_STREAM( "point " << cloud.points[i].pointNr << " vs. " << cloud.points[i-1].pointNr << " for index: " << i);
      sorted = false;
      break;
    }
  }
  return sorted; 
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


void testEvaluateCell (const mrs_laser_maps::SurfelCellInterface& cell)
{
  
  std::cout << "surfel: " << cell.getSurfel().mean_ << std::endl;
}


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
  ros::Time::init();
  
  typedef PointXYZRGBScanLabel PointType;
  typedef pcl::PointCloud<PointType> PointCloudType;
  typedef pcl::PointCloud<PointType>::Ptr PointCloudPtrType;


  mrs_laser_maps::GridCellWithStatistics<PointType, mrs_laser_maps::cell_buffer<PointType>> cell_2(100);

  cell_2.evaluate();
  
//   mrs_laser_maps::MapLevel<PointType, mrs_laser_maps::cell_buffer<PointType>> map_level(64, 0.5, 2500);
//     map_ptr->addCloud(cloud);
// std::cout << " map level created. " << std::endl;

//     mrs_laser_maps::MultiResolutionalMap<PointType, mrs_laser_maps::cell_buffer<PointType>> map(64, 8.0, 4, 2500, "base_link");

//   mrs_laser_maps::MultiResolutionalMap<PointType, mrs_laser_maps::cell_buffer<PointType>>::MapPtr map_ptr(
//     new mrs_laser_maps::MultiResolutionalMap<PointType, mrs_laser_maps::cell_buffer<PointType>>(64, 8.0, 4, 2500, "base_link"));
//     map_ptr->addCloud(cloud);

  PointCloudPtrType cloud(new PointCloudType(*g_cloud));
//   createRandomCloud<PointType>(*cloud, /*1024*1024*10*/ 1000000);

//   if (pcl::io::loadPCDFile<PointType> ("/tmp/1438951337965005.pcd", *cloud) == -1) //* load the file
//   {
//     PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
//     EXPECT_TRUE(false);
//     return;
// 
//   }
  
  std::vector<PointType> sortVec;
  for (PointType p : cloud->points)
	  sortVec.push_back(p);
  std::sort(sortVec.begin(), sortVec.end(), [] ( const PointType a, const PointType b ) {return ( a.pointNr < b.pointNr );	});
  sortVec.erase( unique( sortVec.begin(), sortVec.end(),[] ( const PointType a, const PointType b ) {return ( a.pointNr == b.pointNr );	} ), sortVec.end() );
 
  PointCloudPtrType cloud_sorted(new PointCloudType());
  for (PointType p : sortVec)
    cloud_sorted->points.push_back(p);
  
  int map_levels = 1;
  
  mrs_laser_maps::MultiResolutionalMap<PointType, boost::circular_buffer_space_optimized<PointType>> map(64, 8.0, map_levels, 2500, "base_link");	
  pcl::StopWatch watch_add;
  map.addCloud(cloud_sorted, false);
  std::cout << " adding to standard map wo: " << watch_add.getTime() << std::endl;
  mrs_laser_maps::MultiResolutionalMap<PointType, boost::circular_buffer_space_optimized<PointType>> map_occ(64, 8.0, map_levels, 2500, "base_link");	
  watch_add.reset();
  map_occ.addCloud(cloud_sorted, true);
  std::cout << " adding to standard map with: " << watch_add.getTime() << " and " << cloud_sorted->points.size() << std::endl;
  
  
  mrs_laser_maps::MultiResolutionalMapRefineable<PointType, mrs_laser_maps::cell_buffer<PointType>> map_ref(64, 8.0, map_levels, 2500, "base_link");	
  watch_add.reset();
  map_ref.addCloud(cloud_sorted, false);
  std::cout << " adding wo: " << watch_add.getTime() << std::endl;
  mrs_laser_maps::MultiResolutionalMapRefineable<PointType, mrs_laser_maps::cell_buffer<PointType>>  map_ref_occ(64, 8.0, map_levels, 2500, "base_link");	
  watch_add.reset();
  map_ref_occ.addCloud(cloud_sorted, true);
  std::cout << " adding with: " << watch_add.getTime() << std::endl;
  
  
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
   
   
   
   std::cout << " size: " << sizeof(pcl::PointXYZ) << " " << sizeof(PointType) <<  std::endl;
   
   mrs_laser_maps::GridCellWithStatistics<PointType, boost::circular_buffer_space_optimized<PointType>> cell(5000);
   
   for (unsigned int i = 0; i < g_cloud_sorted->points.size(); ++i)
   {
     cell.addPoint(g_cloud_sorted->points[i]);
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
   GridCellTestType cell_ptr(5000);

   for (unsigned int i = 0; i < g_cloud_sorted->points.size(); ++i)
   {
      cell_ptr.addPoint(g_cloud_sorted->points[i]);
//       cell_ptr.addPoint(point_ptr);
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
      
      
      
    boost::shared_ptr<PointType> point_ptr3 = boost::make_shared<PointType>(cloud->points[0]); // = buffer.front_ptr();
    for (unsigned int i = 0; i < 1000; i++)
    {
      mrs_laser_maps::cell_buffer<PointType> buffer(5);
      buffer.push_front(point_ptr3);
      std::cout << " ref count " << point_ptr3.use_count() << std::endl;

      for (unsigned int i = 0; i < cloud->points.size(); ++i)
      {
	PointType p = cloud->points[i];
	p.scanlineNr = 1;
	buffer.push_front(cloud->points[i]);
      }
            std::cout << " ref count2 " << point_ptr3.use_count() << std::endl;
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


TEST(MapTests, testLabelMethods)
{
  
//   typedef typename mrs_laser_maps::MultiResolutionalMapRefineable<PointType, mrs_laser_maps::cell_buffer<PointType>> MapType;
//   
//   
//   MapType map(64, 8.0, 1, 10000, "base_link");	
//   
//   
//   map.addCloud(g_cloud_sorted, false);
//   
//   
//   PointCloudPtrType map_cloud(boost::make_shared<PointCloudType>());
//   map.getCellPoints(map_cloud);
  
  
  PointCloudPtrType cell_cloud(boost::make_shared<PointCloudType>());
  typedef mrs_laser_maps::GridCellWithStatistics<PointType, mrs_laser_maps::cell_buffer<PointType>> CellType ;
  CellType cell(g_cloud_sorted->size());
  
  for (unsigned int i = 0; i < g_cloud_sorted->points.size(); ++i)
  {
    cell.addPoint(g_cloud_sorted->points[i]);
  }
    
  EXPECT_TRUE(isSortedAsc(*g_cloud_sorted));

  /*
   * test if getting -> updating without changing works
   */
  for (typename CellType::CircularBufferIterator it_cell = cell.getPoints()->begin(); it_cell != cell.getPoints()->end();
	it_cell++)
  {
    cell_cloud->points.push_back(*it_cell);
    
  }
  
  EXPECT_TRUE(equalByLabel(*g_cloud, *cell_cloud));
  EXPECT_TRUE(isSortedDesc(*cell_cloud));
  
  for (auto const &point : g_cloud_sorted->points)
  {
    PointType point_in_cell;
    cell.getPointByLabel(point.pointNr, point_in_cell);
    cell.updateByLabel(point_in_cell);
  }
  
  cell_cloud->clear();
  for (typename CellType::CircularBufferIterator it_cell = cell.getPoints()->begin(); it_cell != cell.getPoints()->end();
	it_cell++)
  {
    cell_cloud->points.push_back(*it_cell);
 
  }
  
  EXPECT_TRUE(equalByLabel(*g_cloud, *cell_cloud));

  EXPECT_TRUE(isSortedDesc(*cell_cloud));

  
  /*
   * 
   * test if getting -> updating with changing works
   */
  for (auto const &point : g_cloud_sorted->points)
  {
    PointType point_in_cell;
    cell.getPointByLabel(point.pointNr, point_in_cell);
    point_in_cell.x = 42.f;
    EXPECT_TRUE(cell.updateByLabel(point_in_cell));
  }
  
  cell_cloud->clear();
  for (typename CellType::CircularBufferIterator it_cell = cell.getPoints()->begin(); it_cell != cell.getPoints()->end();
	it_cell++)
  {
    cell_cloud->points.push_back(*it_cell);
  }
  EXPECT_FALSE(equalByLabel(*g_cloud, *cell_cloud));
  
  
  cell.getPoints()->clear();
  for (unsigned int i = 0; i < g_cloud_sorted->points.size(); ++i)
  {
    cell.addPoint(g_cloud_sorted->points[i]);
  }
  
  /*
   * test accessing by scan label. first count how many in the pointcloud then in the cell
   */
  unsigned int scan_label = g_cloud->points[0].scanNr;
  int points_for_label = 0;
  pcl::StopWatch watch_scan_label;
  float cummulative_x_for_scan = 0.f;
  for (auto const &point : g_cloud_sorted->points)
  {
    if (point.scanNr == scan_label)
    {
      points_for_label++;
      cummulative_x_for_scan += point.x;
    }
  }
  std::cout << "scan_label in cloud took: " << watch_scan_label.getTime() << std::endl;
  watch_scan_label.reset();
  int points_for_label_cell = 0;
  float cummulative_x_for_scan_cell = 0.f;
  
  std::vector<unsigned int> labels_for_scan;
  cell.getPoints()->find_scan_id(scan_label, labels_for_scan);
  for (auto const &point_label: labels_for_scan)
  {
    PointType p;
    cell.getPointByLabel(point_label, p);
    points_for_label_cell++;
    cummulative_x_for_scan_cell += p.x;
  }
  std::cout << "scan_label in cell took: " << watch_scan_label.getTime() << std::endl;
  std::cout << " for label " << scan_label << ": " << points_for_label_cell << " points vs. " << points_for_label<< std::endl;
  
  EXPECT_EQ(points_for_label, points_for_label_cell);
  EXPECT_EQ(cummulative_x_for_scan, cummulative_x_for_scan_cell);
  
  /*
   * test if insert/erase works
   */

  
  PointCloudPtrType cloud_erase_test(boost::make_shared<PointCloudType>(*g_cloud_sorted));
  std::vector<PointType> points_to_remove;
  for (int i = 0; i < 50; ++i)
  {
    unsigned int random_point_index = static_cast<unsigned int>( rand() / (RAND_MAX + 1.0) * cloud_erase_test->size() ); 
    EXPECT_TRUE(random_point_index >= 0 && random_point_index < cloud_erase_test->size());
    points_to_remove.push_back(cloud_erase_test->at(random_point_index));
    cloud_erase_test->erase(cloud_erase_test->begin()+random_point_index);  
  }
  
  // erase from grid cell
  for (const auto& point : points_to_remove)
  { 
    for (typename CellType::CircularBufferIterator it_cell = cell.getPoints()->begin(); it_cell != cell.getPoints()->end(); it_cell++)
    {
      if ( (*it_cell).pointNr == point.pointNr)
      {
	cell.getPoints()->erase(it_cell);
      }
    }
  }
  
  // add to point cloud 
  cell_cloud->clear();
  for (typename CellType::CircularBufferIterator it_cell = cell.getPoints()->begin(); it_cell != cell.getPoints()->end();
	it_cell++)
  {
    cell_cloud->points.push_back(*it_cell);
  }
  
  EXPECT_TRUE(equalByLabel(*cloud_erase_test, *cell_cloud));
  EXPECT_TRUE(isSortedDesc(*cell_cloud));
  EXPECT_TRUE(isSortedAsc(*cloud_erase_test));
  
  
  for (const auto& point : points_to_remove)
  {
    cloud_erase_test->points.push_back(point);
  
    for (typename CellType::CircularBufferIterator it_cell = cell.getPoints()->begin(); it_cell != cell.getPoints()->end(); it_cell++)
    {
      if ((*it_cell).pointNr <= point.pointNr) 
      {
	cell.getPoints()->insert(it_cell, point); 
	break;
      }
    }
  }
  
  // add to point cloud 
  cell_cloud->clear();
  for (typename CellType::CircularBufferIterator it_cell = cell.getPoints()->begin(); it_cell != cell.getPoints()->end();
	it_cell++)
  {
    cell_cloud->points.push_back(*it_cell);
    
    static int c = 0;
    if (c++ < 10)
      std::cout << (*it_cell).pointNr << std::endl;
  }
  
  EXPECT_TRUE(equalByLabel(*g_cloud_sorted, *cloud_erase_test));
  EXPECT_TRUE(equalByLabel(*cloud_erase_test, *cell_cloud));
  EXPECT_TRUE(isSortedDesc(*cell_cloud));
//   EXPECT_TRUE(isSortedAsc(*cloud_erase_test));
  
  
/*
  
  for (auto const &point : g_cloud_sorted->points)
  {
    
    PointType point_in_cell;
    cell.getPointByLabel(point.pointNr, point_in_cell);
    point_in_cell.x = 42.f;
    EXPECT_TRUE(cell.updateByLabel(point_in_cell));
  }
  
  cell_cloud->clear();
  for (typename CellType::CircularBufferIterator it_cell = cell.getPoints()->begin(); it_cell != cell.getPoints()->end();
	it_cell++)
  {
    cell_cloud->points.push_back(*it_cell);
  }
  EXPECT_FALSE(equalByLabel(*g_cloud, *cell_cloud));*/
  
  
  
}


int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  srand(static_cast<unsigned>(time(0)));

  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  {
    ros::console::notifyLoggerLevelsChanged();
  }
  
  
  if (pcl::io::loadPCDFile<PointType> ("/tmp/1438951337965005.pcd", *g_cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    EXPECT_TRUE(false);
    return -1;

  }
  
  std::vector<PointType> sortVec;
  for (PointType p : g_cloud->points)
	  sortVec.push_back(p);
  std::sort(sortVec.begin(), sortVec.end(), [] ( const PointType a, const PointType b ) {return ( a.pointNr < b.pointNr );	});
  sortVec.erase( unique( sortVec.begin(), sortVec.end(),[] ( const PointType a, const PointType b ) {return ( a.pointNr == b.pointNr );	} ), sortVec.end() );
 
  for (PointType p : sortVec)
    g_cloud_sorted->points.push_back(p);

  return RUN_ALL_TESTS();
}
