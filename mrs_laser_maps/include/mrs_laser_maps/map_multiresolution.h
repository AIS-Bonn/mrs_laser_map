/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Computer Science Institute VI, University of Bonn
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of University of Bonn, Computer Science Institute
 *     VI nor the names of its contributors may be used to endorse or
 *     promote products derived from this software without specific
 *     prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef _MULTI_RESOLUTIONAL_MAP_H_
#define _MULTI_RESOLUTIONAL_MAP_H_

#include <ros/time.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>

#include <boost/circular_buffer.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/range/adaptor/reversed.hpp>

#include <mrs_laser_maps/map_point_types.h>
#include <mrs_laser_maps/surfel_map_interface.h>
#include <mrs_laser_maps/map_level.h>

namespace mrs_laser_maps
{
template <typename MapPointType = pcl::PointXYZ, typename BufferType = boost::circular_buffer_space_optimized<MapPointType> >
class MultiResolutionalMap : public SurfelMapInterface
{
public:
  
  typedef typename mrs_laser_maps::MapLevel<MapPointType, BufferType>::GridCellType GridCellType;

  typedef mrs_laser_maps::MapLevel<MapPointType, BufferType> MapLevelType;
  typedef boost::shared_ptr<MapLevelType> MapLevelPtr;
  typedef boost::shared_ptr<MultiResolutionalMap<MapPointType>> MapPtr;
  typedef boost::shared_ptr<MultiResolutionalMap<MapPointType>> Ptr;
  
  typedef std::vector<MapLevelPtr> LevelList;
  typedef typename LevelList::iterator LevelListIterator;
  typedef typename LevelList::reverse_iterator LevelListReverseIterator;

  typedef std::vector<GridCellType, Eigen::aligned_allocator<GridCellType>> AlignedCellVector;
  
  typedef typename std::vector<AlignedCellVector, Eigen::aligned_allocator<AlignedCellVector>> AlignedCellVectorVector;

  typedef std::vector<GridCellType*> CellPointerVector;
  typedef std::vector<CellPointerVector> CellPointerVectorVector;

  typedef pcl::PointCloud<MapPointType> PointCloud;
  typedef typename PointCloud::Ptr PointCloudPtr;

  typedef typename MapLevel<MapPointType, BufferType>::CircularBufferPtr CircularBufferPtr;
  typedef typename MapLevel<MapPointType, BufferType>::CircularBufferIterator CircularBufferIterator;

  MultiResolutionalMap(int size = 16, float resolution = 4, int levels = 4, int cell_capacity = 100,
                       std::string frame_id = "base_link");

  virtual ~MultiResolutionalMap()
  {

  };

  MultiResolutionalMap(const MultiResolutionalMap& map);

  void initMap();

  void decreaseAll(float decrease_rate)
  {
    for (const MapLevelPtr& level : level_maps_)
    {
      level->decreaseAll(decrease_rate);
    }
    evaluated_ = false;
  }

  virtual void set(const MapPointType& p, bool update_occupancy = true);

  void setLevel(const MapPointType& p, size_t level)
  {
    if (level >= 0 && level < level_maps_.size())
    {
      level_maps_[level]->set(p);
    }
  }
  
  void setLevel(const PointCloudPtr& points, size_t level)
  {
    if (level >= 0 && level < level_maps_.size())
    {
      level_maps_[level]->set(points);
    }
  }

  void setAllEndPointFlags( bool flag )
  {
    for (unsigned int i = 0; i < level_maps_.size(); i++)
    {
      level_maps_[i]->setAllEndPointFlags( flag );
    }
  }
  
  void setEndPointFlag(const MapPointType& p, const Eigen::Matrix4f& origin)
  {
    for (unsigned int i = 0; i < level_maps_.size(); i++)
    {
      level_maps_[i]->setEndPointFlag(p, origin);
    }
  }

  virtual void insertRay(const MapPointType& p, const Eigen::Matrix4f& origin)
  {
    for (auto& level : boost::adaptors::reverse(level_maps_) )
    {
      level->insertRay(p, origin);
    }
    evaluated_ = false;
  }

  void evaluateAll();

  void unEvaluateAll();

  inline double getResolution() const
  {
    return resolution_;
  }

  inline unsigned int getLevels() const
  {
    return level_maps_.size();
  }

  double getCellSize(int level) const
  {
    if (level >= 0 && level < (int)level_maps_.size())
    {
      return level_maps_[level]->getCellSize();
    }
    return 0;
  }

  double getCellSize() const
  {
    return getCellSize(0);
  }
  void getOccupiedCells(std::vector<pcl::PointXYZ>& cells );
  void getOccupiedCells(std::vector<pcl::PointXYZ>& cells, int level );

  void getOccupiedCells(AlignedCellVector& cells, int level);

  void getCellPoints(PointCloudPtr points);
  void getCellPointsDownsampled(PointCloudPtr points, unsigned int pointsPerCell);

  void getOccupiedCellsWithOffset(AlignedCellVector& cells, std::vector<Eigen::Vector3f>& offsets, int level);
  
  void getOccupiedCellsWithOffset(mrs_laser_maps::SurfelMapInterface::CellPtrVector& cells, std::vector<Eigen::Vector3f>& offsets, int level) ;
  
  void getOccupiedCellsWithOffset(CellPointerVector& cells, std::vector<Eigen::Vector3f>& offsets);
  
  void getOccupiedCellsWithOffset(mrs_laser_maps::SurfelMapInterface::CellPtrVector& cells, std::vector<Eigen::Vector3f>& offsets);

  void getCellsWithOffset(CellPointerVector& cells, std::vector<Eigen::Vector3f>& offsets, int level,
                          float occupancyThreshold = OCCUPANCY_UNKNOWN);

  int getCellPoints(const pcl::PointXYZ& point, PointCloudPtr cell_points)
  {
    int pointsNr = 0;

    for (unsigned int i = 0; i < level_maps_.size(); i++)
    {
      pointsNr += level_maps_[i]->getCellPoints(point, cell_points);
    }
    return pointsNr;
  }

  inline bool getCell(const Eigen::Vector3f& point, GridCellType*& cell_ptr, Eigen::Vector3f& cell_offset, int& level)
  {
    for (unsigned int i = 0; i < level_maps_.size(); i++)
    {
      if (level_maps_[i]->getCell(point, cell_ptr, cell_offset))
      {
        level = i;
        return true;
      }
    }
    return false;
  }

  bool getCell(const Eigen::Vector3f& point, SurfelCellInterface*& cell_ptr, Eigen::Vector3f& cellOffset, int& level) 
  {
    return getCell(point, cell_ptr, cellOffset, level);
  }
  
  void getOccupiedCells(mrs_laser_maps::SurfelMapInterface::CellPtrVector& cells, int level) 
  {
    cells.clear();

    std::vector<Eigen::Vector3f> offsets;

    if (level >= 0 && level < static_cast<int>(level_maps_.size()))
    {
      level_maps_[level]->getOccupiedCellsWithOffset(cells, offsets, (level > 0));
    }
  }

    
  bool getCell(const Eigen::Vector3f& point, mrs_laser_maps::SurfelMapInterface::CellPtrVector& cellPtrs,
                      std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& cellOffsets,
                      std::vector<int>& levels, int neighbors = 0, bool reverse = false);
  
  void lock()
  {
    mutex_.lock();
  }

  void unlock()
  {
    mutex_.unlock();
  }

  void translateMap(const Eigen::Vector3f& translation);

  void getCellPoints(PointCloudPtr points, int level);

  unsigned int getNumCellPoints();

  inline std::string getFrameId() const
  {
    return frame_id_;
  }

  void setLastUpdateTimestamp(ros::Time ts)
  {
    last_update_stamp_ = ts;
  }

  inline ros::Time getLastUpdateTimestamp() const
  {
    return last_update_stamp_;
  }

  virtual void addCloudInner(PointCloudPtr cloud, bool updateOccupancy = false,
                        const Eigen::Matrix4f& origin = Eigen::Matrix4f::Identity());

  virtual void addCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool updateOccupancy = false, 
                        const Eigen::Matrix4f& origin = Eigen::Matrix4f::Identity());
  
  virtual void addCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, bool updateOccupancy = false, 
                        const Eigen::Matrix4f& origin = Eigen::Matrix4f::Identity());

  virtual void addCloud(pcl::PointCloud<PointXYZRGBScanLabel>::Ptr cloud, bool updateOccupancy = false, 
                        const Eigen::Matrix4f& origin = Eigen::Matrix4f::Identity());

  virtual void addCloud(pcl::PointCloud<PointXYZScanLine>::Ptr cloud, bool updateOccupancy = false, 
                        const Eigen::Matrix4f& origin = Eigen::Matrix4f::Identity());
  
  void setScanNumber(PointCloudPtr cloud){};
  
  int getSizeInMeters() const
  {
    return size_;
  }

  int getCellCapacity() const
  {
    return cell_capacity_;
  }

  void setUpdateMask(std::vector<std::vector<std::vector<bool>>>& updateMask);

  void setConicalUpdateMask(const Eigen::Vector3f& position, const Eigen::Vector3f& orientation, float angle);

  void setOccupancyParameters(float clampingThreshMin, float clampingThreshMax, float probHit, float probMiss);

  float getClampingThreshMin()
  {
    return level_maps_[0]->getClampingThreshMin();
  }
  float getClampingThreshMax()
  {
    return level_maps_[0]->getClampingThreshMax();
  }

  unsigned int getScanNumber() const
  {
    return scan_number_;
  }
  
  bool isEvaluated() const
  {
    return evaluated_;
  }
  
protected:

  unsigned int size_;
  unsigned int resolution_;
  std::string frame_id_;

  int cell_capacity_;

  ros::Time last_update_stamp_;

  std::vector<MapLevelPtr> level_maps_;

  boost::mutex mutex_;
  
  unsigned int scan_number_;
  
  bool evaluated_;
};

//template <>
//  inline void MultiResolutionalMap<PointXYZRGBScanLabel, mrs_laser_maps::cell_buffer<PointXYZRGBScanLabel>>::setScanNumber(pcl::PointCloud<PointXYZRGBScanLabel>::Ptr cloud);

  template <>
  inline void MultiResolutionalMap<PointXYZRGBScanLabel, boost::circular_buffer_space_optimized<PointXYZRGBScanLabel>>::setScanNumber(pcl::PointCloud<PointXYZRGBScanLabel>::Ptr cloud);
}

#include <impl/map_multiresolution.hpp>

#endif

