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

#include <mrs_laser_maps/map_point_types.h>
#include <mrs_laser_maps/map_level.h>

namespace mrs_laser_maps
{
template <typename MapPointType = pcl::PointXYZ>
class MultiResolutionalMap
{
public:
  typedef GridCellWithStatistics<MapPointType> GridCellType;

  typedef boost::shared_ptr<mrs_laser_maps::MapLevel<MapPointType>> MapLevelPtr;
  typedef boost::shared_ptr<MultiResolutionalMap<MapPointType>> MapPtr;

  typedef std::vector<MapLevelPtr> LevelList;
  typedef typename LevelList::iterator LevelListIterator;
  typedef typename LevelList::reverse_iterator LevelListReverseIterator;

  typedef std::vector<GridCellType, Eigen::aligned_allocator<GridCellType>> AlignedCellVector;

  typedef typename std::vector<AlignedCellVector, Eigen::aligned_allocator<AlignedCellVector>> AlignedCellVectorVector;

  typedef std::vector<GridCellType*> CellPointerVector;
  typedef std::vector<CellPointerVector> CellPointerVectorVector;

  typedef pcl::PointCloud<MapPointType> PointCloud;
  typedef boost::shared_ptr<PointCloud> PointCloudPtr;

  typedef typename MapLevel<MapPointType>::CircularBuffer CircularBuffer;
  typedef typename MapLevel<MapPointType>::CircularBufferPtr CircularBufferPtr;
  typedef typename MapLevel<MapPointType>::CircularBufferIterator CircularBufferIterator;

  MultiResolutionalMap(int size = 16, float resolution = 4, int levels = 4, int cell_capacity = 100,
                       std::string frame_id = "base_link");

  virtual ~MultiResolutionalMap()
  {
    for (unsigned int i = 0; i < level_maps_.size(); i++)
    {
      level_maps_[i].reset();
    }
  };

  MultiResolutionalMap(MultiResolutionalMap& map);

  void initMap();

  void decreaseAll(float decrease_rate)
  {
    for (auto level : level_maps_)
    {
      level->decreaseAll(decrease_rate);
    }
  }

  void set(const MapPointType& p, bool update_occupancy = true)
  {
    for (unsigned int i = 0; i < level_maps_.size(); i++)
    {
      level_maps_[i]->set(p, update_occupancy);
    }
  }

  void setLevel(const MapPointType& p, size_t level)
  {
    if (level >= 0 && level < level_maps_.size())
    {
      level_maps_[level]->set(p);
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

  void insertRay(const MapPointType& p, const Eigen::Matrix4f& origin)
  {
    for (MapLevelPtr l : level_maps_)
    {
      l->insertRay(p, origin);
    }
  }

  void evaluateAll()
  {
    for (unsigned int i = 0; i < level_maps_.size(); i++)
    {
      level_maps_[i]->evaluateAll();
    }
  }



  void unEvaluateAll()
  {
    for (unsigned int i = 0; i < level_maps_.size(); i++)
    {
      level_maps_[i]->unEvaluateAll();
    }
  }

  double getResolution()
  {
    return resolution_;
  }

  unsigned int getLevels() const
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

  void getOccupiedCellsWithOffset(CellPointerVector& cells, std::vector<Eigen::Vector3f>& offsets, int level);

  void getOccupiedCellsWithOffset(CellPointerVector& cells, std::vector<Eigen::Vector3f>& offsets);

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

  inline bool getCell(const Eigen::Vector3f& point, GridCellType*& cellPtr, Eigen::Vector3f& cellOffset, int& level)
  {
    for (unsigned int i = 0; i < level_maps_.size(); i++)
    {
      if (level_maps_[i]->getCell(point, cellPtr, cellOffset))
      {
        level = i;
        return true;
      }
    }
    return false;
  }

  inline bool getCell(const Eigen::Vector3f& point, std::vector<GridCellType*>& cellPtrs,
                      std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& cellOffsets,
                      std::vector<int>& levels, int neighbors = 0, bool reverse = false)
  {
    if (reverse)
    {
      for (int i = level_maps_.size() - 1; i >= 0; --i)
      {
        if (level_maps_[i]->getCell(point, cellPtrs, cellOffsets, levels, i, neighbors))
        {
          return true;
        }
      }
    }
    else
    {
      for (unsigned int i = 0; i < level_maps_.size(); i++)
      {
        if (level_maps_[i]->getCell(point, cellPtrs, cellOffsets, levels, i, neighbors))
        {
          return true;
        }
      }
    }
    return false;
  }

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

  std::string getFrameId()
  {
    return frame_id_;
  }

  void setLastUpdateTimestamp(ros::Time ts)
  {
    last_update_stamp_ = ts;
  }

  ros::Time getLastUpdateTimestamp()
  {
    return last_update_stamp_;
  }

  virtual void addCloudInner(PointCloudPtr cloud, bool updateOccupancy = false,
                        const Eigen::Matrix4f& origin = Eigen::Matrix4f::Identity());
	
	void addCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool updateOccupancy = false, 
                        const Eigen::Matrix4f& origin = Eigen::Matrix4f::Identity());
	
	void addCloud(pcl::PointCloud<PointXYZRGBScanLabel>::Ptr cloud, bool updateOccupancy = false, 
                        const Eigen::Matrix4f& origin = Eigen::Matrix4f::Identity());
	
	void addCloud(pcl::PointCloud<PointXYZScanLine>::Ptr cloud, bool updateOccupancy = false, 
                        const Eigen::Matrix4f& origin = Eigen::Matrix4f::Identity());

  inline pcl::PointXYZ getOdomentryIncrement(int level);

  int getSizeInMeters()
  {
    return size_;
  }

  int getCellCapacity()
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

protected:
  void moveMap(const Eigen::Vector3f& translation);

  // private:
  unsigned int size_;
  unsigned int resolution_;
  std::string frame_id_;

  int cell_capacity_;

  ros::Time last_update_stamp_;

  std::vector<MapLevelPtr> level_maps_;

  boost::mutex mutex_;
};

// typedef PointXYZRGBScanLabel MapPointType;
// typedef PointXYZScanLine InputPointType;
typedef MultiResolutionalMap<MapPointType>::GridCellType GridCellType;
typedef boost::shared_ptr<MultiResolutionalMap<MapPointType>> MultiResolutionalMapPtr;
}

#include <impl/map_multiresolution.hpp>

#endif

