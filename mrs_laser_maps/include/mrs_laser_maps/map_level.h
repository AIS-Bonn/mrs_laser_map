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

#ifndef _MAP_LEVEL_H_
#define _MAP_LEVEL_H_

#include <type_traits>

#include <tbb/tbb.h>
#include <tbb/blocked_range3d.h>

#include <boost/circular_buffer.hpp>

#include <pcl/common/time.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <mrs_laser_maps/grid_cell_with_statistics.h>
#include <mrs_laser_maps/cell_buffer.h>
#include <mrs_laser_maps/surfel_map_interface.h>

#define PARALLEL 1

namespace mrs_laser_maps
{
	
  static const float OCCUPANCY_UNKNOWN = 0.5f;

template <typename PointT = pcl::PointXYZ, typename BufferType = boost::circular_buffer_space_optimized<PointT> >
class MapLevel
{
public:  
  
  typedef boost::shared_ptr<MapLevel<PointT, BufferType>> Ptr;
  typedef boost::shared_ptr<MapLevel<PointT, BufferType>> MapLevelPtr;

  typedef pcl::PointCloud<PointT> PointCloud;
  typedef typename PointCloud::Ptr PointCloudPtr;  
  
  typedef typename boost::shared_ptr<BufferType> CircularBufferPtr;
  typedef typename BufferType::iterator CircularBufferIterator;
  typedef GridCellWithStatistics<PointT, BufferType> GridCellType;
  
  typedef std::vector<GridCellType, Eigen::aligned_allocator<GridCellType>> AlignedCellVector;
  typedef std::vector<GridCellType*> CellPointerVector;

  
  typedef typename std::map<uint32_t, CircularBufferIterator> PointByIdMap;
  
  MapLevel(int size_in_meters = 10, float resolution = 4, int cell_capacity = 100);
  MapLevel(const MapLevel &level );

  ~MapLevel();

  void initMap();

  void decreaseAll(float decrease_rate);

  void evaluateAll();

//   template<typename U = PointT, typename std::enable_if<!std::is_same<U, pcl::PointXYZ>::value, int>::type = 0>
  void evaluateAll ( unsigned int skipScan );

  void evaluateWithScanLine (const std::vector<unsigned int>& labels)
  {
    for( size_t iz = 0; iz < map_.size(); iz++ ) 
  {
    for( size_t iy = 0; iy < map_[ iz ].size(); iy++ ) 
    {
      for( size_t ix = 0; ix < map_[ iz ][ iy ].size(); ix++ ) 
      {
	map_[ iz ][ iy ][ ix ].surfel_.clear();
	map_[ iz ][ iy ][ ix ].evaluateByScanLine(labels);
      }
    }
  }
  }

  void evaluateAll ( const std::vector<unsigned int>& labels );
  
  void unEvaluateAll();

  bool set(const PointT& p, bool update_occupancy = true);
  
  void set(const PointCloudPtr& points_in_map_frame, bool update_occupancy = true);
  
  inline double getResolution()
  {
    return resolution_;
  }

  inline double getCellSize()
  {
    return 1.f / resolution_;
  }

  bool inCenter(int x, int y, int z);

  void getOccupiedCells(std::vector<pcl::PointXYZ>& cells, bool omit_center = false );

  void getOccupiedCells(AlignedCellVector& cells, bool omit_center = false);

  void getOccupiedCellsWithOffset(AlignedCellVector& cells, std::vector<Eigen::Vector3f>& offsets, bool omit_center);
  
  void getOccupiedCellsWithOffset(CellPointerVector& cells, std::vector<Eigen::Vector3f>& offsets, bool omit_center);
 
  void getOccupiedCellsWithOffset(mrs_laser_maps::SurfelMapInterface::CellPtrVector& cells, std::vector<Eigen::Vector3f>& offsets, bool omit_center)
  {
    getCellsWithOffset(cells, offsets, omit_center, OCCUPANCY_UNKNOWN); 
  }
  
  void getCellsWithOffset(mrs_laser_maps::SurfelMapInterface::CellPtrVector& cells, std::vector<Eigen::Vector3f>& offsets, bool omit_center,
                          float occupancy_threshold = OCCUPANCY_UNKNOWN);
  
  void getCellsWithOffset(CellPointerVector& cells, std::vector<Eigen::Vector3f>& offsets, bool omit_center,
                          float occupancy_threshold = OCCUPANCY_UNKNOWN);

  void getCellPoints(PointCloudPtr points, bool omit_center = false);

  void getCellPointsDownsampled(PointCloudPtr points, unsigned int points_per_cell);

  unsigned int getNumCellPoints();

  bool getCell(const Eigen::Vector3f& point, mrs_laser_maps::SurfelMapInterface::CellPtrVector& cell_ptrs,
               std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& cell_offsets,
               std::vector<int>& levels, int level, int neighbors = 0);

  inline void setAllOccupancies(float occupancy);

  bool getCell(const Eigen::Vector3f& point, SurfelCellInterface*& cell_ptr, Eigen::Vector3f& cell_offset,
               bool include_empty = false)
  {
    return getCell(point, cell_ptr, cell_offset, include_empty);
  }

  bool getCell(const Eigen::Vector3f& point, GridCellType*& cell_ptr, Eigen::Vector3f& cell_offset,
               bool include_empty = false);
  
  int getCellPoints(const pcl::PointXYZ& point, PointCloudPtr cell_points);

  inline int getLocalCell(const pcl::PointXYZ& point, CircularBufferIterator& begin, CircularBufferIterator& end,
                          float& x_off, float& y_off, float& z_off);

  int getLocalCellPoints(const pcl::PointXYZ& point, CircularBufferIterator& begin, CircularBufferIterator& end,
                         float& x_off, float& y_off, float& z_off);

  bool toCellCoordinate(const PointT& point, PointT& point_in_cell_coords, int& x, int& y, int& z);

  bool toCellCoordinate(const PointT& point, PointT& point_in_cell_coords)
  {
    int x, y, z;
    return toCellCoordinate(point, point_in_cell_coords, x, y, z);
  }

  bool insertRay(const PointT& p, const Eigen::Matrix4f& sensor_transform );
  
  void translateMap(const Eigen::Vector3f& translation);
  void moveMap(const Eigen::Vector3f& translation);

  void setUpdateMask(std::vector<std::vector<std::vector<bool>>>& update_mask);

  void setConicalUpdateMask(const Eigen::Vector3f& position, const Eigen::Vector3f& orientation, float angle);

  /*
  * retain points from coarser level to init grid cell ix, iy, iz
  */
  void retainPoints(int iz, int iy, int ix);

  /*
  * set pointer to coarser level
  */
  void setCoarserLevel(MapLevelPtr coarser_level);

  /*
  * set log odds parameters for occupancy updating
  */
  void setOccupancyParameters(float clamping_thresh_min, float clamping_thresh_max, float prob_hit, float prob_miss);

  float getClampingThreshMin()
  {
    return clamping_thresh_min_;
  }
  float getClampingThreshMax()
  {
    return clamping_thresh_max_;
  }

  void setAllEndPointFlags(bool end_point);

  void setEndPointFlag(const PointT& p, const Eigen::Matrix4f& sensor_transform);

  void getCellPointsByScanLabel(PointCloudPtr points, unsigned int scan_id, bool omit_center = false);

  void getCellPointsByScanLabel(PointCloudPtr points, unsigned int scan_id, PointByIdMap& point_map, bool omit_center = false);
  
  void getCellPointsByScanLabel(PointCloudPtr points, unsigned int scan_id, unsigned int scan_line_id,
                                bool omit_center = false);

  void deleteCellPointsByScanLabel(unsigned int scan_id);

  void deleteCellPointsByScanLabel(PointCloudPtr points, unsigned int scan_id);
  
  void deleteCellPointsByScanLabel(PointCloudPtr points, unsigned int scan_id, unsigned int scan_line_id, 
				   bool omit_center = false);

  void deleteCellPointsByPointLabel(const std::vector<uint32_t>& point_ids);
  
  void clearCellsByOccupancy(std::vector<PointT>& deleted_points);
  
  void clearCellPoints()
  {
    for (size_t iz = 0; iz < map_.size(); iz++)
    {
      for (size_t iy = 0; iy < map_[iz].size(); iy++)
      {
	for (size_t ix = 0; ix < map_[iz][iy].size(); ix++)
	{
	    map_[iz][iy][ix].getPoints()->clear();
	}
      }
    }
  }
 
  void clearUnoccupiedCells()
  {
    for (size_t iz = 0; iz < map_.size(); iz++)
    {
      for (size_t iy = 0; iy < map_[iz].size(); iy++)
      {
	for (size_t ix = 0; ix < map_[iz][iy].size(); ix++)
	{
	  if (map_[iz][iy][ix].getOccupancy() <= OCCUPANCY_UNKNOWN)
	    map_[iz][iy][ix].clearCell();
	}
      }
    }
  }
 
#ifdef DEBUG_CELL_HITS  
  void printCellDebugInfo();
#endif

  std::vector<PointT> dynamic_point_ids_;

  std::vector<GridCellType*> dynamic_cells_;
  
protected:
  inline bool calcIndices(const PointT& p, int& x, int& y, int& z) const;
  inline bool calcIndices(const Eigen::Vector3f& p, int& x, int& y, int& z) const;

  template <typename U = PointT, typename std::enable_if<!std::is_same<U, pcl::PointXYZ>::value, int>::type = 0>
  inline bool calcIndices(const pcl::PointXYZ& p, int& x, int& y, int& z) const
  {
    return calcIndices(p.getVector3fMap(), x, y, z);
  }

  /*
  * transform point from cell frame to map frame
  */
  void cellToMapFrame(const PointT& point_in_cell_coords, PointT& point_in_map, int x, int y, int z);

  /*
  * transform cell frame to map frame
  */
  void cellToMapFrame(Eigen::Vector3f& point_in_map, int x, int y, int z);

  
   
private:
  unsigned int size_;
  float resolution_;
  int cell_capacity_;

  boost::circular_buffer<boost::circular_buffer<boost::circular_buffer<GridCellType>>> map_;

  pcl::PointXYZ translation_increment_;

  std::vector<std::vector<std::vector<bool>>> update_mask_;

  MapLevelPtr coarser_level_;

  float clamping_thresh_min_;
  float clamping_thresh_max_;
  float prob_hit_;
  float prob_miss_;
  
  class EvaluateFunctor
  {
  public:
    EvaluateFunctor(boost::circular_buffer<boost::circular_buffer<boost::circular_buffer<GridCellType>>>* buffer, bool skip_scan_id, unsigned int scan_id )
    : skip_scan_id_(skip_scan_id)
    , scan_id_(scan_id)
    {
      buffer_ = buffer;
      
    }

    ~EvaluateFunctor()
    {
    }

    void operator()(const tbb::blocked_range3d<int>& r) const
    {
      for(int i=r.pages().begin(), i_end=r.pages().end(); i<i_end; i++)
      {
	for(int j=r.rows().begin(), j_end=r.rows().end(); j<j_end; j++)
	{
	  for(int k=r.cols().begin(), k_end=r.cols().end(); k<k_end; k++)
	  {
	    (*buffer_)[i][j][k].surfel_.clear();
	    if (skip_scan_id_)
	    {
	      (*buffer_)[i][j][k].evaluate(scan_id_);
	    }
	    else
	    {
	      (*buffer_)[i][j][k].evaluate();
	    }
	  }
	}
      }
    }
    bool skip_scan_id_;
    unsigned int scan_id_;
    boost::circular_buffer<boost::circular_buffer<boost::circular_buffer<GridCellType>>>* buffer_;
    
  };

  class DeleteByScanLabelFunctor
  {
  public:
    DeleteByScanLabelFunctor(boost::circular_buffer<boost::circular_buffer<boost::circular_buffer<GridCellType>>>* buffer, unsigned int scan_id )
    : scan_id_(scan_id)
    {
      buffer_ = buffer;
      
    }

    ~DeleteByScanLabelFunctor()
    {
    }

    void operator()(const tbb::blocked_range3d<int>& r) const
    {
      for(int i=r.pages().begin(), i_end=r.pages().end(); i<i_end; i++)
      {
	for(int j=r.rows().begin(), j_end=r.rows().end(); j<j_end; j++)
	{
	  for(int k=r.cols().begin(), k_end=r.cols().end(); k<k_end; k++)
	  {
	    if ((*buffer_)[i][j][k].getOccupancy() <= OCCUPANCY_UNKNOWN)
	    {
	      (*buffer_)[i][j][k].getPoints()->clear();
	      continue;
	    }
	    CircularBufferPtr cell_points = (*buffer_)[i][j][k].getPoints();
	    CircularBufferIterator it = cell_points->begin();
	    while ( it != cell_points->end())
	    {
	      if ((*it).scanNr == scan_id_)
	      {
		it = cell_points->erase(it);
	      }
	      else
	      {
		++it;
	      }
	    }
	    if (cell_points->size() == 0)
	    {
	      (*buffer_)[i][j][k].clearCell();
	    }
	  }
	}
      }
    }
    unsigned int scan_id_;
    boost::circular_buffer<boost::circular_buffer<boost::circular_buffer<GridCellType>>>* buffer_;
    
  };

  
  class EvaluateLabelListFunctor
  {
  public:
    EvaluateLabelListFunctor(boost::circular_buffer<boost::circular_buffer<boost::circular_buffer<GridCellType>>>* buffer, const std::vector<unsigned int>& labels )
    : labels_(labels)
    {
      buffer_ = buffer;
      
    }

    ~EvaluateLabelListFunctor()
    {
    }

    void operator()(const tbb::blocked_range3d<int>& r) const
    {
      for(int i=r.pages().begin(), i_end=r.pages().end(); i<i_end; i++)
      {
	for(int j=r.rows().begin(), j_end=r.rows().end(); j<j_end; j++)
	{
	  for(int k=r.cols().begin(), k_end=r.cols().end(); k<k_end; k++)
	  {
	    (*buffer_)[i][j][k].surfel_.clear();
	    
	    (*buffer_)[i][j][k].evaluate(labels_);
	  }
	}
      }
    }
    const std::vector<unsigned int>& labels_;
    boost::circular_buffer<boost::circular_buffer<boost::circular_buffer<GridCellType>>>* buffer_;
    
  };
  
  
  class GetCellPointsFunctor
  {
  public:
    GetCellPointsFunctor(boost::circular_buffer<boost::circular_buffer<boost::circular_buffer<GridCellType>>>* buffer, MapLevel *level_ptr,  boost::shared_ptr<tbb::concurrent_vector<PointCloudPtr>> result_clouds, bool omit_center )
    : level_ptr_(level_ptr)
    , omit_center_(omit_center)
    , result_clouds_(result_clouds)
    {
      buffer_ = buffer;      
    }

    ~GetCellPointsFunctor()
    {
    }

    void operator()(const tbb::blocked_range3d<int>& r) const
    {
      for(int i=r.pages().begin(), i_end=r.pages().end(); i<i_end; i++)
      {
	for(int j=r.rows().begin(), j_end=r.rows().end(); j<j_end; j++)
	{
	  for(int k=r.cols().begin(), k_end=r.cols().end(); k<k_end; k++)
	  {
	    PointCloudPtr temp_cloud = boost::make_shared<PointCloud>();
	    if (omit_center_ && level_ptr_->inCenter(i, j, k))
	    {
	      continue;
	    }
	    if ((*buffer_)[k][j][i].getOccupancy() <= OCCUPANCY_UNKNOWN)
	    {
	      (*buffer_)[k][j][i].getPoints()->clear();
	      continue;
	    }
	    CircularBufferPtr cell_points = (*buffer_)[k][j][i].getPoints();
	    for (CircularBufferIterator it = cell_points->begin(); it != cell_points->end(); it++)
	    {
	      PointT p = (*it);
	      // transform back to map frame
	      level_ptr_->cellToMapFrame(*it, p, i, j, k);
	      temp_cloud->push_back(p);
	    }
	    result_clouds_->push_back(temp_cloud);
	  }
	}
      }
    }
    MapLevel *level_ptr_;
    bool omit_center_;
    boost::shared_ptr<tbb::concurrent_vector<PointCloudPtr>> result_clouds_;
    boost::circular_buffer<boost::circular_buffer<boost::circular_buffer<GridCellType>>>* buffer_;
  };
  
};
}

#include <impl/map_level.hpp>

#endif


