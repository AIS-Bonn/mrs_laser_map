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

#include <boost/circular_buffer.hpp>

#include <pcl/common/time.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <mrs_laser_maps/grid_cell_with_statistics.h>

namespace mrs_laser_maps
{
	
  static const float OCCUPANCY_UNKNOWN = 0.5f;

template <typename PointT = pcl::PointXYZ>
class MapLevel
{
public:
  typedef GridCellWithStatistics<PointT> GridCellType;

  typedef boost::shared_ptr<MapLevel<PointT>> MapLevelPtr;

  typedef pcl::PointCloud<PointT> PointCloud;
  typedef boost::shared_ptr<PointCloud> PointCloudPtr;

  typedef boost::circular_buffer<PointT> CircularBuffer;
  typedef boost::shared_ptr<boost::circular_buffer_space_optimized<PointT>> CircularBufferPtr;
  typedef typename CircularBuffer::iterator CircularBufferIterator;

  typedef std::vector<GridCellType, Eigen::aligned_allocator<GridCellType>> AlignedCellVector;

  typedef std::vector<GridCellType*> CellPointerVector;

  MapLevel(int size_in_meters = 10, float resolution = 4, int cell_capacity = 100);
  ~MapLevel();

  void initMap();

  void decreaseAll(float decrease_rate);

  void evaluateAll();



  void unEvaluateAll();

  void set(const PointT& p, bool update_occupancy = true);

  inline double getResolution()
  {
    return resolution_;
  }

  inline double getCellSize()
  {
    return 1.f / resolution_;
  }

  inline double getCellSize(int l)
  {
    return getCellSize();
  }

  bool inCenter(int x, int y, int z);

  // TODO: pcl::PointXYZ to Eigen::Vector3f>
  void getOccupiedCells(std::vector<pcl::PointXYZ>& cells, bool omit_center = false );

  void getOccupiedCells(AlignedCellVector& cells, bool omit_center = false);

  void getOccupiedCellsWithOffset(AlignedCellVector& cells, std::vector<Eigen::Vector3f>& offsets, bool omit_center);

  void getOccupiedCellsWithOffset(CellPointerVector& cells, std::vector<Eigen::Vector3f>& offsets, bool omit_center);

  void getCellsWithOffset(CellPointerVector& cells, std::vector<Eigen::Vector3f>& offsets, bool omit_center,
                          float occupancy_threshold = OCCUPANCY_UNKNOWN);

  void getCellPoints(PointCloudPtr points, bool omit_center = false);

  void getCellPointsDownsampled(PointCloudPtr points, unsigned int points_per_cell);

  unsigned int getNumCellPoints();

  bool getCell(const Eigen::Vector3f& point, std::vector<GridCellType*>& cell_ptrs,
               std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& cell_offsets,
               std::vector<int>& levels, int level, int neighbors = 0);

  inline void setAllOccupancies(float occupancy);

  bool getCell(const Eigen::Vector3f& point, GridCellType*& cellPtr, Eigen::Vector3f& cell_offset,
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

  inline pcl::PointXYZ getOdomentryIncrement()
  {
    return translation_increment_;
  }

  void insertRay(const PointT& p, const Eigen::Matrix4f& sensor_transform);

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

  void getCellPointsByScanLabel(PointCloudPtr points, unsigned int scan_id, unsigned int scan_line_id,
                                bool omit_center = false);

  void deleteCellPointsByScanLabel(PointCloudPtr points, unsigned int scan_id, bool omit_center = false);

protected:
  bool calcIndices(const PointT& p, int& x, int& y, int& z) const;
  bool calcIndices(const Eigen::Vector3f& p, int& x, int& y, int& z) const;

  template <typename U = PointT, typename std::enable_if<!std::is_same<U, pcl::PointXYZ>::value, int>::type = 0>
  bool calcIndices(const pcl::PointXYZ& p, int& x, int& y, int& z) const
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
};
}

#include <impl/map_level.hpp>

#endif


