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

#ifndef _SURFEL_CELL_INTERFACE_H_
#define _SURFEL_CELL_INTERFACE_H_

#include <ros/time.h>

#include <mrs_laser_maps/surfel.h>

namespace mrs_laser_maps
{
  
 
class SurfelCellInterface
{
public:
  
  virtual ~SurfelCellInterface(){};
  
  virtual void evaluate() = 0;
  
  virtual const mrs_laser_maps::Surfel& getSurfel() const = 0;

};
  
class SurfelMapInterface
{
public:
  
  typedef std::vector<SurfelCellInterface, Eigen::aligned_allocator<SurfelCellInterface>> AlignedCellVector;
   
  typedef std::vector<SurfelCellInterface*> CellPtrVector;

  
  virtual ~SurfelMapInterface() {};
  
  virtual void evaluateAll() = 0;

  virtual void unEvaluateAll() = 0;
  
  virtual bool isEvaluated() const = 0;
  
  virtual unsigned int getLevels() const = 0;

  virtual double getResolution() const = 0;
  
  virtual double getCellSize(int level) const = 0;
  
  virtual double getCellSize() const = 0;
  
  virtual void getOccupiedCells(CellPtrVector& cells, int level) = 0;
 
  virtual void getOccupiedCellsWithOffset(CellPtrVector& cells, std::vector<Eigen::Vector3f>& offsets) = 0;

  virtual bool getCell(const Eigen::Vector3f& point, SurfelCellInterface*& cellPtr, Eigen::Vector3f& cellOffset, int& level) = 0;

  virtual bool getCell(const Eigen::Vector3f& point, CellPtrVector& cellPtrs,
                      std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& cellOffsets,
                      std::vector<int>& levels, int neighbors = 0, bool reverse = false) = 0;

  virtual unsigned int getNumCellPoints() = 0;

  virtual ros::Time getLastUpdateTimestamp() const = 0;
protected:

};    
}

#endif
