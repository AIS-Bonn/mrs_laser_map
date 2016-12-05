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

#ifndef _CELL_BUFFER_H_
#define _CELL_BUFFER_H_

#include <boost/circular_buffer.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>
#include <boost/thread/thread.hpp>
#include <boost/call_traits.hpp>
#include <boost/progress.hpp>
#include <boost/bind.hpp>
#include <boost/iterator/iterator_facade.hpp>

#include <iostream>

namespace mrs_laser_maps
{

template <class Iterator, class T>
class iterator_wrapper
    : public boost::iterator_adaptor< iterator_wrapper<Iterator, T>, Iterator >
{
    typedef boost::iterator_adaptor< iterator_wrapper<Iterator, T>, Iterator > super_t;

    friend class iterator_core_access;

  public:
    iterator_wrapper() {}

    explicit iterator_wrapper(Iterator x) 
	: super_t(x) {}

    template<class OtherIterator>
    iterator_wrapper(
	iterator_wrapper<OtherIterator, T> const& r
	, typename boost::enable_if_convertible<OtherIterator, Iterator>::type* = 0
	)
	: super_t(r.base())
    {}

    
    T& operator * () const {
	return (*((super_t::operator*()).get())); ;
      
    }
    
      T& operator * () {
	return *((super_t::operator*()).get()); 
      
    }
    
    typename super_t::reference getPointer () const {
	return this->base() ;
      
    }
    
  private:

};


// template <class T, class container_pointer>
// class scan_iterator_wrapper 
//   : public boost::iterator_facade<
//         scan_iterator_wrapper<T, container_pointer>
//       , T
//       , container_pointer
//       , boost::forward_traversal_tag
//     >
// {
//  public:
//   
//     explicit scan_iterator_wrapper(std::vector<unsigned int> labels, container_pointer container )
//       : labels_(labels)
//       , container_(container)
//       , index_(0){}
// 
//  private:
//     friend class boost::iterator_core_access;
// 
//     void increment() { index_++;}
// 
//     bool equal(scan_iterator_wrapper const& other) const
//     {
//         return false;
//     }
// 
//     T& dereference() const 
//     { 
//       T point;
//       container_->find( labels_[index_], point);
//       return * point; 
//       
//     }
// 
//     std::vector<unsigned int> labels_;
//     
//     size_t index_;
//     
//     container_pointer container_;
// };


template <class T>
class cell_buffer
{
public:
  typedef boost::shared_ptr<T> pointer_type;
  typedef boost::circular_buffer_space_optimized<pointer_type> container_type;
  typedef typename boost::shared_ptr<boost::circular_buffer_space_optimized<pointer_type>> container_pointer;
  typedef typename container_type::size_type size_type;
  typedef typename container_type::value_type value_type;
  typedef typename container_type::iterator iterator_base;
  typedef typename container_type::allocator_type allocator_type;
  typedef typename std::map<unsigned int, pointer_type> map_type;
  typedef typename std::pair<unsigned int, pointer_type> map_element;
  typedef typename map_type::iterator map_iterator_type;
  typedef typename map_type::reverse_iterator map_reverse_iterator_type;
  typedef typename std::map<unsigned int, std::vector<unsigned int>> scan_map_type;
  typedef typename std::pair<unsigned int, std::vector<unsigned int>> scan_map_element;
  typedef typename std::vector<unsigned int> scan_map_element_ids;
  typedef typename scan_map_type::iterator scan_map_iterator_type;
  typedef typename std::vector<unsigned int>::iterator scan_map_elemnet_iterator_type;
  //	typedef typename boost::call_traits<value_type>::param_type param_type;
  //
  
//   typedef iterator_wrapper< container_type, boost::cb_details::nonconst_traits<allocator_type> > iterator_w;
  typedef iterator_wrapper< iterator_base, T > iterator;
//   typedef scan_iterator_wrapper< T, container_pointer > scan_iterator;

  
  explicit cell_buffer(size_type capacity) : m_container(capacity)
  {
  }

  explicit cell_buffer(const cell_buffer& cb)
  : m_container(cb.m_container)
  , map_(cb.map_)
  , scan_map_(cb.scan_map_)
  {
    if (cb.m_container.size() > 0)
      ROS_WARN_STREAM_THROTTLE(0.1, "cell_buffer copy constructor " << cb.m_container.size());

  }
  
  ~cell_buffer()
  {
//     ROS_INFO("destructor of cell_buffer");
    
  }

    
  T dereference(pointer_type ptr)
  {
      return *(ptr.get());
  }
  
  void push_front(T item)
  {
//     ROS_INFO_STREAM_THROTTLE(1.0, "push front in cell buffer");
    pointer_type item_ptr = boost::make_shared<T>(item);
    ROS_WARN_STREAM_THROTTLE(0.1, "use_count before push_front: " << item_ptr.use_count() );
    
    push_front(item_ptr); 
    ROS_WARN_STREAM_THROTTLE(0.1, "use_count after push_front: " << item_ptr.use_count() );

//     map_[item.pointNr] = item_ptr;
  }

  //@TODO: why slower when sorted asc vs. desc
  void push_front(const pointer_type& item)
  {
//     ROS_WARN_STREAM_THROTTLE(0.1, "use_count before: " << item.use_count() );

    if (m_container.size() == m_container.capacity())
    {
      map_iterator_type it = map_.begin();
      
      if (it != map_.end())
      {
	unsigned int scan_nr = (*it).second->scanNr;
	map_.erase(it);
	
	scan_map_iterator_type s_it = scan_map_.begin();
	if (s_it != scan_map_.end())
	{
	  if ((*s_it).first < scan_nr)
	  {
	    scan_map_.erase(s_it);
	  }
	}
      }
      
      
    }
    m_container.push_front(item); 
    map_[item->pointNr] = item;
    (scan_map_[item->scanNr]).push_back(item->pointNr);
    
//     ROS_WARN_STREAM_THROTTLE(0.1, "use_count after: " << item.use_count() );
    
    if (m_container.size() != map_.size())
    {
      ROS_WARN_STREAM_THROTTLE(0.1, "m_container.size() != map_.size() with m_container.size(): " << m_container.size() << " and map_.size(): " << map_.size() );
    }
    
    
//     if (map_.size() > 100)
//       ROS_INFO_STREAM_THROTTLE(0.1, "push front in cell buffer with id:" << item->pointNr 
//       << " map size is " << map_.size()
//       << " buffer size is " << m_container.size() 
// 	<< " front().pointNr : " << front().pointNr
// 	<< " end().pointNr : " << back().pointNr 
// 	<< " m_container.size()  : " << m_container.size() 
// 	<< " m_container.capacity() : " << m_container.capacity()
//       );	
  }
  
  bool update(const T& item)
  {

    map_iterator_type it = map_.find(item.pointNr);
    if (it != map_.end())
    {
      (*it->second) = item;
      ROS_INFO_STREAM_THROTTLE(0.1, "update with: " << item.pointNr << " to: " << (*it->second).pointNr << " with use count: " << it->second.use_count() );
      return true;
    }
    return false;
  }
  
  bool find (T item, T& point )
  {
    return find(item.pointNr, point);
  }
  
  bool find (unsigned int label, T& point )
  {
    map_iterator_type it = map_.find(label);
    if (it == map_.end())
    {
      return false;
    }
    ROS_WARN_STREAM_THROTTLE(0.1, "use_count before: " << (*it).second.use_count() );

    point = dereference((*it).second);
    
    ROS_WARN_STREAM_THROTTLE(0.1, "use_count after: " << (*it).second.use_count() );

    return true;
  }
  
  bool find_scan_id(unsigned int scan_label, std::vector<unsigned int>& point_ids)
  {
    scan_map_iterator_type it = scan_map_.find(scan_label);
    if (it == scan_map_.end())
      return false;
    
    for (auto l : scan_map_[scan_label])
      point_ids.push_back(l);
//     point_ids = scan_map_[scan_label];
    return true;
  }
  
//   void pop_back(T* pItem)
//   {
//     m_container.pop_back(dereference(pItem));
//     map_.erase(pItem->pointNr);
//   }

  T back()
  {
    
    return dereference(m_container.back());
  }

  T front()
  {
    
    return dereference(m_container.front());
  }
  
  pointer_type back_ptr()
  {
    
    return m_container.back();
  }
  
  pointer_type front_ptr()
  {
    return m_container.front();
  }

  iterator begin()
  {
    return iterator(m_container.begin());
  }
  
  iterator end()
  {
    return iterator(m_container.end());
  }
  
  iterator erase( iterator pos )
  {
    map_iterator_type it = map_.find( (*pos).pointNr);
    if (it != map_.end())
      map_.erase(it);
    
    scan_map_element_ids& id_vector_for_scan = scan_map_[(*pos).scanNr];
    scan_map_elemnet_iterator_type scan_it = id_vector_for_scan.begin();
    for (; scan_it != id_vector_for_scan.end(); scan_it++)
    {
      if ((*scan_it) == (*pos).pointNr)
      {
	id_vector_for_scan.erase(scan_it);
	break;
      }
    }
    return iterator(m_container.erase(pos.base()));
  }
  
  iterator insert( iterator pos, T item)
  {
    pointer_type ptr = boost::make_shared<T>(item);
    map_iterator_type it = map_.lower_bound(item.pointNr);
    if (it != map_.end())
    {
      map_.insert(map_element(item.pointNr, ptr));
//     map_[item.pointNr] = ptr;
    }
    else 
      map_[item.pointNr] = ptr;

    
    ROS_WARN_STREAM_THROTTLE(0.1, "use_count at insert1: " << ptr.use_count() );
    (scan_map_[item.scanNr]).push_back(item.pointNr);
    
    iterator rv(m_container.insert(pos.base(), ptr));
    
    ROS_WARN_STREAM_THROTTLE(0.1, "use_count at insert: " << ptr.use_count() );

    return rv;
  }
  
  void clear()
  {
  
    m_container.clear();
    map_.clear();
    scan_map_.clear();
  }

  size_t size()
  {
    return m_container.size();
    
  }

private:
//   cell_buffer(const cell_buffer&);             // Disabled copy constructor
  cell_buffer& operator=(const cell_buffer&);  // Disabled assign operator

 
//   size_type m_unread;
  container_type m_container;
  map_type map_;
  scan_map_type scan_map_;
//   boost::mutex m_mutex;
//   boost::condition m_not_empty;
//   boost::condition m_not_full;
  
  
};
}

#endif
