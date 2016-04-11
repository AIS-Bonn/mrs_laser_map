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

#ifndef _SYNCHRONIZED_CIRCULAR_BUFFER_H_
#define _SYNCHRONIZED_CIRCULAR_BUFFER_H_

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>
#include <boost/thread/thread.hpp>
#include <boost/call_traits.hpp>
#include <boost/progress.hpp>
#include <boost/bind.hpp>

#include <iostream>

#include <fstream>

namespace mrs_laser_maps
{
template <class T>
class synchronized_circular_buffer
{
public:
  typedef boost::circular_buffer<T> container_type;
  typedef typename container_type::size_type size_type;
  typedef typename container_type::value_type value_type;
  //	typedef typename boost::call_traits<value_type>::param_type param_type;
  //
  explicit synchronized_circular_buffer(size_type capacity) : m_unread(0), m_container(capacity)
  {
  }

  void push_front(T item)
  {
    boost::mutex::scoped_lock lock(m_mutex);
    m_not_full.wait(lock, boost::bind(&synchronized_circular_buffer<value_type>::is_not_full, this));
    m_container.push_front(item);
    ++m_unread;
    lock.unlock();
    m_not_empty.notify_one();
  }

  void pop_back(T* pItem)
  {
    boost::mutex::scoped_lock lock(m_mutex);
    m_not_empty.wait(lock, boost::bind(&synchronized_circular_buffer<value_type>::is_not_empty, this));
    *pItem = m_container[--m_unread];

    m_not_full.notify_one();
  }

  T back()
  {
    boost::mutex::scoped_lock lock(m_mutex);

    return m_container.back();
  }

  T front()
  {
    boost::mutex::scoped_lock lock(m_mutex);

    return m_container.front();
  }

  void clear()
  {
    boost::mutex::scoped_lock lock(m_mutex);

    m_container.clear();

    m_unread = 0;
    lock.unlock();
    m_not_full.notify_one();
  }

  size_t size()
  {
    return m_unread;
  }

private:
  synchronized_circular_buffer(const synchronized_circular_buffer&);             // Disabled copy constructor
  synchronized_circular_buffer& operator=(const synchronized_circular_buffer&);  // Disabled assign operator

  bool is_not_empty() const
  {
    return m_unread > 0;
  }
  bool is_not_full() const
  {
    return m_unread < m_container.capacity();
  }

  size_type m_unread;
  container_type m_container;
  boost::mutex m_mutex;
  boost::condition m_not_empty;
  boost::condition m_not_full;
};
}

#endif
