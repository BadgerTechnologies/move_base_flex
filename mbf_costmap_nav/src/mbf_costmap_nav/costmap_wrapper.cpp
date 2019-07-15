/*
 *  Copyright 2019, Magazino GmbH, Sebastian Pütz, Jorge Santos Simón
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *  3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  costmap_wrapper.cpp
 *
 *  authors:
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *    Jorge Santos Simón <santos@magazino.eu>
 *
 */

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_3d/costmap_3d_ros.h>

#include "mbf_costmap_nav/costmap_wrapper.h"


namespace mbf_costmap_nav
{


template<typename CostmapNDROS>
CostmapWrapper<CostmapNDROS>::CostmapWrapper(const std::string &name, const TFPtr &tf_listener_ptr) :
  CostmapNDROS(name, *tf_listener_ptr),
  shutdown_costmap_(false), costmap_users_(0), private_nh_("~")
{
  // even if shutdown_costmaps is a dynamically reconfigurable parameter, we
  // need it here to decide whether to start or not the costmap on starting up
  private_nh_.param("shutdown_costmaps", shutdown_costmap_, false);
  private_nh_.param("clear_on_shutdown", clear_on_shutdown_, false);

  if (shutdown_costmap_)
    // initialize costmap stopped if shutdown_costmaps parameter is true
    CostmapNDROS::stop();
  else
    // otherwise costmap_users_ is at least 1, as costmap is always active
    ++costmap_users_;
}

template<typename CostmapNDROS>
CostmapWrapper<CostmapNDROS>::~CostmapWrapper()
{
  shutdown_costmap_timer_.stop();
}

template<typename CostmapNDROS>
void CostmapWrapper<CostmapNDROS>::reconfigure(double shutdown_costmap, double shutdown_costmap_delay)
{
  shutdown_costmap_delay_ = ros::Duration(shutdown_costmap_delay);
  if (shutdown_costmap_delay_.isZero())
    ROS_WARN("Zero shutdown costmaps delay is not recommended, as it forces us to enable costmaps on each action");

  if (shutdown_costmap_ && !shutdown_costmap)
  {
    checkActivate();
    shutdown_costmap_ = shutdown_costmap;
  }
  if (!shutdown_costmap_ && shutdown_costmap)
  {
    shutdown_costmap_ = shutdown_costmap;
    checkDeactivate();
  }
}

template<typename CostmapNDROS>
void CostmapWrapper<CostmapNDROS>::clear()
{
  // lock and clear costmap
  boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*this->getCostmap()->getMutex());
  CostmapNDROS::resetLayers();
}

template<typename CostmapNDROS>
void CostmapWrapper<CostmapNDROS>::checkActivate()
{
  boost::mutex::scoped_lock sl(check_costmap_mutex_);

  shutdown_costmap_timer_.stop();

  // Activate costmap if we shutdown them when not moving and they are not already active. This method must be
  // synchronized because start costmap can take up to 1/update freq., and concurrent calls to it can lead to segfaults
  if (shutdown_costmap_ && !costmap_users_)
  {
    CostmapNDROS::start();
    ROS_DEBUG_STREAM("" << CostmapNDROS::name_ << " activated");
  }
  ++costmap_users_;
}

template<typename CostmapNDROS>
void CostmapWrapper<CostmapNDROS>::checkDeactivate()
{
  boost::mutex::scoped_lock sl(check_costmap_mutex_);

  --costmap_users_;
  ROS_ASSERT_MSG(costmap_users_ >= 0, "Negative number (%d) of active users count!", costmap_users_);
  if (shutdown_costmap_ && !costmap_users_)
  {
    // Delay costmap shutdown by shutdown_costmap_delay so we don't need to enable at each step of a normal
    // navigation sequence, what is terribly inefficient; the timer is stopped on costmap re-activation and
    // reset after every new call to deactivate
    shutdown_costmap_timer_ =
      private_nh_.createTimer(shutdown_costmap_delay_, &CostmapWrapper<CostmapNDROS>::deactivate, this, true);
  }
}

template<typename CostmapNDROS>
void CostmapWrapper<CostmapNDROS>::deactivate(const ros::TimerEvent &event)
{
  boost::mutex::scoped_lock sl(check_costmap_mutex_);

  ROS_ASSERT_MSG(!costmap_users_, "Deactivating costmap with %d active users!", costmap_users_);
  if (clear_on_shutdown_)
    clear();  // do before stop, as some layers (e.g. obstacle and voxel) reactivate their subscribers on reset
  CostmapNDROS::stop();
  ROS_DEBUG_STREAM("" << CostmapNDROS::name_ << " deactivated");
}

template class CostmapWrapper<costmap_2d::Costmap2DROS>;
template class CostmapWrapper<costmap_3d::Costmap3DROS>;

} /* namespace mbf_costmap_nav */
