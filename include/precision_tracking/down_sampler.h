/*
 * down_sampler.h
 *
 *  Created on: May 12, 2014
 *      Author: davheld
 *
 * Different methods for down-sampling a point cloud.
 *
 */

#ifndef __PRECISION_TRACKING__DOWN_SAMPLER_H_
#define __PRECISION_TRACKING__DOWN_SAMPLER_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <precision_tracking/params.h>

namespace precision_tracking {

class DownSampler {
public:
  DownSampler(const bool stochastic, const Params *params);
  virtual ~DownSampler();

  void downSamplePoints(
      const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& points,
      const int target_num_points,
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr& down_sampled_points) const;

  // Randomly samples points from the input cloud to try to get the output
  // to have as close as possible to targetNumPoints points.
  static void downSamplePointsStochastic(
      const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& points,
      const int targetNumPoints,
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr& downSampledPoints);

  // Deterministically samples every N points from the input cloud to try to
  // get the output to have as close as possible to targetNumPoints points.
  template <typename PointT>
  static void downSamplePointsDeterministic(
      const typename pcl::PointCloud<PointT>& points,
      const int targetNumPoints,
      typename pcl::PointCloud<PointT>::Ptr& downSampledPoints,
      const bool use_ceil);

private:
  const Params *params_;
  bool stochastic_;
};

template <typename PointT>
void DownSampler::downSamplePointsDeterministic(
    const typename pcl::PointCloud<PointT>& points,
    const int target_num_points,
    typename pcl::PointCloud<PointT>::Ptr& down_sampled_points,
    const bool use_ceil)
{
  const size_t num_points = points.size();

  // Check if the points are already sufficiently down-sampled.
  if (target_num_points >= num_points * 0.8){
    *down_sampled_points = points;
    return;
  }

  // Select every N points to reach the target number of points.
  int everyN = 0;
  if (use_ceil) {
    everyN = ceil(static_cast<double>(num_points) /
                  static_cast<double>(target_num_points));
  } else {
    everyN = static_cast<double>(num_points) /
        static_cast<double>(target_num_points);
  }

  // Allocate space for the new points.
  down_sampled_points->reserve(target_num_points);

  //Just to ensure that we don't end up with 0 points, add 1 point to this
  down_sampled_points->push_back(points[0]);

  // Select every N points to reach the target number of points.
  for (size_t i = 1; i < num_points; ++i) {
    if (i % everyN == 0){
      const PointT& pt = points[i];
      down_sampled_points->push_back(pt);
    }
  }
}

} // namespace precision_tracking

#endif /* __PRECISION_TRACKING__DOWN_SAMPLER_H_ */
