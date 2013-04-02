/*
 * stereo_snapshot.h
 *
 *  Created on: Mar 31, 2013
 *      Author: sled
 */

#ifndef STEREO_SNAPSHOT_H_
#define STEREO_SNAPSHOT_H_

#include <image_geometry/stereo_camera_model.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>

#include <stereo_msgs/DisparityImage.h>

namespace stereo_extractor {

using namespace sensor_msgs;
using namespace stereo_msgs;

struct StereoSnapshot {

  // constructor
  StereoSnapshot(   const ros::Time& timestamp,
                    const ImageConstPtr & l_image_msg,
                    const ImageConstPtr & l_image_rect_msg,
                    const CameraInfoConstPtr & l_info_msg,
                    const ImageConstPtr & r_image_msg,
                    const ImageConstPtr & r_image_rect_msg,
                    const CameraInfoConstPtr & r_info_msg,
                    const DisparityImageConstPtr & disp_msg,
                    const PointCloud2ConstPtr & ptcloud2_msg
                  ) : timestamp_(timestamp),
                      l_image_msg_(l_image_msg),
                      l_image_rect_msg_(l_image_rect_msg),
                      l_info_msg_(l_info_msg),
                      r_image_msg_(r_image_msg),
                      r_image_rect_msg_(r_image_rect_msg),
                      r_info_msg_(r_info_msg),
                      disp_msg_(disp_msg),
                      ptcloud2_msg_(ptcloud2_msg) {
    // ...
  }

  const ros::Time timestamp_;
  const ImageConstPtr l_image_msg_;
  const ImageConstPtr l_image_rect_msg_;
  const CameraInfoConstPtr l_info_msg_;
  const ImageConstPtr r_image_msg_;
  const ImageConstPtr r_image_rect_msg_;
  const CameraInfoConstPtr r_info_msg_;
  const DisparityImageConstPtr disp_msg_;
  const PointCloud2ConstPtr ptcloud2_msg_;
};


typedef boost::shared_ptr< ::stereo_extractor::StereoSnapshot const> StereoSnapshotConstPtr;

}


#endif /* STEREO_SNAPSHOT_H_ */
