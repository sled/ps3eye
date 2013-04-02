/*
 * stereo_extractor.cpp
 *
 *  Created on: Mar 31, 2013
 *  	Author: sled
 */

#include "stereo_extractor/stereo_extractor.h"

namespace stereo_extractor {

StereoExtractor::StereoExtractor( ros::NodeHandle comm_nh,
                                  ros::NodeHandle param_nh,
                                  const boost::shared_ptr<StereoExporter>& stereo_exporter
                                  ) : node_(comm_nh), pnode_(param_nh), stereo_exporter_(stereo_exporter) {

  pnode_.param<bool>("colored", has_color_, false);
  pnode_.param<int>("queue_size", queue_size_, 5);

  // resolve names for topics

  try {
    l_info_  = ros::names::resolve("left/camera_info");
    l_image_   = ros::names::resolve("left/image_raw");

    r_info_  = ros::names::resolve("right/camera_info");
    r_image_   = ros::names::resolve("right/image_raw");

    if(has_color_) {
      l_image_rect_ = ros::names::resolve("left/image_rect_color");
      r_image_rect_ = ros::names::resolve("right/image_rect_color");
    }
    else
    {
      l_image_rect_ = ros::names::resolve("left/image_rect");
      r_image_rect_ = ros::names::resolve("right/image_rect");
    }

    disparity_  = ros::names::resolve("disparity");
    ptcloud2_   = ros::names::resolve("points2");

  } catch(ros::InvalidNameException &ex) {
    ROS_ERROR(ex.what());
  }

  ROS_INFO_STREAM("left info: "<<l_info_<<"\n");
  ROS_INFO_STREAM("left image: "<<l_image_<<"\n");
  ROS_INFO_STREAM("left image rect: "<<l_image_rect_<<"\n");

  ROS_INFO_STREAM("right info: "<<r_info_<<"\n");
  ROS_INFO_STREAM("right image: "<<r_image_<<"\n");
  ROS_INFO_STREAM("right image rect: "<<r_image_rect_<<"\n");

  ROS_INFO_STREAM("disparity: "<<disparity_<<"\n");
  ROS_INFO_STREAM("point cloud 2: "<<ptcloud2_<<"\n");

  // create pointer
  it_.reset(new image_transport::ImageTransport(node_));

  // indicate we want raw data
  image_transport::TransportHints hints("raw", ros::TransportHints(), pnode_);

  sub_l_image_      .subscribe(*it_, l_image_, 1, hints);
  sub_l_image_rect_ .subscribe(*it_, l_image_rect_, 1, hints);
  sub_l_info_       .subscribe(node_, l_info_, 1);

  sub_r_image_      .subscribe(*it_, r_image_, 1, hints);
  sub_r_image_rect_ .subscribe(*it_, r_image_rect_, 1, hints);
  sub_r_info_       .subscribe(node_, r_info_, 1);

  sub_disparity_    .subscribe(node_, disparity_, 1);
  sub_ptcloud2_     .subscribe(node_, ptcloud2_,1);

  /*
  sub_l_image_.registerCallback(boost::bind(&StereoExtractor::ping, this, "LEFT IMAGE"));
  */

  // start worker thread
  ok_ = true;
  worker_thread_ = boost::thread(boost::bind(&StereoExtractor::processSnapshot, this));

  // setup sync
  exact_sync_.reset(  new ExactSync(  ExactPolicy(queue_size_),
                                      sub_l_image_, sub_l_image_rect_, sub_l_info_,
                                      sub_r_image_, sub_r_image_rect_, sub_r_info_,
                                      sub_disparity_, sub_ptcloud2_) );

  exact_sync_->registerCallback(  boost::bind(  &StereoExtractor::processCb,
                                                this, _1, _2, _3, _4, _5, _6, _7, _8));

  ROS_INFO("All hooked up, ready to get messages...");
}


void StereoExtractor::ping(std::string origin) {
  std::cout<<"PINGG FROM "<<origin<<std::endl;
}

void StereoExtractor::processSnapshot() {

  while(ok_) {
    // current snapshot
    StereoSnapshotConstPtr snapshot;
    snapshots_.wait_and_pop(snapshot);
    ROS_INFO("Popped a snapshot of the queue!\n");

    stereo_exporter_->exportSnapshot(snapshot);

    // make sure thread can be joined
    boost::this_thread::interruption_point();
  }
}

void StereoExtractor::processCb(  const ImageConstPtr& l_image_msg,
                                  const ImageConstPtr& l_image_rect_msg,
                                  const CameraInfoConstPtr& l_info_msg,
                                  const ImageConstPtr& r_image_msg,
                                  const ImageConstPtr& r_image_rect_msg,
                                  const CameraInfoConstPtr& r_info_msg,
                                  const DisparityImageConstPtr& disp_msg,
                                  const PointCloud2ConstPtr& ptcloud2_msg) {

  ROS_INFO_STREAM("Got some messages! kapooya!\n");

  // left image is reference
  // TODO: check that all messages have equal timestamps!
  ros::Time timestamp = l_image_msg->header.stamp;

  // put a snapshot on the queue
  snapshots_.push(StereoSnapshotConstPtr(new StereoSnapshot(timestamp,
                                          l_image_msg,
                                          l_image_rect_msg,
                                          l_info_msg,
                                          r_image_msg,
                                          r_image_rect_msg,
                                          r_info_msg,
                                          disp_msg,
                                          ptcloud2_msg  )));


}

StereoExtractor::~StereoExtractor() {
  // join thread
  ok_ = false;
  worker_thread_.interrupt();
  worker_thread_.join();
}

}
