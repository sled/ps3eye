/*
 * stereo_extractor.h
 *
 *  Created on: Mar 31, 2013
 *      Author: sled
 */

#ifndef STEREO_EXTRACTOR_H_
#define STEREO_EXTRACTOR_H_

#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include <ros/ros.h>
#include <ros/time.h>
#include <ros/exceptions.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include <image_geometry/stereo_camera_model.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>

#include <stereo_msgs/DisparityImage.h>

#include <map>

#include "stereo_snapshot.h"
#include "concurrent_queue.h"
#include "stereo_exporter.h"

namespace stereo_extractor {

// too long to type every time
using namespace sensor_msgs;
using namespace message_filters::sync_policies;
using namespace stereo_msgs;




class StereoExtractor {
public:
	StereoExtractor(ros::NodeHandle comm_nh, ros::NodeHandle param_nh, const boost::shared_ptr<StereoExporter>& stereo_exporter);
	~StereoExtractor();

	// callback when data is gathered
	void processCb( const ImageConstPtr& l_image_msg,
	                const ImageConstPtr& l_image_rect_msg,
	                const CameraInfoConstPtr& l_info_msg,
	                const ImageConstPtr& r_image_msg,
	                const ImageConstPtr& r_image_rect_msg,
	                const CameraInfoConstPtr& r_info_msg,
	                const DisparityImageConstPtr& disp_msg,
	                const PointCloud2ConstPtr& ptcloud2_msg);

	void processSnapshot();

	void ping(std::string);

private:
	ros::NodeHandle node_, pnode_;

	// image transport
	boost::shared_ptr<image_transport::ImageTransport> it_;

	// mappings
	std::string l_info_, l_image_, l_image_rect_;
	std::string r_info_, r_image_, r_image_rect_;
	std::string disparity_, ptcloud2_;

	// queue size for sync
	int queue_size_;

	// flags
	bool has_color_;
	volatile bool ok_; // thread flag signal

	// subscriptions

	// reference is the left image... sync upon left image!
	image_transport::SubscriberFilter sub_l_image_, sub_r_image_, sub_l_image_rect_, sub_r_image_rect_;
	message_filters::Subscriber<DisparityImage> sub_disparity_;
	message_filters::Subscriber<CameraInfo> sub_l_info_, sub_r_info_;
	message_filters::Subscriber<PointCloud2> sub_ptcloud2_;


	// shortcuts
	typedef ExactTime<Image, Image, CameraInfo, Image, Image, CameraInfo, DisparityImage, PointCloud2> ExactPolicy;
	typedef message_filters::Synchronizer<ExactPolicy> ExactSync;

	boost::shared_ptr<ExactSync> exact_sync_;

	// queue to hold snapshot for processing
	ConcurrentQueue<StereoSnapshotConstPtr> snapshots_;

	// worker thread which processes the queue
	boost::thread worker_thread_;

	// exporter
	boost::shared_ptr<StereoExporter> stereo_exporter_;

};




}




#endif /* STEREO_EXTRACTOR_H_ */
