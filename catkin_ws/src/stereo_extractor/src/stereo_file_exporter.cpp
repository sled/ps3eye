/*
 * stereo_file_exporter.cpp
 *
 *  Created on: Apr 1, 2013
 *      Author: sled
 */

#include "stereo_extractor/stereo_file_exporter.h"

namespace stereo_extractor {

  void StereoFileExporter::exportSnapshot(StereoSnapshotConstPtr& snapshot) {
    // create folder path
    boost::filesystem::path output_path = output_path_ / formatFolderName(snapshot->timestamp_);

    // touch folder if necessary
    try {
      if(!boost::filesystem::exists(output_path)) {
        boost::filesystem::create_directories(output_path);
      }
    } catch(const boost::filesystem::filesystem_error& ex) {
      ROS_ERROR("Could not create directory: %s", ex.what());
    }

    // export left and right camera info
    saveCameraInfo(output_path / "left_camera_info.yml", "left", snapshot->l_info_msg_);
    saveCameraInfo(output_path / "right_camera_info.yml", "right", snapshot->r_info_msg_);

    // export left and right raw image
    saveImage(output_path / "left.ppm", snapshot->l_image_msg_);
    saveImage(output_path / "right.ppm", snapshot->r_image_msg_);

    // export left and right rectified image
    saveImage(output_path / "left_rect.ppm", snapshot->l_image_rect_msg_);
    saveImage(output_path / "right_rect.ppm", snapshot->r_image_rect_msg_);

    // export disparity map as matrix including meta data
    saveDisparityMatrix(output_path / "disparity.yml", snapshot->disp_msg_);

    // export disparity map as grayscale image
    saveDisparityImage(output_path / "disparity.pgm", snapshot->disp_msg_);

    // export pointcloud2
    savePointCloud2(output_path / "points2.pcd", snapshot->ptcloud2_msg_);

  }

  // save disparity matrix as yaml file including meta data
  void StereoFileExporter::saveDisparityMatrix(const boost::filesystem::path& matrix_path, const DisparityImageConstPtr& disparity) {

    cv_bridge::CvImageConstPtr cv_img_ptr;

    try {
      // get shared cv pointer
      cv_img_ptr = cv_bridge::toCvShare(disparity->image, disparity, disparity->image.encoding);
    } catch(const cv_bridge::Exception &ex) {
      ROS_ERROR("cv_bdrige exception (converting disparity map to image): %s", ex.what());
      return;
    }

    try {
      cv::FileStorage fs(matrix_path.string(), cv::FileStorage::WRITE);

      // meta data

      fs  <<"f"<<(float)disparity->f;
      fs  <<"T"<<(float)disparity->T;
      fs  <<"valid_window"
          <<"{"
          // problem: opencv has no overloaded operator for uint32_t!
            <<"x_offset"<<(int)disparity->valid_window.x_offset
            <<"y_offset"<<(int)disparity->valid_window.y_offset
            <<"height"<<(int)disparity->valid_window.height
            <<"width"<<(int)disparity->valid_window.width
            <<"do_rectify"<<(bool)disparity->valid_window.do_rectify
          <<"}";

      fs  <<"min_disparity"<<(float)disparity->min_disparity;
      fs  <<"max_disparity"<<(float)disparity->max_disparity;
      fs  <<"delta_d"<<(float)disparity->delta_d;

      // image (meta-)data
      fs  <<"image"
          <<"{"
            <<"height"<<(int)disparity->image.height
            <<"width"<<(int)disparity->image.width
            <<"encoding"<<(std::string)disparity->image.encoding
            <<"is_bigendian"<<(int)disparity->image.is_bigendian
            <<"step"<<(int)disparity->image.step
            <<"data"<<cv_img_ptr->image
          <<"}";

      fs.release();

      ROS_INFO("Saved disparity matrix and metadata %s", matrix_path.c_str());
    } catch(const cv::Exception& ex) {
      ROS_ERROR("OpenCV Error, could not save disparity matrix: %s", ex.what());
    }
  }

  // converts to 32F matrix to CV_8U
  void StereoFileExporter::saveDisparityImage(const boost::filesystem::path& image_path, const DisparityImageConstPtr& disparity) {
    // scaling options
    float min_disparity = disparity->min_disparity;
    float max_disparity = disparity->max_disparity;

    float multiplier    = 255.0f/(max_disparity-min_disparity);

    cv_bridge::CvImageConstPtr cv_img_ptr;

    try {
      // get shared cv pointer
      cv_img_ptr = cv_bridge::toCvShare(disparity->image, disparity, disparity->image.encoding);
    } catch(const cv_bridge::Exception &ex) {
      ROS_ERROR("cv_bdrige exception (converting disparity map to image): %s", ex.what());
      return;
    }

    // transform to CV_8U
    cv::Mat scaled_img;
    cv_img_ptr->image.convertTo(scaled_img, CV_8U, multiplier, -min_disparity);


    // try to save
    if(cv::imwrite(image_path.c_str(), scaled_img)) {
      ROS_INFO("Saved disparity map as image %s", image_path.c_str());
    }
    else
    {
      ROS_ERROR("Could not save disparity map as image %s", image_path.c_str());
    }

    // free memory
    scaled_img.release();

  }

  void StereoFileExporter::savePointCloud2(const boost::filesystem::path& pcd_file_path, const PointCloud2ConstPtr& ptcloud2) {
    if((ptcloud2->width * ptcloud2->height) == 0) {
      ROS_ERROR("Cannot export PointCloud2, dimensionless {width=%d, height=%d}", (int)ptcloud2->width, (int)ptcloud2->height);
      return;
    }

    ROS_INFO( "Exporting %d data points in frame %s with the following fields: %s",
               (int)ptcloud2->width * ptcloud2->height,
               ptcloud2->header.frame_id.c_str (),
               pcl::getFieldsList((*ptcloud2)).c_str());

    ROS_INFO_STREAM("Saved PointCloud2 as PCD to '"<<pcd_file_path.string()<<"'");

    // parameters:
    // filename - origin - orentiation - binary_mode

    // Caution: PointCloud structures containing an RGB field have traditionally used packed float values to store RGB data.
    // Storing a float as ASCII can introduce variations to the smallest bits, and thus significantly alter the data.
    // This is a known issue, and the fix involves switching RGB data to be stored as a
    // packed integer in future versions of PCL.
    pcl::io::savePCDFile(pcd_file_path.string(), *ptcloud2, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), false);

  }

  void StereoFileExporter::saveCameraInfo(const boost::filesystem::path& info_file_path, const std::string& camera_name, const CameraInfoConstPtr& camera_info) {
    if(!camera_calibration_parsers::writeCalibration(info_file_path.string(), camera_name, *camera_info))
      ROS_ERROR_STREAM("Could not save camera_info for camera '"<<camera_name<<"' to '"<<info_file_path.string()<<"'");
    else
      ROS_INFO_STREAM("Saved camera_info for camera '"<<camera_name<<"' to '"<<info_file_path.string());
  }

  void StereoFileExporter::saveImage(const boost::filesystem::path& image_path, const ImageConstPtr& image) {

    try {
      // TODO: check encoding type from image! (mono/color)
      cv_bridge::CvImagePtr cv_img_ptr = cv_bridge::toCvCopy(image, enc::BGR8);


      if(!cv_img_ptr->image.empty()) {
        if(cv::imwrite(image_path.c_str(), cv_img_ptr->image)) {
          ROS_INFO("Saved image %s", image_path.c_str());
        }
        else
        {
          ROS_ERROR("Could not save image %s", image_path.c_str());
        }
      }
      else
      {
        ROS_ERROR("Could not save image: Image was empty!");
      }
    } catch(const cv_bridge::Exception &ex) {
      ROS_ERROR("cv_bdrige exception: %s", ex.what());
    }
  }

  std::string StereoFileExporter::formatFolderName(const ros::Time& t) {
    return (folder_format_ % t.sec % t.nsec).str();
  }


}

