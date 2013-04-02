/*
 * stereo_file_exporter.h
 *
 *  Created on: Apr 1, 2013
 *      Author: sled
 */

#ifndef STEREO_FILE_EXPORTER_H_
#define STEREO_FILE_EXPORTER_H_

#include <ros/ros.h>
#include "stereo_exporter.h"

// boost stuff
#include <boost/format.hpp>
#include <boost/filesystem.hpp>

// opencv stuff
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

// pcl stuff
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// eigen
#include <Eigen/Core>

#include "camera_calibration_parsers/parse.h"

namespace stereo_extractor {

namespace enc = sensor_msgs::image_encodings;

class StereoFileExporter : public StereoExporter {
public:

  StereoFileExporter( const boost::filesystem::path& output_path,
                      const std::string& output_folder_prefix="data"
                    ) : output_path_(output_path),
                        output_folder_prefix_(output_folder_prefix),
                        folder_format_(std::string(output_folder_prefix + "_%i_%i")){
   // output folder format: prefix_secs_nsecs/
  }

  void exportSnapshot(StereoSnapshotConstPtr&);


  ~StereoFileExporter() {}
private:
  // disable default constructor
  StereoFileExporter() { }

  boost::format folder_format_;
  boost::filesystem::path output_path_;

  std::string output_folder_prefix_;

  // little helper to construct path
  std::string formatFolderName(const ros::Time& t);

  // helper to save parts

  // camera info message
  void saveCameraInfo(const boost::filesystem::path& info_file_path, const std::string& camera_name, const CameraInfoConstPtr& camera_info);

  // save disparity image, matrix and meta data

  // converts to CV_8U
  void saveDisparityImage(const boost::filesystem::path& image_path, const DisparityImageConstPtr& disparity);
  // dump byte array
  void saveDisparityMatrix(const boost::filesystem::path& matrix_path, const DisparityImageConstPtr& disparity);
  // write out meta data as YAML
  //void saveDisparityMeta(const boost::filesystem::path& meta_path, const DisparityImageConstPtr& disparity);


  // save point cloud 2 using PCL
  void savePointCloud2(const boost::filesystem::path& pcd_file_path, const PointCloud2ConstPtr& ptcloud2);

  // helper to save arbitrary image
  void saveImage(const boost::filesystem::path& image_path, const ImageConstPtr& image);

};

}




#endif /* STEREO_FILE_EXPORTER_H_ */
