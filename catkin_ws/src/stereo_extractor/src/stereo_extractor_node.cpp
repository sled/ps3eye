#include <ros/ros.h>
#include <string>
#include <boost/filesystem.hpp>
#include "stereo_extractor/stereo_extractor.h"
#include "stereo_extractor/stereo_file_exporter.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "stereo_to_matlab");

  ros::NodeHandle node  = ros::NodeHandle();
  ros::NodeHandle pnode = ros::NodeHandle("~");

  std::string str_output_path, output_folder_prefix;

  if(node.getNamespace() == "/") {
    ROS_WARN("Started in global namespace! Use ROS_NAMESPACE=my_camera rosrun stereo_extractor stereo_extractor");
  }

  pnode.param<std::string>("output_folder_prefix", output_folder_prefix, "data");

  if(!pnode.getParam("output_path", str_output_path)) {
    ROS_ERROR("Failed to get param 'output_path'");
    // todo: is this clean?
    ros::shutdown();
    return 1;
  }

  boost::filesystem::path output_path(str_output_path);

  try {
    if(!boost::filesystem::exists(output_path) || !is_directory(output_path)) {
      ROS_ERROR_STREAM("Output path '"<<str_output_path<<"; cannot be found or is not a directory");
      ros::shutdown();
    }
  } catch(const boost::filesystem::filesystem_error& ex) {
    ROS_ERROR("Error while resolving output path: %s", ex.what());
    ros::shutdown();
  }

  // construct path


  // define exporter
  boost::shared_ptr<stereo_extractor::StereoFileExporter> exporter(new stereo_extractor::StereoFileExporter(output_path, output_folder_prefix));

  stereo_extractor::StereoExtractor extractor(node, pnode, exporter);

  ros::spin();
  return 0;
}
