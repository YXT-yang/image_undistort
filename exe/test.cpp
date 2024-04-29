#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "camera_models/Camera.h"
#include "camera_models/CataCamera.h"
#include "camera_models/PinholeCamera.h"
#include "camera_models/EquidistantCamera.h"

int image_out_scale_;
std::string bag_path_;
std::string out_path_;
std::string image_topic_;
std::string image_out_topic_;
std::string image_yaml_path_;
std::string camera_model_type_;
camodocal::CataCamera cataCam;
camodocal::EquidistantCamera equiCam;
cv::Mat map1, map2;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lo_node");
  ros::NodeHandle nh;
  nh.param<int>("common/image_out_scale", image_out_scale_, 1);
  nh.param<std::string>("common/bag_path", bag_path_, "~/");
  nh.param<std::string>("common/out_path", out_path_, "~/");
  nh.param<std::string>("common/image_topic", image_topic_, "/cam");
  nh.param<std::string>("common/image_out_topic", image_out_topic_, "/cam_out");
  nh.param<std::string>("common/image_yaml_path", image_yaml_path_, "~/");
  nh.param<std::string>("common/camera_model_type", camera_model_type_, "~/");

  if (camera_model_type_.compare("MEI") == 0)
  {
    camodocal::CataCamera::Parameters params;
    if (params.readFromYamlFile(image_yaml_path_))
    {
      cataCam.setParameters(params);
      cataCam.initUndistortRectifyMap(map1, map2,
                                      -1.0, -1.0,
                                      cv::Size(params.imageHeight() * image_out_scale_,
                                               params.imageWidth() * image_out_scale_));
    }
    else
      std::cout << "MEI read empty"
                << "\n";
  }
  else if (camera_model_type_.compare("KANNALA_BRANDT") == 0)
  {
    camodocal::EquidistantCamera::Parameters params;
    if (params.readFromYamlFile(image_yaml_path_))
    {
      equiCam.setParameters(params);
      equiCam.initUndistortRectifyMap(map1, map2,
                                      -1.0, -1.0,
                                      cv::Size(params.imageHeight() * image_out_scale_,
                                               params.imageWidth() * image_out_scale_));
    }
    else
      std::cout << "KANNALA_BRANDT read empty"
                << "\n";
  }
  else
    std::cout << "No Models correspond!"
              << "\n";

  rosbag::Bag inp_bag,
      out_bag;
  inp_bag.open(bag_path_, rosbag::bagmode::Read);
  out_bag.open(out_path_, rosbag::bagmode::Write);
  rosbag::View view_full;
  view_full.addQuery(inp_bag);

  int size_ = view_full.size();
  int num_ = 0;

  for (const rosbag::MessageInstance &m : view_full)
  {
    if (m.getTopic() == image_topic_)
    {
      // sensor_msgs::ImagePtr image_msg_ = m.instantiate<sensor_msgs::Image>();
      sensor_msgs::CompressedImagePtr image_msg_ = m.instantiate<sensor_msgs::CompressedImage>();
      if (image_msg_ == nullptr)
        continue;
      cv::Mat mapIn = cv_bridge::toCvCopy(image_msg_,
                                          sensor_msgs::image_encodings::BGR8)
                          ->image;
      cv::Mat mapOut;
      cv::remap(mapIn, mapOut, map1, map2, cv::INTER_LINEAR, cv::BORDER_WRAP);

      sensor_msgs::ImagePtr msg_out_ = cv_bridge::CvImage(
                                           image_msg_->header,
                                           "bgr8",
                                           mapOut)
                                           .toImageMsg();
      out_bag.write(image_out_topic_, msg_out_->header.stamp, msg_out_);
    }
    ROS_WARN("Current Progress: %f%%", 100.0 * num_++ / size_);
  }
  ROS_WARN("Finish: %f%%", 100.0);

  return 0;
}