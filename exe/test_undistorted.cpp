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

std::string topic_i_image;
std::string topic_o_image;
std::string bag_i_file;
std::string bag_o_file;
std::string model_i_type;
std::string model_i_file;
std::string model_o_type;
std::string model_o_file;
std::string image_o_path;
double image_o_star_rate;
double image_o_stop_rate;
int bag_i_size;
camodocal::Camera* cam_i_model;
camodocal::Camera* cam_o_model;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lo_node");
    ros::NodeHandle nh;
    nh.param<std::string>("common/topic_i_image", topic_i_image, "~/");
    nh.param<std::string>("common/topic_o_image", topic_o_image, "~/");
    nh.param<std::string>("common/bag_i_file", bag_i_file, "~/");
    nh.param<std::string>("common/bag_o_file", bag_o_file, "~/");
    nh.param<std::string>("common/model_i_type", model_i_type, "~/");
    nh.param<std::string>("common/model_i_file", model_i_file, "~/");
    nh.param<std::string>("common/model_o_type", model_o_type, "~/");
    nh.param<std::string>("common/model_o_file", model_o_file, "~/");
    nh.param<std::string>("common/image_o_path", image_o_path, "~/");
    nh.param<double>("common/image_o_star_rate", image_o_star_rate, 0);
    nh.param<double>("common/image_o_stop_rate", image_o_stop_rate, 0);

    // ---------------------------------------------------------------------------------
    // input the initial camera model
    if (model_i_type.compare("MEI") == 0){
        camodocal::CataCamera::Parameters params;
        if (params.readFromYamlFile(model_i_file)){
            cam_i_model = new camodocal::CataCamera(params);
        }
    }else if (model_i_type.compare("KANNALA_BRANDT") == 0){
        camodocal::EquidistantCamera::Parameters params;
        if(params.readFromYamlFile(model_i_file)){
            cam_i_model = new camodocal::EquidistantCamera(params);
        }
    }else if (model_i_type.compare("PINHOLE") == 0){
        camodocal::PinholeCamera::Parameters params;
        if(params.readFromYamlFile(model_i_file)){
            cam_i_model = new camodocal::PinholeCamera(params);
        }
    }

    // ------------------------------------------------------------------
    // input the goal camera model
    if (model_o_type.compare("MEI") == 0){
        camodocal::CataCamera::Parameters params;
        if (params.readFromYamlFile(model_o_file)){
            cam_o_model = new camodocal::CataCamera(params);
        }
    }else if (model_o_type.compare("KANNALA_BRANDT") == 0){
        camodocal::EquidistantCamera::Parameters params;
        if(params.readFromYamlFile(model_o_file)){
            cam_o_model = new camodocal::EquidistantCamera(params);
        }
    }else if (model_o_type.compare("PINHOLE") == 0){
        camodocal::PinholeCamera::Parameters params;
        if(params.readFromYamlFile(model_o_file)){
            cam_o_model = new camodocal::PinholeCamera(params);
        }
    }

    // ----------------------------------------------------------------------
    // open ros bag
    rosbag::Bag i_bag, o_bag;
    i_bag.open(bag_i_file, rosbag::bagmode::Read);
    o_bag.open(bag_o_file, rosbag::bagmode::Write);
    rosbag::View view_full;
    view_full.addQuery(i_bag);
    bag_i_size = view_full.size();
    int num_tag(0);

    
    Eigen::Vector2d point_o;
    Eigen::Vector2d point_i;
    Eigen::Vector3d point_sphere;
    for (const rosbag::MessageInstance &m : view_full)
    {
        if (m.getTopic() == topic_i_image)
        {
            sensor_msgs::CompressedImagePtr msg_i = m.instantiate<sensor_msgs::CompressedImage>();
            if (msg_i == nullptr) continue;
            cv::Mat map_i = cv_bridge::toCvCopy(msg_i, sensor_msgs::image_encodings::BGR8)->image;
            cv::Mat map_o = cv::Mat::zeros(cam_o_model->imageHeight(), cam_o_model->imageWidth(), map_i.type());

            for (int y = 0; y < cam_o_model->imageHeight(); ++y) {
                for (int x = 0; x < cam_o_model->imageWidth(); ++x){
                    point_o << x, y;
                    cam_o_model->liftSphere(point_o, point_sphere);
                    cam_i_model->spaceToPlane(point_sphere, point_i);
                    int x0 = static_cast<int>(point_i(0));
                    int y0 = static_cast<int>(point_i(1));
                    double dx = point_i(0) - double(x0);
                    double dy = point_i(1) - double(y0);
                    if (x0 - 1 >= 0 && x0 + 1 < cam_i_model->imageWidth() - 1 && //
                        y0 - 1 >= 0 && y0 + 1 < cam_o_model->imageHeight() - 1){
                        cv::Vec3b p00 = map_i.at<cv::Vec3b>(y0, x0);
                        cv::Vec3b p10 = map_i.at<cv::Vec3b>(y0, x0 + 1);
                        cv::Vec3b p01 = map_i.at<cv::Vec3b>(y0 + 1, x0);
                        cv::Vec3b p11 = map_i.at<cv::Vec3b>(y0 + 1, x0 + 1);
                        cv::Vec3b color = (1 - dx) * (1 - dy) * p00 + //
                                          dx * (1 - dy) * p10 + //
                                          (1 - dx) * dy * p01 + //
                                          dx * dy * p11;
                        map_o.at<cv::Vec3b>(y, x) = color;
                    }else if (x0 - 1 == 0 || x0 + 1 == cam_i_model->imageWidth() - 1 && //
                        y0 - 1 == 0 && y0 + 1 == cam_o_model->imageHeight() - 1){
                        cv::Vec3b p00 = map_i.at<cv::Vec3b>(y0, x0);
                        map_o.at<cv::Vec3b>(y, x) = p00;
                    }
                }
            }
            sensor_msgs::ImagePtr msg_o_i = cv_bridge::CvImage(msg_i->header,"bgr8",map_i).toImageMsg();
            sensor_msgs::ImagePtr msg_o_o = cv_bridge::CvImage(msg_i->header,"bgr8",map_o).toImageMsg();
            o_bag.write(topic_i_image, m.getTime(), msg_o_i);
            o_bag.write(topic_o_image, m.getTime(), msg_o_o);

            if ((num_tag * 1.0 / bag_i_size) > image_o_star_rate && (num_tag * 1.0 / bag_i_size) < image_o_stop_rate)
            {
                cv::imwrite(image_o_path + "/image_in_" + std::to_string(num_tag) + ".png", map_i);
                cv::imwrite(image_o_path + "/image_out_" + std::to_string(num_tag) + ".png", map_o);
            }
        }
        if (float(num_tag * 1.0 / bag_i_size) > image_o_stop_rate) break;
        ROS_WARN("Current Progress: %f%%", 100.0 * num_tag++ / bag_i_size);
    }
    ROS_WARN("Finish: %f%%", 100.0);
    if (cam_i_model != nullptr) delete cam_i_model;
    if (cam_o_model != nullptr) delete cam_o_model;

    return 0;
}