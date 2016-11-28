// just a strange format which I guess is raw format used by clams
#include <fstream>
#include <sstream>
#include <string>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/thread/mutex.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <opencv2/core/eigen.hpp>

namespace po = boost::program_options;
sensor_msgs::Image::ConstPtr img_depth_global, img_rgb_global;
std::string base_dir_global("/tmp/freiburg");

void
save()
{
  static boost::mutex lock;
  lock.lock();
  if(img_depth_global && img_rgb_global)
  {
    // if when both exist
    ros::Time now = ros::Time::now();
    
    // rescale depth, then save both
    cv_bridge::CvImageConstPtr cv_rgb_ptr = cv_bridge::toCvShare(img_rgb_global, sensor_msgs::image_encodings::BGR8);
    cv_bridge::CvImageConstPtr cv_depth_ptr = cv_bridge::toCvShare(img_depth_global, sensor_msgs::image_encodings::TYPE_16UC1);
    ROS_INFO("%s", base_dir_global.c_str());
    
    std::stringstream name_stream;
    name_stream << now;
    
    // timestamp
    std::ofstream timestamp;
    timestamp.open((base_dir_global + "/" + name_stream.str() + ".clk").c_str(), std::ofstream::app);
    timestamp << now << std::endl;
    timestamp.close();
    
    // rgb image
    cv::imwrite(base_dir_global + "/" + name_stream.str() + ".png", cv_rgb_ptr->image);
//     cv::imwrite(base_dir_global + "/" + name_stream.str() + ".eig", cv_depth_ptr->image);
    // write depth map from raw data
    std::ofstream depth;
    const int width = cv_depth_ptr->image.size().width;
    const int height = cv_depth_ptr->image.size().height;
    const int bytes = 2; // short type 2 bytes
    depth.open((base_dir_global + "/" + name_stream.str() + ".eig").c_str(), std::ofstream::binary);
    depth.write((const char*)&bytes, sizeof(int));
    depth.write((const char*)&height, sizeof(int));
    depth.write((const char*)&width, sizeof(int));
    std::vector<ushort> buffer(0, width * height);
    
//     depth.write((const char*)cv_depth_ptr->image.data, bytes * width * height);
    Eigen::Matrix<ushort, Eigen::Dynamic, Eigen::Dynamic> depth_mat;
    cv2eigen(cv_depth_ptr->image, depth_mat);
    depth.write((const char*)depth_mat.data(), bytes * width * height);
    depth.close();
    
    img_depth_global.reset();
    img_rgb_global.reset();
  }
  lock.unlock();
}

void
cb_rgb(const sensor_msgs::Image::ConstPtr &msg)
{
  img_rgb_global = msg;
  save();
}

void
cb_depth(const sensor_msgs::Image::ConstPtr &msg)
{
  img_depth_global = msg;
  save();
}

int
main(int argn, char** argc)
{
  ros::init(argn, argc, "record_freiburg");
  ros::NodeHandle nh;
  
  // parameters
  po::options_description desc;
  desc.add_options()
    ("dst,d", po::value<std::string>()->required(), "directory to store");
  po::variables_map mapping; 
  po::store(po::parse_command_line(argn, argc, desc), mapping);
  if(mapping.count("dst"))
  {
    base_dir_global = mapping["dst"].as<std::string>();
  }
    
  if(boost::filesystem::is_directory(boost::filesystem::path(base_dir_global)))
  {
    boost::filesystem::remove_all(boost::filesystem::path(base_dir_global)); // delete all
  }
  boost::filesystem::create_directory(boost::filesystem::path(base_dir_global));
  
  // fake model file to avoid stream sequence pcl wrapper
  std::ofstream fake_model;
  fake_model.open((base_dir_global + "/primesense_model").c_str(), std::ofstream::app);
  fake_model.close();

  ros::Subscriber sub_rgb = nh.subscribe<sensor_msgs::Image>("/camera/rgb/image_rect_color", 10, cb_rgb);
  ros::Subscriber sub_depth = nh.subscribe<sensor_msgs::Image>("/camera/depth_registered/sw_registered/image_rect_raw", 10, cb_depth);
  
  ros::spin();
  return 0;
}
