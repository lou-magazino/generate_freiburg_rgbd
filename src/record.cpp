#include <fstream>
#include <sstream>
#include <string>
#include <boost/filesystem.hpp>
#include <boost/thread/mutex.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

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
    if(!boost::filesystem::is_directory(boost::filesystem::path(base_dir_global)))
    {
      boost::filesystem::create_directory(boost::filesystem::path(base_dir_global));
      boost::filesystem::create_directory(boost::filesystem::path(base_dir_global + "/depth"));
      boost::filesystem::create_directory(boost::filesystem::path(base_dir_global + "/rgb"));
    }
    std::ofstream assoc;
    assoc.open((base_dir_global + "/assoc.txt").c_str(), std::ofstream::app);
    assoc << now << " rgb/" << now << ".png "
          << now << " depth/" << now << ".png" << std::endl;
    assoc.close();
    
    // rescale depth, then save both
    cv_bridge::CvImageConstPtr cv_rgb_ptr = cv_bridge::toCvShare(img_rgb_global, sensor_msgs::image_encodings::BGR8);
    cv_bridge::CvImageConstPtr cv_depth_ptr = cv_bridge::toCvShare(img_depth_global, sensor_msgs::image_encodings::TYPE_16UC1);
    ROS_INFO("%s", base_dir_global.c_str());
    //cv::Mat mat_depth;
    std::stringstream name_stream;
    name_stream << now << ".png";
    //cv_depth_ptr->image.convertTo(mat_depth, CV_16UC1);
    cv::imwrite(base_dir_global + "/rgb/" + name_stream.str(), cv_rgb_ptr->image);
    cv::imwrite(base_dir_global + "/depth/" + name_stream.str(), cv_depth_ptr->image * 5);
    
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
  if(boost::filesystem::is_directory(boost::filesystem::path(base_dir_global)))
  {
    boost::filesystem::remove_all(boost::filesystem::path(base_dir_global)); // delete all
  }

  ros::Subscriber sub_rgb = nh.subscribe<sensor_msgs::Image>("/camera/rgb/image_rect_color", 10, cb_rgb);
  ros::Subscriber sub_depth = nh.subscribe<sensor_msgs::Image>("/camera/depth_registered/sw_registered/image_rect_raw", 10, cb_depth);
  
  ros::spin();
  return 0;
}