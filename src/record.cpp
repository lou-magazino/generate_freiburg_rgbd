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

sensor_msgs::Image::ConstPtr img_depth, img_rgb;
std::ofstream assoc;

void
save()
{
  static boost::mutex lock;
  lock.lock();
  if(img_depth && img_rgb)
  {
    std::string base_dir("/tmp/freiburg");
    ROS_INFO("%s", base_dir.c_str());
    // if when both exist
    ros::Time now = ros::Time::now();
    if(!boost::filesystem::is_directory(boost::filesystem::path(base_dir)))
    {
      boost::filesystem::create_directory(boost::filesystem::path(base_dir));
      boost::filesystem::create_directory(boost::filesystem::path(base_dir + "/depth"));
      boost::filesystem::create_directory(boost::filesystem::path(base_dir + "/rgb"));
    }
    assoc.open((base_dir + "/assoc.txt").c_str(), std::ofstream::app);
    assoc << now << " rgb/" << now << ".png "
          << now << " depth/" << now << ".png" << std::endl;
    assoc.close();
    
    // rescale depth, then save both
    cv_bridge::CvImageConstPtr cv_rgb_ptr = cv_bridge::toCvShare(img_rgb, sensor_msgs::image_encodings::BGR8);
    cv_bridge::CvImageConstPtr cv_depth_ptr = cv_bridge::toCvShare(img_rgb, sensor_msgs::image_encodings::MONO16);
    std::stringstream name_stream;
    name_stream << now << ".png";
    cv::imwrite(base_dir + "/rgb/" + name_stream.str(), cv_rgb_ptr->image);
    cv::imwrite(base_dir + "/depth/" + name_stream.str(), cv_depth_ptr->image * 5);
    
    img_depth.reset();
    img_rgb.reset();
  }
  lock.unlock();
}

void
cb_rgb(const sensor_msgs::Image::ConstPtr &msg)
{
  img_rgb = msg;
  save();
}

void
cb_depth(const sensor_msgs::Image::ConstPtr &msg)
{
  img_depth = msg;
  save();
}

int
main(int argn, char** argc)
{
  ros::init(argn, argc, "record_freiburg");
  ros::NodeHandle nh;

  ros::Subscriber sub_rgb = nh.subscribe<sensor_msgs::Image>("/camera/rgb/image_raw", 10, cb_rgb);
  ros::Subscriber sub_depth = nh.subscribe<sensor_msgs::Image>("/camera/depth_registered/sw_registered/image_rect_raw", 10, cb_depth);
  
  ros::spin();
  return 0;
}