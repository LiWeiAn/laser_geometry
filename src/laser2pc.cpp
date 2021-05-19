#include <iostream>
#include <iomanip>
#include <chrono>
#include <functional>
#include <memory>
#include <utility>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "laser_geometry/laser_geometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"


using namespace std::chrono_literals;
using std::placeholders::_1;



class MinimalPublisher : public rclcpp::Node
{
public:
    MinimalPublisher()
            : Node("minimal_publisher"), count_(0)
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("laser_pointcloud", 10);
        //*****************subscriber for simulation (gazebo):

         //subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", rclcpp::QoS(rclcpp::SystemDefaultsQoS()), std::bind(&MinimalPublisher::scanCallback, this, _1));
         //this->set_parameter(rclcpp::Parameter("use_sim_time", true));
        
        //*****************subscriber for real life scanner:
        
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("serena/scan", 10 , std::bind(&MinimalPublisher::scanCallback, this, _1));

    }


private:
    void scanCallback (const sensor_msgs::msg::LaserScan::SharedPtr scan_in)
    {

        // clock_ =  this->get_clock();
        // tf2_ros::Buffer buffer_(clock_);
        // listener_ = std::make_shared<tf2_ros::TransformListener>(buffer_);

        // rclcpp::Rate rate(20.0);

        // std::string transform_error;
        //  if(!buffer_.canTransform("laser", "base_link",
        //          tf2_ros::fromMsg(scan_in->header.stamp) + tf2::durationFromSec(scan_in->ranges.size()*scan_in->time_increment),
        //          tf2::durationFromSec(1.0), &transform_error)){
        //if(buffer_.canTransform("laser", "base_link", tf2::TimePoint(), &transform_error)){
        //    //RCLCPP_INFO(this->get_logger(), "waiting");
        //    rate.sleep();
        //}

        sensor_msgs::msg::PointCloud2 cloud;
        
        //projector_.transformLaserScanToPointCloud("laser", *scan_in, cloud, buffer_);
        projector_.projectLaser(*scan_in, cloud);
        //cloud.header.frame_id = "laser";
        rclcpp::Time t = rclcpp::Node::now();
        
        
        cloud.header.stamp = scan_in->header.stamp;
        publisher_->publish(cloud);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    laser_geometry::LaserProjection projector_;
    // std::shared_ptr<tf2_ros::Buffer> buffer_;
    // std::shared_ptr<tf2_ros::TransformListener> listener_;
    size_t count_;
    rclcpp::Clock::SharedPtr clock_;
    

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}

