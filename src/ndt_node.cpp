#include <cstdio>
#include <memory>
#include <string>
#include <iostream>

#include "ndt_matching/ndt_lib.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>



static pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud;

void print_usage()
{
  printf("Usage for listener app:\n");
  printf("listener [-t topic_name] [-h]\n");
  printf("options:\n");
  printf("-h : Print this help function.\n");
  printf("-t topic_name : Specify the topic on which to subscribe. Defaults to chatter.\n");
}

// Create a Listener class that subclasses the generic rclcpp::Node base class.
// The main function below will instantiate the class as a ROS node.
class Listener : public rclcpp::Node
{
public:
  explicit Listener(const std::string & topic_name, const std::string & topic_name2 = "map") 
  : Node("listener")
  {

    RCLCPP_INFO(this->get_logger(), "Listener started.");         

    // Create a callback function for when messages are received.
    // Variations of this function also exist using, for example UniquePtr for zero-copy transport.
    auto callback =
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr filtered_points) -> void
      {
        RCLCPP_INFO(this->get_logger(), "filtered_points received: [%s]", filtered_points->header.frame_id.c_str());
        //TODO:
        // here you call NdtLib function and pass in the msg as input
        // return a pose message and publish it as https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/PoseStamped.msg
        

        if (ndt.isMapLoaded()){
          
          ndt.setInputSource(filtered_points);  

          
          Eigen::Matrix<float, 4, 4> guess = ndt.getFinalTransformation();

          ndt.align(guess);

          if (ndt.isConverged()){
            geometry_msgs::msg::PoseStamped ndt_pose_msg = ndt.getNdtPose();
            publisher_->publish(ndt_pose_msg);
            RCLCPP_INFO(this->get_logger(), "ndt_pose_msg published.");           
          }

        }
        else{
          RCLCPP_INFO(this->get_logger(), "No map received. filtered_points msg is ignored!");
        }        


      };



    ndt.setInputTarget(map_cloud);



    auto callback2 =
    [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) -> void
      {
        RCLCPP_INFO(this->get_logger(), "Map received: [%s]", msg->header.frame_id.c_str());
        //TODO: here you get your map point cloud (one time only)
        //ndt.setInputTarget(msg);
      };

    // Create a subscription to the topic which can be matched with one or more compatible ROS
    // publishers.
    // Note that not all publishers on the same topic with the same type will be compatible:
    // they must have compatible Quality of Service policies.
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(topic_name, callback);
    sub2_ = create_subscription<sensor_msgs::msg::PointCloud2>(topic_name2, callback2);
    // TODO: create a pose publisher, see for reference
    // https://github.com/ros2/demos/blob/master/demo_nodes_cpp/src/topics/talker.cpp
    publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>("akif_pose");

  }

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub2_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  ndt_matching::NdtLib ndt;
};







int main(int argc, char * argv[])
{
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  if (rcutils_cli_option_exist(argv, argv + argc, "-h")) {
    print_usage();
    return 0;
  }




  std::cout<<"Reading cloud map..."<<std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("map.pcd", *cloud) == -1) //* load the file
  {
    std::cout<<"Couldn't read file map.pcd"<<std::endl;
    return (-1);
  }
  std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from map.pcd."
            << std::endl;

  map_cloud = cloud;
  std::cout<<"Reading cloud map has FINISHED..."<<std::endl;



  // Initialize any global resources needed by the middleware and the client library.
  // You must call this before using any other part of the ROS system.
  // This should be called once per process.
  rclcpp::init(argc, argv);

  // Parse the command line options.
  auto topic = std::string("filtered_points");
  char * cli_option = rcutils_cli_get_option(argv, argv + argc, "-t");
  if (nullptr != cli_option) {
    topic = std::string(cli_option);
  }

  // Create a node.
  auto node = std::make_shared<Listener>(topic);

  // spin will block until work comes in, execute work as it becomes available, and keep blocking.
  // It will only be interrupted by Ctrl-C.
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
