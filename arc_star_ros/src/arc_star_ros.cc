#include <ros/ros.h>
#include <dvs_msgs/EventArray.h>
#include <chrono>

#include "acd/arc_star_detector.h"

typedef std::chrono::high_resolution_clock Clock;

acd::ArcStarDetector detector = acd::ArcStarDetector();
ros::Publisher corner_pub;

void EventMsgCallback(const dvs_msgs::EventArray::ConstPtr &event_msg) {

  const int n_event = event_msg->events.size();

 ROS_INFO("n_event:%d",n_event);

  if (n_event == 0) {return;}

  dvs_msgs::EventArray corner_msg;
  corner_msg.header = event_msg->header;
  corner_msg.width = event_msg->width;
  corner_msg.height = event_msg->height;

  ROS_INFO("width:%d,height:%d", corner_msg.width,corner_msg.height);

  auto t_init = Clock::now();
  for (const auto& e : event_msg->events) {
   
    // ROS_INFO("here!!!!!!!!!");
    //  ROS_INFO("event_t:%d,event_x:%d,event_y:%d,,event_p:%d",e.ts.toSec(),e.x, e.y,e.polarity);

      // Unroll event array and detect Corners
    if (detector.isCorner(e.ts.toSec(), e.x, e.y, e.polarity)) {

      // ROS_INFO("here!!!!!!!!!");
      //  ROS_INFO("push_event_t:%d,push_event_x:%d,push_event_y:%d,,push_event_p:%d",e.ts.toSec(),e.x, e.y,e.polarity);//完全没有，那么就算iscorner有问题
      corner_msg.events.push_back(e);
    }
  }
  auto t_end = Clock::now();

  // Summary from the processed event-package
  // Note: DO NOT use this ROS wrapper for timing benchmarking. Use the stand-alone implementation instead.
  // #########################
  const double elapsed_time = std::chrono::duration_cast<std::chrono::nanoseconds>(t_end - t_init).count();
  const int n_corner = corner_msg.events.size();
  const double percentage_corners = (double(n_corner)/n_event)*100;
  const double time_per_event = elapsed_time/n_event; // Average time to process one event [ns/ev]
  const double event_rate = 1/(time_per_event*1e-3) ; // Average Event Rate [Million ev / s]

  ROS_INFO("Percetange of corners: %.1f%%. Avg. timing: %0.0f ns/ev. Max event rate: %0.2f Mev/s",
           percentage_corners, time_per_event, event_rate);
//  #######################

  // Send detected corner events
  corner_pub.publish(corner_msg);
}


int main(int argc, char **argv) {

  ros::init(argc, argv, "arc_star_ros");
  ros::NodeHandle nh;

//  detector = acd::ArcStarDetector();

  corner_pub = nh.advertise<dvs_msgs::EventArray>("corners", 1);
  // ROS_INFO("here!!!!!!!!!");
  ros::Subscriber event_sub = nh.subscribe("events", 0, &EventMsgCallback);

  while (ros::ok()) {ros::spinOnce();} // Preferred over ros::spin() for performance

  return 0;
}

