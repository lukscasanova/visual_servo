#include <ros/ros.h>
#include <vtec_msgs/TrackingResult.h>
#include <geometry_msgs/Twist.h>
#include <control_toolbox/pid.h>

ros::Publisher* cmd_vel_pub_ptr;

control_toolbox::Pid ang_vel_pid, lin_vel_pid;

ros::Time last;

float desired_center_x = 320;
float desired_xy_scale = 1.0;

void trackingCallback(vtec_msgs::TrackingResult track){
   geometry_msgs::Twist vel;
   vel.linear.x = 0.0;
   vel.linear.y = 0.0;
   vel.linear.z = 0.0;
   vel.angular.x = 0.0;
   vel.angular.y = 0.0;
   vel.angular.z = 0.0;

   ros::Time now = ros::Time::now();
   
   // If result not good enough, hold still.
   if(track.score < 0.5){
      cmd_vel_pub_ptr->publish(vel);
   }else{
      // Find center point
      float center_x=0.0, center_y=0.0;
      for(int i = 0 ; i < 4; i++){
         center_x+= track.corners[i].x;
         center_y+= track.corners[i].y;
      }
      center_x /= 4;
      center_y /= 4;

      // Find size
      double x_scale = track.homography[0];
      double y_scale = track.homography[4];

      double xy_scale = x_scale*y_scale;

      // Control in angular velocity
      double ang_vel = ang_vel_pid.computeCommand(desired_center_x - center_x, now-last);
      double lin_vel = lin_vel_pid.computeCommand(desired_xy_scale - xy_scale, now-last);
      vel.linear.x = lin_vel;
      vel.angular.z = ang_vel;
      cmd_vel_pub_ptr->publish(vel);
   }

   last = now;
}

int main(int argc, char **argv){

   ros::init(argc, argv, "homography_servo");
   ros::NodeHandle nh;
   ros::NodeHandle nhPrivate("~");

   std::string track_topic = "tracking";
   std::string cmd_vel_topic = "cmd_vel";
   nhPrivate.getParam("track_topic", track_topic);
   nhPrivate.getParam("cmd_vel_topic", cmd_vel_topic);
   nhPrivate.getParam("desired_xy_scale", desired_xy_scale);
   nhPrivate.getParam("desired_center_x", desired_center_x);

   ang_vel_pid.initParam("~ang_vel_pid");
   lin_vel_pid.initParam("~lin_vel_pid");
   
   // Subscribers 
   ros::Subscriber track_sub = nh.subscribe(track_topic, 1, trackingCallback);

   // Publishers
   ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1);
   cmd_vel_pub_ptr = &cmd_vel_pub;

   last = ros::Time::now();

   while(ros::ok()){
      ros::spinOnce();
   }

}


