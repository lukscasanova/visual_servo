#include <ros/ros.h>
#include <vtec_msgs/TrackingResult.h>
#include <geometry_msgs/Twist.h>
#include <control_toolbox/pid.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Float64.h>

#define GRAVITY_THRUST     0.496
#define GRAVITY            9.81

ros::Publisher* cmd_vel_pub_ptr;
ros::Publisher cmd_att_pub;
ros::Publisher angle_error_pub;

control_toolbox::Pid fx_pid, fy_pid, fz_pid;
control_toolbox::Pid yaw_pid;

mavros_msgs::AttitudeTarget cmd_att_msg;
vtec_msgs::TrackingResult track;
std_msgs::Float64 angle_error_msg;

ros::Time last;

int desired_bbox_x_size = 200;
int desired_bbox_y_size = 200;
int desired_center_x = 320;
int desired_center_y = 320;

std::vector<geometry_msgs::Point> desired_corners, corner_errors;

tf::Vector3 desired_force(0.0, 0.0, 0.0);
tf::Vector3 desired_orientation_rpy(0.0, 0.0, 0.0);

double thrust_factor = GRAVITY_THRUST/GRAVITY;

bool has_tracking = false;

void printVector(const tf::Vector3& v , const std::string& name){
    ROS_INFO_STREAM(name << ": " << v.getX() << ", " << v.getY() << ", " << v.getZ());
}

double applyLimits(const double v, const double lo, const double hi){
    if(v < lo) return lo;
    if(v > hi) return hi;
    return v;
}

void createDesiredCornersList(
   const int desired_bbox_x_size,
   const int desired_bbox_y_size,
   const int desired_center_x,
   const int desired_center_y,
   std::vector<geometry_msgs::Point>& desired_corners){

   desired_corners.clear();

   geometry_msgs::Point up_left, up_right, down_left, down_right;
   down_left.x = desired_center_x - desired_bbox_x_size/2.0;
   down_left.y = desired_center_y - desired_bbox_y_size/2.0;

   up_left.x = desired_center_x - desired_bbox_x_size/2.0;
   up_left.y = desired_center_y + desired_bbox_y_size/2.0;

   down_right.x = desired_center_x + desired_bbox_x_size/2.0;
   down_right.y = desired_center_y - desired_bbox_y_size/2.0;

   up_right.x = desired_center_x + desired_bbox_x_size/2.0;
   up_right.y = desired_center_y + desired_bbox_y_size/2.0;

   desired_corners.push_back(down_left);
   desired_corners.push_back(up_left);
   desired_corners.push_back(down_right);
   desired_corners.push_back(up_right);
  
}


/**
 * @brief      Builds an attitude target message.
 *
 * @param[in]  cmd_dfc_acc   The command direct force control
 * @param[in]  cmd_attitude  The command attitude
 * @param      cmd_att_msg   The command att mavros message
 */
void buildAttTargetMsg(
    const tf::Vector3& cmd_dfc_acc,
    const tf::Vector3& cmd_attitude,
    mavros_msgs::AttitudeTarget& cmd_att_msg){

    cmd_att_msg.header.frame_id = "base_link";
    cmd_att_msg.header.stamp = ros::Time::now();

    cmd_att_msg.type_mask = 
        mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE  |
        mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
        mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE; 

    tf::Quaternion q;
    q.setRPY(   
                cmd_attitude.getX(),
                cmd_attitude.getY(),
                cmd_attitude.getZ() );

    cmd_att_msg.orientation.x = q.getX();
    cmd_att_msg.orientation.y = q.getY();
    cmd_att_msg.orientation.z = q.getZ();
    cmd_att_msg.orientation.w = q.getW();
// 
    double thrust_x = cmd_dfc_acc.getX()*thrust_factor;
    // double thrust_x = f_des.getX()*thrust_factor;
    thrust_x = applyLimits(thrust_x, -1.0f, 1.0);

    double thrust_y = cmd_dfc_acc.getY()*thrust_factor;
    // double thrust_y = f_des.getY()*thrust_factor;
    thrust_y = applyLimits(thrust_y, -1.0f, 1.0);

    double thrust_z = cmd_dfc_acc.getZ()*thrust_factor;
    // double thrust_z = f_des.getZ()*thrust_factor;
    thrust_z = applyLimits(thrust_z, -1.0f, 1.0);

    
    // thrust_x = 0.0;
    // thrust_y = 0.0;
    // thrust_z = 0.6;
    
    // ENU TO NED
    cmd_att_msg.body_rate.x = thrust_x;
    cmd_att_msg.body_rate.y = -thrust_y;
    cmd_att_msg.body_rate.z = -thrust_z;

    cmd_att_msg.thrust = 
    sqrt(thrust_x*thrust_x +
         thrust_y*thrust_y + 
         thrust_z*thrust_z);
   
}

void updateControl(const ros::Duration delta_t, 
   mavros_msgs::AttitudeTarget& cmd){

   // If result not good enough, hold still.
     // if(track.score < 0.5){
      // cmd_vel_pub_ptr->publish(vel);
   // }else{
      // Find center point
   float center_x=0.0, center_y=0.0;
   float dx = 0.0, dy = 0.0;

   for(int i = 0 ; i < 4; i++){
      center_x += track.corners[i].x;
      center_y += track.corners[i].y;
   }

   center_x /= 4;
   center_y /= 4;

   double bbox_size_x;
   double bbox_size_y;

   double bbox_left_size_y = track.corners[1].y-track.corners[0].y;
   double bbox_right_size_y = track.corners[3].x-track.corners[2].y;

   double angle_error = (bbox_right_size_y - bbox_left_size_y);

   bbox_size_x = (track.corners[2].x + track.corners[3].x) - (track.corners[0].x + track.corners[1].x);
   bbox_size_y = (track.corners[1].y + track.corners[3].y) - (track.corners[0].y + track.corners[2].y);

   ROS_INFO_STREAM("bbox_size_x: " << bbox_size_x);
   ROS_INFO_STREAM("bbox_size_y: " << bbox_size_y);

   double size_xy_avg = (bbox_size_y + bbox_size_x)/2.0;

   double size_error = desired_bbox_x_size + desired_bbox_y_size - size_xy_avg;
   ROS_INFO_STREAM("size_error pre normalization: " << size_error);
   size_error /= fabs(size_xy_avg);
   ROS_INFO_STREAM("size_error pos normalization: " << size_error);


   angle_error_msg.data = angle_error;
   angle_error_pub.publish(angle_error_msg);
   ROS_INFO_STREAM("ANGLE ERROR: " << angle_error_msg.data);
   
   ROS_INFO_STREAM("center_x: " << center_x);
   ROS_INFO_STREAM("center_y: " << center_y);
   // Find size
   double x_scale = track.homography[0];
   double y_scale = track.homography[4];

   double xy_scale = x_scale*y_scale;
      // Control in angular velocity

   double f_x = fx_pid.computeCommand(size_error, delta_t);
   double f_y = fy_pid.computeCommand(desired_center_x - center_x, delta_t);
   double f_z = fz_pid.computeCommand(desired_center_y - center_y, delta_t) + GRAVITY;

   ROS_INFO_STREAM("F_Z: " << f_z);
   desired_force.setX(f_x);
   desired_force.setY(f_y);
   desired_force.setZ(f_z);

   buildAttTargetMsg(desired_force, desired_orientation_rpy, cmd);
}

void trackingCallback(const vtec_msgs::TrackingResult::ConstPtr& track_msg){
   track = *track_msg;
   has_tracking = true;
}



int main(int argc, char **argv){

   ros::init(argc, argv, "homography_servo");
   ros::NodeHandle nh;
   ros::NodeHandle nhPrivate("~");

   std::string track_topic = "tracking";
   std::string cmd_vel_topic = "cmd_vel";
   nhPrivate.getParam("track_topic", track_topic);
   nhPrivate.getParam("cmd_vel_topic", cmd_vel_topic);
   nhPrivate.getParam("desired_center_x", desired_center_x);
   nhPrivate.getParam("desired_bbox_x_size", desired_bbox_x_size);
   nhPrivate.getParam("desired_bbox_y_size", desired_bbox_y_size);

   fx_pid.initParam("~fx_pid");
   fy_pid.initParam("~fy_pid");
   fz_pid.initParam("~fz_pid");

   createDesiredCornersList(desired_bbox_x_size, desired_bbox_y_size, desired_center_x, desired_center_y, desired_corners);

   
   // Subscribers 
   ros::Subscriber track_sub = nh.subscribe<vtec_msgs::TrackingResult>(track_topic, 1, trackingCallback);

   // Publishers
   ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1);
   cmd_vel_pub_ptr = &cmd_vel_pub;
   cmd_att_pub = nh.advertise<mavros_msgs::AttitudeTarget>
                    ("mavros/setpoint_raw/attitude", 10);
   angle_error_pub = nh.advertise<std_msgs::Float64>("/angle_error",10);

   last = ros::Time::now();

   ros::Rate rate(30.0);
   while(ros::ok()){

      ros::spinOnce();
      rate.sleep();

      ros::Time now = ros::Time::now();
      ros::Duration delta_t = now - last;

      if(delta_t>ros::Duration(0.001) && has_tracking){
         updateControl(delta_t, cmd_att_msg);
         cmd_att_pub.publish(cmd_att_msg);
      }

      last = now;
   }

}


