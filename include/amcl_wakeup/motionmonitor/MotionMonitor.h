#ifndef AMCL_MOTION_MONITOR
#define AMCL_MOTION_MONITOR

#include <algorithm>
#include <vector>
#include <map>
#include <cmath>
#include <memory>
#include <mutex>

#include "move_base_msgs/MoveBaseActionGoal.h"
#include "nav_msgs/Odometry.h"


#include "ros/ros.h"
#include "amcl/pf/pf_vector.h"
#include "geometry_msgs/Pose.h"

// For transform support
#include "tf2/LinearMath/Transform.h"
#include "tf2/convert.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

using namespace std;

#define MAP_SIZE_X 0.05
#define MAP_SIZE_Y 0.05
#define MAP_SIZE_Z M_PI*10/180.0

typedef struct
{
  // received from wakeup class, origin_poses
  pf_vector_t pose;
  // cluster_poses_tmp
  move_base_msgs::MoveBaseActionGoal goal;

} pose_goal;


class MotionMonitor
{
 public:
    MotionMonitor();
    MotionMonitor(const vector<pf_vector_t>& goal_poses, const vector<pf_vector_t>& orgin_poses);
    ~MotionMonitor();

    // callback functions
    void getAMCLupdate(const geometry_msgs::PoseWithCovarianceStamped& msg);
    void getCurrentOdom(const nav_msgs::OdometryConstPtr& msg);    
    void updateAMCL(const geometry_msgs::PoseWithCovarianceStamped& msg);
    move_base_msgs::MoveBaseActionGoal* getCurrentGoal();

 private:
    pf_vector_t current_pose;
    pf_vector_t last_pose;
    pf_vector_t* amcl_pose = NULL;
    mutex mutex_;
    bool current_pose_init;
    vector<pose_goal> tracking_pose_goal_pairs;

    void updateTrackingposes();
};

// debug
MotionMonitor::MotionMonitor()
{
    ROS_INFO("test mode");
}


MotionMonitor::MotionMonitor(const vector<pf_vector_t>& goal_poses, const vector<pf_vector_t>& origin_poses)
{
    current_pose_init = false;
    ROS_ASSERT(goal_poses.size()==origin_poses.size());
    for (int i=0; i<origin_poses.size(); i++)
    {
        pose_goal pg;
        pg.pose = origin_poses[i];

        move_base_msgs::MoveBaseActionGoal goal;
        goal.header.seq = i;
        goal.header.stamp = ros::Time::now();
        goal.goal.target_pose.header.frame_id = "map";
        goal.goal.target_pose.pose.position.x = goal_poses[i].v[0];
        goal.goal.target_pose.pose.position.y = goal_poses[i].v[1];
        goal.goal.target_pose.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, goal_poses[i].v[2]);
        geometry_msgs::Quaternion q_;
        q_ = tf2::toMsg(q);
        goal.goal.target_pose.pose.orientation = q_;

        pg.goal = goal;
        tracking_pose_goal_pairs.push_back(pg);
    }
    
    ROS_INFO("MotionMonitor initializaed!");
}

MotionMonitor::~MotionMonitor()
{
    ROS_INFO("Target goal reached, MotionMonitor destructed!");
    delete amcl_pose;
}

void MotionMonitor::getCurrentOdom(const nav_msgs::OdometryConstPtr& msg)
{
    tf2::Quaternion q(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);

    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    mutex_.lock();
    current_pose.v[0] = msg->pose.pose.position.x;
    current_pose.v[1] = msg->pose.pose.position.y;
    current_pose.v[2] = yaw;
    mutex_.unlock();

    if (current_pose_init)
        updateTrackingposes();

    last_pose = current_pose;
    current_pose_init = true;
}

void MotionMonitor::updateTrackingposes()
{
    // lock current_pose
    mutex_.lock();
    pf_vector_t delta_pose;
    delta_pose.v[0] = current_pose.v[0] - last_pose.v[0];
    delta_pose.v[1] = current_pose.v[1] - last_pose.v[1];
    delta_pose.v[2] = current_pose.v[2] - last_pose.v[2];
    mutex_.unlock();

    for (int i=0; i<tracking_pose_goal_pairs.size(); i++)
    {
        tracking_pose_goal_pairs[i].pose.v[0] += delta_pose.v[0];
        tracking_pose_goal_pairs[i].pose.v[1] += delta_pose.v[1];
        tracking_pose_goal_pairs[i].pose.v[2] += delta_pose.v[2];
    }
}

// from amcl pose find target pose
void MotionMonitor::getAMCLupdate(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
    if (amcl_pose==NULL)
        amcl_pose = new pf_vector_t;

    tf2::Quaternion q(
    msg.pose.pose.orientation.x,
    msg.pose.pose.orientation.y,
    msg.pose.pose.orientation.z,
    msg.pose.pose.orientation.w);

    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    amcl_pose->v[0] = msg.pose.pose.position.x;
    amcl_pose->v[1] = msg.pose.pose.position.y;
    amcl_pose->v[2] = yaw;
}

move_base_msgs::MoveBaseActionGoal* MotionMonitor::getCurrentGoal()
{
    move_base_msgs::MoveBaseActionGoal *current_goal = NULL;
    double dist = 0.0;
    for (int i=0; i<tracking_pose_goal_pairs.size(); i++)
    {
        double tmp_dist;
        tmp_dist = sqrt(pow(abs(amcl_pose->v[0] - tracking_pose_goal_pairs[i].pose.v[0])/MAP_SIZE_X, 2) + 
                        pow(abs(amcl_pose->v[1] - tracking_pose_goal_pairs[i].pose.v[1])/MAP_SIZE_Y, 2) + 
                        pow(abs(amcl_pose->v[2] - tracking_pose_goal_pairs[i].pose.v[2])/MAP_SIZE_Z, 2));
        if (current_goal == NULL || tmp_dist < dist)
        {
            dist = tmp_dist;
            current_goal = &(tracking_pose_goal_pairs[i].goal);
        }
    }
    // update time

    return current_goal;
}
    
#endif