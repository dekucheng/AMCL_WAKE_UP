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
    MotionMonitor(const vector<pf_vector_t>& goal_poses, const vector<pf_vector_t>& orgin_poses);
    ~MotionMonitor();

    // callback functions
    void getAMCLupdate(const geometry_msgs::PoseWithCovarianceStamped& msg);
    void getCurrentOdom(const nav_msgs::OdometryConstPtr& msg);    
    void updateAMCL(const geometry_msgs::PoseWithCovarianceStamped& msg);
    void getTrackingPoses(vector<pf_vector_t>& tp);
    move_base_msgs::MoveBaseActionGoal* getCurrentGoal();

 private:
    pf_vector_t current_pose;
    pf_vector_t last_pose;
    pf_vector_t* amcl_pose;
    mutex mutex_;
    bool current_pose_init;
    vector<pose_goal> tracking_pose_goal_pairs;

    void updateTrackingposes();
};
    
#endif