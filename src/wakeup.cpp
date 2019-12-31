#include <algorithm>
#include <vector>
#include <map>
#include <cmath>
#include <memory>
#include <mutex>

// roscpp
#include "ros/ros.h"
#include <signal.h>

#include "amcl_wakeup/map/map.h"
#include "amcl_wakeup/pf/pf_vector.h"
#include "portable_utils.hpp"
#include "amcl_wakeup/motionmonitor/MotionMonitor.h"

// Messages that I need
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/SetMap.h"
#include "std_srvs/Empty.h"
#include <visualization_msgs/Marker.h>
#include "move_base_msgs/MoveBaseActionGoal.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include "amcl_wakeup/PoseWithWeight.h"
#include "amcl_wakeup/PoseArrayWithWeight.h"


// For transform support
#include "tf2/LinearMath/Transform.h"
#include "tf2/convert.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "message_filters/subscriber.h"

typedef struct
{
  pf_vector_t pose;

  double weight;

  int id;

} pose_with_weight;


using namespace std;

class WakeUp
{
 public:
    WakeUp();
    ~WakeUp();
 
 private:
    bool DEBUG;

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber cluster_pose_sub, odom_sub, amcl_sub, move_base_goal_sub;
    ros::Publisher marker_pub, marker_pub_, goal_pub, tracking_poses_pub, goal_visualize_pub;
    ros::ServiceClient clear_costmaps;
    mutex mutex_;

    int max_beams;
    double max_range;
    double min_range;
    double grid_size;
    double search_width;
    bool laser_simu_req;
    vector<pf_vector_t> cluster_poses;
    vector<pf_vector_t> cluster_poses_tmp;
    vector<vector<double>> fake_readings_clust;

    map_t* map_;
    bool map_converted;
    bool use_map_topic;
    double max_occ_dist;

    bool IFVISUALIZE_FAKE_LASER;
    bool IFVISUALIZE_SEARCH;
    bool IF_PRINT;
    double wait_time;

    void requestMap();
    void handleMapMessage(const nav_msgs::OccupancyGrid& msg);
    void freememory();
    map_t* convertMap(const nav_msgs::OccupancyGrid& map_msg);
    void clusterPoseReceived(const amcl_wakeup::PoseArrayWithWeightConstPtr& msg);
    bool laserSimulate();
    // cluster callbacks
    void handleClusterPose(const amcl_wakeup::PoseArrayWithWeight& msg);
    void calc_fake_readings_stats();
    void grid_map_search();
    void updateAMCL(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
    void checkmovestatus(const actionlib_msgs::GoalStatusArrayConstPtr& msg);
    void relabel_clusters(vector<pose_with_weight>& PWW, vector<vector<pose_with_weight>>& PWW_labeled);
    void dfs_label(vector<pose_with_weight>& PWW, pose_with_weight& pww, vector<pose_with_weight>& pww_labeled, int& clst_id);

    double z_hit, z_rand, sigma_hit;
    double score_threshold;
    void SimuBeamModelConfig();
    double LikelihoodFieldModel(const vector<double> &fake_reading, const vector<pf_vector_t> &poses);

    // communicate with move_base
    MotionMonitor* mtmonitor;
    move_base_msgs::MoveBaseActionGoal* current_goal;
    int last_tracking_goal_id;
    int check_clust_num;
    bool CHECK_CLUST_NUM_CHANGE;
};

WakeUp::WakeUp() :
      private_nh_("~"),
      current_goal(NULL),
      last_tracking_goal_id(-1)
{
    use_map_topic = false;
    laser_simu_req = true;
    map_converted = false;
    grid_size = 0.05;
    max_occ_dist = 2.0;
    check_clust_num = -1;
    CHECK_CLUST_NUM_CHANGE = false;

    cluster_pose_sub = nh_.subscribe("cluster_poses", 5, &WakeUp::clusterPoseReceived, this);
    marker_pub = nh_.advertise<visualization_msgs::Marker>("visualization_fakelaser_marker", 2);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_search_marker", 2);    
    goal_pub = nh_.advertise<move_base_msgs::MoveBaseActionGoal>("move_base/goal", 2);
    tracking_poses_pub = nh_.advertise<visualization_msgs::Marker>("visualization_tracking_poses", 10);
    goal_visualize_pub = nh_.advertise<visualization_msgs::Marker>("visualization_goal", 2);
    clear_costmaps = nh_.serviceClient<std_srvs::Empty>("move_base/clear_costmaps");

    if (!use_map_topic)
    {
        SimuBeamModelConfig();
        requestMap();
    }
}
WakeUp::~WakeUp()
{
  freememory();
  delete mtmonitor;
}

void WakeUp::clusterPoseReceived(const amcl_wakeup::PoseArrayWithWeightConstPtr& msg)
{
  if (laser_simu_req && map_converted)
    handleClusterPose(*msg);
}

void WakeUp::handleClusterPose(const amcl_wakeup::PoseArrayWithWeight& msg)
{
  laser_simu_req = false;
  cluster_poses.clear();
  vector<pose_with_weight> PWW;
  for (int i=0; i<msg.poses.size(); i++)
  {
    // drop out clusters with low confidence
    if (msg.poses[i].weight < 0.001)
      continue;
    pose_with_weight p;
    tf2::Quaternion q(
      msg.poses[i].orientation.x,
      msg.poses[i].orientation.y,
      msg.poses[i].orientation.z,
      msg.poses[i].orientation.w);

    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    p.pose.v[0] = msg.poses[i].position.x;
    p.pose.v[1] = msg.poses[i].position.y;
    p.pose.v[2] = yaw;
    p.weight = msg.poses[i].weight;
    p.id = -1;
    PWW.push_back(p);
  }
  cout << PWW.size() << " clusters got from topic cluster_poses, ready to re-clustering" << endl;

  // workspace
  vector<vector<pose_with_weight>> PWW_labeled;
  relabel_clusters(PWW, PWW_labeled);
  
  // size of PWW should equal to the number of clusters
  for (int i=0; i<PWW_labeled.size(); i++)
  {
    vector<pose_with_weight>* pww;
    pww = &PWW_labeled[i];
    double weight = 0.0;
    for (int j=0; j<pww->size(); j++)
    {
      weight += (*pww)[j].weight;

    }

    cout << "weight for each sub lab pww is " << weight << endl;


    pf_vector_t cl_p;
    cl_p.v[0] = cl_p.v[1] = cl_p.v[2] = 0.0;

    for (int j=0; j<pww->size(); j++)
    {
      cl_p.v[0] += (*pww)[j].weight/weight * (*pww)[j].pose.v[0];
      cl_p.v[1] += (*pww)[j].weight/weight * (*pww)[j].pose.v[1];
      cl_p.v[2] += (*pww)[j].weight/weight * (*pww)[j].pose.v[2];
      cout << "the cl_p is " << (*pww)[j].pose.v[0] << "    " << (*pww)[j].pose.v[1] << "     " << (*pww)[j].pose.v[2] << endl;
    }

    cluster_poses.push_back(cl_p);
  }

  ROS_INFO("%d cluster poses got!!!", cluster_poses.size());
    // lock_guard<mutex> guard(mutex_);
  if (cluster_poses.size()==1)
  {
    ROS_INFO("amcl pose converges, no need to do grid map searching!");
    laser_simu_req = true;
  }
  else
  {
    // keep updating current searching grid poses
    if (DEBUG)
      {
        ROS_INFO("Starting DEBUG MODE!!!");
        if (laserSimulate());
          double L2 = LikelihoodFieldModel(fake_readings_clust[0], cluster_poses);
      }
    else
      grid_map_search();
    // laser_simu_req = true;
  }
}

void WakeUp::relabel_clusters(vector<pose_with_weight>& PWW, vector<vector<pose_with_weight>>& PWW_labeled)
{
  int clst_id = 0;
  for (int i=0; i<PWW.size(); i++)
  {
    vector<pose_with_weight> pww_labeled;
    if(PWW[i].id>=0)
      continue;
    PWW[i].id = clst_id;
    pww_labeled.push_back(PWW[i]);
    dfs_label(PWW, PWW[i], pww_labeled, clst_id);
    PWW_labeled.push_back(pww_labeled);
    clst_id++;
  }

  ROS_ASSERT(PWW_labeled.size()==clst_id);
}

void WakeUp::dfs_label(vector<pose_with_weight>& PWW, pose_with_weight& pww, vector<pose_with_weight>& pww_labeled, int& clst_id)
{
  for (int i=0; i<PWW.size(); i++)
  {
    if (PWW[i].id >= 0)
      continue;
    double dist;
    dist = sqrt(pow(abs(PWW[i].pose.v[0] - pww.pose.v[0])/MAP_SIZE_X, 2) + 
                pow(abs(PWW[i].pose.v[1] - pww.pose.v[1])/MAP_SIZE_Y, 2) + 
                pow(abs(PWW[i].pose.v[2] - pww.pose.v[2])/MAP_SIZE_Z, 2));
    if (dist < 3)
    {
      PWW[i].id = clst_id;
      pww_labeled.push_back(PWW[i]);
      dfs_label(PWW, PWW[i], pww_labeled, clst_id);
    }
  }
}

// when new amcl_pose estimate comes in
void WakeUp::updateAMCL(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
  mutex_.lock();
  mtmonitor->getAMCLupdate(*msg);
  move_base_msgs::MoveBaseActionGoal* goal;
  goal = mtmonitor->getCurrentGoal();
  cout << "current goal id is: " << goal->header.seq << endl;

  if (current_goal == NULL || current_goal->header.seq != goal->header.seq)
  {
    // clear cost maps to navigate
    std_srvs::Empty emp;
    clear_costmaps.call(emp);
    current_goal = goal;
    goal_pub.publish(*current_goal);
    ROS_INFO("new goal published!!!");

    // visualize the published goal
    visualization_msgs::Marker goal_marker;
    goal_marker.header.frame_id = "map";
    goal_marker.ns = "current_goal";
    goal_marker.header.stamp = ros::Time::now();
    goal_marker.action = visualization_msgs::Marker::ADD;
    goal_marker.pose.orientation.w = 1.0;
    goal_marker.id = 0;
    goal_marker.type = visualization_msgs::Marker::CUBE;
    goal_marker.scale.x = 0.3;
    goal_marker.scale.y = 0.3;
    goal_marker.scale.z = 0.01;

    // Points are green
    goal_marker.color.r = 1.0f;
    goal_marker.color.a = 1.0;
    goal_marker.pose.position.x = current_goal->goal.target_pose.pose.position.x;
    goal_marker.pose.position.y = current_goal->goal.target_pose.pose.position.y;
    goal_marker.pose.position.z = 0.0;
    goal_visualize_pub.publish(goal_marker);
  }

  vector<pf_vector_t> tracking_poses;
  mtmonitor->getTrackingPoses(tracking_poses);

  visualization_msgs::Marker points;
  points.header.frame_id = "map";
  points.ns = "tracking_poses";
  points.header.stamp = ros::Time::now();
  points.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = 1.0;
  points.id = 0;
  points.type = visualization_msgs::Marker::POINTS;
  points.scale.x = 0.1;
  points.scale.y = 0.1;

  // Points are green
  points.color.r = 1.0f;
  points.color.b = 0.75;
  points.color.a = 1.0;

  for(int i=0; i<tracking_poses.size(); i++)
  {
    geometry_msgs::Point p;
    p.x = tracking_poses[i].v[0];
    p.y = tracking_poses[i].v[1];
    p.z = 0.0;
    points.points.push_back(p); 
  }
  // tracking_poses_pub.publish(points);

  mutex_.unlock();
}

bool WakeUp::laserSimulate()
{
  // clear fake readings
  fake_readings_clust.clear();
  // check if all tmp poses are valid

  pf_vector_t P;
  if (DEBUG)
    P = cluster_poses[0];
  else
  {
    P = cluster_poses_tmp[0];
    for (int i=0; i<cluster_poses_tmp.size(); i++)
    {
      double x,y;
      x = cluster_poses_tmp[i].v[0];
      y = cluster_poses_tmp[i].v[1];
      int mi_, mj_;
      mi_ = MAP_GXWX(map_, x);
      mj_ = MAP_GXWX(map_, y);

      if (!MAP_VALID(map_, mi_, mj_))
        return false;

      else
      {
        int occ_state;
        double occ_dist;
        occ_state = map_->cells[MAP_INDEX(map_,mi_,mj_)].occ_state;
        occ_dist = map_->cells[MAP_INDEX(map_,mi_,mj_)].occ_dist;
        if (occ_state != -1 || occ_dist < 0.20)
            return false;
      }
    }
  }

  double theta = P.v[2];
  double dtheta;
  dtheta = (2*M_PI / (double)max_beams);

  vector<double> fake_readings;
  for (int j=0; j<max_beams; j++)
  {
    // simulate each ray casting
    theta += j*dtheta;
    double range = min_range;
    double simx, simy;
    int mi, mj, last_mi, last_mj;
    simx = range*cos(theta) + P.v[0];
    simy = range*sin(theta) + P.v[1];
    mi = MAP_GXWX(map_, simx);
    mj = MAP_GYWY(map_, simy);
    // ROS_INFO("!!! fake scan simx is %f, \n"
            //  "fake scan simy is %f \n"
            //  "range*cos(theta) is %f \n"
            //  "range*sin(theta) is %f \n"
            //  "pose_x is %f pose_y is %f !!!", simx, simy, range*cos(theta), range*sin(theta), cluster_poses[i].v[0], cluster_poses[i].v[1]);

    if (MAP_VALID(map_, mi, mj))
    {
      last_mi = mi;
      last_mj = mj;    
      do
      {
        range += grid_size/2.0;
        simx = range*cos(theta) + P.v[0];
        simy = range*sin(theta) + P.v[1];
        mi = MAP_GXWX(map_, simx);
        mj = MAP_GYWY(map_, simy);

        if(mi!=last_mi || mj!=last_mj)
        {
          // only happens on min_range?
          if(!MAP_VALID(map_, mi, mj))
            break;
          else
          {
            int occ_state;
            occ_state = map_->cells[MAP_INDEX(map_,mi,mj)].occ_state;
            if (occ_state != -1)
              break;
          }
          last_mi = mi;
          last_mj = mj;
        }
        else
          continue;
      } while (range <= max_range-grid_size/2.0);
    }

    if (range>max_range)
      range = max_range;
    fake_readings.push_back(range);
  }
  fake_readings_clust.push_back(fake_readings);

  return true;
}

void WakeUp::checkmovestatus(const actionlib_msgs::GoalStatusArrayConstPtr& msg)
{
  if (msg->status_list.empty())
    return;
  const actionlib_msgs::GoalStatus* gst;
  int c_id;
  c_id = msg->status_list.size()-1;
  gst = &(msg->status_list[c_id]);
  if (gst->goal_id.stamp.sec!= last_tracking_goal_id && gst->status == 3)
  {
    last_tracking_goal_id = gst->goal_id.stamp.sec;
    ROS_INFO("current goal has been reached, deleting motionmonitor!");

    amcl_sub.shutdown();
    odom_sub.shutdown();
    delete mtmonitor;
    current_goal = NULL;
    laser_simu_req = true;
    ROS_INFO("hang off for amcl convergence!");
    ros::Duration d(wait_time);
    d.sleep();
  }
}

void WakeUp::grid_map_search()
{
  // cluster_poses_tmp = cluster_poses;
  if (cluster_poses.size()==0)
  {
    ROS_WARN("Cluster Poses not available, grid map search failed");
    return;    
  }
  
  // pick one pose from cluster poses (to do: pick the one with highes score)
  double theta_ = cluster_poses[0].v[2];
  double dtheta_ = 0.0 - theta_;
  double step = grid_size;
  double search_width_t = grid_size; // first search_width
  int loop_width_count = 0;

  vector<double> theta_turned, theta_loop;
  theta_turned.resize(cluster_poses.size());
  theta_loop.resize(cluster_poses.size());

  while (search_width_t <= search_width)
  {
    // clear tmp cluster poses
    cluster_poses_tmp.clear();
    loop_width_count++;
    // loop over each grid in one spiral

    for (int c_loop_count=1; c_loop_count<=loop_width_count*8; c_loop_count++)
    {
      // for each grid loop over all cluster poses
      for (int i=0; i<cluster_poses.size(); i++)
      {
        // change loop directions
        if (c_loop_count==2 || c_loop_count==loop_width_count+1 || c_loop_count==loop_width_count*3+1 ||
            c_loop_count==loop_width_count*5+1 || c_loop_count==loop_width_count*7+1)
            {theta_loop[i] -= M_PI/2.0;}

        if (c_loop_count==1)
        {
          // Initialize temporary cluster poses
          pf_vector_t tmp_pose;
          theta_turned[i] = theta_loop[i] = cluster_poses[i].v[2] + dtheta_;
          tmp_pose.v[0] = cluster_poses[i].v[0] + search_width_t*cos(theta_turned[i]);
          tmp_pose.v[1] = cluster_poses[i].v[1] + search_width_t*sin(theta_turned[i]); 
          tmp_pose.v[2] = cluster_poses[i].v[2];
          cluster_poses_tmp.push_back(tmp_pose);
        }
        else 
        {
          double t = theta_loop[i];
          ROS_ASSERT(cluster_poses_tmp.size() == cluster_poses.size());

          cluster_poses_tmp[i].v[0] += step*cos(t);
          cluster_poses_tmp[i].v[1] += step*sin(t);
        }
      }


      if (laserSimulate()) // Simulate Succeed, which means tmp poses are valid
      {
        if (IFVISUALIZE_SEARCH)
        {
          visualization_msgs::Marker points;
          points.header.frame_id = "map";
          points.ns = "grid_search";
          points.header.stamp = ros::Time::now();
          points.action = visualization_msgs::Marker::ADD;
          points.pose.orientation.w = 1.0;
          points.id = 0;
          points.type = visualization_msgs::Marker::POINTS;
          points.scale.x = 0.3;
          points.scale.y = 0.3;

          // Points are green
          points.color.b = 1.0f;
          points.color.a = 1.0;

          for(int i=0; i<cluster_poses_tmp.size(); i++)
          {
            geometry_msgs::Point p;
            p.x = cluster_poses_tmp[i].v[0];
            p.y = cluster_poses_tmp[i].v[1];
            p.z = 0.0;
            points.points.push_back(p); 
          }
          marker_pub_.publish(points);
        }
        double L2;
        L2 = LikelihoodFieldModel(fake_readings_clust[0], cluster_poses_tmp);
        if(IF_PRINT)
          cout << "fake reading L2 distance for current searching is :  " << L2 << "  !!!" << endl;
        if (L2 > score_threshold)
        {
          ROS_INFO("Distinctive grids found!!! Check tmp poses for navigation, L2 is %f", L2);
          mtmonitor = new MotionMonitor(cluster_poses_tmp, cluster_poses);
          odom_sub = nh_.subscribe("odom", 10, &MotionMonitor::getCurrentOdom, mtmonitor);
          amcl_sub = nh_.subscribe("amcl_pose", 2, &WakeUp::updateAMCL, this);
          move_base_goal_sub = nh_.subscribe("move_base/status", 2, &WakeUp::checkmovestatus, this);
          return;
        }
        // calc_fake_readings_stats();
        // if (!if_mean_init)
        // {
        //   mean.push_back(mean_cov_fake_readings);
        //   if (mean.size()==6)
        //     if_mean_init = true;
        // }
        // else
        // {
        //   mean.erase(mean.begin());
        //   mean.push_back(mean_cov_fake_readings);
        // }
        // double mean_ = 0;

        // for (auto x:mean) {mean_+=x/6.0;}

        // if (mean_ > 1.5)
        //   {
        //     ROS_INFO("Distinctive grids found!!! Check tmp poses for navigation mean cov is %f", mean_);
        //     return;
        //   }
      }
    }
    search_width_t += grid_size;  
  }
  ROS_INFO("SEARCHING FINISHED, no distinctive girds found!!!");
  return;
}


void WakeUp::freememory()
{
  if( map_ != NULL ) {
    map_free( map_ );
    map_ = NULL;
  }
}

void WakeUp::requestMap()
{
  // get map via RPC
  nav_msgs::GetMap::Request  req;
  nav_msgs::GetMap::Response resp;
  ROS_INFO("Requesting the map from wakeup node ...");
  while(!ros::service::call("static_map", req, resp))
  {
    ROS_WARN("Request for map failed; trying again...");
    ros::Duration d(0.5);
    d.sleep();
  }
  handleMapMessage( resp.map );

  // update occ distance in map
  map_update_cspace(this->map_, max_occ_dist);
  map_converted = true;
  ROS_INFO("map converted");
}

void WakeUp::handleMapMessage(const nav_msgs::OccupancyGrid& msg)
{
    cout << "MAP GOT!!!" << endl;
    map_ = convertMap(msg);
}

void WakeUp::SimuBeamModelConfig()
{
  private_nh_.param("laser_z_hit", z_hit, 0.5);
  private_nh_.param("laser_z_rand", z_rand, 0.5);
  private_nh_.param("laser_sigma_hit", sigma_hit, 0.2);
  private_nh_.param("laser_max_beams", max_beams, 48);
  private_nh_.param("wakeup_score_threshold", score_threshold, 50.0);
  private_nh_.param("laser_max_range", max_range, 2.0);
  private_nh_.param("laser_min_range", min_range, 0.1);
  private_nh_.param("if_debug", DEBUG, false);
  private_nh_.param("if_visualize_laser", IFVISUALIZE_FAKE_LASER, true);
  private_nh_.param("if_visualize_search", IFVISUALIZE_SEARCH, true);
  private_nh_.param("if_print", IF_PRINT, true);
  private_nh_.param("search_width", search_width, 4.0);
  private_nh_.param("wait_time", wait_time, 10.0);
}

double WakeUp::LikelihoodFieldModel(const vector<double> &fake_reading, const vector<pf_vector_t> &poses)
{
  int i,j;
  double z,pz;
  double p;
  double total_score = 0.0;
  pf_vector_t hit, pose;

  visualization_msgs::Marker points;
  points.header.frame_id = "map";
  points.ns = "fake_scan";
  points.header.stamp = ros::Time::now();
  points.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = 1.0;
  points.id = 0;
  points.type = visualization_msgs::Marker::POINTS;
  points.scale.x = 0.03;
  points.scale.y = 0.03;

  // Points are green
  points.color.r = 1.0f;
  points.color.g = 1.0f;
  points.color.a = 1.0;

  vector<vector<double>> clust_scores;
  vector<double> one_cls_score;
  for (j=0; j<poses.size(); j++)
  {
    one_cls_score.clear();

    pose = poses[j];
    p = 1.0;

    double z_hit_denom = 2 * sigma_hit * sigma_hit;
    double z_rand_mult = 1.0/max_range;

    double obs_range;
    double obs_bearing = pose.v[2];

    for (i=0; i<fake_reading.size(); i++)
    {
      obs_bearing += (2.0*M_PI/max_beams) * i;
      obs_range = fake_reading[i];

      if (obs_range >= max_range)
      {
        one_cls_score.push_back(0.0);
        continue;
      }
      
      pz = 0.0;

      hit.v[0] = pose.v[0] + obs_range * cos(obs_bearing);
      hit.v[1] = pose.v[1] + obs_range * sin(obs_bearing);

      int mi, mj;
      mi = MAP_GXWX(map_, hit.v[0]);
      mj = MAP_GYWY(map_, hit.v[1]);

      if(!MAP_VALID(map_, mi, mj))
        z = map_->max_occ_dist;
      else
        z = map_->cells[MAP_INDEX(map_,mi,mj)].occ_dist;

      // score for a single reading
      pz += z_hit * exp(-(z * z) / z_hit_denom);
      // Part 2: random measurements
      pz += z_rand * z_rand_mult;

      assert(pz <= 1.0);
      assert(pz >= 0.0);

      double pzc;
      pzc = pz*pz*pz;
      p += pzc;

      // visualize fake readings
      geometry_msgs::Point pt;
      pt.x = MAP_WXGX(map_,mi);
      pt.y = MAP_WYGY(map_,mj);
      pt.z = 0.0;
      points.points.push_back(pt); 

      one_cls_score.push_back(pzc);

      // cout << i << "th cluster pzc is: " << pzc << endl;

    }
    assert(one_cls_score.size()==max_beams);
    total_score += p;
    clust_scores.push_back(one_cls_score);
    // print total score for each cluster pose
    if (IF_PRINT)
      cout << "total score for cluster" << j << "is :" << p << "reading size is" << fake_reading.size() << endl;
  }

  // calculate L2 distance of total score
  ROS_ASSERT(clust_scores.size()>1);
  double max_L2;
  for (int i=1; i<clust_scores.size(); i++)
  {
    double L2 = 0.0;
    for (int j=0; j<max_beams; j++)
    {
      double truth = clust_scores[0][j];          // which is the score of fake readings
      L2 += pow(truth-clust_scores[i][j], 2);
    }
    L2 = sqrt(L2);
    max_L2 = max(max_L2, L2);
  }

  // double mean = total_score/(double)poses.size();
  // double var=0.0;
  // for (int i=0; i<clust_scores.size(); i++)
  // {
  //   var += 1/(double)clust_scores.size() * pow((clust_scores[i]-mean), 2);
  // }
   
  // vector<double> m_;
  // m_.resize(max_beams, 0.0);
  // double var = 0.0;
  // int sz = clust_scores.size();
  // // each laser
  // for (int i=0; i<max_beams; i++)
  // {
  //   for (int j=0; j<sz; j++) // each clust
  //   {
  //     m_[i] += clust_scores[j][i]/(double)sz;
  //   }

  //   for (int j=0; j<sz; j++)
  //   {
  //     var += pow(clust_scores[j][i]-m_[i], 2.0)/(double)sz;
  //   }
  // }

  if (IFVISUALIZE_FAKE_LASER)
    marker_pub.publish(points);

  return max_L2/(double)max_beams;
}


map_t* WakeUp::convertMap( const nav_msgs::OccupancyGrid& map_msg )
{
  map_t* map = map_alloc();
  ROS_ASSERT(map);

  map->size_x = map_msg.info.width;
  map->size_y = map_msg.info.height;
  map->scale = map_msg.info.resolution;
  cout << "map scale is " << map->scale << endl;
  map->origin_x = map_msg.info.origin.position.x + (map->size_x / 2) * map->scale;
  map->origin_y = map_msg.info.origin.position.y + (map->size_y / 2) * map->scale;
  // Convert to player format
  map->cells = (map_cell_t*)malloc(sizeof(map_cell_t)*map->size_x*map->size_y);
  ROS_ASSERT(map->cells);
  for(int i=0;i<map->size_x * map->size_y;i++)
  {
    if(map_msg.data[i] == 0)
      map->cells[i].occ_state = -1;
    else if(map_msg.data[i] == 100)
      map->cells[i].occ_state = +1;
    else
      map->cells[i].occ_state = 0;
  }

  return map;
}

void sigintHandler(int sig)
{
    ROS_INFO("Shutting Down ...");
    ros::shutdown();
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "wakeup");
    boost::shared_ptr<WakeUp> WakeUp_node(new WakeUp());
    signal(SIGINT, sigintHandler);

    ros::spin();
    return (0);
}