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
    bool search_req;
    vector<pose_with_weight> cluster_poses;
    vector<sample_vector_t> cluster_poses_tmp;
    vector<double> fake_readings;
    vector<vector<double>> cluster_scores;

    map_t* map_;
    bool map_converted;
    bool use_map_topic;
    double max_occ_dist;

    // some parameters
    bool IFVISUALIZE_FAKE_LASER;
    bool IFVISUALIZE_SEARCH;
    bool IF_PRINT;
    bool IF_USE_L2;
    double wait_time;
    int CONVERGE_ITE_LIMIT;

    int ITE_COUNT;

    void requestMap();
    void handleMapMessage(const nav_msgs::OccupancyGrid& msg);
    void freememory();
    map_t* convertMap(const nav_msgs::OccupancyGrid& map_msg);
    void clusterPoseReceived(const amcl_wakeup::PoseArrayWithWeightConstPtr& msg);
    bool check_tmp_poses() const;
    void laserSimulate(int index);
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
    void LikelihoodFieldModel(const vector<double>& fake_reading, vector<sample_vector_t>& poses);
    double evaluate_scores_L2(const vector<vector<double>>& clst_scores, vector<sample_vector_t>& tmp_poses);
    sample_vector_t evaluate_scores_prob(const vector<vector<double>>& clst_scores, vector<sample_vector_t>& tmp_poses);

    // communicate with move_base
    MotionMonitor* mtmonitor;
    move_base_msgs::MoveBaseActionGoal* current_goal;
    int last_tracking_goal_id;
    int last_clust_num;
    int current_clust_num;
    bool check_clust_num_change;
    int ite_count;
};

WakeUp::WakeUp() :
      private_nh_("~"),
      current_goal(NULL),
      last_tracking_goal_id(-1),
      use_map_topic(false),
      search_req(true),
      map_converted(false),
      grid_size(0.05),
      max_occ_dist(2.0),
      last_clust_num(-1),
      current_clust_num(-1),
      check_clust_num_change(false),
      ite_count(0)
{

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
  if ((search_req || check_clust_num_change) && map_converted) {
    assert(search_req != check_clust_num_change);
    handleClusterPose(*msg);
  }
}

void WakeUp::handleClusterPose(const amcl_wakeup::PoseArrayWithWeight& msg)
{
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

    // convert cluster poses to the desired datatype
    pose_with_weight cl_p;
    cl_p.pose.v[0] = cl_p.pose.v[1] = cl_p.pose.v[2] = 0.0;
    cl_p.id = i;
    cl_p.weight = weight;
    for (int j=0; j<pww->size(); j++)
    {
      cl_p.pose.v[0] += (*pww)[j].weight/weight * (*pww)[j].pose.v[0];
      cl_p.pose.v[1] += (*pww)[j].weight/weight * (*pww)[j].pose.v[1];
      cl_p.pose.v[2] += (*pww)[j].weight/weight * (*pww)[j].pose.v[2];
    }

    cluster_poses.push_back(cl_p);
  }

  sort(cluster_poses.begin(), cluster_poses.end(), [](pose_with_weight a, pose_with_weight b) {return a.weight>b.weight;});
  
  for (int i=0; i<cluster_poses.size(); i++)
  {
    // relabel id
    cluster_poses[i].id = i;
    cout << "the total weight for cluster " << cluster_poses[i].id << " is : " << cluster_poses[i].weight << endl;
  }

  ROS_INFO("%d cluster poses got!!!", cluster_poses.size());
    // lock_guard<mutex> guard(mutex_);

  if (check_clust_num_change) {
    current_clust_num = cluster_poses.size();
    ite_count++;
    // if size = 1, this will hold and res will be printed
    if (current_clust_num != last_clust_num) {
      check_clust_num_change = false;
      search_req = true;
      ROS_INFO("cluster dropped!");
      cout << ite_count << " ites happened before cluster dropped, the ite limit is " << CONVERGE_ITE_LIMIT << endl;
      ite_count = 0;
    }
  }
  if (cluster_poses.size()==1)
  {
    ROS_INFO("amcl pose converges, no need to do grid map searching!");
    search_req = false;
  }
  else
  {
    // keep updating current searching grid poses
    if (DEBUG) {
        ROS_INFO("Starting DEBUG MODE!!!");
        if (check_tmp_poses());
          laserSimulate(0);
          // double L2 = LikelihoodFieldModel(fake_readings[0], cluster_poses);
      }
    else if(search_req) {
      search_req = false;
      last_clust_num = current_clust_num = cluster_poses.size();
      grid_map_search();
    }
    // search_req = true;
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

bool WakeUp::check_tmp_poses() const
{
  for (int i=0; i<cluster_poses_tmp.size(); i++)
  {
    double x,y;
    x = cluster_poses_tmp[i].pose.v[0];
    y = cluster_poses_tmp[i].pose.v[1];
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
  return true;
}

// take in an index?
void WakeUp::laserSimulate(int index)
{
  // clear fake readings
  fake_readings.clear();
  pf_vector_t P;
  if (DEBUG)
    P = cluster_poses[0].pose;
  else  
    P = cluster_poses_tmp[index].pose;


  double theta = P.v[2];
  double dtheta;
  dtheta = (2*M_PI / (double)max_beams);

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
            //  "pose_x is %f pose_y is %f !!!", simx, simy, range*cos(theta), range*sin(theta), cluster_poses[i].pose.v[0], cluster_poses[i].pose.v[1]);

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
    if (IF_USE_L2) {
      search_req = true;
      ROS_INFO("hang off for amcl convergence!");
      ros::Duration d(wait_time);
      d.sleep();
    }
    else {
      check_clust_num_change = true;
    }
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

  // pick one pose from cluster poses (to do: pick the one with highest score)
  double theta_ = cluster_poses[0].pose.v[2];
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
          sample_vector_t tmp_pose;
          theta_turned[i] = theta_loop[i] = cluster_poses[i].pose.v[2] + dtheta_;
          tmp_pose.pose.v[0] = cluster_poses[i].pose.v[0] + search_width_t*cos(theta_turned[i]);
          tmp_pose.pose.v[1] = cluster_poses[i].pose.v[1] + search_width_t*sin(theta_turned[i]); 
          tmp_pose.pose.v[2] = cluster_poses[i].pose.v[2];
          tmp_pose.pr = cluster_poses[i].weight;
          cluster_poses_tmp.push_back(tmp_pose);
        }
        else 
        {
          double t = theta_loop[i];
          ROS_ASSERT(cluster_poses_tmp.size() == cluster_poses.size());

          cluster_poses_tmp[i].pose.v[0] += step*cos(t);
          cluster_poses_tmp[i].pose.v[1] += step*sin(t);
        }
        cluster_poses_tmp[i].id = cluster_poses[i].id;
      }

      // add for loop here to simulate all required readings
      if (check_tmp_poses()) // Simulate Succeed, which means tmp poses are valid
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
            p.x = cluster_poses_tmp[i].pose.v[0];
            p.y = cluster_poses_tmp[i].pose.v[1];
            p.z = 0.0;
            points.points.push_back(p); 
          }
          marker_pub_.publish(points);
        }
        
        // current laserSimulating pose
        int simu_index = 0;
        bool grid_found = false;
        if (IF_USE_L2) {
          laserSimulate(simu_index);
          LikelihoodFieldModel(fake_readings, cluster_poses_tmp); 
          double L2;
          L2 = evaluate_scores_L2(cluster_scores, cluster_poses_tmp);
          if (L2 > score_threshold)
            grid_found = true;
          if(IF_PRINT)
            cout << "fake reading L2 distance for current searching is :  " << L2 << "  !!!" << endl;
        }
        else {
          double lhs_known_prob = 0.0;
          // if total 1000 particles, this means less than 1 partile left in bad cluster
          // after CONVERGE_ITE_LIMIT
          double prob_threshold = 0.001;
          // simulate fake readings of all clusters
          // work space ofr E[E[wi,N|p(i true)]]
          vector<double> total_expect(cluster_poses.size(), 0.0);
          grid_found = true;
          for (simu_index=0; simu_index<cluster_poses.size(); simu_index++) {
            laserSimulate(simu_index);
            LikelihoodFieldModel(fake_readings, cluster_poses_tmp); 
            sample_vector_t min_po_pose = evaluate_scores_prob(cluster_scores, cluster_poses_tmp);
            
            double simul_prob = min_po_pose.po * cluster_poses_tmp[simu_index].pr;
            if (simul_prob >= prob_threshold) {
              grid_found = false;
              break;
            }
            else {
              lhs_known_prob += simul_prob;
              prob_threshold -= lhs_known_prob;
            }
          }
        }

        if (grid_found) {
          mtmonitor = new MotionMonitor(cluster_poses_tmp, cluster_poses);
          odom_sub = nh_.subscribe("odom", 10, &MotionMonitor::getCurrentOdom, mtmonitor);
          amcl_sub = nh_.subscribe("amcl_pose", 5, &WakeUp::updateAMCL, this);
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
  private_nh_.param("wakeup_score_threshold", score_threshold, 0.03);
  private_nh_.param("laser_max_range", max_range, 2.0);
  private_nh_.param("laser_min_range", min_range, 0.1);
  private_nh_.param("if_debug", DEBUG, false);
  private_nh_.param("if_visualize_laser", IFVISUALIZE_FAKE_LASER, true);
  private_nh_.param("if_visualize_search", IFVISUALIZE_SEARCH, true);
  private_nh_.param("if_print", IF_PRINT, true);
  private_nh_.param("search_width", search_width, 4.0);
  private_nh_.param("wait_time", wait_time, 10.0);
  private_nh_.param("if_use_L2", IF_USE_L2, true);
  private_nh_.param("converge_ite_limit", CONVERGE_ITE_LIMIT, 16);
  
}

void WakeUp::LikelihoodFieldModel(const vector<double> &fake_reading, vector<sample_vector_t> &poses) 
{
  int i,j;
  double z,pz;
  double p, total=0.0;
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
  
  cluster_scores.clear();
  vector<double> one_cls_score;
  for (j=0; j<poses.size(); j++)
  {
    one_cls_score.clear();

    pose = poses[j].pose;
    p = 0.0;

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
    }
    assert(one_cls_score.size()==max_beams);
    cluster_scores.push_back(one_cls_score);
    // add score to tmp poses
    poses[j].score = p;
    total += p;
    // print total score for each cluster pose
    if (IF_PRINT)
      cout << "total score for cluster" << j << "is :" << p << "reading size is" << fake_reading.size() << endl;
  }
  // normalize
  for (auto &x:poses) {
    x.score /= total;
    assert(x.score<1.0);
  }

  if (IFVISUALIZE_FAKE_LASER)
    marker_pub.publish(points);

  ROS_ASSERT(cluster_scores.size()>1);
}

double WakeUp::evaluate_scores_L2(const vector<vector<double>>& clst_scores, vector<sample_vector_t>& tmp_poses) 
{
    // calculate L2 distance of total score
  double max_L2;
  for (int i=1; i<clst_scores.size(); i++)
  {
    double L2 = 0.0, score = 0.0;
    for (int j=0; j<max_beams; j++)
    {
      double truth = clst_scores[0][j];          // which is the score of fake readings
      L2 += pow(truth-clst_scores[i][j], 2);
    }
    L2 = sqrt(L2);
    max_L2 = max(max_L2, L2);
  }
    return max_L2/(double)max_beams;
}

sample_vector_t WakeUp::evaluate_scores_prob(const vector<vector<double>>& clst_scores, vector<sample_vector_t>& tmp_poses)
{
  sample_vector_t min_po_sample;
  min_po_sample.po = 1.0;
  // for each iteration, normalize
  assert(CONVERGE_ITE_LIMIT > 0);

  for (int ite=0; ite<CONVERGE_ITE_LIMIT; ite++)
  {
    double norm = 0.0;
    for (int i=0; i<tmp_poses.size(); i++) {
      if (ite==0)
        tmp_poses[i].po = tmp_poses[i].pr;
      else {
        tmp_poses[i].po *= tmp_poses[i].score;
        norm += tmp_poses[i].po;
      }
    }
    if (ite>0) {
      for (int i=0; i<tmp_poses.size(); i++) {
        tmp_poses[i].po /= norm;
        if (tmp_poses[i].po < min_po_sample.po) {
          min_po_sample = tmp_poses[i];
        }
        // cout << "tmp_cluster_poses " << i << "'s pr po score are " << tmp_poses[i].pr << " " << tmp_poses[i].po
        // << "  " << tmp_poses[i].score << endl;
      }
    }
  }
  return min_po_sample;
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