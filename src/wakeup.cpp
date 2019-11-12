#include <algorithm>
#include <vector>
#include <map>
#include <cmath>
#include <memory>
#include <mutex>

// roscpp
#include "ros/ros.h"
#include <signal.h>

#include "amcl/map/map.h"
#include "amcl/pf/pf_vector.h"
#include "portable_utils.hpp"

// Messages that I need
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/SetMap.h"
#include "std_srvs/Empty.h"
#include <visualization_msgs/Marker.h>

// #include "amcl_wakeup/Handleclusterpose.h"

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
    ros::NodeHandle nh_;
    ros::Subscriber cluster_pose_sub;
    ros::Publisher marker_pub;    
    mutex mutex_;

    int range_count;
    int max_beams;
    double max_range;
    double min_range;
    double grid_size;
    double search_width;
    bool laser_simu_req;
    vector<pf_vector_t> cluster_poses;
    vector<pf_vector_t> cluster_poses_tmp;
    vector<vector<double>> fake_readings_clust;
    double mean_cov_fake_readings;

    map_t* map_;
    bool use_map_topic;
    bool IFVISUALIZE;

    void requestMap();
    void handleMapMessage(const nav_msgs::OccupancyGrid& msg);
    void freememory();
    map_t* convertMap(const nav_msgs::OccupancyGrid& map_msg);
    void clusterPoseReceived(const geometry_msgs::PoseArrayPtr& msg);
    void laserSimulate();
    // cluster callbacks
    void handleClusterPose(const geometry_msgs::PoseArray& msg);
    void calc_fake_readings_stats();
    void grid_map_search();
    

};

WakeUp::WakeUp()
{
    use_map_topic = false;
    laser_simu_req = true;
    max_range = 2.0;
    min_range = 0.1;
    grid_size = 0.05;
    search_width = 1.0; // meters
    max_beams = 48;
    IFVISUALIZE = false;
    cluster_pose_sub = nh_.subscribe("cluster_poses", 2, &WakeUp::clusterPoseReceived, this);
    marker_pub = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 2);
    if (!use_map_topic)
    {
        requestMap();
    }
}
WakeUp::~WakeUp()
{
    freememory();
}

void WakeUp::clusterPoseReceived(const geometry_msgs::PoseArrayPtr& msg)
{
  if (laser_simu_req)
    handleClusterPose(*msg);
  ros::Duration(2.0).sleep();
}

void WakeUp::handleClusterPose(const geometry_msgs::PoseArray& msg)
{
  cluster_poses.clear();
  fake_readings_clust.clear();
  for (int i=0; i<msg.poses.size(); i++)
  {
    pf_vector_t p;
    tf2::Quaternion q(
      msg.poses[i].orientation.x,
      msg.poses[i].orientation.y,
      msg.poses[i].orientation.z,
      msg.poses[i].orientation.w);

    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    p.v[0] = msg.poses[i].position.x;
    p.v[1] = msg.poses[i].position.y;
    p.v[2] = yaw;
    cluster_poses.push_back(p);
  }
  
    // lock_guard<mutex> guard(mutex_);
  if (cluster_poses.size()==1)
    ROS_INFO("amcl pose converges, no need to do grid map searching!");
  else
  {
    grid_map_search();
    laser_simu_req = false;
  }
}

void WakeUp::laserSimulate()
{

  // for each potential cluster pose
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
  

  for(int i=0; i<cluster_poses_tmp.size(); i++)
  {
    double theta = cluster_poses_tmp[i].v[2];
    double theta_start, dtheta;

    theta -= M_PI;
    dtheta = (2*M_PI / (double)max_beams);
    vector<double> fake_readings;
    for (int j=0; j<max_beams; j++)
    {
      // simulate each ray casting
      theta += j*dtheta;
      double range = min_range;
      double simx, simy;
      int mi, mj, last_mi, last_mj;
      simx = range*cos(theta) + cluster_poses_tmp[i].v[0];
      simy = range*sin(theta) + cluster_poses_tmp[i].v[1];
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
          simx = range*cos(theta) + cluster_poses_tmp[i].v[0];
          simy = range*sin(theta) + cluster_poses_tmp[i].v[1];
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
              if (occ_state == 1)
                break;
            }
            last_mi = mi;
            last_mj = mj;
          }
          else
            continue;
        } while (range <= max_range-grid_size/2.0);
      }

      geometry_msgs::Point p;
      p.x = MAP_WXGX(map_,mi);
      p.y = MAP_WYGY(map_,mj);
      p.z = 0.0;
      points.points.push_back(p); 
  
      if (range>max_range)
        range = max_range;
      fake_readings.push_back(range);
    }
    fake_readings_clust.push_back(fake_readings);
    // ROS_INFO("simu_reading !!! size is %d", simu_readings_clust.size());
  }
  if (IFVISUALIZE)
    marker_pub.publish(points);
  
  calc_fake_readings_stats();
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
  // loop over each spiral
  int ct=0;
  while (search_width_t <= search_width)
  {
    // clear tmp cluster poses
    cluster_poses_tmp.clear();
    loop_width_count++;
    // bool if_first = true;
    // loop over each grid in one spiral
    for (int c_loop_count=1; c_loop_count<=loop_width_count*8; c_loop_count++)
    {
      ct++;
      cout << "current looping: ct is" << ct << endl;
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
      // set if_first to false
      // if_first = false;

      laserSimulate();
      ROS_INFO("the mean cov fake readings is %f ", mean_cov_fake_readings); 
      if (mean_cov_fake_readings > 100.0)
        return;
    }

    search_width_t += grid_size;  
  }
  ROS_INFO("SEARCHING finished, no distinctive girds found!!!");
}
  


void WakeUp::calc_fake_readings_stats()
{
  int i, j;
  vector<double> mean_fake_readings;
  vector<double> cov_fake_readings;
  mean_fake_readings.resize(max_beams, 0);
  // cov_fake_readings.resize(max_beams, 0);
  double cov_accumulate = 0.0;
  for (i=0; i<fake_readings_clust.size(); i++)
  {
    for (j=0; j<max_beams; j++)
    {
      ROS_ASSERT(fake_readings_clust[i].size() == max_beams);
      mean_fake_readings[j] += 1.0/fake_readings_clust.size() * fake_readings_clust[i][j];
    }
  }
  for (i=0; i<fake_readings_clust.size(); i++)
  {
    for (j=0; j<max_beams; j++)
    {
      cov_accumulate += 1.0/fake_readings_clust.size() 
                        * pow(fake_readings_clust[i][j]-mean_fake_readings[j], 2);
    }
  }
  mean_cov_fake_readings = cov_accumulate;
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
}

void WakeUp::handleMapMessage(const nav_msgs::OccupancyGrid& msg)
{
    cout << "MAP GOT!!!" << endl;
    map_ = convertMap(msg);
}

void sigintHandler(int sig)
{
    ROS_INFO("Shutting Down ...");
    ros::shutdown();
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

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wakeup");
    boost::shared_ptr<WakeUp> WakeUp_node(new WakeUp());
    signal(SIGINT, sigintHandler);

    ros::spin();
    return (0);
}