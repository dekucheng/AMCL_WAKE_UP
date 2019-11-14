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
    ros::NodeHandle private_nh_;
    ros::Subscriber cluster_pose_sub;
    ros::Publisher marker_pub;    
    ros::Publisher marker_pub_;
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
    bool map_converted;
    bool use_map_topic;
    double max_occ_dist;

    bool IFVISUALIZE_FAKE_LASER;
    bool IFVISUALIZE_SEARCH;

    void requestMap();
    void handleMapMessage(const nav_msgs::OccupancyGrid& msg);
    void freememory();
    map_t* convertMap(const nav_msgs::OccupancyGrid& map_msg);
    void clusterPoseReceived(const geometry_msgs::PoseArrayPtr& msg);
    bool laserSimulate();
    // cluster callbacks
    void handleClusterPose(const geometry_msgs::PoseArray& msg);
    void calc_fake_readings_stats();
    void grid_map_search();

    double z_hit, z_rand, sigma_hit;
    void SimuBeamModelConfig();
    double LikelihoodFieldModel(const vector<double> &fake_reading, const vector<pf_vector_t> &poses);
};

WakeUp::WakeUp() :
      private_nh_("~")
{
    use_map_topic = false;
    laser_simu_req = true;
    map_converted = false;

    max_range = 2.0;
    min_range = 0.1;
    grid_size = 0.05;
    search_width = 4.0; // meters
    max_beams = 72;
    max_occ_dist = 2.0;

    IFVISUALIZE_FAKE_LASER = true;
    IFVISUALIZE_SEARCH = true;
    cluster_pose_sub = nh_.subscribe("cluster_poses", 2, &WakeUp::clusterPoseReceived, this);
    marker_pub = nh_.advertise<visualization_msgs::Marker>("visualization_fakelaser_marker", 2);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_search_marker", 2);    
    if (!use_map_topic)
    {
        SimuBeamModelConfig();
        requestMap();
    }
}
WakeUp::~WakeUp()
{
    freememory();
}

void WakeUp::clusterPoseReceived(const geometry_msgs::PoseArrayPtr& msg)
{
  if (laser_simu_req && map_converted)
    handleClusterPose(*msg);
  ros::Duration(2.0).sleep();
}

void WakeUp::handleClusterPose(const geometry_msgs::PoseArray& msg)
{
  cluster_poses.clear();
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
  ROS_INFO("%d cluster poses got!!!", cluster_poses.size());
    // lock_guard<mutex> guard(mutex_);
  if (cluster_poses.size()==1)
    ROS_INFO("amcl pose converges, no need to do grid map searching!");
  else
  {
    // grid_map_search();
    
    laserSimulate();
    double score = LikelihoodFieldModel(fake_readings_clust[0], cluster_poses);
    cout << "the cluster scores varriance is : --- " << score << "---!!" << endl;
    laser_simu_req = true;
  }
}

bool WakeUp::laserSimulate()
{
  laser_simu_req = false;
  // clear fake readings
  fake_readings_clust.clear();
  // for each potential cluster pose

  // check if all tmp poses are valid
  // for (int i=0; i<cluster_poses_tmp.size(); i++)
  // {
  //   double x,y;
  //   x = cluster_poses_tmp[i].v[0];
  //   y = cluster_poses_tmp[i].v[1];
  //   int mi_, mj_;
  //   mi_ = MAP_GXWX(map_, x);
  //   mj_ = MAP_GXWX(map_, y);

  //   if (!MAP_VALID(map_, mi_, mj_))
  //     return false;

  //   else
  //   {
  //     int occ_state;
  //     double occ_dist;
  //     occ_state = map_->cells[MAP_INDEX(map_,mi_,mj_)].occ_state;
  //     occ_dist = map_->cells[MAP_INDEX(map_,mi_,mj_)].occ_dist;
  //     if (occ_state != -1 || occ_dist < 0.15)
  //       {
  //         return false;
  //       }
  //   }
  // }

  for(int i=0; i<1; i++)
  {
    double theta = cluster_poses[i].v[2];
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
      simx = range*cos(theta) + cluster_poses[i].v[0];
      simy = range*sin(theta) + cluster_poses[i].v[1];
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
          simx = range*cos(theta) + cluster_poses[i].v[0];
          simy = range*sin(theta) + cluster_poses[i].v[1];
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
    // ROS_INFO("simu_reading !!! size is %d", simu_readings_clust.size());
  }
  return true;
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
    // bool if_first = true;
    // loop over each grid in one spiral
    vector<double> mean;
    
    bool if_mean_init = false;
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
        calc_fake_readings_stats();
        if (!if_mean_init)
        {
          mean.push_back(mean_cov_fake_readings);
          if (mean.size()==6)
            if_mean_init = true;
        }
        else
        {
          mean.erase(mean.begin());
          mean.push_back(mean_cov_fake_readings);
        }
        double mean_ = 0;

        for (auto x:mean) {mean_+=x/6.0;}

        if (mean_ > 1.5)
          {
            ROS_INFO("Distinctive grids found!!! Check tmp poses for navigation mean cov is %f", mean_);
            return;
          }
      }
    }
    search_width_t += grid_size;  
  }
  ROS_INFO("SEARCHING FINISHED, no distinctive girds found!!!");
}
  


void WakeUp::calc_fake_readings_stats()
{
  int i, j;
  vector<double> mean_fake_readings;
  vector<double> cov_fake_readings;
  mean_fake_readings.resize(max_beams, 0.0);
  // cov_fake_readings.resize(max_beams, 0);
  double cov_accumulate = 0.0;
  for (i=0; i<fake_readings_clust.size(); i++)
  {
    for (j=0; j<max_beams; j++)
    {
      mean_fake_readings[j] += 1.0/fake_readings_clust.size() * fake_readings_clust[i][j];
    }
  }
  for (i=0; i<fake_readings_clust.size(); i++)
  {
    for (j=0; j<max_beams; j++)
    {
      cov_accumulate += 1.0/fake_readings_clust.size() 
                        * pow(fake_readings_clust[i][j]-mean_fake_readings[j], 2);
      // ROS_INFO("fake range for cluster %d is: %f", i, fake_readings_clust[i][j]);
    }
  }
  mean_cov_fake_readings = cov_accumulate; 
  ROS_INFO("the mean cov fake readings is %f ", mean_cov_fake_readings); 

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
  private_nh_.param("laser_z_hit", z_hit, 0.95);
  private_nh_.param("laser_z_rand", z_rand, 0.05);
  private_nh_.param("laser_sigma_hit", sigma_hit, 0.2);
}

double WakeUp::LikelihoodFieldModel(const vector<double> &fake_reading, const vector<pf_vector_t> &poses)
{
  int i,j;
  double z,pz;
  double p;
  double total_score = 0.0;
  pf_vector_t hit, pose;

  vector<double> clust_scores;

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

  for (j=0; j<poses.size(); j++)
  {
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
        continue;
      
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

      p += pz*pz*pz;

      // visualize fake readings
      geometry_msgs::Point p;
      p.x = MAP_WXGX(map_,mi);
      p.y = MAP_WYGY(map_,mj);
      p.z = 0.0;
      points.points.push_back(p); 

    }
    clust_scores.push_back(p);
    total_score += p;
    cout << "p for cluster" << j << "is :" << p << endl;
    
  }

  // calculate varriance 
  double mean = total_score/(double)poses.size();
  double var=0.0;
  for (int i=0; i<clust_scores.size(); i++)
  {
    var += 1/(double)clust_scores.size() * pow((clust_scores[i]-mean), 2);
  }
   
  if (IFVISUALIZE_FAKE_LASER)
    marker_pub.publish(points);

  return var;
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