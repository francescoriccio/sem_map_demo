/*
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */


#include <algorithm>
#include <vector>
#include <map>

#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>

#include "ros/assert.h"
// roscpp
#include "ros/ros.h"

// Messages that I need
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/GetMap.h"
#include "std_srvs/Empty.h"

// For transform support
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"

#include "glocalizer/localizer.hh"
#include "glocalizer/LocalizerRanges.h"

#include <iostream>
#include <fstream>

using namespace std;

class GLocalizerNode
{
  public:
    GLocalizerNode();
    ~GLocalizerNode();

    int process();

  private:
    tf::TransformBroadcaster* tfb_;
    tf::TransformListener* tf_;

    bool sent_first_transform_;

    tf::Transform latest_tf_;
    bool latest_tf_valid_;

    // Message callbacks
    bool globalLocalizationCallback(std_srvs::Empty::Request& req,
                                    std_srvs::Empty::Response& res);
    void laserReceived(const sensor_msgs::LaserScanConstPtr& laser_scan);
    void initialPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
    void initialPoseReceivedOld(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
    void mapReceived(const nav_msgs::OccupancyGridConstPtr& msg);

    void handleMapMessage(const nav_msgs::OccupancyGrid& msg);
    void freeMapDependentMemory();
    LocalizeMap* convertMap( const nav_msgs::OccupancyGrid& map_msg );

    double getYaw(tf::Pose& t);

    //parameter for what odom to use
    std::string odom_frame_id_;
    std::string laser_topic_;
    //parameter for what base to use
    std::string base_frame_id_;
    std::string global_frame_id_;

    bool use_map_topic_;
    bool first_map_only_;

    ros::Duration gui_publish_period;
    ros::Time save_pose_last_time;
    ros::Duration save_pose_period;

    geometry_msgs::PoseWithCovarianceStamped last_published_pose;

    
    message_filters::Subscriber<sensor_msgs::LaserScan>* laser_scan_sub_;
    tf::MessageFilter<sensor_msgs::LaserScan>* laser_scan_filter_;
    message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped>* initial_pose_sub_;
    tf::MessageFilter<geometry_msgs::PoseWithCovarianceStamped>* initial_pose_filter_;
    std::vector< LaserParameters* > lasers_;
    std::vector< bool > lasers_update_;
    std::map< std::string, int > frame_to_laser_;

    DPose2 odomPose;

    ros::Duration cloud_pub_interval;
    ros::Time last_cloud_pub_time;

    void requestMap();

    // Helper to get odometric pose from transform system
    bool getOdomPose(tf::Stamped<tf::Pose>& pose,
                     DPose2& pose_,
                     const ros::Time& t, const std::string& f);

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Publisher pose_pub_;
    ros::Publisher particlecloud_pub_;
	ros::Publisher distances_pub_;
    ros::ServiceServer global_loc_srv_;
    ros::Subscriber initial_pose_sub_old_;
    ros::Subscriber map_sub_;

  Localizer* localizer;
  MotionModel* motionModel;
  bool pf_init_;

  int particles;
  double minWeight, minRange, maxRange, distanceMapThreshold;
  bool dynamicRestart;
  
  DPose2 init_pose;


  double init_cov_[3];
  bool first_map_received_;
  boost::recursive_mutex configuration_mutex_;

  glocalizer::LocalizerRanges laserRangesMsg;
  
  bool startGlobal;

};

#define USAGE "USAGE: glocalizer"

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "glocalizer");
  ros::NodeHandle nh;
  GLocalizerNode an;

  ros::spin();

  // To quote Morgan, Hooray!
  return(0);
}

#define __installParam(nodehandle, param, variable, defaultval)	\
  if (! nodehandle.getParam(param, variable)){					\
    std::cerr << "default" << param << " " << (variable=defaultval) << std::endl; \
  } else { \
    std::cerr << "read" << param << " " << variable << std::endl; \
  } \


GLocalizerNode::GLocalizerNode() :
        sent_first_transform_(false),
        latest_tf_valid_(false),
        private_nh_("~"),
        first_map_received_(false)
{
  boost::recursive_mutex::scoped_lock l(configuration_mutex_);
  startGlobal=true;

  // Grab params off the param server
  private_nh_.param("use_map_topic", use_map_topic_, false);
  private_nh_.param("first_map_only", first_map_only_, false);

  double tmp;
  private_nh_.param("gui_publish_rate", tmp, -1.0);
  gui_publish_period = ros::Duration(1.0/tmp);
  private_nh_.param("save_pose_rate", tmp, 0.5);
  save_pose_period = ros::Duration(1.0/tmp);

  cerr << private_nh_.hasParam("particles") << endl;
  cerr << private_nh_.hasParam("initial_pose_x") << endl;
  cerr << private_nh_.hasParam("initial_pose_y") << endl;
  cerr << private_nh_.hasParam("initial_pose_a") << endl;

  __installParam( private_nh_, "particles", particles, 1000);
  private_nh_.param("min_range", minRange, 0.1);
  private_nh_.param("max_range", maxRange, 10.);
  private_nh_.param("dynamic_restart", dynamicRestart, false);
  private_nh_.param("min_weight", minWeight, 0.001);
  private_nh_.param("distance_threshold", distanceMapThreshold, 2.0);
  
  private_nh_.param("laser_topic", laser_topic_, std::string("scan"));
  private_nh_.param("odom_frame_id", odom_frame_id_, std::string("odom"));
  private_nh_.param("base_frame_id", base_frame_id_, std::string("base_link"));
  private_nh_.param("global_frame_id", global_frame_id_, std::string("map"));

  if (private_nh_.getParam("initial_pose_x", init_pose.x())) {
    __installParam( private_nh_, "initial_pose_x", init_pose.x(), 0.0);
    __installParam( private_nh_, "initial_pose_y", init_pose.y(), 0.0);
    __installParam( private_nh_, "initial_pose_a", init_pose.theta(), 0.0);
    startGlobal=false;
  } 
  private_nh_.param("initial_cov_xx", init_cov_[0], 0.25 * 0.25);
  private_nh_.param("initial_cov_yy", init_cov_[1], 0.25 * 0.25);
  private_nh_.param("initial_cov_aa", init_cov_[2], 
                               (M_PI/12.0) * (M_PI/12.0));

  cloud_pub_interval.fromSec(1.0);
  tfb_ = new tf::TransformBroadcaster();
  tf_ = new tf::TransformListener();

  pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 2);
  particlecloud_pub_ = nh_.advertise<geometry_msgs::PoseArray>("particlecloud", 2);
  global_loc_srv_ = nh_.advertiseService("global_localization", 
					 &GLocalizerNode::globalLocalizationCallback,
                                         this);
  laser_scan_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh_, laser_topic_, 100);
  laser_scan_filter_ = 
          new tf::MessageFilter<sensor_msgs::LaserScan>(*laser_scan_sub_, 
                                                        *tf_, 
                                                        odom_frame_id_, 
                                                        100);

  cerr << "subscriptions:" << endl;
  cerr << "\tlaser_topic_: [" << laser_topic_ << "]" << endl;
  cerr << "\odom_frame_id_: [" << odom_frame_id_ << "]" << endl;
  cerr << "\global_frame_id_: [" << global_frame_id_ << "]" << endl;
  cerr << "\tlaser_frame_id_: " << endl;

  laser_scan_filter_->registerCallback(boost::bind(&GLocalizerNode::laserReceived,
                                                   this, _1));
  initial_pose_sub_ = new message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped>(nh_, "initialpose", 2);
  initial_pose_filter_ = new tf::MessageFilter<geometry_msgs::PoseWithCovarianceStamped>(*initial_pose_sub_, *tf_, global_frame_id_, 2);
  initial_pose_filter_->registerCallback(boost::bind(&GLocalizerNode::initialPoseReceived, this, _1));

  initial_pose_sub_old_ = nh_.subscribe("initialpose", 2, &GLocalizerNode::initialPoseReceivedOld, this);

  localizer = new Localizer();
  motionModel = new MotionModel();
  LocalizerParameters* params = new LocalizerParameters();
  params->distanceMapThreshold=distanceMapThreshold;
  params->dynamicRestart=dynamicRestart;
  params->minWeight=minWeight;
  localizer->params = params;
  localizer->motionModel = motionModel;

  motionModel->ff=0.1;
  motionModel->fs=0.025;
  motionModel->fr=0.05;
  motionModel->ss=0.025;
  motionModel->sr=0.05;
  motionModel->rr=0.1;

  cerr << "num particles=" <<  particles << endl;
  if (! startGlobal) {
    cerr << "initial pose x=" <<  init_pose.x() << endl;
    cerr << "initial pose y=" <<  init_pose.y() << endl;
    cerr << "initial pose theta=" <<  init_pose.theta() << endl;
  } else {
    cerr << "global localization" <<  endl;
  }
  pf_init_ = false;
  if(use_map_topic_) {
    map_sub_ = nh_.subscribe("map", 1, &GLocalizerNode::mapReceived, this);
    ROS_INFO("Subscribed to map topic.");
  } else {
    requestMap();
  }
}


void
GLocalizerNode::requestMap()
{
  boost::recursive_mutex::scoped_lock ml(configuration_mutex_);

  // get map via RPC
  nav_msgs::GetMap::Request  req;
  nav_msgs::GetMap::Response resp;
  ROS_INFO("Requesting the map...");
  while(!ros::service::call("static_map", req, resp))
  {
    ROS_WARN("Request for map failed; trying again...");
    ros::Duration d(0.5);
    d.sleep();
  }
  handleMapMessage( resp.map );
}

void
GLocalizerNode::mapReceived(const nav_msgs::OccupancyGridConstPtr& msg)
{
  if( first_map_only_ && first_map_received_ ) {
    return;
  }

  handleMapMessage( *msg );

  first_map_received_ = true;
}

void
GLocalizerNode::handleMapMessage(const nav_msgs::OccupancyGrid& msg)
{
  boost::recursive_mutex::scoped_lock cfl(configuration_mutex_);

  ROS_INFO("Received a %d X %d map @ %.3f m/pix\n",
           msg.info.width,
           msg.info.height,
           msg.info.resolution);


  LocalizeMap* map= convertMap(msg);
  ofstream os("test.pgm");
  map->saveToPGM(os);
  os.close();

  cerr << "initializing localizer with " << particles << " particles" << endl;
  localizer->init(*map, particles, true);

  if (! startGlobal)
    localizer->setPose(init_pose, .5*(init_cov_[0]+init_cov_[1]), init_cov_[2]);
  else 
    localizer->startGlobal();
  pf_init_ = true;
  delete map;
  cerr << "localizer initialized" << endl;
}

LocalizeMap*
GLocalizerNode::convertMap( const nav_msgs::OccupancyGrid& map_msg )
{
  
  IVector2 size(map_msg.info.width, map_msg.info.height);
  double resolution = map_msg.info.resolution;
  DVector2 offset(map_msg.info.origin.position.x,
		  map_msg.info.origin.position.y);
  LocalizeMapCell unknown;
  unknown.occupancy=0;
  unknown.distance=0;
  unknown.visited=false;
  LocalizeMap* lmap = new LocalizeMap(size, resolution, offset, unknown);
  int i=0;
  for(int y=0; y<size.y(); y++) {
    for(int x=0; x<size.x(); x++){
      
      IVector2 indices(x,y);
      LocalizeMapCell& c=lmap->cell(indices);
      c.visited=true;
      if(map_msg.data[i] == 0)
	c.occupancy=0.;
      else if(map_msg.data[i] == 100)
	c.occupancy=1;
      else
	c=unknown;
      i++;
    }
  }
  return lmap;
}


GLocalizerNode::~GLocalizerNode()
{
  delete laser_scan_filter_;
  delete laser_scan_sub_;
  delete initial_pose_filter_;
  delete initial_pose_sub_;
  delete tfb_;
  delete tf_;
  delete localizer;
  delete motionModel;
  // TODO: delete everything allocated in constructor
}

bool
GLocalizerNode::getOdomPose(tf::Stamped<tf::Pose>& odom_pose,
                      DPose2& p,
                      const ros::Time& t, const std::string& base_frame_id)
{
  // Get the robot's pose
  tf::Stamped<tf::Pose> ident (tf::Transform(tf::createQuaternionFromRPY(0,0,0),
                                           tf::Vector3(0,0,0)), t, base_frame_id);
  //tf::Stamped<tf::Transform> odom_pose;
  try
  {
    tf_->transformPose(odom_frame_id_, ident, odom_pose);
  }
  catch(tf::TransformException e)
  {
    ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
    return false;
  }
  double yaw = tf::getYaw(odom_pose.getRotation());
  
  p.x()=odom_pose.getOrigin().x();
  p.y()=odom_pose.getOrigin().y();
  p.theta()=yaw;
  
  return true;
}

bool
GLocalizerNode::globalLocalizationCallback(std_srvs::Empty::Request& req,
                                     std_srvs::Empty::Response& res)
{
  if( ! pf_init_ ) {
    return false;
  }
  boost::recursive_mutex::scoped_lock gl(configuration_mutex_);
  ROS_INFO("Initializing with uniform distribution");
  localizer->startGlobal();
  return true;
}

void
GLocalizerNode::laserReceived(const sensor_msgs::LaserScanConstPtr& laser_scan)
{

  if( ! pf_init_ ) {
    return;
  }
  boost::recursive_mutex::scoped_lock lr(configuration_mutex_);
  int laser_index = -1;

  // Do we have the base->base_laser Tx yet?
  if(frame_to_laser_.find(laser_scan->header.frame_id) == frame_to_laser_.end())
  {
    ROS_INFO("Setting up laser %d (frame_id=%s)\n", (int)frame_to_laser_.size(), laser_scan->header.frame_id.c_str());
    lasers_.push_back(0);
    lasers_update_.push_back(true);
    laser_index = frame_to_laser_.size();

    // ERRORE QUI !!!
    //tf::Stamped<tf::Pose> ident (btTransform::getIdentity(),
    //                             ros::Time(), laser_scan->header.frame_id);
    //tf::Stamped<tf::Pose> laser_pose;
    
    tf::Stamped<tf::Pose> ident;
    tf::Stamped<tf::Transform> laser_pose;
    ident.setIdentity();
    ident.frame_id_ = laser_scan->header.frame_id;
    ident.stamp_ = laser_scan->header.stamp;
  
    try
    {
      this->tf_->transformPose(base_frame_id_, ident, laser_pose);
    }
    catch(tf::TransformException& e)
    {
      ROS_ERROR("Couldn't transform from %s to %s, "
                "even though the message notifier is in use",
                laser_scan->header.frame_id.c_str(),
                base_frame_id_.c_str());
      return;
    }

    laserRangesMsg.header = laser_scan->header;
    double lx = laser_pose.getOrigin().x();
    double ly = laser_pose.getOrigin().y();

    double yaw, pitch, roll;
    laser_pose.getBasis().getEulerYPR(yaw, pitch, roll);
    double lth = yaw;
    // laser mounting angle gets computed later -> set to 0 here!
    double a_min=laser_scan->angle_min;
    double a_delta=laser_scan->angle_increment;
    int beams=laser_scan->ranges.size();
    double maxrange=laser_scan->range_max;
    lasers_[laser_index]=new LaserParameters(beams, a_min, a_delta, maxrange);
    lasers_[laser_index]->minRange=laser_scan->range_min;
    lasers_[laser_index]->laserPose=DTransformation2(lx,ly,lth);
    ROS_DEBUG("Received laser's pose wrt robot: %.3f %.3f %.3f",
              lx,
              ly,
              lth);

    frame_to_laser_[laser_scan->header.frame_id] = laser_index;
  } else {
    // we have the laser pose, retrieve laser index
    laser_index = frame_to_laser_[laser_scan->header.frame_id];
  }

  // Where was the robot when this scan was taken?
  tf::Stamped<tf::Pose> odom_pose;
  DPose2 pose;
  if(!getOdomPose(odom_pose, pose,
                  laser_scan->header.stamp, base_frame_id_))
  {
    ROS_ERROR("Couldn't determine robot's pose associated with laser scan");
    return;
  }


  DTransformation2 deltaT;
  DTransformation2 tNew(pose);
  
  deltaT=DTransformation2(odomPose).inv()*tNew;
  lasers_update_[laser_index] = true;

  bool force_publication = true;
  
  odomPose = pose;

  // Filter is now initialized
  pf_init_ = true;

  // Should update sensor data
  for(unsigned int i=0; i < lasers_update_.size(); i++)
    lasers_update_[i] = true;

  // If the robot has moved, update the filter
  localizer->updateMotion(deltaT.toPoseType());
  
  bool updated =false;

  LaserParameters* lparams=lasers_[laser_index];
  if (lparams->beams.size()!=laser_scan->ranges.size()){
    ROS_DEBUG("laser parameters changed\n");
  }

  std::vector<double> dranges(laser_scan->ranges.size());
  for (size_t i=0; i<dranges.size(); i++) {
    dranges[i] = laser_scan->ranges[i];
    if (dranges[i] < this->minRange || dranges[i]>this->maxRange){
      dranges[i] = lparams->maxRange;
    }
  }
  updated=localizer->updateObservation(dranges, *lparams, false);
  

  
  // Publish the resulting cloud
  // TODO: set maximum rate for publishing
  if (updated) {
    geometry_msgs::PoseArray cloud_msg;
    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.header.frame_id = global_frame_id_;
    cloud_msg.poses.resize(localizer->particles.size());
    for(size_t i=0;i<localizer->particles.size();i++)
      {
	tf::Transform particlePose(tf::Matrix3x3(tf::createQuaternionFromYaw(localizer->particles[i].pose.theta())),
				  tf::Vector3(localizer->particles[i].pose.x(),
					    localizer->particles[i].pose.y(), 0));

						
	tf::poseTFToMsg(particlePose, cloud_msg.poses[i]);
	
      }
    particlecloud_pub_.publish(cloud_msg);
  }
    

  DPose2 mean;
  CovarianceMatrix _covariance;
  bool isBounded;
  bool isLocalized=localizer->hasConverged(mean, _covariance, isBounded);
  
  if(isLocalized) {

    geometry_msgs::PoseWithCovarianceStamped p;
    // Fill in the header
    p.header.frame_id = global_frame_id_;
    p.header.stamp = laser_scan->header.stamp;
    // Copy in the pose
    p.pose.pose.position.x = mean.x();
    p.pose.pose.position.y = mean.y();
    tf::quaternionTFToMsg(tf::createQuaternionFromYaw(mean.theta()),p.pose.pose.orientation);
    // Copy in the covariance, converting from 3-D to 6-D
    for(int i=0; i<2; i++)
      {
	for(int j=0; j<2; j++)
	  {
	    // Report the overall filter covariance, rather than the
	    // covariance for the highest-weight cluster
	    //p.covariance[6*i+j] = hyps[max_weight_hyp].pf_pose_cov.m[i][j];
	    p.pose.covariance[6*i+j] = _covariance.values[i][j];
	  }
      }
      // Report the overall filter covariance, rather than the
      // covariance for the highest-weight cluster
      //p.covariance[6*5+5] = hyps[max_weight_hyp].pf_pose_cov.m[2][2];
    p.pose.covariance[6*5+5] = _covariance.values[2][2];

    /*
      printf("cov:\n");
      for(int i=0; i<6; i++)
      {
      for(int j=0; j<6; j++)
      printf("%6.3f ", p.covariance[6*i+j]);
      puts("");
      }
    */

    pose_pub_.publish(p);
    last_published_pose = p;

    /**compute the distances of the observations in the fitting scans, in meters.*/
    DVector2Vector cartesianRanges;
    std::vector<double> distances;
    localizer->currentObservationFitting(cartesianRanges, distances);
    
    assert(cartesianRanges.size()==distances.size());
    laserRangesMsg.x.resize(cartesianRanges.size());
    laserRangesMsg.y.resize(cartesianRanges.size());
    laserRangesMsg.dist.resize(cartesianRanges.size());
    for(size_t i=0; i<cartesianRanges.size();i++){
      laserRangesMsg.x[i]=cartesianRanges[i].x();
      laserRangesMsg.y[i]=cartesianRanges[i].y();
      laserRangesMsg.dist[i]=distances[i];
    }
    distances_pub_.publish(laserRangesMsg);
	
	
    DPose2 deltaMapOdom=(DTransformation2(mean)*DTransformation2(odomPose).inv()).toPoseType();
    tf::Transform tmp_tf(tf::createQuaternionFromYaw(deltaMapOdom.theta()),
                             tf::Vector3(deltaMapOdom.x(),
                                         deltaMapOdom.y(),
					 0.0));

    tf::StampedTransform map_to_odom(tmp_tf,
				     laser_scan->header.stamp,
				     global_frame_id_, odom_frame_id_);
    //ROS_INFO("Sending bloody transform from %s to %s ",global_frame_id_.c_str(),odom_frame_id_.c_str());
    
      this->tfb_->sendTransform(map_to_odom);
      sent_first_transform_ = true;
  }
  else
    {
      //ROS_ERROR("Not localized!");
    }
}

double
GLocalizerNode::getYaw(tf::Pose& t)
{
  double yaw, pitch, roll;
  tf::Matrix3x3 mat = t.getBasis();
  mat.getEulerYPR(yaw,pitch,roll);
  return yaw;
}

void
GLocalizerNode::initialPoseReceivedOld(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
  // Support old behavior, where null frame ids were accepted.
  if(msg->header.frame_id == "")
  {
    ROS_WARN("Received initialpose message with header.frame_id == "".  This behavior is deprecated; you should always set the frame_id");
    initialPoseReceived(msg);
  }
}

void
GLocalizerNode::initialPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
  boost::recursive_mutex::scoped_lock prl(configuration_mutex_);
  // In case the client sent us a pose estimate in the past, integrate the
  // intervening odometric change.
  tf::StampedTransform tx_odom;
  try
  {
    tf_->lookupTransform(base_frame_id_, ros::Time::now(),
                         base_frame_id_, msg->header.stamp,
                         global_frame_id_, tx_odom);
  }
  catch(tf::TransformException e)
  {
    // If we've never sent a transform, then this is normal, because the
    // global_frame_id_ frame doesn't exist.  We only care about in-time
    // transformation for on-the-move pose-setting, so ignoring this
    // startup condition doesn't really cost us anything.
    if(sent_first_transform_)
      ROS_WARN("Failed to transform initial pose in time (%s)", e.what());
    tx_odom.setIdentity();
  }

  tf::Pose pose_old, pose_new;
  tf::poseMsgToTF(msg->pose.pose, pose_old);
  pose_new = tx_odom.inverse() * pose_old;

  ROS_INFO("Setting pose (%.6f): %.3f %.3f %.3f",
           ros::Time::now().toSec(),
           pose_new.getOrigin().x(),
           pose_new.getOrigin().y(),
           getYaw(pose_new));
  // Re-initialize the filter
  DPose2 newPose(pose_new.getOrigin().x(),
		 pose_new.getOrigin().y(),
		 getYaw(pose_new));
  localizer->setPose(newPose);
}

