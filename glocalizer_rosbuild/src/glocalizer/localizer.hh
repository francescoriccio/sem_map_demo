#ifndef LOCALIZER_HH
#define LOCALIZER_HH

#include "localizemap.hh"
#include "motionmodel.hh"
#include "laserparameters.hh"
#include "imuparameters.hh"
#include "../stuff/pf.hh"
#include <vector>



/**Class which encloses the parameters of the localizer.*/
struct LocalizerParameters{
  /** minimum weight to set the particles after normalizing the likelihood of a laser range measurement*/
  double minWeight;

  /** minimum weight to set the particles after normalizing the likelihood of an IMU measurement (only the yaw)*/
  double minIMUWeight;

  /** radius and angle in which the particles should be contained
      for claiming convergence [in meters and radiants]
  */
  double convergenceRadius; 
  double convergenceAngle;  

  /** distance the robot has to travel 
      before entering in the converged status [in meters]
  */
  double traveledConvergenceDistance; 
  

  /** distance the robot has to travel for exiting from the 
      converged status [in meters]  */
  double traveledDivergenceDistance;  

  /** enable the filter restart when particles enter in the 
      unknown area */
  bool   dynamicRestart;

  /** distance the robot has to travel before performing an update
      [in meters] */
  double linearUpdate;

  /** angle the robot has to turn before performing an update
      [in radiants] */
  double angularUpdate;

  /** threshold for computing the distance map [in meters] */
  double distanceMapThreshold;

  /** threshold for considering a cell as full*/
  double fullThreshold;

  /** density of the points to consider in computing the likelihood
      how many points per grid cell. */
  double obsPointDensity;
  
  /** standard deviation of the noise in a reading [in meters] */
  double obsSigma;

  /** standard deviation of the noise during static convergence*/
  double staticSigmaX, staticSigmaY, staticSigmaTheta;
 
  /** constructor, initializes the params to default values for SICK-LMS      in indoor environments
  */
  LocalizerParameters();
};


/**Localization particle*/
struct LocalizeParticle{
  double weight;
  DPose2 pose;
  inline operator double() const {return weight;}
  inline LocalizeParticle& operator=(const double& d) {weight=d; return *this;}
};

typedef std::vector<LocalizeParticle> LocalizeParticleVector;
typedef std::vector<DVector2> DVector2Vector;
typedef std::vector<IVector2> IVector2Vector;
typedef SMatrix3<double> CovarianceMatrix;


/**Base localizer class*/
struct Localizer{
  /** parameters of the localizer */
  LocalizerParameters *params;
  /** simple motion modes */
  MotionModel         *motionModel;
  /** map used for localization */
  LocalizeMap         localizeMap;
  /** pose hypotheses */
  LocalizeParticleVector particles;
  
  /** This method initializes the structures of the localizer.
      @param lmap the localization map
      @param nParticles the particles to be used in this instance
      @param recomputeDistanceMap if true the loaded distance map is overridden.
      It constructs the distance map based on the input occupancy map.
      The input map should contain a valid information in the
      "visited" field of every cell. From this "visited" flag the edmissible region
      of the map in which to draw new samples is computed.
      After invoking init, the localizer is in global localization mode,
      and the particles are spreaded around in all 
      unknown regions of the environment.
      It brings the localizer in the *diverged state".
  */
  void init(const LocalizeMap& lmap, int nParticles=1000, bool recomputeDistanceMap=false);

  
  /**Sets the position of the robot in the localized state, 
     and all particles are set in position @param pose +,- a noise
     @param sigmaRadial : the sigma of the radius around pose.x and pose.y where to spread the particles (translation)
     @param sigmaTheta  : the sigma of the orientation of the robot
  */
  void setPose(const DPose2& pose, double sigmaRadial=0.2, double sigmaTheta=M_PI/8);

  /**Sets the localizer in global localization mode, by spreading the particles
     in all free area of the map;
   */
  void startGlobal();

  /**Updates the motion of the robot, according to the odometry movement.
     @param motion: the movement
  */
  void updateMotion(const DPose2& motion);
  
  /**Measurement update.
     @param ranges: the laser readings (in meters)
     @param laserParameters: the parameters of the readings
     @param enableStaticConvergence: 
            if true the localizer updates the robot even when not moving, 
	    while injecting noise into the poses proportional to the sx, sy and sth variables
	    of th parameters.
     @returns true if an update has been carried on
   */

  bool updateObservation(const std::vector<double> ranges,   
			 const LaserParameters& laserParameters, bool enableStaticConvergence=false);


  /**Measurement update for an IMU attitude estimate.
     @param heading: the heading measured by the IMU
     @param imuParameters: the parameters of the IMU
   */
  bool updateIMU(double heading, const IMUParameters& imuParameters);

  /**Tells about the convergence status of the filter
     @param mean: the returned mean of the particles
     @param cov:  the returned covariance of the particles
     @param isBounded:  true if the particles are concentrated in
     a small area within the convergenceRadius and convergenceAngle*/
  bool hasConverged(DPose2& mean, CovarianceMatrix& cov, bool& isBounded) const;

  /**remaps a the cells spanned by a scan according to the given pose
     @param indices: the map cell indices of the scan endpoints
     @param pose: the robot pose in world coordinates
   */
  void remapScan(IVector2Vector& indices, const DPose2& pose);

  /**@returns the pose of the best particle*/
  inline const DPose2& bestPose() const {return _bestPose;}

  /**@returns the index of the best particle*/
  inline int bestIndex()    const {return _bestIndex;}

  /**@returns the observation fitting score*/
  inline double observationFitting() const {return _observationFitting;}

  /**compute the distances of the observations in the fitting scans, in meters.*/

  void currentObservationFitting(DVector2Vector &cartesianRanges,std::vector<double>& distances);

protected:
  /**accumulators for the traveled distance. Based on these values the algorithm
     decises when t perform the update.*/
  double _traveledLinearDistance, 
         _traveledAngularDistance;

  /**vector of the poses of the free cells. When a particle ends in an invalid region,
   it is replaced with a particle located in one fo the free cells.*/
  DVector2Vector freeCells;
  
  /**Computes the admissible region of the map, in which the robot can be.
     This region consists of all the cells which are not occupied and have been visited.
  */
  void computeFreeCells();

  /**samples a particle in one of the free areas of the map*/
  DPose2 sampleFromFreeCells() const;

  /**computes the weith of a particle, based on a reading mapped in the cartesian space:
   @param ranges: the reading;
   @param pose: the particle pose;
   @returns: the log weight
  */

  double computeWeight(const DVector2Vector& ranges, const DPose2& pose);

  /**computes the weith of a particle, based on a set of map indices obtained by translating
     the cartesian ranges in map cells. Performs integer computation and it is 
     faster than the previous method.
     @param indices: the indices;
     @param pose: the particle pose;
     @returns: the log weight
  */
  double computeWeight(const IVector2Vector& indices, const DPose2& pose);

  /**Updates the status of the filter, based on the particle positions.
     Namely it computes mean and variance of the particles.
   */
  void updateStatus();

  /**returns the average distance of the observation in the fitting scans, in meters.*/
  double observationFitting(const IVector2Vector& ranges, const DPose2& pose);


  //internal variables, do not touch
  DPose2 _cumMotion;
  DPose2 _bestPose;
  int _bestIndex;
  DVector2Vector _cartesianRanges;

  DPose2 _mean;
  CovarianceMatrix _covariance;

  double _convergenceCumDistance;
  double _divergenceCumDistance;
  double _observationFitting;

  IVector2Vector _cartesianIndices;
  bool _isLocalized;
  bool _isBounded;
  bool _updateIMU;
};

/** helper function for retrieving the yaw from a quaternion */
double getYaw(double qw, double qx, double qy, double qz);

#endif 
