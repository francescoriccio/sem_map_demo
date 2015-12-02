#include <values.h>
#include "localizer.hh"
#include <cstdlib>
#include <limits>
using namespace std;

#include "../stuff/transformation3.hh"
#include "../stuff/stat.hh"

double getYaw(double qw, double qx, double qy, double qz){
  typedef Vector3<double> DVector3;
  typedef Quaternion<double> DQuaternion;

  DQuaternion q(qw, qx, qy, qz);
  DVector3 angles=q.toAngles();
  return angles.yaw();
} 


static const int trigTable_size=1024;
static int sinTable[trigTable_size+1], cosTable[trigTable_size+1]; 
static void trigTable_init(){
  double alpha=(2*M_PI)/trigTable_size;
  for (int i=0; i<trigTable_size+1; i++){
    sinTable[i]=(int)(1024.*sin(alpha*i));
    cosTable[i]=(int)(1024.*cos(alpha*i));
  }
}

inline void remap (int& x, int& y, int xoff, int yoff, int ic, int is){
  int px=x, py=y;
  x=xoff+((ic*px-is*py)>>10);
  y=yoff+((is*px+ic*py)>>10);
}


LocalizerParameters::LocalizerParameters(){
  minWeight=0.01;
  minIMUWeight=0.5;
  convergenceRadius=0.5;
  convergenceAngle=0.3;
  traveledConvergenceDistance=5.;
  traveledDivergenceDistance=3.;
  dynamicRestart=false;
  linearUpdate=0.1;  // 0.5
  angularUpdate=0.1;  // 0.5
  obsSigma=0.1;
  obsPointDensity=1;
  distanceMapThreshold=3.0;
  fullThreshold=0.51;
  staticSigmaX=0.1;
  staticSigmaY=0.1;
  staticSigmaTheta=0.1;
}

void Localizer::init(const LocalizeMap& pmap, int nParticles, bool recomputeDistanceMap){
  trigTable_init();
  localizeMap=pmap;
  if (recomputeDistanceMap)
    localizeMap.distanceMap(params->distanceMapThreshold, params->fullThreshold);
  particles.resize(nParticles);
  computeFreeCells();
  cerr << "FreeCells=" << freeCells.size() << endl;
  startGlobal();
  _cumMotion=DPose2(0.,0.,0.);
  _updateIMU=false;
}

void Localizer::setPose(const DPose2& pose, double sigmaRadial, double sigmaTheta){
  for (int i=0; i<(int)particles.size(); i++){
    particles[i].weight=0;
    // sample a radius
    double radius=triangularSample(sigmaRadial,0);
    double theta1=(drand48()-0.5)*2*M_PI;
    double x=radius*cos(theta1);
    double y=radius*sin(theta1);
    double theta=triangularSample(sigmaTheta, 0);
    particles[i].pose=DPose2( pose.x() + x, pose.y() + y, pose.theta() + theta);
  }
  _isLocalized=true;
  _convergenceCumDistance=0;
  _divergenceCumDistance=0;
  _traveledLinearDistance = std::numeric_limits<double>::max();
  _traveledAngularDistance = std::numeric_limits<double>::max();
}

void Localizer::startGlobal(){
  for (int i=0; i<(int)particles.size(); i++){
    particles[i].weight=0;
    particles[i].pose=sampleFromFreeCells();
  }
  _isLocalized=false;
  _convergenceCumDistance=0;
  _divergenceCumDistance=0;
}

void Localizer::updateMotion(const DPose2& motion){
  DTransformation2 cm(_cumMotion);
  DTransformation2 dm(motion);
  cm=cm*dm;
  _cumMotion=cm.toPoseType();;

  _traveledLinearDistance+=sqrt(motion.x()*motion.x()+motion.y()*motion.y());
  _traveledAngularDistance+=fabs(motion.theta());
}


bool Localizer::updateObservation(const std::vector<double> ranges, const LaserParameters& laserParams, bool enableStaticConvergence){

  // construct a reading in cartesian coordinates centered in the *robot* frame;
  DVector2Vector cartesianRanges (ranges.size());
  _cartesianIndices.resize(ranges.size());
  DVector2 lastValidPoint(MAXDOUBLE, MAXDOUBLE);
  double res2=localizeMap.resolution;
  res2*=res2*params->obsPointDensity;
  int count=0;
  for (int i=0; i<(int)ranges.size(); i++){
    DVector2 v;
    if (ranges[i]<laserParams.maxRange){
     
      v=laserParams.laserPose * laserParams.beams[i] * DVector2(ranges[i], 0);
      DVector2 delta = v-lastValidPoint;
      if (delta*delta>res2){
	lastValidPoint=v;
	cartesianRanges[count]=v;
	IVector2 iv((int)(v.x()/localizeMap.resolution),(int)(v.y()/localizeMap.resolution));
	_cartesianIndices[count]=iv;
	count++;
      } 
    } 
  }

  cartesianRanges.resize(count);
  _cartesianRanges=cartesianRanges;
  _cartesianIndices.resize(count);

  _observationFitting=observationFitting(_cartesianIndices, _mean);

  // if static convergence is enabled,  set the motion model to inject static noise 
  if (enableStaticConvergence){
    motionModel->sx=params->staticSigmaX;
    motionModel->sy=params->staticSigmaY;
    motionModel->sth=params->staticSigmaTheta;
  } else {
    motionModel->sx=0;
    motionModel->sy=0;
    motionModel->sth=0;
  }

  if (_traveledLinearDistance<params->linearUpdate && 
      _traveledAngularDistance<params->angularUpdate && ! enableStaticConvergence)
    return false;

  // evolve the particles according to the accumulated motion commands
  motionModel->prepareSampling(_cumMotion);
  for (int i=0; i<(int)particles.size(); i++){
    DPose2 np=motionModel->sampleMotion(_cumMotion);
    DTransformation2 t(particles[i].pose);
    DTransformation2 nt(np);
    t=t*nt;
    particles[i].pose=t.toPoseType();

    DVector2 p(particles[i].pose.x(), particles[i].pose.y());
    IVector2 ip=localizeMap.world2map(p);
    if (! localizeMap.isInside(ip)){
      particles[i].pose=sampleFromFreeCells();
    } else if (params->dynamicRestart && !localizeMap.cell(ip).visited){
      particles[i].pose=sampleFromFreeCells();
    } 
  }


  _cumMotion=DPose2(0.,0.,0.);


  double maxWeight=-MAXDOUBLE;
  double minWeight=MAXDOUBLE;
  _bestIndex=-1;
  int worstIndex=-1;

  for (int i=0; i<(int)particles.size(); i++){
    particles[i].weight=computeWeight(_cartesianIndices, particles[i].pose);
    if (particles[i].weight>maxWeight){
      maxWeight=particles[i].weight;
      _bestIndex=i;
    }
    if (particles[i].weight<minWeight){
      minWeight=particles[i].weight;
      worstIndex=i;
    }
  }

  _bestPose=particles[_bestIndex].pose;

  normalizeWeights(particles, particles.size(), params->minWeight);
  std::vector<int> indices;
  resample(indices, particles);
  std::vector<LocalizeParticle> resampled=particles;
  repeatIndexes(resampled, indices, particles);
  particles=resampled;

  updateStatus();

  _traveledLinearDistance=0;
  _traveledAngularDistance=0;

  _updateIMU=true;
  return true;
}

bool Localizer::updateIMU(double heading, const IMUParameters& imuParameters){
  if (!_updateIMU)
    return false;

  double maxWeight=-MAXDOUBLE;
  double minWeight=MAXDOUBLE;
  _bestIndex=-1;
  int worstIndex=-1;

  for (int i=0; i<(int)particles.size(); i++){
    double delta=localizeMap.north()-heading;
    delta=atan2(sin(delta),cos(delta));
    particles[i].weight=-delta*delta/imuParameters.yawCov;
    if (particles[i].weight>maxWeight){
      maxWeight=particles[i].weight;
      _bestIndex=i;
    }
    if (particles[i].weight<minWeight){
      minWeight=particles[i].weight;
      worstIndex=i;
    }
  }

  _bestPose=particles[_bestIndex].pose;

  normalizeWeights(particles, particles.size(), params->minIMUWeight);
  std::vector<int> indices;
  resample(indices, particles);
  std::vector<LocalizeParticle> resampled=particles;
  repeatIndexes(resampled, indices, particles);
  particles=resampled;

  updateStatus();
  _updateIMU=false;
  return true;
}

bool Localizer::hasConverged(DPose2& mean, CovarianceMatrix& cov, bool& isBounded) const{
  DTransformation2 tmean(_mean);
  DTransformation2 tmotion(_cumMotion);
  tmean=tmean*tmotion;
  mean=tmean.toPoseType();

  cov=_covariance;
  isBounded=_isBounded;
  return _isLocalized;
}

void Localizer::updateStatus(){
  // compute the mean
  double mc=0, ms=0, mx=0, my=0;
  for (int i=0; i<(int)particles.size(); i++){
    DPose2 p=particles[i].pose;
    mx+=p.x();
    my+=p.y();
    mc+=cos(p.theta());
    ms+=sin(p.theta());
  }
  _mean=DPose2(mx/particles.size(), my/particles.size(), atan2(ms, mc));

  // compute the covariance
  double d=0;
  _covariance=_covariance*d;

  for (int i=0; i<(int)particles.size(); i++){
    DPose2 p=particles[i].pose;
    double dx=p.x()-_mean.x();
    double dy=p.y()-_mean.y();
    double dth=p.theta()-_mean.theta();
    dth=atan2(sin(dth), cos(dth));

    _covariance.values[0][0]+=dx*dx;
    _covariance.values[0][1]+=dx*dy;
    _covariance.values[1][0]+=dx*dy;
    _covariance.values[1][1]+=dy*dy;
    _covariance.values[0][2]+=dx*dth;
    _covariance.values[2][0]+=dx*dth;
    _covariance.values[1][2]+=dy*dth;
    _covariance.values[2][1]+=dy*dth;
    _covariance.values[2][2]+=dth*dth;
  }
  d=1./particles.size();
  _covariance=_covariance*d;


  // determine the area spanned by the metric part of the covariance
  double ma=sqrt(_covariance.values[0][0]*_covariance.values[1][1]-_covariance.values[1][0]*_covariance.values[0][1]);
  double da=sqrt(_covariance.values[2][2]);

  _isBounded=ma<params->convergenceRadius && da<params->convergenceAngle;

  // perform the state transition according to the particle status
  if (_isLocalized){
    if (!_isBounded){
      _divergenceCumDistance+=_traveledLinearDistance;
      if (_divergenceCumDistance>params->traveledConvergenceDistance){
	_divergenceCumDistance=0;
	_convergenceCumDistance=0;
	_isLocalized=false;
      }
    } else {
      _divergenceCumDistance=0;
    } 
  } else { // ! localized
    if (_isBounded){
      _convergenceCumDistance+=_traveledLinearDistance;
      if (_convergenceCumDistance>params->traveledDivergenceDistance){
	_divergenceCumDistance=0;
	_convergenceCumDistance=0;
	_isLocalized=true;
      }
    } else {
      _convergenceCumDistance=0;
    } 
  }

  // compute the mean
}

void Localizer::computeFreeCells(){
  freeCells.clear();
  for (int x=0; x<localizeMap.size.x(); x++){
    for (int y=0; y<localizeMap.size.y(); y++){
      if (localizeMap.cell(IVector2(x,y)).visited){
	DVector2 v=localizeMap.map2world(IVector2(x,y));
	freeCells.push_back(v);
      }
    }
  }
}

DPose2 Localizer::sampleFromFreeCells() const{
  int i=(int) (drand48()*freeCells.size());
  DVector2 t=freeCells[i];
  double theta=(drand48()-0.5)*2*M_PI;
  return DPose2(t.x(), t.y(), theta);
}


double Localizer::computeWeight(const DVector2Vector& ranges, const DPose2& pose){
  double f=0;
  DTransformation2 p(pose);
  int count=0;
  for (int i=0; i<(int)ranges.size(); i++){
    DVector2 b=ranges[i];
    DVector2 tb=p*b;
    count++;
    float d=params->distanceMapThreshold;
    IVector2 itb=localizeMap.world2map(tb);
    if (localizeMap.isInside(itb)){
      d= localizeMap.cell(itb).distance;
    }
    if (d>params->distanceMapThreshold)
      d=params->distanceMapThreshold;
    f+=d;
  }
  f/=(count*params->obsSigma);
  return -f;
}


double Localizer::computeWeight(const IVector2Vector& ranges, const DPose2& pose){
  DTransformation2 p(pose);
  IVector2 offset=localizeMap.world2map(DVector2(pose.x(), pose.y()));
  double t=pose.theta();
  t=atan2(sin(t), cos(t));
  if (t<0){
    t+=2*M_PI;
  }
  int ic=cosTable[(int) (t*(trigTable_size/(2*M_PI)))];
  int is=sinTable[(int) (t*(trigTable_size/(2*M_PI)))];
  int count=0;
  double f=0;
  for (int i=0; i<(int)ranges.size(); i++){
    int x=ranges[i].x();
    int y=ranges[i].y();
    remap(x,y,offset.x(), offset.y(), ic, is);
    IVector2 itb(x,y);
    double d=params->distanceMapThreshold;
    if (localizeMap.isInside(itb)){
      d= localizeMap.cell(itb).distance;
    }
    f-=d;
    count++;
  }
  f/=(count*params->obsSigma);
  return f;
}

double Localizer::observationFitting(const IVector2Vector& ranges, const DPose2& pose){
  DTransformation2 p(pose);
  IVector2 offset=localizeMap.world2map(DVector2(pose.x(), pose.y()));
  double t=pose.theta();
  t=atan2(sin(t), cos(t));
  if (t<0){
    t+=2*M_PI;
  }
  int ic=cosTable[(int) (t*(trigTable_size/(2*M_PI)))];
  int is=sinTable[(int) (t*(trigTable_size/(2*M_PI)))];
  int count=0;
  double f=0;
  for (int i=0; i<(int)ranges.size(); i++){
    int x=ranges[i].x();
    int y=ranges[i].y();
    remap(x,y,offset.x(), offset.y(), ic, is);
    IVector2 itb(x,y);
    double d=params->distanceMapThreshold;
    if (localizeMap.isInside(itb)){
      d= localizeMap.cell(itb).distance;
    }
    f+=d;
    count++;
  }
  f/=count;
  return f;
}

void Localizer::currentObservationFitting(DVector2Vector &cartesianRanges, std::vector<double>& distances) {
  
  DTransformation2 p(_mean);
  IVector2 offset=localizeMap.world2map(DVector2(_mean.x(), _mean.y()));
  double t=_mean.theta();
  t=atan2(sin(t), cos(t));
  if (t<0){
    t+=2*M_PI;
  }
  int ic=cosTable[(int) (t*(trigTable_size/(2*M_PI)))];
  int is=sinTable[(int) (t*(trigTable_size/(2*M_PI)))];
  distances.resize(_cartesianIndices.size());
  for (int i=0; i<(int)_cartesianIndices.size(); i++){
    int x=_cartesianIndices[i].x();
    int y=_cartesianIndices[i].y();
    remap(x,y,offset.x(), offset.y(), ic, is);
    IVector2 itb(x,y);
    double d=params->distanceMapThreshold;
    if (localizeMap.isInside(itb)){
      d = localizeMap.cell(itb).distance;
    }
    distances[i] = d;
  }
  cartesianRanges = _cartesianRanges;
}


void Localizer::remapScan(IVector2Vector& ranges, const DPose2& pose){
  ranges=_cartesianIndices;
  DTransformation2 p(pose);
  IVector2 offset=localizeMap.world2map(DVector2(pose.x(), pose.y()));
  double t=pose.theta();
  t=atan2(sin(t), cos(t));
  if (t<0){
    t+=2*M_PI;
  }
  int ic=cosTable[(int) (t*(trigTable_size/(2*M_PI)))];
  int is=sinTable[(int) (t*(trigTable_size/(2*M_PI)))];
  for (int i=0; i<(int)ranges.size(); i++){
    int x=ranges[i].x();
    int y=ranges[i].y();
    remap(x,y,offset.x(), offset.y(), ic, is);
    ranges[i].x()=x;
    ranges[i].y()=y;
  }
}
