#include <iostream>
#include <fstream>
#include <sstream>
#include <cstring>
#include <string>
#include "localizer.hh"
#include <cstdio>
#include <sys/time.h>
#include "../stuff/transformation3.hh"
#include "gridlinetraversal.hh"

using namespace std;

const char *message[]={
  "glocalize_test: runs the localizer in off-line mode, given an map and a clf file",
  "usage glocalize_test [options]  <lmap_file>  <carmen file>",
  "options:",
  " -initPose <x> <y> <theta>",
  0
};


// this dumps the status of the localizer in a pgm image
void dumpStatus(char* filename, Localizer& l, LocalizeMap& background){
  LocalizeMap image(background);
  for (int i=0; i<(int)l.particles.size(); i++){
    IVector2 p=l.localizeMap.world2map(DVector2(l.particles[i].pose.x(), l.particles[i].pose.y()));
    image.cell(p).occupancy=1.;//l.particles[i].weight;
    image.cell(p).visited=1;
  }

  IVector2Vector ib;
  l.remapScan(ib,l.bestPose());
  for (int i=0; i<(int)ib.size(); i++){
    IVector2 p=ib[i];
    if (l.localizeMap.isInside(p)){
      image.cell(p).occupancy=0;
      image.cell(p).visited=1;
    }
  }

  ofstream os(filename);
  image.saveToPGM(os);
  os.close();
}


int main (int argc, const char**argv){

  if (argc<2){
    const char**v=message;
    while (*v){
      cout << *v << endl;
      v++;
    }
    return 0;
  }

  bool doInit=true;
  double off_x=0., off_y=0., off_theta=0.;
  int c = 1;
  if (!strcmp(argv[c],"-initPose")){
    c++;
    off_x=atof(argv[c]);
    c++;
    off_y=atof(argv[c]);
    c++;
    off_theta=atof(argv[c]);
    c++;
  }
  const char* mapfile=argv[c++];
  const char* logfile=argv[c++];
  

  ifstream is (mapfile);
  if (! is){
    cerr << "map file " << mapfile << " not found" << endl;
    return 0;
  }

  cerr << "loading localize map.... ";
  LocalizeMap om;
  is >> om;
  cerr << "Done!" << endl;

  cerr << " Offset=" << om.offset.x() << " " << om.offset.y() << endl;
  cerr << " Size=" <<   om.size.x() << " " << om.size.y() << endl;
  cerr << " Resolution=" << om.resolution << endl;
  cerr << " North=" << om.north() << endl;

  
  // the default parameters are ok
  LocalizerParameters params;
  // this encapsulates a laser configuration
  LaserParameters laserParams(181, -M_PI/2, M_PI/180., 50);
  
  // laser pose wrt robot frame
  laserParams.laserPose =  DTransformation2(0.6,0,0);
  
  // this encapsulates a IMU configuration
  //IMUParameters imuParams;

  // a trivial motion model 
  MotionModel motionModel;
  motionModel.ff=0.05;
  motionModel.fs=0.025;
  motionModel.fr=0.05;
  motionModel.ss=0.025;
  motionModel.sr=0.05;
  motionModel.rr=0.05;

  
  //the localizer....
  Localizer localizer;

  double forcedMaxrange=50;
  int particles=10000;
    
  //BEGIN: alex 66.carmen.log
  motionModel.ff=0.1;
  motionModel.fs=0.025;
  motionModel.fr=0.1;
  motionModel.ss=0.025;
  motionModel.sr=0.05;
  motionModel.rr=0.1;
  params.distanceMapThreshold=3;
  params.dynamicRestart=true;
  params.minWeight=0.001;
  forcedMaxrange=19;
  particles=5000;
  //END: alex
  
  
  localizer.params=&params;
  localizer.motionModel=&motionModel;
  
  //should be initialized with a map and a number of particles
  cerr << "Initializing... ";
  localizer.init(om, particles,true);
  cerr << "Done!" << endl;

  //after initialization it recomputes the distance map,
  //according to the given parameters (you may change the distance threshold


  //these dumps are for seeing the shit we are working on
  ofstream os("localizer_ogrid.pgm");
  localizer.localizeMap.saveToPGM(os);
  os.close();

  os.open("localizer_dgrid.pgm");
  localizer.localizeMap.saveDistanceMapToPGM(os,true);
  os.close();

  os.open("localizer.lmap");
  os << localizer.localizeMap;
  os.close();

  // ofstream ods("distances.txt");
  ofstream opath("path.txt");


  LocalizeMap pathMap = localizer.localizeMap;

  //this is my naive log reading :-).
  //I *do not want* any dependency at this level
  std::vector<double> ranges;
  std::vector<double> distances;
  DPose2 pose;
  ifstream log(logfile);

  bool firstRead=false;
  DPose2 oldPose;

  DPose2 oldLocalizedPose;
  bool oldPoseGood=false;
#define BUFSIZE 20000

  int count=0;
  LocalizeMap image(localizer.localizeMap);
  std::vector<DPose2> path;
  while (log){
    char line[BUFSIZE];
    log.getline(line, BUFSIZE);
    istringstream ss(line);
    string tag;
    ss>> tag;

    int type;
    double angle, fov, res, maxrange, acc, remission_mode;
    bool newlaser=false;

    bool laserFound=false;
    bool imuFound=false;

    struct timeval tv;
    struct timeval ntv;

    if (tag=="ROBOTLASER1"){
      newlaser=true;
      ss >> type >> angle >> fov >> res >> maxrange >> acc >> remission_mode;
      
      laserFound=true;
    } 
    if (tag=="FLASER"){
      laserFound=true;
    }
    if (tag=="IMU"){
      imuFound=true;
    }

    bool updated=false;

    if (laserFound){

      int beams;
      ss >> beams;

      if (newlaser){
	laserParams=LaserParameters(beams, angle, res, maxrange);      
      }
      
      ranges.resize(beams); distances.resize(beams);
      for (int i=0; i<beams; i++){
	ss >> ranges[i];
      }

      if (tag=="ROBOTLASER1"){
	int rems;
	ss >> rems;
	for (int i=0; i<rems; i++){
	  double z;
	  ss >> z;
	}
      }

      double x,y,theta;
      ss >> x >> y >> theta;
      pose=DPose2(x,y,theta);
      //cerr << "Beams=" << beams << " pose=" << x << " " << y << " " << theta<< endl;
      cerr << ".";
      
      

      if (! firstRead){
	oldPose=pose;
        if (doInit){
          localizer.setPose(DPose2(off_x, off_y, off_theta));
		  cout << "Set initial pose: " << off_x << " " << off_y << " " << off_theta << endl;
 		  doInit=false;
        }
	firstRead=true;
	updated=true;
      } else {
	
	//in readings are the readings of the laser
	//in pose the current pose
	//in oldpose the previous pose
	//we compute the motion between oldpose and pose,
	//and we update the filter.
	
	DTransformation2 told(oldPose);
	DTransformation2 tnew(pose);
	DTransformation2 tdelta=told.inv()*tnew;
	DPose2 dpose=tdelta.toPoseType();
	localizer.updateMotion(dpose);
	oldPose=pose;
	gettimeofday(&tv, 0);
	
	updated=localizer.updateObservation(ranges, laserParams);
	gettimeofday(&ntv, 0);
      }
    }
    if (imuFound){
      double ax, ay, az, q0, q1, q2, q3, mx, my, mz, gx, gy, gz;
      is >> ax >> ay >> az >> q0 >> q1 >> q2 >> q3 >> mx >> my >> mz >> gx >> gy >> gz;
      double yaw = getYaw(q0, q1, q2, q3);
      //updated=localizer.updateIMU(yaw, imuParams);
    }
    

    //if an update happens, we dump the image and print some bullshits
    if (updated){
      double dt=(ntv.tv_sec-tv.tv_sec)*1e3+(ntv.tv_usec-tv.tv_usec)*1e-3;
 
      cerr << "!(" << dt << ")";
      DPose2 mean;
      CovarianceMatrix covariance;
      bool isBounded;
      bool isLocalized=localizer.hasConverged(mean, covariance, isBounded);
      cerr << "[" << (isBounded?"B":"N") << (isLocalized?"L":"U") <<"]:f="<< localizer.observationFitting() << endl; 


      bool currentPoseGood = isLocalized && isBounded;
      if (oldPoseGood && currentPoseGood){
	DVector2 pOld(oldLocalizedPose.x(), oldLocalizedPose.y());
	DVector2 pNew(mean.x(), mean.y());
	IVector2 iOld = pathMap.world2map(pOld);
	IVector2 iNew = pathMap.world2map(pNew);
	GridLineTraversalLine line;
	GridLineTraversal::gridLine(iOld, iNew, &line ) ;
	for (int i=0; i<line.num_points; i++){
	  LocalizeMapCell& c=pathMap.cell(line.points[i]);
	  c= LocalizeMapCell(1., 0., true);
	}
      }

      oldLocalizedPose = mean;
      oldPoseGood = currentPoseGood;
      count++;
      /*
      char dumpName[1000];
      sprintf(dumpName, "ldump-%05d.pgm", count);
      dumpStatus(dumpName, localizer, image);
      */
      
      DVector2Vector cartesianRanges;
      if (isLocalized) {
	
	  DPose2 meanLaser;
	  DTransformation2 tmean(mean);
	  DTransformation2 tLP(laserParams.laserPose);
	  DTransformation2 tmL=tmean*tLP;
	  
	  meanLaser = tmL.toPoseType();

	  opath << count << " " << meanLaser.x() << " " <<  meanLaser.y() << " " << meanLaser.theta() << endl;

	  localizer.currentObservationFitting(cartesianRanges,distances);
	  // ods << count << " " ;
	  for (size_t k=0; k<distances.size(); k++) {
	  //   ods << cartesianRanges[k].x() << " " << cartesianRanges[k].y() << " " << distances[k] << "      ";
	   
    
	    char p[32];
	    sprintf(p,"frames/laser%04d.txt",(int)count);
	    ofstream ods2(p);
	    for (size_t k=0; k<distances.size(); k++) 
		ods2 << cartesianRanges[k].x() << " " << cartesianRanges[k].y() << " " << distances[k] << endl;
	    ods2.close();

	  }
	 // ods << endl;
	  
      }
			
    }else {
      cerr <<".";
    }
  }
  
  // ods.close();
  opath.close();
  
  cerr << "saving trajectory in the map" << endl;
  os.open("path.pgm");
  pathMap.saveToPGM(os);
  os.close();

}

