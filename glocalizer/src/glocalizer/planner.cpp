#include "planner.hh"
#include <values.h>
#include <set>
#include <cassert>

using namespace std;

PlannerParameters::PlannerParameters(){
  robotRadius=0.3;
  obstacleGain=1.;
  distanceGain=1.;
}


struct BorderComparator{
  bool operator()(const LocalizeMapCell* a, const LocalizeMapCell* b){
    return (a->cost+a->hcost)<(b->cost+b->hcost);
  }
};

typedef std::set<LocalizeMapCell*, BorderComparator> PlannerBorder;

Planner::Planner(PlannerParameters* params, LocalizeMap* lmap){
  _params=params;
  _lmap=lmap;
  _markedCells.reserve(_lmap->size.x()*lmap->size.y());
  for (int x=0; x<lmap->size.x(); x++)
    for (int y=0; y<lmap->size.y(); y++){
      LocalizeMapCell& c=lmap->cell(IVector2(x,y));
      c.ix=x;
      c.iy=y;
    }
}

Planner::~Planner(){
}

double Planner::heuristic(LocalizeMapCell* current, LocalizeMapCell* goal){
  DVector2 delta=
    _lmap->map2world(IVector2(current->ix, current->iy))-
    _lmap->map2world(IVector2(goal->ix, goal->iy));
  return sqrt(delta*delta);
}


bool Planner::computePath(const DVector2& _start, const DVector2& _goal, LocalizeCellList& path){
  path.clear();
  _markedCells.clear();
  //translate everything into map coordinates
  IVector2 istart=_lmap->world2map(_start);
  IVector2 iend  =_lmap->world2map(_goal);
  if (! _lmap->isInside(istart) || !_lmap->cell(istart).visited){
    return false;
  }
  if (! _lmap->isInside(iend) || !_lmap->cell(iend).visited){
    return false;
  }

  LocalizeMapCell* goal=&_lmap->cell(iend);
  goal->parent=0;
  
  PlannerBorder border;
  border.insert(goal);
  _markedCells.push_back(goal);
  goal->cost=0;
  bool goalReached=false;
  while (! goalReached && ! border.empty()){
    LocalizeMapCell* current=*(border.begin());
    border.erase(border.begin());
    int cx=current->ix;
    int cy=current->iy;
    for (int xx=cx-1; xx<cx+1; xx++)
      for (int yy=cy-1; yy<cy+1; yy++){
	if (!_lmap->isInside(IVector2(xx,yy)))
	    continue;
	if (xx==cx && yy==cy)
	  continue;
	LocalizeMapCell* neighbor=&_lmap->cell(IVector2(xx,yy));
	if (!neighbor->visited)
	  continue;
	if (neighbor->distance<_params->robotRadius)
	  continue;
	double oldNeighborCost=neighbor->cost+neighbor->hcost;
	
	double newNeighborEffectiveCost=current->cost+_params->distanceGain;
	double newNeighborHeuristicCost=heuristic(current,goal);
	double newNeighborCost=newNeighborEffectiveCost+newNeighborHeuristicCost;
	bool inserted=false;
	if (neighbor->cost==MAXDOUBLE){
	  _markedCells.push_back(neighbor);
	  border.insert(neighbor);
	  inserted=true;
	  neighbor->cost=newNeighborEffectiveCost;
	  neighbor->hcost=newNeighborHeuristicCost;
	  neighbor->parent=current;
	} else if (oldNeighborCost > newNeighborCost){
	  inserted=true;
	  neighbor->cost=newNeighborEffectiveCost;
	  neighbor->hcost=newNeighborHeuristicCost;
	  neighbor->parent=current;
	  std::set<LocalizeMapCell*>::iterator ni=border.find(neighbor);
	  if (ni==border.end()){
	    assert(0);
	  }
	  border.erase(ni);
	  border.insert(neighbor);
	}
	
	if (neighbor==goal)
	  goalReached=true;
      } 
  }
  if (goalReached){
    LocalizeMapCell* aux=goal;
    while (aux->parent!=0){
      path.push_front(aux);
      aux=aux->parent;
    }
  }
  for (std::vector<LocalizeMapCell*>::iterator it=_markedCells.begin(); it!=_markedCells.end(); it++){
    LocalizeMapCell* c=*it;
    c->hcost=MAXDOUBLE;
    c->cost=MAXDOUBLE;
  }
  _markedCells.clear();
  if (goalReached)
    return true;
  return false;
}
