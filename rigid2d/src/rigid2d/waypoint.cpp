#include"rigid2d/waypoint.hpp"

namespace rigid2d
{


  Waypoint::Waypoint(std::vector<Vector2D> PointSequence)
  {
    trajectoryPoints.addElements(PointSequence);
  }

  Waypoint::Waypoint()
  {

  }



}

