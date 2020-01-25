#ifndef WAYPOINT_INCLUDE_GUARD_HPP
#define WAPOINT_INCLUDE_GUARD_HPP
#include <utility>
#include <vector>

namespace rigid2d
{

  struct Point
  {
    double x,y;
  };

  template <typename T>
  struct Node
  {
    Node(T value);
    T value;
    Node *nextElement;
  };
  
  template <typename T>
  class CircularLinkedList
  {
   CircularLinkedList();
   CircularLinkedList(std::vector<T> trajectoryPoints);
   T returnNextElement();

   public:
   T *head,*current;
  };

  class  Waypoint
  {
    Waypoint(std::vector<Point> PointSequence);
    std::vector<double> nextWaypoint();
    private:
    CircularLinkedList<Point> TrajectoryPoints;
  };

}
#endif
