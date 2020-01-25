#ifndef WAYPOINT_INCLUDE_GUARD_HPP
#define WAPOINT_INCLUDE_GUARD_HPP
#include <utility>
#include <vector>
#include "rigid2d/rigid2d.hpp"

namespace rigid2d
{


  template <typename T>
  struct Node
  {
  public:
  T value;
  Node<T> *nextElement;

  Node(T input_value)
  {
    value = input_value;
    nextElement = NULL;

  }
  };

  template <typename T>
  class CircularLinkedList
  {

   public:
  CircularLinkedList()
   {
    head=NULL;
    current=NULL;
    tail = NULL;
   }

  CircularLinkedList(std::vector<T> all_elements)
  {
    addElements(all_elements);
  }


  void addElements(std::vector<T> all_elements)
  {
    int start_index = 0;
    auto temp_current = current;
    if(head == NULL)
    {

    auto nodeElement = new Node<T>(all_elements[0]);
    head = nodeElement;
    current = nodeElement;
    start_index = 1;
    temp_current = head;
    }
    else
    {
      current = tail;
    }

    for (auto it = all_elements.begin()+start_index; it != all_elements.end(); it++) 
    {
      auto nodeElement = new Node<T>(*it);
      current->nextElement = nodeElement;
      current = nodeElement;
    }
    current->nextElement = head;
    tail = current;
   current = temp_current;
  }


  T returnNextElement()
  {
     auto elemValue = current->value;
     current = current->nextElement;
     return elemValue;

   }

   private:
   Node<T> *head,*current, *tail;
  };

  class  Waypoint
  {
    public:
      explicit Waypoint(std::vector<Vector2D> PointSequence);
      std::vector<double> nextWaypoint();
      CircularLinkedList<Vector2D> trajectoryPoints;
      WheelVelocities  nextVelocity(float timeInterval=0.1);
    private:
      DiffDrive diffCar;
      Vector2D currentWaypoint;
      double maxLinearVelocity, maxAngularVelocity;
  };


}
#endif
