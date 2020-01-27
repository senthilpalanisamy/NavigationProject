#ifndef WAYPOINT_INCLUDE_GUARD_HPP
#define WAPOINT_INCLUDE_GUARD_HPP
#include <utility>
#include <vector>
#include "rigid2d/rigid2d.hpp"
/// \file
/// \brief A Library for getting a diffdrive robot to move in a specified
/// trajectory using waypoints

namespace rigid2d
{

  /// \brief A template for defining a node for a linked list. A template
  /// is used so that the node is agnostic to the data that is being stored
  template <typename T>
  struct Node
  {
    public:
    /// The actual value stored in the node
    T value;
    /// Pointer to the next element
    Node<T> *nextElement;
    /// Constructor for initilizing the value and pointer of the node
    Node(T input_value)
    {
      value = input_value;
      nextElement = NULL;

    }
  };

  /// \brief A template for implementing a circular linked list. A template
  /// is used so that the implementation is agnostic to the actual data
  /// being stored
  template <typename T>
  class CircularLinkedList
  {

    public:
    /// \brief Default constructor of a circular linked list. Initializes all pointers
    /// to NULL
    CircularLinkedList()
    {
     head=NULL;
     current=NULL;
     tail = NULL;
    }
    /// \brief A parametrized constructor for the circular linked list
    /// \param all_elements A vector containing all elements in a circular linked
    /// list
    CircularLinkedList(std::vector<T> all_elements)
    {
      addElements(all_elements);
    }

    /// \brief Add the given element to the circular linked list
    /// \param A vector all the elements that need to be added.
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

   /// \brief Returns the next element on the circular linked list
   /// \return The element that is next on the circular linked list
   T returnNextElement()
   {
     auto elemValue = current->value;
     current = current->nextElement;
     return elemValue;

   }

   private:
   /// Pointer to the first element, last element and the current element
   Node<T> *head,*current, *tail;
  };

  /// A class that consolidates functionalities for driving a differential
  /// drive robot to follow a specified trajectory through waypoints
  class  Waypoint
  {
    public:
      /// A parameterized constructor 
      /// \param PointSequence - All the waypoints to be navigated
      explicit Waypoint(std::vector<Vector2D> PointSequence);
      /// Returns the next waypoint
      /// \return A waypoint packed into a standard vector
      std::vector<double> nextWaypoint();
      /// A circular linked list for holding all waypoints
      CircularLinkedList<Vector2D> trajectoryPoints;
      /// Computes teh next velocity to follow the given waypoint
      /// \param timeInterval The time interval of the estimated motion
      /// \return The wheel velocities to move closer to the waypoint
      WheelVelocities  nextVelocity(float timeInterval=0.1);
    private:
      /// All data and functionalities of a differential drive robot
      DiffDrive diffCar;
      /// The waypoint which is currently being followed
      Vector2D currentWaypoint;
      /// The max constraints on the velocity
      double maxLinearVelocity, maxAngularVelocity;
  };


}
#endif
