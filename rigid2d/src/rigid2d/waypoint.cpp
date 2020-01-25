#include<waypoints.hpp>

namespace rigid2d
{

  template<typename T>
  Node<T>::Node(T input_value)
  {
    value = input_value;
    nextElement = NULL;

  }
  template<typename T>
  CircularLinkedList<T>::CircularLinkedList()
  {
    head=NULL;
    current=NULL;
  }

  template<typename T>
  CircularLinkedList<T>::CircularLinkedList(std::vector<T> all_elements)
  {
    auto nodeElement = new Node<T>(all_elements[0]);
    head = &nodeElement;
    current = &nodeElement;
    for (auto it = all_elements.begin()+1; it != all_elements.end(); it++) 
    {
      nodeElement = new Node<T>(*it);
      current->nextElement = &nodeElement;
      current = &nodeElement;
    }
    current->nextElement = head;
   current = head;

  }

  template<typename T>
  T CircularLinkedList<T>::returnNextElement()
  {
   current = &current->nextElement;
   return current.value;
  }



}

