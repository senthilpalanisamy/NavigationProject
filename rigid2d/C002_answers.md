1. The core difference between a struct and a class in C++ is that members in
   struct default to public visibility and members in a class default to private
   visibility.
2. Transform2D has some private data while Vector2D doesn't have any non-public 
   data. Therefore, it is better to use a class for Transform2D and a structure
   for Vector2D to stick with the conventions, thereby promoting code readibility.
   Moreover, since Transform2D contains private data that needs to initialised
   based on the values received for public members through the constructor,
   this technically means that the class has an invariant. Therefore, in 
   accordance with cpp core guidelines, we should be using a class for Transform2D 
   where as for Vector2D, there are no invariants and hence, we should stick
   to a struct.
3. The recommened cpp coding guideline is to declare all single parameter
   constructors explicit by default. What this means is that this disables
   the default implicit conversion for single paramter constructors enabled by
   default. This avoids unintended implicit conversions happening leading to 
   bugs. Unless, we see a need for enabling implicit conversion for a single
   parameter constructor, we should always declare a single parameter construcor
   with explicit keyword.
4. Three design patterns I could think of are 
   1. Have a normalise function within the rigid2d namespace but outside the
      struct Vector2D.
   2. Have a public function within the struct Vector2D
   3. Overload division ("/") operator and then implement a normalise function
      that will use the overloaded definition to normalise the vector
   Design pattern |        Pro                     |        Con
   _________________________________________________________________________________
          1       | A generic normalising function |  A function that is modifiing 
                  | could be implemented that could| object state is placed outside
                  |be used as a library in a       | of the class, which is not a
                  | general purpose way across     | recommended guideline.
                  |different class or struct       |
                  | object or anywhere in the code | 
                  |                                |
           2      | Falls within the suggested     |
                  | pattern of having all function |
                  | that modify a class/struct     |
                  | state within the class/struct  |
                  |                                |
           3      | Overloading the division       | 
                  | operator will give a more      |
                  | intuitive feeling about the    |
                  | data type making a more        |
                  | "regular" datatype thereby     |
                  | promoting user interaction with|
                  | the datatype.                  | 
___________________________________________________________________________________________
  I would prefer design pattern 3 since it would give a more intuitive feel of the
  whole datatype.
5. I have implemented it. 
6. By default, we should be declaring all functions that do not change the 
   object passed to them as "const". This will help us in conveying the precise
   intent of the function, promotes readibility and lets the compiler catch
   our mistakes if we violate the design design decision at some point. Hence,
   inv function whose main intent is to return an inverse of the transform
   passed on to it should never be modifying the object passed to it and hence,
   should be declared as a constant function but a "*=" operator, whose 
   main purpose is to mutiply the current object with the object to its right
   should modify its object content and therefore, should not be declared as
   a const function.


