# ME495 Sensing, Navigation, and Machine Learning
Author: Senthil Palanisamy
# Tasks Submitted
(List the tasks that you worked on and completed here)
A.000
A.001
A.002
A.003
A.004
B.000
B.001
B.002
B.003
B.004


# Tasks Completed
C.000
C.001
C.002
C.003
C.004


One small discrepancy noticed:
The istream overriding for Transform2D was declared outside the class definition.
`std::istream & operator>>(std::istream & is, Transform2D & tf);`

To the best of my knowledge, it is not possible to access private members of 
a class and store values in them from a function that is outside the class. So
I moved the istream overloading function to be with the class definition of
Transform2D
`friend std::istream & operator>>(std::istream & is, Transform2D & tf);`
I noticed this too late and hence, didn't have time to clarify this with you.
If a different implementation was expected I could modify my implementation 
in the next submission
# Tasks Completed
(List the tasks that you finished in previous assignments here)
