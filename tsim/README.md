## Package Description ##
This package contains code for controlling a turtle from turtlesim package
through a feed forward control

 
##  roslaunch command ##
`roslaunch tsim trect.launch`

## Files ##
1. turtle_rect.cpp - The main logic file which holds code for setting up the
                     RoS communications and controlling the turtle
2. turtle_rect.hpp - The header file of the above cpp file. Contains the class
                     definition for that controls the whole pipeline
3. trect.launch - The launch file which launches the turtlesim node and the  
                  turtle_rect node. This also holds parameters that specify
                  the rectangular motion of the turtle.
4. ErrorPose.msg - A custom message definition for storing and publishing the
                   error pose ( Difference between the turtle's belief and 
                   the actual pose)
                  
## Outputs ##
1. Screenshot of turtle
![Image of turtle](./img/turtle_screenshot.png)
2. rqt graph output
![Image of rqt graph Output](./img/rqt_plot.png)
3. [Link to video output](https://drive.google.com/file/d/1CjJGAg3pz2uSBVpcFEVQs__nyGiVkVnt/view?usp=sharing)
