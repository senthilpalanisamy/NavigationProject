# g++ -Wall -Wextra -g -std=c++17 -o rigid2d_test main.cpp rigid2d.cpp
rosrun rigid2d rigid2d_node<test1_input.txt | cmp -s - test1_answer.txt && echo success || echo failure
