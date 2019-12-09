# turtle_go_to_goal is an incomplete ros-package designed as an assignment 

## Current state of this ros-package

- This ros-package contains an `src` directory and two files (i.e., `CMakeLists.txt`, `package.xml`)
- The src directory contains two files that are incomplete ros-nodes:
	1. `turtle_go_to_goal.cpp`
	2. `turtle_go_to_goal.py`

## Steps to complete this assignment

- **Fork** this repository into your GitHub. 
- **Clone** the repository from your GitHub into your local machine's catkin worksapce.
- **Complete the objective** of this assignment.
- Once the objective is complete, **push** the modifications that you made in your turtle_go_to_goal package to your repository on GitHub.

## Objective of this assignment

- Develop the ros-node `turtle_go_to_goal` with the programming langauge that you prefer (i.e., cpp or python). 
	- When you run the ros-node, it should take as input cordinates for the goal position of the turtle.
	- After giving the goal position, the turtle should (1) orient itself in the direction of the goal position, and (2) go to the goal position.
	- The node files `turtle_go_to_goal.cpp` and `turtle_go_to_goal.py` are incomplete but provide an initial overall structure and hints.
- Dont forget to check your `CMakeLists.txt` and `package.xml` before doing catkin_make.
- Once you successfully run and test your node, fill in the results table.

## Results

All of these were calculated starting from the previous one.

| Goal positions| Error in reaching the goal| Number of iterations to reaching the goal  |
| ------------- |:-------------------------:| ------------------------------------------:|
| (7, 4)        |        0.23711            |              46                            |
| (2, 9)        |        0.238446           |              81                            |
| (1,1)         |        0.241254           |              80                            |



