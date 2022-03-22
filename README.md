# ABB_irb_120_control


![gripper](https://user-images.githubusercontent.com/63298005/159411206-c9b4343b-62b4-48d3-9e94-906ba8d97ea3.gif)

A package simulates motion control scenarios for the ABB_irb_120 robot.

the package is capable of simulating simple kinematics motion control scenarios and more complex ones such as pick and place operations.

## abb_config
contains all the needed configurations files for the motion control and the simulation.

these files can be found in the [abb experimental repository](https://github.com/ros-industrial/abb_experimental)


## abb_irb_120_motion control
contains python scripts that were used to apply motion planning scenarios using moveit's motion planning API.
[motion planning API tutorial](http://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/motion_planning_api/motion_planning_api_tutorial.html)

contains python scripts that were used to apply pick and place scenarios using moveit's API.
[pick and place tutorial](http://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/pick_place/pick_place_tutorial.html)



## simulation
 start the simulation by typing the following command in the terminal `roslaunch roboticarm roboticarm.launch`
 
