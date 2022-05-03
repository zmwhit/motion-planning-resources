# motion-planning-resources
collection of motion planning resources

[motion-planning-resources](https://github.com/zmwhit/motion-planning-resources)
# what is motion planning
In contrast of control problem which closed-loop design state and control variables using feedback in frequency domain and accouting for tracking a reference, motion planning problem open-loop design them  in time domain, accouting for avoiding obstacles and producing the smooth reference
# Take the First Step
## mobile wheeled robot
> DWA [The dynamic window approach to collision avoidance](https://ieeexplore.ieee.org/document/580977)
## autonomous driving car
> Frenet [Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame](https://ieeexplore.ieee.org/document/5509799/)
## unmanned aerial vehicle
> MinimumSnap [Minimum snap trajectory generation and control for quadrotors](https://ieeexplore.ieee.org/document/5980409)
# Next Step
choose one classical algorithm for a general scenario and study deeply
* [Basic&Tools](motion-planning-resources.md/#Basic&Tools)
    * [C++](motion-planning-resources.md/##C++)
    * [Curve](motion-planning-resources.md/##Curve)
    * [Collision Check](motion-planning-resources.md/##Collision-Check)
    * [Robotics Basic Algorithm](motion-planning-resources.md/##Robotics-Basic-Algorithm)
    * [ROS](motion-planning-resources.md/##ROS)
* [Algorithm](motion-planning-resources.md/#Algorithm)
    * [DWA](motion-planning-resources.md/##Dynamic-Windows-Approach-(DWA))
    * [TEB](motion-planning-resources.md/##Timed-Elastic-Band-(TEB))
    * [HybridA*](motion-planning-resources.md/##Hybrid-Astar)
    * [RRT](motion-planning-resources.md/##RRT&variants)
    * [MPCC](motion-planning-resources.md/##Model-Predictive-Contouring-Controller-(MPCC))
    * [Multi-Robot Planning](motion-planning-resources.md/##Search-Based-Multi-Robot-Planning)
    * [Frenet Planning](motion-planning-resources.md/##Frenet-Optimal-Trajectory-Planning)
    * [Path Optimizer](motion-planning-resources.md/##Path-Optimizer-(LiJiangnanBit))
    * [Polytope Convex Space](motion-planning-resources.md/##Polytope-Convex-Space-(sikang))
    * [EPSILON](motion-planning-resources.md/##EPSILON)
    * [Spline Based Planning](motion-planning-resources.md/##Spline-Based-Planning)
        * [Btraj](motion-planning-resources.md/##Btraj)
        * [Fast-Planner](motion-planning-resources.md/###Fast-Planner)
        * [Ego-Planner](motion-planning-resources.md/###Ego-Planner)
* [Solver](motion-planning-resources.md/#Solver)
    * [POMDP](motion-planning-resources.md/##POMDP)
    * [QP](motion-planning-resources.md/##QP)
    * [NLP](motion-planning-resources.md/##NLP)
    * [MPC](motion-planning-resources.md/##MPC)
    * [LeastSquare](motion-planning-resources.md/##LeastSquare)
* [Simulation](motion-planning-resources.md/#Simulation)
    * [LGSVL](motion-planning-resources.md/##LGSVL)
    * [Carla](motion-planning-resources.md/##Carla)
    * [Udacity](motion-planning-resources.md/##Udacity)
    * [ROS-Stage](motion-planning-resources.md/##ROS-Stage)
* [Lab&Group](motion-planning-resources.md/#Lab&Group)
* [Open Autonomous Driving Platform](motion-planning-resources.md/#Open-Autonomous-Driving-Platform)
    * [Apollo](motion-planning-resources.md/##Apollo)
    * [Autoware](motion-planning-resources.md/##Autoware)
* [Videos](motion-planning-resources.md/#Videos)
* [Competition](motion-planning-resources.md/#Competition)


# Easy Install
[script](easy_install.sh)
```
mkdir lib && cd lib
sudo chmod 777 easy_install.sh
bash easy_install.sh
```
don't forget to type password when installing, this script may have some mistake, so you can copy and paste commands on your own, this could also also be fast :)
