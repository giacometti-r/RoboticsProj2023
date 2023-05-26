# RoboticsProj2023

Tasks Anthony:
1. Write and test code for b-spline trajectories [75%] --- May 26th
2. Write and test kalman filter with simulated imu/proximity [10%] --- May 26th
3. Implement UCS, A*, RRT, RRT* path planning with circular obstacles --- May 31st
4. Derive dynamics of 2d drone and compute transfer function --- May 28th
5. Design and document tuning of multiloop PID controller --- May 28th
6. Design and document tuning of multiloop Lead/Lag controller --- May 28th

Tasks Daniil and Riccardo:
1. Modify drone code with trajectory planner --- May 26th
2. Modify sim code with [x,y,theta,u,v,w] states/controls plots --- May 28th
3. Write basic test suite for built in PID with various trajectories --- May 28th
4. Apply test suite of trajectories on optimized PID, Kalman and gather data --- May 31th
  a. Change noise models for sensors, add coupling to see what happens
  b. Try high frequency trajectories (sharpo turns)
6. Integrate all control and planning modules

EXTRA
1. Model missile system from https://murray.cds.caltech.edu/index.php/Python-control/Example:_Vertical_takeoff_and_landing_aircraft
2. Tune missile PID
3. Tune missile kalman filter so that it knows where it is because it knows where it isn't
4. Write and test SE(2) trajectory planner (for flip)
5. Design and document tuning of MIMO SE(2) controller for drone/missile

Tasks Report:
1. Anthony - Theory portion of report
2. Anthony - Slides
3. Daniil/Riccardo - Experimental portion of report 
4. Daniil/Riccardo - Demos
