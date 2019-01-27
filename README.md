# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Goals
In this project the goal is to safely navigate a car (the "ego vehicle") around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. The simulator provides the ego vehicles's localization and sensor fusion data, as well as a sparse map list of waypoints around the highway. The ego vehicle should:
* Try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible (other cars will try to change lanes too). 
* Avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. 
* Be able to make one complete loop around the 6946m highway. Since the target velocity is 50 MPH, it should take a little over 5 minutes to complete 1 loop. 
* Not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

### Solution

The ego vehicle is able to successfully navigate around the track at least one time (generally many times).

The entire solution code is in main.cpp, except for the single header file spline.h which was downloaded from http://kluge.in-chemnitz.de/opensource/spline/.

The overall approach is to use a Prediction -> Behavior -> Trajectory cycle for the vehicle. Drive-related variables are defined in lines 229-243, and constants are defined in lines 257-261. 

#### Prediction

In the Prediction module (starting line 295), the algorithm looks through all vehicles included in the sensor fusion information to determine if any are in the current lane ahead of the ego vehicle, within the distance COLLISION_BUFFER (30m). If so, the algorithm begins to consider passing, by examining whether possible passing lanes are clear. When driving in the left or right lanes this is the center lane; while driving in the center lane it is either the left or right lane. "Clear" is defined as a distance of COLLISION_BUFFER ahead and PASSING_BUFFER (20 m) behind.

The determination of whether a given lane is clear is handled by lane_clear() (lines 166-186), which examines each car in the sensor_fusion data to see if it is in the given lane, then projects its current location using its measured velocity, the time increment and the number of previous path steps.

#### Behavior

In the Behavior module (starting line 330), the algorithm first brances on the result of the Prediction module about whether the ego vehicle is too close to a car in front. If so, it decelerates and checks if the appropriate target lane is clear. From the center lane it first tries to pass left, and then right. (Note that modularizing the Prediction and Behavior steps introduce some slight inefficiency, since the Prediction module checks lane clearance for both left and right passing lanes from the center lane, even if the left lane is clear for passing.)

If the ego vehicle is not too close to a leading car but is in slow-down mode, it decelerates or accelerates to match a reduced velocity (80% of the target velocity of 50 mph). This is discussed below.

If the ego vehicle is not too close and not in slow down mode, but is below the target velocity of 50 mph, it accelerates.

Finally, if none of the above conditions apply but the ego vehicle is not in the center lane, it attempts to return there, if it is clear. This reflects the theory that the ego vehicle should normally drive in the center lane to have the best chance of passing traffic in the future.

#### Behavior for Oscillation

The sub-module Behavior for Oscillation (starting line 368) is intended to eliminate a common problem the ego vehicle can experience: getting stuck behind a slow-moving lead car while blocked from passing. In this case the vehicle accelerates and decelerates repeatedly, experiencing oscillating "fallbacks". To get out of this situation, the algorithm tracks these "fallbacks" by recording the last three reference velocities sent to the simulator, and flags a "fallback" as a velocity minimum. These velocity fallbacks are timestamped using std::clock().

Each time a fallback is detected, the separation times between the current and two most recent fallbacks are calculated. If these two time separations are less than FALLBACK_DURATION_SEC (5s) then this is taken as evidence that the oscillating fallbacks are occurring, and the ego vehicle enters "slow-down mode". In this mode, the target velocity drops to 80% of 50 mph (40 mph), which can enable the ego vehicle to "fade back" and eventually be able to pass. This condition lasts for SLOW_DOWN_DURATION_SEC (10s) before returning to normal.

#### Trajectory

In the Trajectory module (starting line 418), the algorithm tries to find the last two points from the previous path, or generates two equivalent points from the ego vehicle's current location and a projection 1 m behind it in the same heading as the ego vehicle's current yaw. The algorithm next calculates three future points, separated by 25 m, in the target lane. Note that if the target lane was changed by the Behavior module, these will be in a different lane.

These five points are transformed into the ego vehicle's reference frame and the spline.h library is used to calculate a spline between them. To determine the appropriate spacing to sample this spline, a "look-ahead distance", the target velocity, and the 50 ms update rate for the simulator are used to calculate the increment. Any points from the previous path that are still ahead of the ego vehicle are added to the updated path, and then the spline is sampled at the appropriate interval at enough points to make a total of 50 for the updated path. This is then transformed back into the map reference frame and sent to the simulator. 

### Results

The ego vehicle is able to navigate all the way around the track successfully, without collisions, road departures, or excessive acceleration or jerk. It successfully changes lanes when too close to a lead vehicle, and successfully returns to the center lane when it is clear.

The sub-module to address fallback oscillations is partly successful. It functions as intended, and in some cases enables the ego vehicle to fall further back and open up passing space to escape a situation where it's blocked from passing. However, this doesn't always work, since the reduced target velocity is 40 mph, and the slowest-moving vehicles on the road match this. To ensure being able to fall back and find a passing zone, it would have to slow even more, but this would risk it being rear-ended. A future extension of this module could include considering this additional slow-down if the lane behind the ego vehicle is clear for an extended distance (although this would not protect against vehicle that change into the lane).

There are also situations where the ego vehicle could potentially speed up over 50 mph to find a passing zone, but these have been disallowed in the current implementation. This could also be part of a future extension.

When hemmed in by slow-moving traffic it completes 4.5 miles in 7:55. When driving in relatively open conditions it completes 4.5 miles in [X].

### Simulator
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
