# Path-Planning-Project WriteUp
Project 1 of Term 3 Self-Driving Car Engineer Nanodegree Program.  Goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit.  Car's localization and sensor fusion data is provided, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible.

## Model Documentation

Implemented model has two Parts: 

1. **Collision Avoidance (Using Sensor Fusion Data):** Model reads sensor fusion data to understand which vehicles are too close to ego vehicle. When system dectect that vehicle in front of it is less than 30m. It reduces it velocity and keep on doing it until fromt vehicle reaches at safe distance. Model manages speed by accelrating until car reaches speed limit.     

2. **Smooth Trangectory generation:** refering the map waypoints, ego vehicle understands which lane it is in. Hence, Model generate waypoints that follow smooth tranjectory with help of splne library APIs.   

## Basic Build Instructions

**NOTE: Please make sure if you rerun the cmake, to use the `-DCMAKE_BUILD_TYPE=RelWithDebInfo` flag!  This will make the spline.h anonymous namespace referenceable.
Otherwise, the path_planning executable will generate an error from the spline.h file, since it is referenced multiple times within main.cpp:**
```
path_planning: /home/jchen/SDCND/SDC-T3-P1/src/spline.h:288: void {anonymous}::tk::spline::set_points(const std::vector<double>&, const std::vector<double>&, bool): Assertion `x.size()==y.size()' failed.
Aborted (core dumped)
```

1. Clone this repo.
2. Change to the build directory: `cd build`
3. Compile: `cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo .. && make`
4. Run it: `./path_planning`.
5. Start the simulator

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


## Spline Library

Creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

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

## Simulator

The latest term 3 simulator (v1.2) downloaded from  [https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).




