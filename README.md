# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

[//]: # (Image References)

[image1]: ./image/path-planning-result.gif "Result"
[image2]: ./image/best_result.png (https://youtu.be/7rDqoTM8v1w)

![Result][image1]


### Simulator.
You can download the simulator which contains the Path Planning Project(https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

### Goals
The goal of this project is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. THe car's localization and sensor fusion data are provided , there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

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

## Refrection

### Predict other cars behavior :  Line 253 - 305
In this part, we will check the other cars' position and speed from sensor fusion data. If the other car is within 30m in front of our car and within 10m behind, our car recognize there is a car.

The code is following:

```cpp
bool car_ahead = false;
bool car_right = false;
bool car_left = false;

double ahead_car_speed = 0;
for(int i=0; i < sensor_fusion.size(); i++)
{
  float d = sensor_fusion[i][6];
  int other_car = 0;
  
  if(d > 0 && d < 4)
  {
    other_car = 0;  // left lane
  }
  else if (d > 4 && d <8)
  {
    other_car = 1; // center lane
  }
  else if (d > 8 && d < 12)
  {
    other_car = 2; // right lane
  }
  if (other_car < 0) {
    continue;
  }
  
  double vx = sensor_fusion[i][3];
  double vy = sensor_fusion[i][4];
  double check_speed = sqrt(vx*vx + vy*vy);
  double check_car_s = sensor_fusion[i][5];

  check_car_s += ((double)prev_size * 0.02 * check_speed);
  
  if((other_car == lane) && (check_car_s > car_s) && (check_car_s - car_s) < 30)
  //the other car is in front of us and the car is within 30m
  {
    car_ahead = true;
    ahead_car_speed = check_speed * 2.24; //convert from m/s to MPH
  }
  else if ((other_car - lane == 1) && (check_car_s - car_s < 30) && (car_s - check_car_s < 10))
  {
    car_right = true;
  }
  else if ((other_car - lane == -1) && (check_car_s - car_s < 30) && (car_s - check_car_s < 10))
  {
    car_left = true;
  }
}

cout << "car_ahead: " << car_ahead <<endl;
cout << "car_left: " << car_left << endl;
cout << "car_right: " << car_right << endl;
cout << "lane number: " << lane << endl << endl;
```


### Control speed and lane change :  Line 307 - 329
In this part, we will regulate acceleration based on the other cars' position and speed which is obtaind at the code section above. If there is a car in front fo our car and no car on the left or right of our car, our car chages the lane. If our car cannot change the lane, it keeps the speed of the car ahead and follow the car. The code is following:

```cpp
double Max_speed = 49.5;
if(car_ahead)
{
  if (!car_left && lane > 0){
    lane--; // change to left lane
  }
  else if (!car_right && lane != 2){
    lane++;  // change to right lane
  }
  else{
    if(ref_vel > ahead_car_speed){
      ref_vel -= 0.23;
    }else{
      ref_vel += 0.23;
    }
  }
}
else{

  if ( ref_vel < Max_speed ) {
    ref_vel += 0.224;
  }
}
```

### Trajectory :  Line 331 - 441
In this part, we will create the car trajectory based on the speed and lane output from the section above and past path points. 

To crate trajectory, 5 points are prepared to use spline calculation. First two points are carried over from last two points of previous trajectory and three points are set on 30m, 60m, 90m ahead of the car. 
The last trajectory points are copied to the new trajectory for continuity on the trajectory.

The code is following:

```cpp
vector<double> ptsx;
vector<double> ptsy;

// reference x,y yaw states
// either we will reference the starting point as where the car is or at the previous paths end point
double ref_x = car_x;
double ref_y = car_y;
double ref_yaw = deg2rad(car_yaw);

//if previous size is almost empty, use the car as starting reference
if(prev_size<2)
{
  //Use two points that make the path tangent to the car
  double prev_car_x = car_x - cos(car_yaw);
  double prev_car_y = car_y - sin(car_yaw);

  ptsx.push_back(prev_car_x);
  ptsx.push_back(car_x);

  ptsy.push_back(prev_car_y);
  ptsy.push_back(car_y);

}else{

  //Redefine reference states as previous path end point
  ref_x = previous_path_x[prev_size-1];
  ref_y = previous_path_y[prev_size-1];

  double ref_x_prev = previous_path_x[prev_size-2];
  double ref_y_prev = previous_path_y[prev_size-2];
  ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

  //use two points that make the path tangent to the previous path's end point
  ptsx.push_back(ref_x_prev);
  ptsx.push_back(ref_x);

  ptsy.push_back(ref_y_prev);
  ptsy.push_back(ref_y);

}


//In Frenet add evenly 30m spaced points ahead of the starting reference
vector<double> next_wp0 = getXY(car_s+30,(2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
vector<double> next_wp1 = getXY(car_s+60,(2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
vector<double> next_wp2 = getXY(car_s+90,(2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

ptsx.push_back(next_wp0[0]);
ptsx.push_back(next_wp1[0]);
ptsx.push_back(next_wp2[0]);

ptsy.push_back(next_wp0[1]);
ptsy.push_back(next_wp1[1]);
ptsy.push_back(next_wp2[1]);

for (int i = 0; i < ptsx.size(); i++) {
  
  // Shift car reference angle to 0 degrees
  double shift_x = ptsx[i] - ref_x;
  double shift_y = ptsy[i] - ref_y;
  
  ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
  ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
}

// create a spline
tk::spline s;

//set (x,y) points to the spline
s.set_points(ptsx, ptsy);

//Define the actual (x, y) points we will use for the planner
vector<double> next_x_vals;
vector<double> next_y_vals;

//Start wil all of the previous path points from last time
for(int i = 0; i < previous_path_x.size(); i++)
{
  next_x_vals.push_back(previous_path_x[i]);
  next_y_vals.push_back(previous_path_y[i]);
}


//Calculate how to break up spline points so that we travel at our desired reference velocity
double target_x = 30.0;
double target_y = s(target_x);
double target_dist = sqrt((target_x * target_x) + (target_y * target_y));

double x_add_on = 0;

for(int i=1; i <= 50-previous_path_x.size(); i++)
{
  double N = (target_dist/(0.02*ref_vel/2.24)); 
  //2.24 convert to meter per second
  double x_point = x_add_on + target_x / N;
  double y_point = s(x_point);

  x_add_on = x_point;

  double x_ref = x_point;
  double y_ref = y_point;

  //Rotate back to normal after rotating it earlier
  x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
  y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

  x_point += ref_x;
  y_point += ref_y;

  next_x_vals.push_back(x_point);
  next_y_vals.push_back(y_point);
}
```

## Result
Here's a [link to my video result](https://youtu.be/7rDqoTM8v1w)

![Result][image2]

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
