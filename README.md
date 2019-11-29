# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Goals
In this project our goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. We are provided with the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

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

## Details of trajectory generation algorithm

### Choice of coordinates
Roads can be very curvy and unpredictable, which brings a lot of complexity when expressed in usual cartesian coordinates. but fortunately there is a practical approach which expresses the path distance s along the road and lateral displacement d to measure the lane position. In this approach it is assumed that we already have a detailed map of the road beforehand. d is perpendicular to s. d can be multiplied by lane width to get the position in the lane.

### Sensing the traffic
We loop over the given sensor fusion data to get nearby vehicles position and speed to determine if there is any car in fron of us and also if there is any vehicle in the left or right lane.

The following code is used to determine if there is a car in front of us which is slower than us.
If it finds a fster lane on the righ or left it tries to change lane by generating a new set of waypoints.

```
// loop over sensor fusion data to find front cars
    for (int i = 0; i < sensor_fusion.size(); ++i){
      // check if front car d Frenet component is within safety distance
      if (( lane_id*lane_width < sensor_fusion[i][6]) && (sensor_fusion[i][6] < (lane_id+1)*lane_width))
      {
        if ((sensor_fusion[i][5] < (end_path_s + safety_dist)) && (sensor_fusion[i][5] > (car_s - 5.0)))
        {           
          // calculate front car absolute speed            
          front_car_speed = std::sqrt( pow(sensor_fusion[i][3],2.0) + pow(sensor_fusion[i][4],2.0) ); 
          // calculate the diference in speed between the two cars
          delta_speed = car_speed - front_car_speed;
          // if front car going slower then match speed
          if (delta_speed >= 0.0){
            ref_speed = front_car_speed;
            front_car_slower = 1;
          } 
        }
      }

      // check if left lane is free
      // make sure car is not in the left most lane
      if ((lane_id > 0) && ((lane_id - 1) * lane_width < sensor_fusion[i][6]) && (sensor_fusion[i][6] < lane_id * lane_width))
      {
        // look for available space in the left lane
        if ((sensor_fusion[i][5] < car_s + 2*safety_dist) && (sensor_fusion[i][5] > car_s - safety_dist/2))
        {       
          left_clear = 0;
        }
        // check for traffic speed in the left lane, if any
        if (sensor_fusion[i][5] > car_s - 5.0){       
          speed_per_lane[lane_id - 1] = std::sqrt( pow(sensor_fusion[i][3],2.0) + pow(sensor_fusion[i][4],2.0) );
        }              
      }

      // check if right lane is free
      // make sure car is not in the right most lane            
      if ((lane_id < 2) && ((lane_id+1) * lane_width < sensor_fusion[i][6]) && (sensor_fusion[i][6] < (lane_id+2) * lane_width) )
      {
        // Right lane clear
        if ((sensor_fusion[i][5] < car_s + 2*safety_dist) && (sensor_fusion[i][5] > car_s - safety_dist/2))
        {
          right_clear = 0;
        }
        // check for traffic speed in the right lane, if any
        if (sensor_fusion[i][5] > car_s - 5.0)
        {       
          speed_per_lane[lane_id + 1] = std::sqrt( pow(sensor_fusion[i][3],2.0) + pow(sensor_fusion[i][4],2.0) );
        }  
      }            
    }   
```
Lane shift code

```
// change lanes if a lane is faster than me and a lane change is not already in progress
    if ( (front_car_slower == 1) && (car_speed < 0.9 * max_speed) && (change_lane == 0)) 
    {
      change_lane = 1;
    }

    if (change_lane == 1){
      if ((lane_id > 0) && (left_clear == 1))
      {
        lane_id = lane_id - 1;
        change_lane = 0;
      }
      else if ((lane_id < 2) && (right_clear == 1))
      {
        lane_id = lane_id + 1;
        change_lane = 0;
      }        
    }

    // transfer previous path's points to new path
    for (int i = 0; i < path_size; ++i)
    {
      next_x_vals.push_back(previous_path_x[i]);
      next_y_vals.push_back(previous_path_y[i]);
    }
```
### Waypoint generation
We tell the car where to move by producing waypoints (x,y). This sequence of points is then followed by the car by moving forward in a given angle and speed.

To get a smooth trajectory we use a spline to generate waypoints that follows the s coordinate and does not exceed max jerk.

The following code is used to generate a smooth path by generating new waypoints
 
```
    // calculate spline points ahead of ego vehicle, spaced by spline_dist
    new_wp = getXY(car_s + spline_dist,(lane_width/2 + lane_width*lane_id), map_waypoints_s, map_waypoints_x, map_waypoints_y);
    ptsx.push_back(new_wp[0]);
    ptsy.push_back(new_wp[1]);
    new_wp = getXY(car_s + 2*spline_dist,(lane_width/2 + lane_width*lane_id), map_waypoints_s, map_waypoints_x, map_waypoints_y);
    ptsx.push_back(new_wp[0]);
    ptsy.push_back(new_wp[1]);
    new_wp = getXY(car_s + 3*spline_dist,(lane_width/2 + lane_width*lane_id), map_waypoints_s, map_waypoints_x, map_waypoints_y);
    ptsx.push_back(new_wp[0]);
    ptsy.push_back(new_wp[1]);

    // transform spline points from map coordinates to car coordinates
    for (int i = 0; i < ptsx.size(); ++i)
    {
      shift_x = ptsx[i] - pos_x;
      shift_y = ptsy[i] - pos_y;
      ptsx[i] = shift_x * cos(0 - angle) - shift_y * sin(0 - angle);
      ptsy[i] = shift_x * sin(0 - angle) + shift_y * cos(0 - angle);
    }         

    // compute spine
    s.set_points(ptsx,ptsy); 
```



