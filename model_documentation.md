# **Path Planning Project**

In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. 

The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

[//]: # (Image References)

[last]: ./last_screenshot.png
[lcr0]: ./lane_change_right_screenshot_0.png
[lcr1]: ./lane_change_right_screenshot_1.png
[lcr2]: ./lane_change_right_screenshot_2.png
[lcr3]: ./lane_change_right_screenshot_3.png
[video1]: ./project_video.mp4

## [Rubric](https://review.udacity.com/#!/rubrics/1020/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
# Writeup / README

## Valid Trajectories

* The following screenshot is taken after finishing 1-loop. The upper right infomation shows that the collision-free and speed limit.

![alt text][last]

### The car is able to drive at least 4.32 miles without incident..

The top right screen of the simulator shows the current/best miles driven without incident. Incidents include exceeding acceleration/jerk/speed, collision, and driving outside of the lanes. Each incident case is also listed below in more detail.

### The car drives according to the speed limit.

The car doesn't drive faster than the speed limit. Also the car isn't driving much slower than speed limit unless obstructed by traffic.

* I use 49.0 MPH as a maximum reference velocity. This value is smaller than 49.5 MPH (the value in the guide video). After several tests, slower reference velocity is needed when lane-changes happen in the curved road. In the curved path, larger interval than reference velocity can be generated for path trajectory.
```
#define MAX_REF_VEL 49.0 // 50 MPH - 22.352 m/s
```
### Max Acceleration and Jerk are not Exceeded.

The car does not exceed a total acceleration of 10 m/s^2 and a jerk of 10 m/s^3.

* I use 50 m as a transition length for lane changes. It is larger than the value in the guide video is 30m. In the tests, I found that transition within 30 m can cause jerk violation.
```
#define MAX_PATH_LEN 50
```
```
            auto next_wp0 = getXY(car_s+TRANSITION_DIST, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            auto next_wp1 = getXY(car_s+TRANSITION_DIST + 40, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            auto next_wp2 = getXY(car_s+TRANSITION_DIST + 80, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
```

### Car does not have collisions.

The car must not come into contact with any of the other cars on the road.

* I use large margin for lane change. My implementation checks range (-20, +30) at the current position when calculating lane change cost.
```
#define LC_SAFE_MARGIN_FRONT 30.0
#define LC_SAFE_MARGIN_REAR 20.0
```
### The car stays in its lane, except for the time between changing lanes.

The car doesn't spend more than a 3 second length out side the lane lanes during changing lanes, and every other time the car stays inside one of the 3 lanes on the right hand side of the road.

### The car is able to change lanes

The car is able to smoothly change lanes when it makes sense to do so, such as when behind a slower moving car and an adjacent lane is clear of other traffic.

![alt text][lcr0]
![alt text][lcr1]
![alt text][lcr2]
![alt text][lcr3]

* The series of pictures above show the smooth transition for lane change to right. The path is a bit smoother than the guide to avoid acceleration and jerk violation.

## Final Result
The result video cut is here: [video_link](https://www.youtube.com/embed/eXf5FlwHKU8 "")

<iframe width="560" height="315" src="https://www.youtube.com/embed/eXf5FlwHKU8" frameborder="0" allowfullscreen></iframe>

## Reflection

There is a reflection on how to generate paths.

* My implementation could not response the sudden car intervention from sides.
* My implementation could not clear prediction process of other cars using sensor_fusion. Instead of it I use large margins for spatial slack.

