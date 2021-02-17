# Head_Pose_Alert

![image1](https://user-images.githubusercontent.com/50894726/108259253-386a5500-71a4-11eb-8fc9-1632c3699440.gif)

Camera-based head pose ROS Package using [Dense-Pose-Estimation](https://github.com/1996scarlet/Dense-Head-Pose-Estimation). 

## Dependencies
- ROS2 (Tested on Dashing)
- tensorflow >= 2.3
- opencv-python

## About

Based on the camera, the head pose is estimated to derive the roll, pitch, and yaw values of the head.

If you set specific left and right angles and vertical angles as thresholds in advance, you can see the direction the head is facing.

If you do not look straight ahead for more than a certain time, a separate notification is delivered. 

## Published Topics

- **/headpose/rpy** ([std_msgs/Float32MultiArray](http://docs.ros.org/en/jade/api/std_msgs/html/msg/Float32MultiArray.html))
  - Publish head roll, pitch, and yaw values.
  - The length of the array is 3, and it means roll, pitch, and yaw from the left.
  
- **/headpose/alert** ([std_msgs/Int16MultiArray](http://docs.ros.org/en/api/std_msgs/html/msg/Int16MultiArray.html))
  - Publish the current heading direction and forward gaze notification.
  - The length of the array is 2, and it means direction, and notification from the left.
  - The value of index 0(direction) is from 0 to 8, and the meaning of each number is as follows.      
    |0|1|2|3|4|5|6|7|8|
    |---|---|---|---|---|---|---|---|---|
    |straight ahead|Left|Right|Up|Down|Left&Up|Left&Down|Right&Up|Right&Down|
  - The value of index 1(notification) is from 0 to 1, 0 is normal, and 1 is a warning.
  
## Demo

- In the Lab

  ![image2](https://user-images.githubusercontent.com/50894726/108259253-386a5500-71a4-11eb-8fc9-1632c3699440.gif)

  Full Video : https://youtu.be/OevwqQCDygE

- In the Car

  ![image3](https://user-images.githubusercontent.com/50894726/108263751-ff34e380-71a9-11eb-9c53-0bd5e685d28c.gif)

  Full Video : https://youtu.be/OPAwfY2Lt7Q
  
## How to Run

- Move the `head_pose_alert` directory to your colcon workspace(e.g. `~/colcon_ws/src`). 
- Build & source it!
  ```
  cd ~/colcon_ws
  colcon build
  source install/setup.bash
  ```
- `ros2 launch head_pose_alert head_pose_alert.launch.py`

## Settings

- **Parameters**
  - You can edit parameters in the `params.yaml` file in the config directory.
  - `video_input` is your camera device index.
  - `threshold` is angle threshold. From index 0, these are the thresholds of left, right, up, and down, respectively.
     For example, `[-35, 35, 10, -30]` means left if the yaw of the head is less than -35°, right if the head is more than 35°, up if the pitch of the head is more than 10°, and down if it is less than -30°.

- **Alert Conditions**
  - I wanted to warn if I didn't look ahead for more than 3 seconds. 
  - The direction of the head is recorded in a queue with a specific length, and a warning is made when there are many cases in which the head is not looking forward. 
  - Let's look the `head_pose_alert.py`
    - Specify the maximum length of the queue in `alert_check = deque(maxlen=90)`. The default is 90.(This is because the camera is about 30hz.)
    - `if len(alert_check) == 90 and alert_check.count(0) <= 10 :` is a condition for sending an alert when 0(straight ahead) is less than 10 out of 90 values.

- **Video Demo**
  - By default, the video is not visible.
  - If you want to watch the video, just uncomment below. 
    ```
    cv2.imshow("demo", frame)
    if cv2.waitKey(1) == ord("q"):
        break
    ```

## TODO

- Send a notification when a face is not recognized.

----------------------------------------------------------------
### Open Source License

#### Dense-Head-Pose-Estimation

- https://github.com/1996scarlet/Dense-Head-Pose-Estimation
- Copyright (c) 2020 Remilia Scarlet
- MIT License
