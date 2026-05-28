[< Previous lesson](../lesson1) -- [**Main Readme**](../README.md) -- [Next lesson >](../lesson3/)

# Lesson 2 - Localizer

The primary task of the localizer is to determine the vehicle's position on the map. This can be done using GNSS ([Global Navigation Satellite System](https://en.wikipedia.org/wiki/Satellite_navigation)), lidar, or a camera. In this lesson, we focus on localization using GNSS, which is the easiest to implement and suffices for simpler experiments.

We will use logged data from the car's [Novatel PwrPak7D](https://novatel.com/products/receivers/enclosures/pwrpak7d) GNSS sensor. The Novatel system combines the absolute GNSS positions with relative IMU (Inertial Measurement Unit) measurements to improve accuracy. The accuracy is further enhanced by using stationary ground base stations within [RTK](https://en.wikipedia.org/wiki/Real-time_kinematic_positioning) (Real-Time Kinematic). In addition to location, it also determines the speed of the vehicle from consecutive locations.

The data is saved in a rosbag file. The GNSS log is recorded in the `/novatel/oem7/inspva` topic. This topic includes two kinds of data:
* **location**, given as latitude, longitude, and height,
* **velocity**, given in north, east, and up directions.

The task of the localizer is straightforward — it must take the latitude and longitude coordinates in [WGS84 system](https://en.wikipedia.org/wiki/World_Geodetic_System) and convert them to the map coordinate system. We use the UTM (Universal Transverse Mercator) zone 35N projection for the map. Using projection to a [Cartesian coordinate system](https://en.wikipedia.org/wiki/Cartesian_coordinate_system) is common in processing geospatial data, making calculating distances between points and other transformations much easier.

### Expected outcome

* Understanding what it means to localize a car on the map
* Your localizer node will convert the measured location from the GNSS system frame to map coordinates
* As a result, we can record a trajectory of the car into a .csv file and can make the car follow that trajectory in the next lesson


## 0. Download the rosbag

Download the demo rosbag file and place it in `shared/data/bags/`:

* [2023-05-25-14-21-10_sensors_Raekoda.bag](https://drive.google.com/file/d/1rFDmUaqjApCEv8PqPAS5zCYA8VJt6xA_/view) — contains all data recorded from the car during a single drive in the center of Tartu.

This bag file will be used across all lessons.

## 1. Explore the localizer node

Open [`lesson2/nodes/localizer.py`](nodes/localizer.py) — this is the starting skeleton for your localizer node. Take a moment to study its structure:

- The node is organized as a class. The ROS node is created in the [`if __name__ == '__main__':`](https://docs.python.org/3/library/__main__.html#idiomatic-usage) block, then `run()` is called.
- The `__init__` method sets up parameters, internal variables, subscribers, and publishers in separate groups.
- Notice that parameter names don't use the `~` prefix — these are shared parameters that other nodes can also read. More on [parameter name resolution](http://wiki.ros.org/Names#Resolving).
- The parameter values are defined in [`shared/config/localization.yaml`](../shared/config/localization.yaml) and loaded via the `rosparam` tag in the launch file.

##### Instructions
Find `TODO 1` in `transform_coordinates` and add: `print(msg.latitude, msg.longitude)`. You can remove the `pass` statement at the end of the method once you start adding code.

##### Validation
* `roslaunch autoware_mini_tutorial lesson2.launch`
   - The launch file plays the rosbag, runs your localizer, and opens RViz.
* You should see latitude/longitude coordinates printed in the console, similar to this (note: exact values depend on the bag file):

```
58.37951071076978 26.72648964125517
58.37951153102539 26.726488117933275
58.379512354077676 26.726486590056282
...
```


## 2. Convert coordinates

Latitude and longitude printed out in the previous task are geographic coordinates in [World Geodetic System](https://en.wikipedia.org/wiki/World_Geodetic_System) (with [EPSG code 4326](https://epsg.io/4326)), or more commonly referred to as GPS coordinates. Our localizer needs to transform these coordinates to UTM zone 35N projection ([with EPSG code 25835](https://epsg.io/25835)) as used for the map. Additionally, we have defined a custom origin point near the Delta building that needs to be subtracted from transformed coordinates. We use the [pyproj](https://pyproj4.github.io/pyproj/stable/) library for coordinate transformations.

##### Instructions

Find `TODO 2` in `localizer.py` — it appears in two places:

1. In `__init__`: Create a coordinate [Transformer](https://pyproj4.github.io/pyproj/stable/api/transformer.html) using the defined [CRS](https://pyproj4.github.io/pyproj/stable/api/crs/crs.html) objects, and transform the origin point so it can be subtracted later:
   ```python
   self.transformer = Transformer.from_crs(self.crs_wgs84, self.crs_utm)
   self.origin_x, self.origin_y = self.transformer.transform(utm_origin_lat, utm_origin_lon)
   ```

2. In `transform_coordinates`: Use `self.transformer` to transform `msg.latitude` and `msg.longitude` to UTM coordinates, then subtract the origin. Replace the previous printout with the transformed x, y values.

##### Validation
* `roslaunch autoware_mini_tutorial lesson2.launch`
* Output should be similar to this:

```
10.085402616416104 -649.6371095692739
9.99668661749456 -649.5454208320007
9.907705496239942 -649.4534196304157
...
```

## 3. Calculate orientation

Before we can publish the vehicle's pose, we need to determine its orientation. From the GNSS topic, we have the azimuth angle in degrees — the angle from north in clockwise direction (we will be ignoring the car's roll and pitch angles currently). We need to convert this to a [quaternion](https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation), which is how orientations are represented in ROS.

The conversion involves three steps:
1. **Correct for [meridian convergence](https://en.wikipedia.org/wiki/Transverse_Mercator_projection#Convergence)**: GNSS azimuth is in the WGS84 system; we must correct it for the UTM zone 35N projection. The correction can be found with the [`get_factors`](https://pyproj4.github.io/pyproj/stable/api/proj.html#pyproj.Proj.get_factors) method and should be subtracted from `msg.azimuth`.
2. **Convert azimuth to yaw**: Azimuth is a **clockwise (CW)** angle from the **y-axis** (north). In ROS, angles are **counterclockwise (CCW)** from the **x-axis** (east). The `convert_azimuth_to_yaw()` function in the node code handles this conversion. It takes corrected azimuth in **radians** as input.
3. **Convert yaw to quaternion**: Use `quaternion_from_euler(0, 0, yaw)` to get the quaternion components.

##### Instructions
Find `TODO 3` in `localizer.py` and implement the orientation calculation.

```python
azimuth_correction = self.utm_projection.get_factors(msg.longitude, msg.latitude).meridian_convergence
yaw = self.convert_azimuth_to_yaw(...)
x, y, z, w = quaternion_from_euler(0, 0, yaw)
orientation = Quaternion(x, y, z, w)
```

## 4. Publish `current_pose`

Now we can publish the transformed coordinates and orientation to a `current_pose` topic. The message type is [PoseStamped](https://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PoseStamped.html), which contains:
   * [Header](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Header.html):
      * `stamp` — timestamp of the message
      * `frame_id` — reference frame name. Use `"map"` here; reference frames will be explained more in future lessons.
   * [Pose](https://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Pose.html):
      * `position` ([Point](https://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Point.html)) — `x` and `y` are the transformed coordinates with subtracted origin. For `z`, take `msg.height` and subtract `self.undulation`. [Undulation](https://globalgpssystems.com/gnss/geoids-ellipsoids-and-undulation-height-understanding-the-basics/) corrects for differences in height measurement between the ellipsoid and geoid models of Earth's shape.
      * `orientation` ([Quaternion](https://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Quaternion.html)) — the quaternion calculated in the previous step.

##### Instructions
Find `TODO 4` in `localizer.py` and implement the following:
1. Create the PoseStamped() message
2. `stamp` should be taken from the INSPVA message header
3. `frame_id` should be `"map"`
4. Fill in position and orientation
5. Publish `current_pose`

```python
current_pose_msg = PoseStamped()
current_pose_msg.header.stamp = 
current_pose_msg.header.frame_id = 
current_pose_msg.pose.position.x = 
current_pose_msg.pose.position.y = 
current_pose_msg.pose.position.z = 
current_pose_msg.pose.orientation = 
self.current_pose_pub.publish(current_pose_msg)
```

##### Validation
* `roslaunch autoware_mini_tutorial lesson2.launch`
* `rostopic echo /localization/current_pose` — you should see messages similar to this:

```
header: 
  seq: 227
  stamp: 
    secs: 1685013676
    nsecs: 114465538
  frame_id: "map"
pose: 
  position: 
    x: -13.349363245186396
    y: -622.8975134063512
    z: 36.22197324112058
  orientation: 
    x: 0.0
    y: 0.0
    z: 0.9080059543110942
    w: 0.4189572614666073
```

## 5. Publish `current_velocity`

Velocity is represented in `/novatel/oem7/inspva` messages with three components: `north_velocity`, `east_velocity`, and `up_velocity`. We will use only the north and east components and take their norm.

Message type in `current_velocity` is [TwistStamped](https://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/TwistStamped.html), and we are going to write the calculated velocity to `twist.linear.x` and say that `header.frame_id` equals to `base_link`. `base_link` is a common way to name the robot's/car's main reference frame where the x axis points forward. That is why we calculate the norm and write it only to `twist.linear.x`.

##### Instructions
Find `TODO 5` in `localizer.py` and implement the following:
1. Calculate the velocity as the norm of the `north_velocity` and `east_velocity`
2. Create TwistStamped message
3. Fill in the message fields:
   * Set frame as `"base_link"` 
   * Take a time stamp from the GNSS message
   * Assign velocity to `twist.linear.x`
4. Publish to `current_velocity`

##### Validation
* `roslaunch autoware_mini_tutorial lesson2.launch`
* `rostopic echo /localization/current_velocity` — similar messages should appear:

```
header: 
  seq: 277
  stamp: 
    secs: 1685013677
    nsecs: 114484297
  frame_id: "base_link"
twist: 
  linear: 
    x: 9.374128787160322
    y: 0.0
    z: 0.0
  angular: 
    x: 0.0
    y: 0.0
    z: 0.0
```


## 6. Create and publish a transform

Lastly, our localizer must publish the transformation between the `map` frame and the `base_link` frame. This allows later conversion of map coordinates (e.g., traffic light locations) to vehicle local coordinates (e.g., in camera reference frame).

For that, you need to use [TransformBroadcaster() from tf2_ros](https://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20broadcaster%20%28Python%29) library. That is already imported and created. It uses message type: [geometry_msgs/TransformStamped](https://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/TransformStamped.html)

Find `TODO 6` in `localizer.py` and implement the following:

1. Create the TransformStamped message
   - `frame_id` is `map`
   - `child_frame_id` should be `base_link`
   - timestamp should come from the GNSS message
2. Transform consists of translation and rotation, which are the current_pose's position and orientation. Basically, to convert (0, 0) in `base_link` frame to `map` frame, you need to add the current position.
3. Use `sendTransform()` to publish the transform (TransformStamped message)


```
# create a transform message
t = TransformStamped()

# fill in the transform message - t

# publish transform
self.br.sendTransform(t)

```

##### Validation 
* `roslaunch autoware_mini_tutorial lesson2.launch`
* RViz should open with a similar visualization. Location information is replayed from the rosbag, your localization node transforms them into UTM Zone 35N coordinates, and also, waypoint recording is launched.

* The big red arrow in front is the location of `current_pose`
* Smaller arrows depict the recorded waypoints; the waypoint itself is at the beginning of the arrow, and the arrow shows the orientation
* The white number on top of the waypoints shows speed in km/h
* The color of the waypoint arrow shows the blinker
   - blue - right turn
   - green - straight
   - red - left turn

If you open the recorded .csv file in `autoware_mini/data/trajectories` folder, the contents should look like this:
```
x,y,z,yaw,velocity,change_flag,steering_flag,accel_flag,stop_flag,event_flag
10.085402616416104,-649.6371095692739,36.23543893620372,133.73629793722796,6.376735101833061,0,0,0,0,0
9.371052170288749,-648.895146233961,36.23321827016771,133.25623449464334,6.504391382059996,0,0,0,0,0
8.647575187613256,-648.1340144658461,36.23230783250183,132.79363659002348,6.628094637194675,0,0,0,0,0
7.916609529813286,-647.3523410074413,36.229860993847254,132.37560712127316,6.748367097827541,0,0,0,0,0
7.179045389697421,-646.5497807534412,36.229174598865214,131.99154370377778,6.869881292426453,0,0,0,0,0
...
```
* `x,y,z` - waypoint coordinates in map frame
* `yaw` - yaw angle in degrees (CCW from x-axis). A negative angle "turns the direction around"
* `velocity` - speed of the car
* `steering_flag` - blinker information
* Other columns are currently not used


## 7. Launch file arguments

If you open the launch file, you can see three arguments in the beginning
* `bag_file` - specify the bag file name
* `interval` - define spacing between waypoints
* `waypoints_file` - define output waypoints file name

These arguments can be specified when you run the launch file. The tab key is beneficial here because it auto-completes. Try to enter the same roslaunch command `roslaunch autoware_mini_tutorial lesson2.launch` by hitting the tab key at various steps.

Additionally, enter the tab key at the end and see the arguments appear. For example, try the command with arguments `roslaunch autoware_mini_tutorial lesson2.launch interval:=5 waypoints_file:=waypoints_5m.csv`
