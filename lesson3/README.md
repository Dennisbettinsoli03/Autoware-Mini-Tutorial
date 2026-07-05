[< Previous lesson](../lesson2/) -- [**Main Readme**](../README.md) -- [Next lesson >](../lesson4/)

# Lesson 3 - Controller

In this lesson, we will implement a simple path-following algorithm (controller) called Pure Pursuit. Path following is about staying on the path as accurately as possible while driving the car. We can control the vehicle through the steering angle (lateral control) and speed (longitudinal control). Your task is to implement the Pure Pursuit algorithm that produces the steering angles to keep the car on the path. Velocities will be taken from the recorded path (result of lesson 2).

Output from your node (steering angle and speed) will go into the `bicycle_simulation` node (reused from `autoware_mini`). It will calculate and update where the vehicle will be in the next time step and its speed and orientation based on its current position and commands from your controller node.

More about Pure Pursuit:
* [Three methods of lateral control](https://web.archive.org/web/20250601131849/https://www.shuffleai.blog/blog/Three_Methods_of_Vehicle_Lateral_Control.html)
* [Pure Pursuit](https://thomasfermi.github.io/Algorithms-for-Automated-Driving/Control/PurePursuit.html)

### Expected outcome
* Understanding of the general task of the vehicle's lateral control
* The ego vehicle can follow the recorded waypoints (path) from lesson 2. Theoretically, the same simple controller could be used with a real car, where low-level drivers will interpret provided longitudinal and lateral commands and control the vehicle accordingly.


## 1. Explore the follower node

Open [`lesson3/nodes/pure_pursuit_follower.py`](nodes/pure_pursuit_follower.py) — this is the starting skeleton for your path follower node. Take a moment to study its structure:

- The node is organized as a class `PurePursuitFollower`, similar to the localizer from lesson 2.
- The `__init__` method sets up internal variables, subscribers, and placeholders for the publisher and parameters you will add.
- Two subscribers are already created: `path` receives the loaded waypoints, and `/localization/current_pose` receives the ego vehicle's position from the simulator.
- A threading `Lock` is used for thread safety — all ROS 1 callbacks run in separate threads, so shared variables (`self.path_linestring`, `self.distance_to_velocity_interpolator`) must be protected. The lock usage in `path_callback` is already provided.

##### Instructions
Find `TODO 1` in `current_pose_callback` and add: `print(msg.pose.position.x, msg.pose.position.y)`.

##### Validation
* `roslaunch autoware_mini_tutorial lesson3.launch`
   - The launch file loads waypoints (uses default file name) created in lesson 2 and visualizes them in RViz (path with waypoints, blinker information, and speed values at the waypoints).
   - If you don't have a waypoint file, you can create one by running `roslaunch autoware_mini_tutorial lesson2.launch`. The assumption is that you have finalized your localizer node in lesson 2.
* The path should be visualized, and no error messages in the console.
* If you are printing out `current_pose`, the output should start with `x: 0.0, y: 0.0`. When placed near the path start using `2D Pose Estimate`, coordinates should be close to the following - `x: 10.0, y: -649.6`.

![loaded waypoints](images/load_practice_3.png)


## 2. Create a vehicle command publisher

After running the launch file, we saw waypoints loaded and visualized in RViz. We can call this array of waypoints a global path, which we want to follow with the car. The `pure_pursuit_follower` must send a vehicle command (contains speed and steering angle) that is picked up by the simulator (`bicycle_simulation`). The simulator calculates where the car will end up within the next time step given the current position, speed, orientation, and the vehicle command. After these calculations, the simulator gives us a new position, orientation, and speed. These updated values go again into the `pure_pursuit_follower` to recalculate a new steering angle and speed. So, there has to be a cycle between these nodes.

In this step, we will publish constant values to confirm that the ego vehicle will start to drive and that this cycle between the simulator and follower works.

##### Instructions
Find `TODO 2` in `pure_pursuit_follower.py` — it appears in two places:

1. In `__init__`: Create a publisher for the vehicle command topic `/control/vehicle_cmd` with message type `VehicleCommand`.

2. In `current_pose_callback`: Find the `TODO 2` block at the bottom (outside the if/else). Complete the vehicle command publishing code:

```python
vehicle_cmd = VehicleCommand()
vehicle_cmd.header.stamp = msg.header.stamp
vehicle_cmd.header.frame_id = "base_link"
vehicle_cmd.steering_angle = 0.2
vehicle_cmd.speed = 10
vehicle_cmd.acceleration = 0
self.vehicle_cmd_pub.publish(vehicle_cmd)
```

Acceleration limits how fast the simulator changes the current speed towards the commanded speed, and steering angle denotes maximum allowed steering angle.

##### Validation
* `roslaunch autoware_mini_tutorial lesson3.launch`
* Place a `2D Pose Estimate` close to the path, and the ego vehicle should drive in a circular pattern.
* Run `rostopic echo /control/vehicle_cmd` to verify what commands are actually published.
* Run [`rqt_graph`](http://wiki.ros.org/rqt_graph) (`Nodes_only` option selected) — you should see a cycle: `/control/pure_pursuit_follower -> /control/vehicle_cmd -> /vehicle/bicycle_simulation -> /localization/current_pose -> /pure_pursuit_follower`.

![node graph](images/rosgraph.png)


## 3. Implement lateral control I

We are publishing the vehicle command with constant values, and the ego vehicle reacts to these commands. Now we need to start calculating meaningful values. As the first step, we need to know where on the path the ego vehicle is. We will convert the path to a shapely [LineString](https://shapely.readthedocs.io/en/stable/reference/shapely.LineString.html) and use its [project](https://shapely.readthedocs.io/en/stable/reference/shapely.LineString.html#shapely.LineString.project) function to find the ego vehicle's distance from the path start.

The `project` function returns the distance along the path to the point closest to the ego vehicle. The [interpolate](https://shapely.readthedocs.io/en/stable/reference/shapely.LineString.html#shapely.LineString.interpolate) function (used in the next section) returns a point at a given distance along the path.

##### Instructions
Find `TODO 3` in `pure_pursuit_follower.py` — it appears in two places:

1. In `path_callback`: Convert waypoints to a shapely LineString and `prepare` it for efficient spatial queries:

```python
path_linestring = LineString([(w.position.x, w.position.y) for w in msg.waypoints])
prepare(path_linestring)
```

2. In `current_pose_callback`: Calculate the ego vehicle's distance from the path start:

```python
current_pose = Point([msg.pose.position.x, msg.pose.position.y])
d_ego_from_path_start = self.path_linestring.project(current_pose)
```

Remove the previous printout (TODO 1) and print `d_ego_from_path_start` instead.

Note: The `current_pose_callback` already checks if `self.path_linestring is None` — this prevents errors when the path hasn't been received yet.

##### Validation
* `roslaunch autoware_mini_tutorial lesson3.launch`
* Place `2D Pose Estimate` and see if the printed distance from the path start seems logical (should increase as you move along the path).

Examples of using `project`, `interpolate`, and `distance` from shapely:

```python
>>> from shapely import LineString, Point
>>> line = LineString([(0, 0), (10, 10)])
>>> point1 = Point(5, 5)
>>> point2 = Point(0, 5)

>>> line.project(point1)
7.0710678118654755
>>> line.interpolate(6)
<POINT (4.243 4.243)>
>>> shapely.distance(point1, point2)
5.0
```


## 4. Implement lateral control II

Now we will calculate the steering angle using the Pure Pursuit formula. A summary and code example can be found in [Three methods of lateral control](https://web.archive.org/web/20250601131849/https://www.shuffleai.blog/blog/Three_Methods_of_Vehicle_Lateral_Control.html).

The steering angle formula:

![pure_pursuit_formula](images/pure_pursuit_formula.png)

* δ - steering angle
* L - wheelbase
* α - the difference in car heading and lookahead point heading
* ld - lookahead point distance

The idea behind the formula: we pick a lookahead point on the path with the constant distance ahead of the car and steer so that the car would drive a circular arc that ends up in that point. The angle α tells how much the direction to the lookahead point differs from where the car is currently heading — the larger the α (or the shorter the lookahead distance ld), the sharper the arc, and the larger the required steering angle δ.

![pure_pursuit_image](images/pure_pursuit_img.png)

**Important Note:** On the image, you can see two terms, but here we use them differently:
* *Trajectory* — means the path we want to follow. In this and following lessons, we use the term **path** to represent what is meant by *trajectory* on the image.
* *Path* — can be interpreted as the **actual path** the ego vehicle takes. It will not align perfectly with the path we want to follow.

The main calculation happens in `current_pose_callback` because whenever we get a new position update, we immediately want to calculate a new steering angle.

A modified drawing with added heading angle and steering angle:

![pure_pursuit_image](images/pure_pursuit_additions.png)

##### Instructions
Find `TODO 4` in `pure_pursuit_follower.py` — it appears in two places:

1. In `__init__`: Read in parameter values:
   - `lookahead_distance` — comes from [`shared/config/control.yaml`](../shared/config/control.yaml). Used to find the lookahead point location on the path.
   - `wheel_base` — comes from `autoware_mini/config/vehicle.yaml`. Important parameter for the steering angle calculation.

```python
self.lookahead_distance = rospy.get_param("~lookahead_distance")
self.wheel_base = rospy.get_param("/vehicle/wheel_base")
```

2. In `current_pose_callback`: Calculate heading, lookahead point, and steering angle. Both headings are angles from the x-axis: the car's heading comes from its orientation quaternion, and the lookahead heading is the direction of the vector from the car to the lookahead point. Their difference is the α in the formula:

```python
# Get heading from current pose orientation
_, _, heading = euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])

# Calculate lookahead point on the path
lookahead_point = ...
# Calculate lookahead heading
lookahead_heading = ... 
# Recalculate the actual lookahead distance (direct distance between points)
ld = ...
# Calculate steering angle using the Pure Pursuit formula
steering_angle = ...
```

Use the calculated `steering_angle` in the vehicle command instead of the constant value.

##### Validation
* `roslaunch autoware_mini_tutorial lesson3.launch`
* Place the car close to the path. It should start following the path and make the correct turns.
* You can experiment by placing the start point at different distances and directions from the path.


## 5. Implement longitudinal control

For longitudinal control, we need to take the speed from the path. Each waypoint has a speed attribute, and the result should be the same speed profile as during the waypoint recording. When the path comes from the map (in later lessons), the waypoints will have maximum speed limit or reference speeds from the map data.

Current waypoints have 1m spacing, but with sparser waypoints there can be long distances between them. The solution is to create a function that does linear interpolation to get a speed value based on the ego vehicle's location on the path. We give it a distance, and it returns a speed.

##### Instructions
Find `TODO 5` in `pure_pursuit_follower.py` — it appears in two places:

1. In `path_callback`: Create the distance-to-velocity interpolator:

```python
# Collect waypoint x and y coordinates
waypoints_xy = np.array([(w.position.x, w.position.y) for w in msg.waypoints])
# Calculate cumulative distances between points
distances = np.cumsum(np.sqrt(np.sum(np.diff(waypoints_xy, axis=0)**2, axis=1)))
# Add 0 distance in the beginning
distances = np.insert(distances, 0, 0)
# Extract velocity values at waypoints
velocities = np.array([w.speed for w in msg.waypoints])
# Create interpolator (experiment with `bounds_error` and `fill_value` to create logical behavior when the ego vehicle is before the first waypoint or after the last one)
distance_to_velocity_interpolator = interp1d(distances, velocities, kind='linear', ...)
```

Assign `distance_to_velocity_interpolator` to the local variable that is then stored via the lock.

2. In `current_pose_callback`: Use the interpolator to get the velocity at the ego vehicle's position:

```python
linear_velocity = float(self.distance_to_velocity_interpolator(d_ego_from_path_start))
```

Replace the constant `linear_velocity = 0.0` in the `else` branch with this interpolated value, and use it in the vehicle command. Since the interpolator is now used in `current_pose_callback`, also add a check for `self.distance_to_velocity_interpolator is None` to the `if` statement at the top of the callback.

##### Validation
* `roslaunch autoware_mini_tutorial lesson3.launch`
* Place the ego vehicle at the start of the path (use `2D Pose Estimate`), and it should start following the path with the speed reflecting the moment of recording.
* If everything works without errors, clean the code (remove unnecessary debugging printouts) and commit to your repo.
* Try different lookahead distances by changing `lookahead_distance` in [`shared/config/control.yaml`](../shared/config/control.yaml). Is the behavior different? What is different and why? (These questions are for your own understanding.)