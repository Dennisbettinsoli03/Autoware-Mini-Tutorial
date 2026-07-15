[< Previous lesson](../lesson3/) -- [**Main Readme**](../README.md) -- [Next lesson >](../lesson5/)

# Lesson 4 - Global Planner

In this lesson, we will use a [Lanelet2](https://github.com/fzi-forschungszentrum-informatik/Lanelet2) map to plan routes. Your task is to write a global planner that finds the shortest route from the vehicle's current location to a goal point selected in RViz. The path will be converted to waypoints so that the `pure_pursuit_follower` from lesson 3 can follow it.

Instead of loading a pre-recorded waypoints file, the global planner creates the path dynamically from the map — you can select any on-road destination and the planner will find a route to it.

### Expected outcome
* Understanding of how a global planner uses a map to create a driveable path
* The car can follow a path created by the global planner (from the ego vehicle's current location to the selected destination on the map)


## 1. Explore the planner node

Open [`lesson4/nodes/lanelet2_global_planner.py`](nodes/lanelet2_global_planner.py) — this is the starting skeleton for your global planner node. Study its structure:

- The node is organized as a class `GlobalPlanner`.
- The `__init__` method loads the Lanelet2 map using a UTM projector and sets up subscribers and a publisher. You will add the traffic rules and routing graph in section 2.
- Two subscribers are already created: `/move_base_simple/goal` receives goal points from RViz (`2D Nav Goal` button), and `/localization/current_pose` receives the vehicle's position from the simulator.
- A `threading.Lock` protects shared variables (`self.current_location`, `self.goal_point`) — the same pattern you saw in lesson 3.
- The `publish_lane_from_waypoints_list` method is complete — it creates a `Path` message and publishes it on a latched topic.

##### Instructions
Find `TODO 1` in `goal_callback` and add a `rospy.loginfo` message that prints the goal coordinates. Note that the log includes node name to easily identify which node is logging the message. This format should be standard for all your log messages in this tutorial, however always be mindful of not logging too frequently in callbacks that trigger often (like `current_pose_callback`) to avoid spamming the console.

```python
rospy.loginfo("%s - goal position (%f, %f, %f) in %s frame", rospy.get_name(),
              msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
              msg.header.frame_id)
```

##### Validation
* `roslaunch autoware_mini_tutorial lesson4.launch`
   - The launch file loads the Lanelet2 map and starts the simulator, map visualizer, and your planner node.
   - Your `pure_pursuit_follower` from lesson 3 is also launched — it subscribes to the global path instead of reading static waypoints file, so it will follow whatever path your planner publishes.
* The map should be visualized in RViz, and no error messages in the console.
* Use `2D Nav Goal` (purple arrow button in RViz) to place a goal point. You should see your loginfo message in the console with the goal coordinates.


## 2. Find the route on the map

Now we will implement the routing logic. Given a current position and a goal point, we need to find the shortest route on the Lanelet2 map. Note that we want the route **without lane changes** — `getRoute` allows them by default, which would produce routes that jump between parallel lanes.

The key steps are:
1. Find which lanelet the vehicle is currently in (using [findNearest](https://github.com/fzi-forschungszentrum-informatik/Lanelet2/blob/master/lanelet2_python/python_api/geometry.cpp))
2. Find which lanelet the goal point is in
3. Find a route between them using the routing graph
4. Extract the shortest path without lane changes

For a more thorough overview of Lanelet2 routing, see the [routing documentation](https://github.com/fzi-forschungszentrum-informatik/Lanelet2/tree/master/lanelet2_routing) and [Python tutorial](https://github.com/fzi-forschungszentrum-informatik/Lanelet2/blob/master/lanelet2_examples/scripts/tutorial.py#L215).

##### Instructions
Find `TODO 2` — it appears in two places:

1. In `__init__`: Create the traffic rules and routing graph needed for route planning. The [Lanelet2 Python tutorial](https://github.com/fzi-forschungszentrum-informatik/Lanelet2/blob/master/lanelet2_examples/scripts/tutorial.py#L215) shows how these are created:

```python
traffic_rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                              lanelet2.traffic_rules.Participants.VehicleTaxi)
self.graph = lanelet2.routing.RoutingGraph(self.lanelet2_map, traffic_rules)
```

2. In `goal_callback`: Implement the routing logic:

```python
# Get the start lanelet; find the goal lanelet the same way using self.goal_point
start_lanelet = findNearest(self.lanelet2_map.laneletLayer, self.current_location, 1)[0][1]
goal_lanelet = ...

# Find route (the third argument is the routing cost id, the last argument disables lane changes)
route = self.graph.getRoute(start_lanelet, goal_lanelet, 0, False)
if route is None:
    rospy.logwarn("%s - No route found to goal position", rospy.get_name())
    return

# Find shortest path; check it for None with a logwarn the same way as above
path = route.shortestPath()
...

# Get path without lane changes
path_no_lane_change = path.getRemainingLane(start_lanelet)
```

`findNearest()` returns a list of `(distance, lanelet)` tuples — we take the first (closest) one with `[0][1]`.

##### Validation
* Add temporary `print(path_no_lane_change)` after the routing logic to verify the output.
* `roslaunch autoware_mini_tutorial lesson4.launch`
* Place a `2D Pose Estimate` to set the vehicle start location, then use `2D Nav Goal` to set a goal. You should see the printed lanelet sequence in the console.
* Try placing a goal where no route exists — you should see the warning message.


## 3. Convert lanelets to a path

We now have a route as a `LaneletSequence`, but the `pure_pursuit_follower` needs a `Path` message containing `Waypoint` objects. Each lanelet has a `centerline` (list of 3D points) that contain `speed_ref` (reference speed in km/h).

##### Instructions
Find `TODO 3` — it appears in two places:

1. In `convert_laneletseq_to_waypoints_list`: Convert the lanelet sequence to waypoints:

```python
for j, lanelet in enumerate(laneletseq):
    # Get speed from lanelet attribute or use global speed limit. The speed limit is in km/h, convert to m/s for the Waypoint message.
    speed = ...

    # Iterate through the centerline points and create waypoints. 
    for i, point in enumerate(lanelet.centerline):
        # Skip first point of every lanelet except the very first (endpoints overlap)
        if i == 0 and j != 0:
            continue
        waypoint = Waypoint()
        waypoint.position.x = point.x
        waypoint.position.y = point.y
        waypoint.position.z = point.z
        waypoint.speed = speed
        waypoints.append(waypoint)
```

2. In `goal_callback`: After the routing logic from section 2, call the conversion and publish methods

Key points:
- `speed_ref` is in **km/h** — convert to **m/s**
- Speed should not exceed `self.speed_limit` (also in km/h from the launch argument)
- The last point of each lanelet and the first point of the next lanelet are the same — skip the duplicate to avoid overlapping waypoints

##### Validation
* `roslaunch autoware_mini_tutorial lesson4.launch`
* Everything should run without errors. Remove any temporary print statements if necessary.
* Run `rqt_graph` (`Nodes only` option) — the nodes should be connected:

![node graph](images/rosgraph.png)

* Place a start and goal — the path should be visualized and the car should start following it.
* Try: `roslaunch autoware_mini_tutorial lesson4.launch speed_limit:=10` and verify speed is limited (echo `/localization/current_velocity`).
* Observe what happens when the ego vehicle reaches the end of the path.

![ego continues driving](images/ego_continues_driving.png)


## 4. Clear the path when goal is reached

The ego vehicle continues driving chaotically after reaching the path end. We need to publish an empty path (clearing the route) when the vehicle is close enough to the goal.

##### Instructions
Find `TODO 4` in `current_pose_callback`. Check the distance to the goal and publish the empty path when close enough.

The `distance_to_goal_limit` parameter comes from `shared/config/planning.yaml` (default 4.0m).

Note: Your `pure_pursuit_follower` from lesson 3 should already handle empty paths — when the path has fewer than 2 waypoints, it publishes stopping commands. The vehicle won't stop instantly because the `bicycle_simulation` has deceleration limits — stopping precisely at the goal will be solved by the local planner in later lessons.

##### Validation
* `roslaunch autoware_mini_tutorial lesson4.launch`
* Set the goal position and observe the car driving to the goal — you should see the "goal position reached" log message, the path should be cleared in RViz, and the vehicle should decelerate to a stop.


## 5. Sync path end with goal point

You may notice that the path end and the goal point are not aligned. The problem: `findNearest()` returns a full lanelet, so the path extends to the lanelet's endpoint — which may be far past where you clicked.

![path vs goal](images/path_vs_goal.png)

This matters because the distance check in section 4 uses `self.goal_point` (where you clicked), but the path may end somewhere else. The local planner (in later lessons) stops at the path end, not at the user's goal — so they must be in the same location.

##### Instructions
Find `TODO 5` in `convert_laneletseq_to_waypoints_list`. Implement a solution that ensures the last waypoint and `self.goal_point` are as closely aligned as possible.

Three approaches to consider:
1. Add the full lanelet to the path and use the last waypoint to overwrite `self.goal_point`. This is the simplest to implement but may cause the vehicle to stop past where you clicked.
2. Find the waypoint closest to `self.goal_point` in the last lanelet, truncate the list afterward, and update `self.goal_point` to match
3. Use shapely `project` and `interpolate` to create a new waypoint at the exact closest point on the path to the goal, then use it as both the path end and `self.goal_point`

##### Validation
* Test your solution by placing goals at various positions along a lanelet (beginning, middle, end).
* Verify that the path end and goal point are aligned — the car should stop near where you clicked.
* Clean the code (remove debugging printouts) and commit to your repo.