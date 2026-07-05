[**Main Readme**](../README.md) -- [Next lesson >](../lesson2/)

# Lesson 1 - Introduction to ROS

In this lesson, you will build a basic ROS (Robot Operating System) setup involving a publisher and subscriber node using Python. You will learn the fundamental concepts of ROS: creating nodes, publishers, subscribers, and launching nodes using launch files.

#### Expected outcome
* Understand the basics of ROS logic (publishers and subscribers)
* You can write your own simple ROS nodes
* You understand the simple class-based structure for ROS nodes
* You can run nodes using launch files


## 1. Implement Publisher

Write a simple publisher node that publishes "Hello World!" to a topic called `/message` with a message type of [std_msgs/String](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/String.html). The publisher should publish the string at a rate of 2Hz.

##### Instructions
1. Create a new file `publisher.py` in the `lesson1/nodes/` directory of your repository.
2. Copy the following code into it:

```python
#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

rospy.init_node('publisher')
rate = rospy.Rate(2)
pub = rospy.Publisher('/message', String, queue_size=10)

while not rospy.is_shutdown():
    pub.publish("Hello world!")
    rate.sleep()
```

3. Make the file executable: `chmod +x lesson1/nodes/publisher.py`

Line by line explanations:
   - `#!/usr/bin/env python3` - the "shebang" line tells the OS how to interpret the script
   - `rospy.init_node('publisher')` - [initializes the ROS node](https://wiki.ros.org/rospy/Overview/Initialization%20and%20Shutdown#Initializing_your_ROS_Node)
   - `rospy.Publisher(...)` - [creates a publisher](https://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers#Publishing_to_a_topic) on the `/message` topic
   - `rospy.Rate()` and `rate.sleep()` - sets the publish frequency; sleeps until it's time to publish again
   - `while not rospy.is_shutdown():` - keeps running until the node is stopped

##### Validation
* In terminal, run the publisher: `python3 lesson1/nodes/publisher.py`
   - You should see: `Unable to register with master node ... master may not be running yet.`
   - This is because the ROS master is not running yet.
* Open another terminal, run `roscore`
* Open another terminal, try these commands:
  - `rostopic list` - should show `/message` in the list
  - `rostopic info /message` - should show your node is publishing
  - `rostopic echo /message` - should display your published message (Ctrl+C to stop)
  - `rostopic hz /message` - should show ~2Hz (keep it running)


## 2. Implement Subscriber

The subscriber node should subscribe to the topic `/message` and print the received string to the console.

##### Instructions
1. Create a new file `subscriber.py` in the `lesson1/nodes/` directory.
2. Copy the following code:

```python
#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def callback(msg):
    print(msg.data)

rospy.init_node('subscriber')
rospy.Subscriber("/message", String, callback)
rospy.spin()
```

3. Make it executable: `chmod +x lesson1/nodes/subscriber.py`

Line by line explanations:
   - `rospy.Subscriber(...)` - [creates a subscriber](https://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers#Subscribing_to_a_topic) that calls `callback` whenever a message arrives
   - `rospy.spin()` - keeps the node alive, processing incoming messages

##### Validation
* Make sure roscore and the publisher are still running (or restart them).
* Open another terminal, run: `python3 lesson1/nodes/subscriber.py`
   - It should start printing "Hello World!" in the console.
* Open another terminal: `rostopic info /message`
   - The topic should now show one publisher and two subscribers (your subscriber node and the `rostopic hz` process if it is still running).


## 3. Convert the nodes to a class-based structure

The two nodes above are written as plain Python scripts. ROS nodes are almost always written as a class, because most nodes need to share state between callbacks, and storing that state on `self` is much cleaner than using global variables. All Autoware Mini nodes and all nodes in the following lessons use the same class-based structure:

```python
class NodeName:
    def __init__(self):
        # 1. Read parameters
        # 2. Initialize internal variables
        # 3. Create publishers
        # 4. Create subscribers
        ...

    def some_callback(self, msg):
        # callbacks are class methods, they can access shared state through self
        ...

    def run(self):
        # keeps the node running
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('node_name')
    node = NodeName()
    node.run()
```

Line by line explanations:
   - `__init__` sets everything up: parameters are read first, then internal variables, publishers and subscribers are created. Subscribers are typically created last — a callback can fire as soon as the subscriber exists, so everything the callback uses must already be in place.
   - Callbacks are methods, so they can read and write shared state through `self`.
   - `if __name__ == '__main__':` block only initializes the node, creates the class instance and calls `run()`.

##### Instructions
1. Rewrite `subscriber.py` using the class-based structure:

```python
#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

class Subscriber:
    def __init__(self):
        # Subscribers
        rospy.Subscriber('/message', String, self.message_callback)

    def message_callback(self, msg):
        print(msg.data)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('subscriber')
    node = Subscriber()
    node.run()
```

2. Rewrite `publisher.py` the same way yourself. Use the following skeleton and fill in the `...` parts — all the code you need is already in your script version:

```python
#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

class Publisher:
    def __init__(self):
        # Internal variables
        self.rate = ...

        # Publishers
        self.pub = ...

    def run(self):
        while not rospy.is_shutdown():
            ...

if __name__ == '__main__':
    ...
```

The published has no callbacks, so run() contains the publishing loop instead of rospy.spin()

##### Validation
* Run `roscore`, the publisher and the subscriber in three terminals exactly like before:
   ```bash
   python3 lesson1/nodes/publisher.py
   python3 lesson1/nodes/subscriber.py
   ```
* The behaviour should be unchanged: the subscriber prints "Hello world!" at 2Hz.

## 4. Build the workspace and run via roslaunch

Stop any running nodes and roscore from the previous sections (Ctrl+C in each terminal).

So far you have run the nodes as plain Python scripts using their file paths. ROS has its own build and run tools that find nodes by package name instead, so you don't need to care where a node's file is located. For that, the nodes must belong to a package inside a [catkin workspace](https://wiki.ros.org/catkin/conceptual_overview):

   - A **catkin workspace** (`~/autoware_mini_ws`) is a directory with a `src/` folder where packages live. Building the workspace creates two additional folders: `build/` for intermediate build files and `devel/` for the results, including the `devel/setup.bash` environment file.
   - A **package** is a directory inside `src/` containing a `package.xml` (metadata) and a `CMakeLists.txt` (build instructions). The `autoware_mini_tutorial` package is already set up in this repository with both files.
   - `catkin build` builds every package in the workspace. Python packages like ours don't need compiling and update on-fly, but building still registers the package so that ROS tools (`rosrun`, `roslaunch`, `catkin list`) can find it by name.

##### Instructions
1. Build the workspace:
   ```bash
   cd ~/autoware_mini_ws
   catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```
2. Source the workspace so ROS can find the newly built packages:
   ```bash
   source devel/setup.bash
   ```
   Tip: add this line to your `~/.bashrc` so it runs automatically in every new terminal:
   ```bash
   echo "source ~/autoware_mini_ws/devel/setup.bash" >> ~/.bashrc
   ```

##### Validation
* `catkin list` should include `autoware_mini_tutorial` in the output.
* You can now run nodes from anywhere using `rosrun` (requires roscore to be running in a separate terminal):
   ```bash
   rosrun autoware_mini_tutorial publisher.py
   rosrun autoware_mini_tutorial subscriber.py
   ```
* You can also launch both nodes at once using the provided launch file. Launch files are convenient way to bundle multiple nodes together. Run the following command (the launch file is located at `lesson1/launch/lesson1.launch`, however `roslaunch` can find it by package and actual file name):
   ```bash
   roslaunch autoware_mini_tutorial lesson1.launch
   ```
   "Hello world!" should be printed in the console. For more on launch file syntax, see [roslaunch/XML](https://wiki.ros.org/roslaunch/XML).


## 5. Parameters in launch files and nodes

What if you want to publish a different message instead of "Hello World!"? Instead of changing the code, you can make it a [parameter](https://wiki.ros.org/rospy_tutorials/Tutorials/Parameters) that is set in the launch file.

##### Instructions
1. Open `lesson1/launch/lesson1.launch` and add a parameter to the publisher node using the [param tag](https://wiki.ros.org/roslaunch/XML/param):
   ```xml
   <launch>
       <node pkg="autoware_mini_tutorial" name="publisher" type="publisher.py" output="screen" required="true" >
            <param name="message" value="Hello ROS!" />
       </node>

       <node pkg="autoware_mini_tutorial" name="subscriber" type="subscriber.py" output="screen" required="true"/>
   </launch>
   ```

2. In `publisher.py`, read the parameter from the ROS parameter server at the top of `__init__` and store it as an internal variable:
   ```python
   # Parameters
   self.message = rospy.get_param('~message', 'Hello World!')
   ```
   The `~` prefix means it's a private parameter (scoped to this node). The second argument is the default value.

3. Replace the hardcoded `"Hello world!"` in `self.pub.publish(...)` with `self.message`.

##### Validation
* `roslaunch autoware_mini_tutorial lesson1.launch`
* The new message "Hello ROS!" should be printed.


## 6. Reusing the node

ROS nodes can be reused — you can run several instances of the same node with different parameters. Let's create two publishers that are publishing to the same topic but with different rates and messages.

##### Instructions
1. Convert the rate to a parameter (just like you did with the message):
   - Add a `rate` param in the launch file
   - Read it with `rospy.get_param('~rate', 2.0)` in `__init__` and use it when creating `self.rate`.
2. In the launch file, duplicate the publisher node with:
   - Different `name` attributes (e.g. `publisher_1` and `publisher_2`) — you can't have two nodes with the same name
   - Different `rate` and `message` values (e.g. 2Hz and 10Hz)

##### Validation
* `roslaunch autoware_mini_tutorial lesson1.launch`
* The console output should show messages from both publishers at different rates.
* `rosnode list` - lists all running nodes
* `rostopic info /message` - shows both publishers and the subscriber
* Before pushing, open `package.xml` in the root of the repository and replace the `TODO` in the `<maintainer>` tag with your name and email.
* Push your changes to Github.