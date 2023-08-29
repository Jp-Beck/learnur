---
icon: material/alert-outline
---
## Functions
A function is declared as follows:

``` py title="sample.urscript" linenums="1"
def add(a, b):
    return a+b
end
```
???+ warning "indentation is important"

    The robots scripting language requires indentation of 4 spaces for it compiler 
    to correcltly understand that the command is withing the scope.
    Also, it is a must to enclose the functions with the ``` end ``` keyword.

The function can then be called like this:
``` py title="sample.urscript" linenums="4"
result = add(1,4) # returns 5
```
It is also possible to give function arguments default values:

``` py title="sample.urscript" linenums="1"
def add(a=0, b=0):
    return a+b
end
```

If default values are given in the declaration, arguments can be either input or skipped as below:
``` py title="sample.urscript" linenums="1"
result = add(0,0) # returns 0
result = add() # returns 0
```
A function, ```add```, is defined to take in two arguments, "a" and "b", and returns the sum of them. 
It can be invoked by passing in specific values as arguments, such as ```result = add(1, 4)```. The
function's arguments can also be given default values, allowing them to be omitted when the
function is called, as in ```result = add()```. 

???+ warning 

    It is important to ensure that the arguments are
    passed in the correct order as defined in the function's declaration, or the function may not work as
    intended.



For example, here is a function that takes in a list of numbers and an optional "factor" argument,
which defaults to 2:

``` py title="sample.urscript" linenums="1"
def multiply_list(numbers, factor=2):
    local result = []
    local i = 0
    while i < length(numbers):
      multiplication = numbers[i] * factor
      append(result, multiplication)
      i = i + 1
    end
    return result
  end
  
  local result = multiply_list([1,2,3])
  

```
and it will take the default value of factor = 2.

It is important to note that in URScript, variables declared within a function are not visible outside
of that function. This is known as variable scope.
URScript also has the ability to pass the variables by reference or by value. By default, URScript
passes variables by reference and it means if you change the variables inside the function, it will be
changed outside the function as well. However, if you would like to keep the value of the original
variable, you have to use the keyword "global" before the variable. The "global" keyword in
URScript is used to specify that a variable is a global variable, rather than a local variable within a
function. This allows you to access and modify the value of a global variable within a function.
Here's an example of how to use the "global" keyword:
``` py title="sample.urscript" linenums="1"
x = 5
def modify_global():
    global x
    x += 1
    print(x) # prints 5
    modify_global() #Call the function
    print(x) # prints 6
end
```
In this example, the variable x is defined as a global variable before the function modify_global() is
defined. Within the function, the "global x" statement is used to indicate that the variable x being
modified inside the function is the global variable, rather than a new local variable with the same
name. When the function is called, it increments the value of the global variable x by 1, and this
change is reflected when we print the value of x after the function call.
It's worth noting that in general, it is considered a best practice to avoid using global variables as
much as possible, as it can make your code harder to understand and reason about. Instead, it's often
better to pass the variables you need as arguments to functions or return them as a result of a
function and then reassign the value.
Also, the global keyword is not required if the variable is already defined in the global scope. You
can modify the value of the variable without any problem.

## 13. Module motion
### 13.1 Functions

#### 13.1.7 end_force_mode()
The end_force_mode() function in URScript is used to stop the robot from executing a force-
controlled move. It stops the robot from moving and holds it in its current position, in force mode.
This function is typically used when the robot is in force mode, which is a control mode that allows
the robot to apply a specific force to an object while moving.
When the robot is in force mode, it can apply a force to an object while moving, which is useful in
applications such as force-controlled assembly, deburring, and polishing. The robot's control system
uses the information from the force/torque sensors to control the force applied to the object, in this
mode.
``` py title="sample.urscript" linenums="1" hl_lines="5"
# Start force mode
force_mode_start()
# movements and actions which need to be performed in force mode
# Stop force mode
end_force_mode()
```

???+ note 

    In this example, the program starts force mode using the force_mode_start() function, then the
    robot move in force mode using the movel function and finally the force mode is ended using the
    end_force_mode() function. It's important to note that to use force mode, the robot needs to be
    equipped with force/torque sensors and these sensors need to be calibrated and properly configured.
    
#### 13.1.8 end_freedrive_mode()

The end_freedrive_mode() function in URScript is used to exit freedrive mode in a UR program.
Freedrive mode is a control mode where the robot's control system allows the user to manually
move the robot's joints while the robot's safety features are still active.

#### 13.1.9 end_screw_driving()

The end_screw_driving() function in URScript is used to exit screw driving mode in a UR program.
Screw driving mode is a specialized control mode that is used to perform screw driving tasks such
as tightening or loosening screws. It is a closed loop control mode that uses the information
provided by the robot's encoders to control the torque and position of the robot's joints.
Here's an example of how to use the end_screw_driving() function in a UR program:
``` py title="sample.urscript" linenums="1" hl_lines="5"
def main():
    # Enter screw driving mode
    screw_driving_start()
    # Perform screw driving task
    # Exit screw driving mode
    end_screw_driving()
end
```
#### 13.1.11 force_mode(task_frame, selection_vector, wrench, type, limits)

- The function is used to control the force/torque of the robot's end-effector.
    The function takes in five parameters:

    - task_frame: The name of the task frame that the force/torque should be applied to.
    - selection_vector: A 3-element vector that specifies which direction the force/torque should
    be applied in.
    - wrench: A 6-element vector that specifies the force and torque values to be applied.
    - type: A string that specifies the type of force mode. This can be "force" or "torque".
    - limits: A 6-element vector that specifies the limits for the force/torque values.

The ```force_mode()``` function is used to control the robot in "force control mode", where the robot's
end-effector is controlled by applying a specific force/torque, rather than by specifying a specific
position or velocity. This is useful in applications where the robot needs to interact with an object in
a precise and controlled manner, such as in assembly or packaging tasks.
```force_mode(task_frame, selection_vector, wrench, type, limits)``` is a function in UR Script, the
programming language used to control Universal Robots (UR) robotic arms. The function is used to
control the robot in force control mode, where the robot's end effector applies a specific force or
torque to the work piece.
Here is an example of how to use the function:
``` py title="sample.urscript" linenums="1" hl_lines="12"
# Define the task frame
task_frame = [0, 0, 0.1, 0, 0, 0]
# Define the selection vector
selection_vector = [1, 0, 0, 0, 0, 0]
# Define the wrench (force/torque)
wrench = [0, 0, -10, 0, 0, 0]
# Define the type of force control (for example, 'force')
type = 'force'
# Define the limits for the force/torque (for example, [-50, 50])
limits = [-50, 50]
# Apply force control
force_mode(task_frame, selection_vector, wrench, type, limits)
```
???+ note

    The above example sets the task frame to be 0.1 meters above the current position, and applies a
    force of -10 N in the Z-direction (downwards) to the workpiece. The selection vector defines which
    elements of the wrench vector are used. The limits parameter defines the minimum and maximum
    force/torque that can be applied. It's important to note that the force_mode function is a blocking
    function which means the program execution will be blocked until the force/torque limits are
    reached. It's recommended to use force_mode_tool(...) instead which is a non-blocking function.
#### 13.1.12 force_mode_example()

```force_mode(p[0.1,0,0,0,0.785], [1,0,0,0,0,0], [20,0,40,0,0,0], 2, [.2,.1,.1,.785,.785,1.57])```
Example Parameters:


- Task frame = p[0.1,0,0,0,0.785] ! This frame is offset from the base frame 100 mm in the x
direction and rotated 45 degrees in the RZ direction
- Selection Vector = [1,0,0,0,0,0] ! The robot is compliant in the x direction of the Task frame
above.
- Wrench = [20,0,40,0,0,0] ! The robot apples 20N in the x direction. It also accounts for a 40N
external force in the z direction.
- Type = 2 ! The force frame is not transformed.
- Limits = [.1,.1,.1,.785,.785,1.57] ! max x velocity is 100 mm/s, max y deviation is 100 mm,
max z deviation is 100 mm, max rx deviation is 45 deg, max ry deviation is 45 deg, max rz
deviation is 90 deg
#### 13.1.13 force_mode_set_damping(damping)

In UR Script, the "force_mode_set_damping" command sets the damping value for the force mode.
Damping is a value between 0 and 1 that determines how quickly the robot's end effector will stop
moving once it reaches its target. A damping value of 0 means the end effector will continue
moving indefinitely, while a value of 1 means it will stop immediately. The "damping" parameter in
the command is the value that you want to set the damping to. You can use this command in
conjunction with other force mode commands such as "force_mode_start" and "force_mode_stop"
to control the robot's end effector in a more precise and intuitive way.
```py title="sample.urscript" linenums="1" hl_lines="4"
# Start force mode
force_mode_start()
# Set the damping value to 0.7
force_mode_set_damping(0.7)
# Apply force to the end effector in the x-direction
set_tool_force(x=10)
# Stop force mode
force_mode_stop()
```
#### 13.1.14 force_mode_set_gain_scaling(scaling)

scaling parameter between 0 and 2, default is 1.
A value larger than 1 can make force mode unstable, e.g. in case of collisions or pushing against
hard surfaces. The value is stored until this function is called again. Add this to the beginning of
your program to ensure it is called before force mode is entered (otherwise default value will be
used).
```py title="sample.urscript" linenums="1" hl_lines="2"
force_mode_start()
force_mode_set_gain_scaling(0.5)
movej([0, -pi/2, pi/3, 0, pi/2, 0], a=1.2, v=1.05, r=0.1, t=0, ro=0)
force_mode_stop()
```
In this example, the script first starts the force mode with the "force_mode_start" command. Then,
it sets the gain scaling to 0.5 with the "force_mode_set_gain_scaling" command. The gain scaling is
a value between 0 and 1 that determines how sensitive the robot's end effector is to external forces.
A scaling value of 0 means the end effector is not sensitive at all, while a value of 1 means it is fully
sensitive. Then, it moves the robot's arm with the "movej" command. The "movej" command is
used to move the robot's end effector to a specific set of joint positions. Finally, the script stops the
force mode with the "force_mode_stop" command.
#### 13.1.15 freedrive_mode (freeAxes=[1, 1, 1, 1, 1, 1], feature=p[0, 0, 0, 0, 0, 0])

Set robot in freedrive mode. In this mode the robot can be moved around by hand in the same
way as by pressing the "freedrive" button.
The robot will not be able to follow a trajectory (eg. a movej) in this mode.
The default parameters enables the robot to move freely in all directions. It is possible to enable
Constrained Freedrive by providing user specific parameters.

**Parameters**

`freeAxes:` A 6 dimensional vector that contains 0’s and 1’s, these indicates in which axes
movement is allowed. The first three values represents the cartesian directions along x, y, z, and
the last three defines the rotation axis, rx, ry, rz. All relative to the selected feature.

`feature:` A pose vector that defines a freedrive frame relative to the base frame. For base and
tool reference frames predefined constants "base", and "tool" can be used in place of pose vectors.

**Example commands:**


- freedrive_mode()
Robot can move freely in all directions.
- freedrive_mode(freeAxes=[1,0,0,0,0,0], feature=p
[0.1,0,0,0,0.785])
- Example Parameters:
- feature = p[0.1,0,0,0,0.785] -> This feature is offset from the base
frame with 100 mm in the x direction and rotated 45 degrees in the rz
direction.
- freedrive_mode(freeAxes=[0,1,0,0,0,0], feature="tool")
- Example Parameters:
- freeAxes = [0,1,0,0,0,0] -> The robot is compliant in the y direction
relative to the "tool" feature.
- feature = "tool" -> The "tool" feature is located in the active TCP.

Note: Immediately before entering freedrive mode, avoid:
- movements in the non-compliant axes
- high acceleration in freedrive mode
- high deceleration in freedrive mode
High acceleration and deceleration can both decrease the control accuracy and cause protective
stops.

#### 13.1.17 get_freedrive_status()

Returns status of freedrive mode for current robot pose.
Constrained freedrive usability is reduced near singularities. Value returned by this function
corresponds to distance to the nearest singularity.
It can be used to advice operator to follow different path or switch to unconstrained freedrive.
Return Value

- 0 - Normal operation.
- 1 - Near singularity.
- 2 - Too close to singularity. High movement resistance in freedrive.
The above function can basically be achieved by below:
```py title="sample.urscript" linenums="1" hl_lines="7"
def main():
# Enable free drive mode
socket_send_string("set_freedrive_mode(True)")
# Wait for the robot to enter free drive mode
sleep(0.5)
# Get the current status of free drive modei
status = get_freedrive_status()
popup("Free drive mode status: ", status)
end
```
#### 13.1.18 get_target_tcp_pose_along_path()

Query the target TCP pose as given by the trajectory being followed.
This script function is useful in conjunction with conveyor tracking to know what the target pose of
the TCP would be if no offset was applied.

**Return Value**

Target TCP pose
```py title="sample.urscript" linenums="1" hl_lines="7"
movej(waypoint, a=1.2, v=0.2)
# Get the target TCP pose along the path
target_tcp_pose = get_target_tcp_pose_along_path()
print(target_tcp_pose)
```
???+ note

    Note that the get_target_tcp_pose_along_path() command should be called after a movement
    command like movej() to get the correct target pose along the path.
    The "get_target_tcp_pose_along_path()" function in UR Script is used to retrieve the target TCP
    (tool center point) pose of the robot along a pre-defined path. The benefit of using this function is
    that it allows you to easily and accurately determine the robot's target position and orientation at any
    point along a path, which can be useful for a variety of applications. For example, you could use
    this function to calculate the exact position and orientation of the robot's end effector at a specific
    point along a pick-and-place path, or to track the robot's progress along a path for monitoring and
    control purposes.

#### 13.1.20 movec(pose_via, pose_to, a=1.2, v=0.25, r =0, mode=0)

Move Circular: Move to position (circular in tool-space)
TCP moves on the circular arc segment from current pose, through pose_via to pose_to.
Accelerates to and moves with constant tool speed v. Use the mode parameter to define the
orientation interpolation.

**Parameters**

***pose_via:*** path point (note: only position is used). Pose_via can also be specified as joint
positions, then forward kinematics is used to calculate the corresponding pose.
pose_to: target pose (note: only position is used in Fixed orientation mode). Pose_to can also
be specified as joint positions, then forward kinematics is used to calculate the corresponding
pose.

***a:*** tool acceleration [m/s^2]
***v:*** tool speed [m/s]
***r:*** blend radius (of target pose) [m]

**mode:**

***0:*** Unconstrained mode. Interpolate orientation from current pose to target pose (pose_to)

***1:*** Fixed mode. Keep orientation constant relative to the tangent of the circular arc (starting from
current pose)

**Example command:**

```movec(p[x,y,z,0,0,0], pose_to, a=1.2, v=0.25, r=0.05, mode=1)```

- Example Parameters:
- Note: first position on circle is previous waypoint.
- pose_via = p[x,y,z,0,0,0] → second position on circle.
- ???+ note
     Rotations are not used so they can be left as zeros.
- ???+ note
     This position can also be represented as joint angles [j0,j1,j2,j3,j4,j5] then forward kinematics is used to calculate the corresponding pose
- pose_to → third (and final) position on circle
- a = 1.2 → acceleration is 1.2 m/s/s
- v = 0.25 → velocity is 250 mm/s
- r = 0 → blend radius (at pose_to) is 50 mm.
- mode = 1 → use fixed orientation relative to tangent of circular arc

#### 13.1.21 movej(q, a=1.4, v=1.05, t=0, r =0)

Move to position (linear in joint-space)
When using this command, the robot must be at a standstill or come from a movej or movel with a
blend. The speed and acceleration parameters control the trapezoid speed profile of the move.
Alternatively, the t parameter can be used to set the time for this move. Time setting has priority
over speed and acceleration settings.

**Parameters**

- q: joint positions (q can also be specified as a pose, then inverse kinematics is used to calculate
the corresponding joint positions)
- a: joint acceleration of leading axis [rad/s^2]
- v: joint speed of leading axis [rad/s]
- t: time [S]
- r: blend radius [m]

If a blend radius is set, the robot arm trajectory will be modified to avoid the robot stopping at the
point.

However, if the blend region of this move overlaps with the blend radius of previous or following
waypoints, this move will be skipped, and an ’Overlapping Blends’ warning message will be
generated.

**Example command:**

``` movej([0,1.57,-1.57,3.14,-1.57,1.57], a=1.4, v=1.05, t=0, r=0) ```

**Example Parameters:**

- q = [0,1.57,-1.57,3.14,-1.57,1.57] base is at 0 deg rotation, shoulder is at 90 deg
rotation, elbow is at -90 deg rotation, wrist 1 is at 180 deg rotation, wrist 2 is at -90
deg rotation, wrist 3 is at 90 deg rotation. Note: joint positions (q can also be
specified as a pose, then inverse kinematics is used to calculate the corresponding
joint positions)

- a = 1.4 → acceleration is 1.4 rad/s/s
- v = 1.05 → velocity is 1.05 rad/s
- t = 0 the time (seconds) to make move is not specified. If it were specified the
command would ignore the a and v values.
- r = 0 → the blend radius is zero meters.
#### 13.1.22 movel(pose, a=1.2, v=0.25, t=0, r=0)
Move to position (linear in tool-space)

#### 13.1.23 movep(pose, a=1.2, v=0.25, r=0)

Move Process
Blend circular (in tool-space) and move linear (in tool-space) to position. Accelerates to and
moves with constant tool speed v.

#### 13.1.24 path_offset_disable(a=20)

Disable the path offsetting and decelerate all joints to zero speed.
Uses the stopj functionality to bring all joints to a rest. Therefore, all joints will decelerate at
different rates but reach stand-still at the same time.
Use the script function path_offset_enable to enable path offsetting

**Parameters**

- a: joint acceleration [rad/s^2] (optional)

#### 13.1.25 path_offset_enable()

The ```path_offset_enable``` command in UR Script, allows the robot to follow a path with an offset.
This means that the robot will follow the path, but it will maintain a certain distance from the path.
The benefit of using this command is that it allows the robot to follow a path while avoiding
obstacles or other objects that may be in the way. 

For example, if a robot is working in a crowded
factory, it can use path offset to avoid collision with other machines or equipment. Additionally, it
can also be useful for tasks such as painting or welding where the robot needs to maintain a certain
distance from the surface it is working on.
Here's an example of how to use the path_offset_enable command in a UR program:
```py title="sample.urscript" linenums="1" hl_lines="5"
  path_offset_enable()
  movej(p[0.5, 0.5, 0.5, 0, 0, 0], a=1.2, v=0.5)
  path_offset_disable()
```

In this example, the ```path_offset_enable``` command is called before the robot moves to the target
position (specified in the ```movej``` command). 

This tells the robot to follow the path while
maintaining an offset. The ```movej``` command then moves the robot to the target position with a
specific acceleration and velocity. 

Once the robot reaches the target position, the
"path_offset_disable" command is called, which disables the offset mode, and the robot moves with
the path in a normal mode.

Note that the path_offset_enable() by default will offset the path with a distance of 0.05m. You can
change the distance by calling the command ```path_offset_enable(distance)``` with the desired distance
as parameter.

**Enable path offsetting.**

Path offsetting is used to superimpose a Cartesian offset onto the robot motion as it follows a
trajectory. This is useful for instance for imposing a weaving motion onto a welding task, or to
compensate for the effect of moving the base of the robot while following a trajectory.

Path offsets can be applied in various frames of reference and in various ways. Please refer to
the script function path_offset_set for further explanation.
Enabling path offsetting doesn’t cancel the effects of previous calls to the script functions 
```path_offset_set_max_offset```  and  ```path_offset_set_alpha_filter``` Path offset
configuration will persist through cycles of enable and disable.

???+ warning

    Using Path offset at the same time as Conveyor Tracking and/or Force can lead to program
    conflict. Do not use this function together with Conveyor Tracking and/or Force.

#### 13.1.26 path_offset_get(type)

The "path_offset_get" command in UR Script is used to retrieve the current offset value of a
specific type of path. The "type" parameter specifies the type of path for which you want to retrieve
the offset value.
The benefit of using this command is that it allows you to retrieve the current offset value of a
specific type of path, so you can use it in other commands or calculations within your program. For
example, you could use the offset value to modify the robot's end effector's position or orientation
during a movement, or use it to calculate the distance between the robot's current position and a
target position.
Additionally, path_offset_get command can be use to check the current offset of the path in force
mode and adjust the path accordingly. It can also be used to check the current offset of a path in
force mode and adjust the path accordingly. This can help to improve the accuracy of the robot's
movements and reduce the risk of collisions.
Here's an example of how to use the path_offset_get command in a UR program:

```py title="sample.urscript" linenums="1" hl_lines="5"
# Get the current offset value of the linear path
offset = path_offset_get("linear")
# Use the offset value to adjust the robot's end effector position
movej(get_inverse_kin(p[offset, 0, 0.1, 0, 0, 0]), a=1.2, v=0.5)
In this example, the "path_offset_get" command is used to retrieve the current offset value of the
linear path. The offset value is then stored in the variable "offset". Next, the offset value is used to
adjust the robot's end effector position in the movej command, by using it as an offset in the
get_inverse_kin function, which calculate the robot's joint angles based on the end effector position.
Another example,
# Get the current offset value of the force path
offset = path_offset_get("force")
# Use the offset value to check the current position in force mode
if offset < 0.01:
popup("The robot is at the desired position")
else:
popup("The robot is not at the desired position, offset is ", offset)
```

```py title="sample.urscript" linenums="1" hl_lines="5"
# Another example,
# Get the current offset value of the force path
offset = path_offset_get("force")
# Use the offset value to check the current position in force mode
if offset < 0.01:
popup("The robot is at the desired position")
else:
popup("The robot is not at the desired position, offset is ", offset)
```
#### 13.1.27 path_offset_set(offset, type)

Specify the Cartesian path offset to be applied.
Use the script function path_offset_enable beforehand to enable offsetting. The calculated offset
is applied during each cycle at 500Hz.
Discontinuous or jerky offsets are likely to cause protective stops. If offsets are not smooth the
function path_offset_set_alpha_filter can be used to engage a simple filter.
The following example uses a harmonic wave (cosine) to offset the position of the TCP along the
Z-axis of the robot base:

```py title="sample.urscript" linenums="1" hl_lines="5"
thread OffsetThread():
  while(True):
    # 2Hz cosine wave with an amplitude of 5mm
    global x = 0.005*(cos(p) - 1)
    global p = p + 4*3.14159/500
    path_offset_set([0,0,x,0,0,0], 1)
    sync()
  end
end
```

**Parameters**

offset: Pose specifying the translational and rotational offset.
type: Specifies how to apply the given offset. 

**Options are:**

1: (BASE) Use robot base coordinates when applying.

2: (TCP) Use robot TCP coordinates when applying.

3: (MOTION) Use a coordinate system following the un-offset trajectory when applying.

This coordinate system is defined as follows. X-axis along the tangent of the translational part of the
un-offset trajectory (rotation not relevant here). Y-axis perpendicular to the X-axis above and the
Z-axis of the tool (X cross Z). Z-axis given from the X and Y axes by observing the right-hand
rule. This is useful for instance for superimposing a weaving pattern onto the trajectory when
welding.

4: (WORLD) This can be used to follow a trajectory in world (inertial) space, while the base
coordinate system of the robot is being translated and/or rotated by something external, e.g. a
mobile robot or another robot arm. The offset is thus the pose of the robot base relative to the
world coordinate system, and it is also the world coordinate system which the commanded
trajectory should be understood relative to.
Another way to perform the above program is below:

```py title="sample.urscript" linenums="1" hl_lines="5"
# Define the amplitude and frequency of the cosine function
amplitude = 0.1
frequency = 0.1
# Set the initial time value
time = 0
while True:
  # Calculate the offset value using the cosine function
  offset = amplitude * cos(2 * math.pi * frequency * time)
  # Set the offset for the TCP along the Z-axis
  path_offset_set(offset, "z")
  # Update the time value
  time += 0.01
  # Sleep for a short period of time
  sleep(0.01)
```

#### 13.1.28 path_offset_set_alpha_filter(alpha)

Engage offset filtering using a simple alpha filter (EWMA) and set the filter coefficient.
When applying an offset, it must have a smooth velocity profile in order for the robot to be able to
follow the offset trajectory. This can potentially be cumbersome to obtain, not least as offset
application starts, unless filtering is applied.
The alpha filter is a very simple 1st order IIR filter using a weighted sum of the commanded offset
and the previously applied offset: ```filtered_offset= alpha*offset+ (1-alpha)*filtered_offset```

See more details and examples in the UR Support Site: Modify Robot Trajectory

**Parameters**

***alpha:*** The filter coefficient to be used - must be between 0 and 1.

A value of 1 is equivalent to no filtering.
For welding; experiments have shown that a value around 0.1 is a good compromise between
robustness and offsetting accuracy.
The necessary alpha value will depend on robot calibration, robot mounting, payload mass,
payload center of gravity, TCP offset, robot position in workspace, path offset rate of change and
underlying motion.

```py title="sample.urscript" linenums="1" hl_lines="5"
def main():
  # Set the alpha filter value to 0.2
  path_offset_set_alpha_filter(0.2)
  # Generate the offset path
  offset_path = path_offset(original_path, offset_distance)
  # Move the robot along the offset path
  movej(offset_path)
end
```
In this example, the alpha filter value is set to 0.2 before the offset path is generated. This means
that the path offset algorithm will apply a moderate level of filtering to the final path, resulting in a
smoother path for the robot to follow. 
The robot then moves along the offset path using the movej
command. 

???+ note 

    Note that the original_path variable should be the path that you want to offset and offset_distance is the distance you want the path to be offset by.

#### 13.1.29 path_offset_set_max_offset(transLimit, rotLimit)

The benefit of using path_offset_set_max_offset(transLimit, rotLimit) in a UR program is that it
allows for precise control over the maximum allowed deviation from the programmed path for the
robot's movement. The transLimit parameter sets the maximum allowed deviation in linear
movement (measured in millimeters), and the rotLimit parameter sets the maximum allowed
deviation in rotational movement (measured in degrees). By setting these limits, the robot can be
programmed to stay within a specific tolerance range for its movement, ensuring greater precision
and accuracy in tasks such as welding or assembly. This can also help to prevent collisions or other
potential hazards.
Set limits for the maximum allowed offset.
Due to safety and due to the finite reach of the robot, path offsetting limits the magnitude of the
offset to be applied. Use this function to adjust these limits. Per default limits of 0.1 meters and
30 degrees (0.52 radians) are used.

**Parameters**

***transLimit:*** The maximum allowed translational offset distance along any axis in meters.

***rotLimit:*** The maximum allowed rotational offset around any axis in radians.

#### 13.1.30 pause_on_error_code(code, argument)

Makes the robot pause if the specified error code occurs. The robot will only pause during
program execution.
This setting is reset when the program is stopped. Call the command again before/during
program execution to re-enable it.

```pause_on_error_code(173, 3)```
In the above example, the robot will pause on errors with code 173 if its argument equals 3
(corresponding to ’C173A3’ in the log).

```pause_on_error_code(173)```

In the above example, the robot will pause on error code 173 for any argument value.

**Parameters**


* code: The code of the error for which the robot should pause (int)
- argument: The argument of the error. If this parameter is omitted the robot will pause on any argument for the specified error code (int)

Notes: 

- This script command currently only supports error codes 173 and 174.
- Error codes appear in the log as CxAy where 'x' is the code and 'y' is the argument.

#### 13.1.31 position_deviation_warning(enabled, threshold=0.8)

When enabled, this function generates warning messages to the log when the robot deviates
from the target position. This function can be called at any point in the execution of a program. It
has no return value.

>position_deviation_warning(True)

**Parameters**

***enabled:*** (Boolean) Enable or disable position deviation log messages.

***threshold:*** (Float) Optional value in the range [0;1], 

where 0 is no position deviation and 1 is the
maximum position deviation (equivalent to the amount of position deviation that causes a
protective stop of the robot). If no threshold is specified by the user, a default value of 0.8 is used.

***Example command:*** position_deviation_warning(True, 0.8)

**Example Parameters:**

- Enabled = True → Logging of warning is turned on

- Threshold = 0.8 80% of deviation that causes a protective stop causes a warning to
be logged in the log history file.

#### 13.1.32 reset_revolution_counter(qNear=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

Reset the revolution counter, if no offset is specified. This is applied on joints which safety limits
are set to "Unlimited" and are only applied when new safety settings are applied with limitted joint
angles.

```reset_revolution_counter()```

**Parameters**

- qNear: Optional parameter, reset the revolution counter to one close to the given qNear joint

- vector. If not defined, the joint’s actual number of revolutions are used.

**Example command:**

```reset_revolution_counter(qNear=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])```

In the above example, the function has been enabled. This means that log messages will be

generated whenever a position deviation occurs. The optional "threshold" parameter can be
used to specify the level of position deviation that triggers a log message.

**Example Parameters:**

- qNear = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] -> Optional parameter, resets the revolution
counter of wrist 3 to zero on UR3 robots to the nearest zero location to joint rotations
represented by qNear.

#### 13.1.33 screw_driving(f, v_limit)

Enter screw driving mode. The robot will exert a force in the TCP Z-axis direction at limited
speed. This allows the robot to follow the screw during tightening/loosening operations.

**Parameters**

***f:*** The amount of force the robot will exert along the TCP Z-axis (Newtons).

***v_limit:*** Maximum TCP velocity along the Z axis (m/s).

Notes:

Zero the F/T sensor without the screw driver pushing against the screw.
Call end_screw_driving when the screw driving operation has completed.

```py title="sample.urscript" linenums="1" hl_lines="3"
def testScrewDriver():
  # Zero F/T sensor
  zero_ftsensor()
  sleep(0.02)

  # Move the robot to the tightening position
  # (i.e. just before contact with the screw)
  ...

  # Start following the screw while tightening
  screw_driving(5.0, 0.1)

  # Wait until screw driver reports OK or NOK
  ...

  # Exit screw driving mode
  end_screw_driving()
end
```

#### 13.1.34 servoc(pose, a=1.2, v=0.25, r=0)

Servo Circular

Servo to position (circular in tool space). Accelerates to, and moves with, constant tool speed v.

**Parameters**

***pose:*** target pose (pose can also be specified as joint positions, then
forward kinematics is used to calculate the corresponding pose)

***a:*** tool acceleration [m/s^2]

***v:*** tool speed [m/s]

***r:*** blend radius (of target pose) [m]

```servoc(p[0.2,0.3,0.5,0,0,3.14], a=1.2, v=0.25, r=0)```

Example Parameters:

```pose = p[0.2,0.3,0.5,0,0,3.14]``` → position in base frame of x
= 200 mm, y = 300 mm, z = 500 mm, rx = 0, ry = 0, rz = 180 deg.

- a = 1.2 → acceleration of 1.2 m/s^2
- v = 0.25 → velocity of 250 mm/s
- r = 0 → the blend radius at the target position is zero meters.

#### 13.1.35 servoj(q, a, v, t=0.002, lookahead_time=0.1, gain=300)

Servoj can be used for online realtime control of joint positions.
The gain parameter works the same way as the P-term of a PID controller, where it adjusts the
current position towards the desired (q). The higher the gain, the faster reaction the robot will
have. The parameter lookahead_time is used to project the current position forward in time with the
current velocity. A low value gives fast reaction, a high value prevents overshoot.
Note: A high gain or a short lookahead time may cause instability and vibrations. Especially if the
target positions are noisy or updated at a low frequency
It is preferred to call this function with a new setpoint (q) in each time step (thus the default
t=0.002)
You can combine with the script command get_inverse_kin() to perform servoing based on
cartesian positions:

``` q = get_inverse_kin(x)```

``` servoj(q, lookahead_time=0.05, gain=500)```

Here x is a pose variable with target cartesian positions, received over a socket or RTDE
registers.

**Example command: **

```servoj([0.0,1.57,-1.57,0,0,3.14], 0, 0, 0.002, 0.1, 300)```

**Example Parameters:**

- q = [0.0,1.57,-1.57,0,0,3.14] joint angles in radians representing rotations of base, shoulder, elbow, wrist1, wrist2 and wrist3
- a = 0 → not used in current version
- v = 0 → not used in current version
- t = 0.002 time where the command is controlling the robot. The function is blocking
- for time t [S].
- lookahead time = .1 time [S], range [0.03,0.2] smoothens the trajectory with this
- lookahead time


gain = 300 proportional gain for following target position, range [100,2000]

The servoj(q, a, v, t=0.002, lookahead_time=0.1, gain=300) function in a UR program is used to
control the motion of a robotic arm. The function takes in several parameters to control the motion
of the arm:

The benefit of using this function is that it allows for precise and smooth control of the robotic arm's
motion. The function uses a control loop with a proportional gain to track the desired position,
velocity, and acceleration. The lookahead time and sampling time parameters allow for the control
loop to react to changes in the desired setpoints and provide smooth and accurate motion. The gain
parameter allows for fine-tuning of the control loop's responsiveness. Overall, the servoj function
provides a powerful and flexible tool for controlling the motion of a robotic arm in a UR program.

#### 13.1.36 set_conveyor_tick_count(tick_count, absolute_encoder_resolution=0)
#### 13.1.37 set_pos(q)


#### 13.1.38 set_safety_mode_transition_hardness(type)

Set joint positions of simulated robot

***q:*** joint positions

**Example command:** 

```set_pos([0.0,1.57,-1.57,0,0,3.14])```

**Example Parameters:**
```py title="sample.urscript" linenums="1" hl_lines="8"
q = [0.0,1.57,-1.57,0,0,3.14] -> the position of the simulated robot with joint angles in
radians representing rotations of base, shoulder, elbow, wrist1, wrist2 and wrist3
# Set the safety mode transition hardness to "strict"
set_safety_mode_transition_hardness("strict")
# Perform some robot movements or actions
movej([0, 0, 0, 0, 0, 0], a=1.0, v=0.5)
# Set the safety mode transition hardness to "normal"
set_safety_mode_transition_hardness("normal")
# Perform some more robot movements or actions
movel([0, 0, 0, 0, 0, 0], a=1.0, v=0.5)
```
#### 13.1.39 speedj(qd, a, t)

Joint speed

Accelerate linearly in joint space and continue with constant joint speed. The time t is optional; if
provided the function will return after time t, regardless of the target speed has been reached. If
the time t is not provided, the function will return when the target speed is reached.

**Parameters**

- qd: joint speeds [rad/s]
- a: joint acceleration [rad/s^2] (of leading axis)
- t: time [s] before the function returns (optional)

**Example command:**

```speedj([0.2,0.3,0.1,0.05,0,0], 0.5, 0.5)```

**Example Parameters:**

- qd -> Joint speeds of: base=0.2 rad/s, shoulder=0.3 rad/s, elbow=0.1 rad/s,
- wrist1=0.05 rad/s, wrist2 and wrist3=0 rad/s
- a = 0.5 rad/s^2 -> acceleration of the leading axis (shoulder in this case)
- t = 0.5 s -> time before the function returns

```py title="sample.urscript" linenums="1" hl_lines="8"
# Define the target joint speed, acceleration, and time
qd = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6] # in rad/s
a = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5] # in rad/s^2
t = 2 # in seconds
# Move the robot to the target joint speed using speedj
speedj(qd, a, t)
```

Stop (linear in joint space)

Decelerate joint speeds to zero

**Parameters**

***a:*** joint acceleration [rad/s^2] (of leading axis)

Example command: stopj(2)

- Example Parameters:
- a = 2 rad/s^2 -> rate of deceleration of the leading axis.

## 14. Module Internals
### 14.1 Functions
#### 14.1.1 force()

Returns the force exerted at the TCP
Return the current externally exerted force at the TCP. The force is the norm of Fx, Fy, and Fz
calculated using get_tcp_force().
Return Value
The force in Newton (float)
???+ note


    Note: Refer to force_mode() for taring the sensor.

#### 14.1.2 get_actual_positions()

Returns the actual angular positions of all joints

The angular actual positions are expressed in radians and returned as a vector of length 6. Note
that the output might differ from the output of get_target_joint_positions(), especially
during acceleration and heavy loads.

**Return Value**

The current actual joint angular position vector in rad : [Base, Shoulder, Elbow, Wrist1, Wrist2, Wrist3]

```get_actual_joint_positions()``` returns the current actual joint positions of the robot arm in radians,
while get_actual_tcp() returns the current actual position and orientation of the tool center point
(TCP) of the robot arm in millimeters and radians.

```get_actual_joint_positions()``` is used to retrieve the current position of each joint in the robot arm,
while get_actual_tcp() is used to retrieve the current position and orientation of the end-effector
(TCP) of the robot arm. The former is useful for monitoring the position of each joint and ensuring
they are at the desired positions, while the latter is useful for monitoring the position and
orientation of the end-effector for tasks such as pick-and-place operations.

#### 14.1.3 get_actual_joint_positions_history(steps=0)
Returns the actual past angular positions of all joints

This function returns the angular positions as reported by the function ```get_actual_joint_
positions()``` which indicates the number of controller time steps occurring before the current
time step.
An exception is thrown if indexing goes beyond the buffer size.

**Parameters**

steps: The number of controller time steps required to go back. 0 corresponds to ```get_actual_joint_positions()```

**Return Value:**

The joint angular position vector in rad : [Base, Shoulder, Elbow, Wrist1, Wrist2, Wrist3] that was
actual at the provided number of steps before the current time step.

#### 14.1.4 get_actual_joint_speeds()
Returns the actual angular velocities of all joints
The angular actual velocities are expressed in radians. second and returned as a vector of
length 6. 
???+ 
  
    Note that the output might differ from the output of get_target_joint_speeds(),
    especially during acceleration and heavy loads.

**Return Value**

The current actual joint angular velocity vector in rad/s: [Base, Shoulder, Elbow, Wrist1, Wrist2,
Wrist3]
You can use the ```get_actual_joint_speeds()``` command in various scenarios, such as:
- Monitoring the robot's performance: You can use this command to check the joint speeds of
the robot in real-time and ensure that they are within safe and efficient operating ranges.

- Feedback control: You can use the joint speeds as feedback in a control algorithm to adjust
the robot's motion, for example, to achieve a specific trajectory or a specific position.

- Safety: You can use the joint speeds to detect and prevent potential collisions. For example,
if the joint speeds are too high, you can reduce the speed to avoid a collision.

- Debugging: You can use this command to debug issues with the robot's motion, such as
unexpected changes in joint speed that may indicate a problem with the robot's hardware or
software.

- Algorithms that require the knowledge of the actual joint speeds like visual servoing, force
control, motion planning, etc.

#### 14.1.5 get_actual_tcp_pose()

Returns the current measured tool pose
Returns the 6d pose representing the tool position and orientation specified in the base frame.
The calculation of this pose is based on the actual robot encoder readings.

**Return Value**

The current actual TCP vector [X, Y, Z, Rx, Ry, Rz]

#### 14.1.6 get_actual_tcp_speed()

Returns the current measured TCP speed
The speed of the TCP retuned in a pose structure. The first three values are the cartesian speeds
along x,y,z, and the last three define the current rotation axis, rx,ry,rz, and the length |rz,ry,rz|
defines the angular velocity in radians/s.

**Return Value**

The current actual TCP velocity vector [X, Y, Z, Rx, Ry, Rz]
#### 14.1.7 get_actual_tool_flange_pose()

Returns the current measured tool flange pose
Returns the 6d pose representing the tool flange position and orientation specified in the base
frame, without the Tool Center Point offset. The calculation of this pose is based on the actual
robot encoder readings.
**Return Value**

The current actual tool flange vector: [X, Y, Z, Rx, Ry, Rz]
??? note
  
    See get_actual_tcp_pose for the actual 6d pose including TCP offset.

The get_actual_tool_flange_pose() function in UR Script is used to retrieve the current flange pose
(position and orientation) of the robot's tool. This information can be used for various purposes,
such as:

- Verifying the current position of the tool for a given task or operation

- Comparing the current tool pose to a desired or target pose for accurate movement or control

- Recording the tool's position and orientation for later use or analysis

- Monitoring the tool's movement and detecting any deviations from the expected path or trajectory
  Overall, the get_actual_tool_flange_pose() function is a useful tool for monitoring and controlling
  the robot's tool position and orientation, which is essential for accurate and precise robotic
  automation tasks.

***These are high level examples which require XML-rpc as it contains now ursctipt commands***
```py title="sample.urscript" linenums="1" hl_lines="8"
1. Verifying tool position before starting a task:
# Define the desired tool position and orientation
desired_pose = [0, 0, 0.2, 0, 0, 0]
# Retrieve the current tool flange pose
actual_pose = get_actual_tool_flange_pose()
# Compare the actual and desired poses
if (desired_pose[0] == actual_pose[0]) and (desired_pose[1] == actual_pose[1]) and
(desired_pose[2] == actual_pose[2]) and (desired_pose[3] == actual_pose[3]) and
(desired_pose[4] == actual_pose[4]) and (desired_pose[5] == actual_pose[5]):
# Start the task
movej(...)
else:
# Print an error message and stop the program
popup("Error: Tool is not in the correct position.")
stopj(1)

```
```py title="sample.urscript" linenums="1" hl_lines="8"
2. Monitoring tool movement and detecting deviations: This does not work directly
# Define the desired tool path
desired_path = [[0, 0, 0.2, 0, 0, 0], [0, 0.1, 0.2, 0, 0, 0], [0, 0.2, 0.2, 0, 0, 0]]
# Initialize a variable to store the current tool path
actual_path = []
# Start the task
movej(...)
# Retrieve the current tool flange pose at regular intervals
while not get_inverse_kin()[-1]:
actual_path.append(get_actual_tool_flange_pose())
sleep(0.1)
# Compare the actual and desired paths
for i in range(len(desired_path)): #I may do an RPC call tp a python function.
if (desired_path[i][0] != actual_path[i][0]) or (desired_path[i][1] != actual_path[i][1]) or
(desired_path[i][2] != actual_path[i][2]) or (desired_path[i][3] != actual_path[i][3]) or
(desired_path[i][4] != actual_path[i][4]) or (desired_path[i][5] != actual_path[i][5]):
# Print an error message and stop the program
print("Error: Tool deviated from the desired path.")
stopj(...)
# Print a success message
popup("Task completed successfully.")
```

3.Recording tool position and orientation for later use:
```py title="sample.urscript" linenums="1" hl_lines="8"  
# Define the recording duration (in seconds)
duration = 60
# Initialize a variable to store the recorded tool poses
recorded_poses = []
# Start the recording
start_time = time()
while (time() - start_time) < duration:
recorded_poses.append(get_actual_tool_flange_pose())
sleep(0.1)
# Save the recorded poses to a file #I may do an RPC call tp a python function.
with open("recorded_poses.txt", "w") as f:
for pose in recorded_poses:
f.write(str(pose) + "\n")
# Print a success message
print("Recording completed successfully.")
```
#### 14.1.8 get_controller_temp()

Returns the temperature of the control box
The temperature of the robot control box in degrees Celcius.

**Return Value**

A temperature in degrees Celcius (float)
Example 1: Monitoring and Logging Controller Temperature
```py title="sample.urscript" linenums="1" hl_lines="8"  
# Define a variable to store the current temperature
temp = 0
# Continuously retrieve the controller temperature and log it to a file
while True:#I may do an RPC call tp a python function.
temp = get_controller_temp()
log_file.write("Current controller temperature: {} degrees Celsius\n".format(temp))
sleep(1)
```

In this example, the program continuously retrieves the current controller temperature and logs it to
a file every second. This can be useful for monitoring the temperature over time and detecting any
potential issues or overheating.


Example 2: Automatic Cooling System

```py title="sample.urscript" linenums="1" hl_lines="8"  
# Define the maximum temperature threshold
max_temp = 60
# Continuously check the controller temperature and activate the cooling system if necessary
while True:
temp = get_controller_temp()
if temp > max_temp:
activate_cooling_system()#I may do an RPC call to a python function.
sleep(1)
```

#### 14.1.9 get_forward_kin(q=’current_joint_positions’, tcp=’active_tcp’)

Calculate the forward kinematic transformation (joint space -> tool space) using the calibrated
robot kinematics. If no joint position vector is provided the current joint angles of the robot arm
will be used. If no tcp is provided the currently active tcp of the controller will be used.
Parameters

q: joint position vector (Optional)

tcp: tcp offset pose (Optional)

**Return Value**

tool pose

**Example command:** 
  - get_forward_kin([0.,3.14,1.57,.785,0,0], p[0,0,0.01,0,0,0])

**Example Parameters:**

- q = [0.,3.14,1.57,.785,0,0] -> joint angles of j0=0 deg, j1=180 deg, j2=90 deg, j3=45
- deg, j4=0 deg, j5=0 deg.
- tcp = p[0,0,0.01,0,0,0] -> tcp offset of x=0mm, y=0mm, z=10mm and rotation vector of rx=0 deg., ry=0 deg, rz=0 deg

## What is Kinematics?
Kinematics is the branch of mechanics that deals with the motion of objects without considering the
forces that cause the motion. In robotics, kinematics is the study of the movement and positioning
of robotic systems and their individual components, such as joints, links, and end effectors.

Kinematics in robotics involves the analysis and modeling of the robotic system's movement,
including the position, velocity, and acceleration of the robot's joints and links. It also includes the
study of the relationship between the robot's input variables, such as joint angles and velocities, and
its output variables, such as the position and orientation of the end effector.

Kinematics is a fundamental aspect of robotics, as it enables the design, control, and optimization of
robotic systems for a wide range of applications. It is used in the development of robotic systems
for manufacturing, automation, medical, and service applications. For example, in the case of a
robotic arm, the kinematics of the arm enable the calculation of the position and orientation of the
end effector based on the joint angles, which is essential for the robot to be able to complete its task.

Kinematics is closely related to other fields such as dynamics, control and planning, which are all
important in the development of robotic systems. The study of kinematics also helps to develop
mathematical models that can be used to control and program the robot's movements, which is
essential for accurate and precise automation tasks.

## What is forward kinematics?

Forward kinematics in Universal Robots is the process of determining the position and orientation
of the end effector (tool or gripper) of the robot based on the current joint angles and the robot's
kinematic model.

In Universal Robots, forward kinematics is typically used to calculate the robot's current flange (end
effector) position and orientation, which is essential for monitoring and controlling the robot's
movement. 

**This information can be used for various purposes, such as:**

- Verifying the current position of the robot's flange for a given task or operation

- Comparing the current flange pose to a desired or target pose for accurate movement or control

- Recording the robot's flange position and orientation for later use or analysis

- Monitoring the robot's flange movement and detecting any deviations from the expected
  path or trajectory

The forward kinematics is also used to calculate the inverse kinematics, which is the process of
determining the joint angles required to reach a desired flange position and orientation.
Universal Robots provides an API that enables developers to use the forward kinematics
calculations, the API is called URScript, and it is a script-based programming language that allows
you to control the robot and access the robot's kinematic information, for example, you can use the
function get_actual_tool_flange_pose() to get the flange pose of the robot.
Overall, forward kinematics plays a critical role in the programming and control of Universal
Robots, as it enables accurate and precise movement of the robot's end effector, which is essential
for various automation tasks.

The get_forward_kin() function in UR Script is used to calculate the forward kinematics of a
Universal Robots robotic arm. It can be used in several use scenarios such as:

- Position Control: The get_forward_kin() function can be used to calculate the robot's current
  flange (end effector) position and orientation based on the current joint angles. This
  information can be used to verify the robot's current position and ensure that it is in the
  correct location for a given task or operation.

- Trajectory Planning: The get_forward_kin() function can be used to calculate the robot's
  flange position and orientation at different points along a planned trajectory. This
  information can be used to ensure that the robot's movement is accurate and precise, and to
  detect and correct any deviations from the planned trajectory.

- Error Detection and Correction: The get_forward_kin() function can be used to calculate the
  robot's flange position and orientation based on the current joint angles and compare it to the
  desired or target position and orientation. Any discrepancies between the two can be used to
  detect and correct errors in the robot's movement or control.

- Robotic Vision: The get_forward_kin() function can be used to calculate the position and
  orientation of the robot's flange in relation to the camera or other sensors. This information
  can be used to accurately align the robot's end effector with objects in the camera's field of
  view.

- Safety Measures: The get_forward_kin() function can be used to calculate the position and
  orientation of the robot's flange in relation to the environment and other objects. This
  information can be used to ensure that the robot's movement does not pose a risk to people
  or other objects in its vicinity.\

Overall, the get_forward_kin() function is a powerful tool in UR Script that allows developers to
accurately and precisely control the robot's movement, verify its position and orientation, and
ensure the robot's safety.
```py title="sample.urscript" linenums="1" hl_lines="8"  
# Define the camera's position and orientation
camera_pose = [0, 0, 0.5, 0, 0, 0]
# Define the desired position and orientation of the robot's flange
desired_flange_pose = [0.5, 0.5, 0.5, 0, 0, 0]
# Retrieve the current joint angles of the robot
current_joint_angles = get_actual_joint_positions()
# Calculate the current flange position and orientation using forward kinematics
current_flange_pose = get_forward_kin(current_joint_angles)
# Compare the current flange position and orientation to the desired position and orientation

if current_flange_pose == desired_flange_pose:
  print("Flange is correctly aligned with the camera")
else:
  print("Flange is not correctly aligned with the camera. Adjusting joint angles...")
end
# Calculate the necessary adjustments to the joint angles
joint_angle_adjustments = calculate_joint_angle_adjustments(current_flange_pose,
desired_flange_pose)
# Move the robot to the desired flange position and orientation
movej(joint_angle_adjustments)
```

In this example, the program first defines the position and orientation of the camera in relation to
the robot's base frame (camera_pose). It then defines the desired position and orientation of the
robot's flange in relation to the camera (desired_flange_pose).

The program then retrieves the current joint angles of the robot and uses the get_forward_kin()
function to calculate the current flange position and orientation based on the current joint angles.
It then compares the current flange position and orientation to the desired position and orientation,
and if they do not match, the program calculates the necessary adjustments to the joint angles using
a separate function (calculate_joint_angle_adjustments()) and moves the robot to the desired flange
position and orientation using the movej() function.

This example shows how the get_forward_kin() function can be used to accurately align the robot's
flange with a camera or other sensor, allowing the robot to perform vision-based tasks such as
object detection and recognition.

```py title="sample.urscript" linenums="1" hl_lines="8"  
# Define the camera's position and orientation
camera_pose = [0,0,0,0,0,0]
# Continuously retrieve the current joint angles and calculate the flange pose
while True:
  joint_angles = get_actual_joint_positions()
  flange_pose = get_forward_kin(joint_angles)
  print("Current flange pose: ", flange_pose)
  # Calculate the difference between the camera and flange poses
  diff_pose = subtract_pose(camera_pose, flange_pose)
  print("Difference between camera and flange poses: ", diff_pose)
  # Check if the difference is within a certain tolerance
  if abs(diff_pose[0]) < 0.1 and abs(diff_pose[1]) < 0.1 and abs(diff_pose[2]) < 0.1:
    print("Robot's flange is correctly aligned with the camera")
  else:
    print("Robot's flange is not correctly aligned with the camera")
  sleep(0.5)
```
In this example, the program defines a variable "camera_pose" that represents the position and
orientation of a camera. The program then enters a loop that retrieves the current joint angles of the
robot using the get_actual_joint_positions() function. The program then uses the get_forward_kin()
function to calculate the current flange pose based on the joint angles.

The program then calculates the difference between the flange pose and the camera pose using the
subtract_pose() function. The program then checks if the difference is within a certain tolerance (in
this example, 0.1) in each of the x,y,z axis. If the difference is within the tolerance, the program
prints "Robot's flange is correctly aligned with the camera". Otherwise, it prints "Robot's flange is
not correctly aligned with the camera".

This example demonstrates how the get_forward_kin() function can be used to calculate the robot's
flange position and orientation in relation to the camera, allowing for accurate alignment of the
robot's end effector with objects in the camera's field of view.

It's important to note that this is a simplified example, in a real scenario, you will have to take into
consideration the position and orientation of the camera in relation to the robot, but also the
parameters of the camera such as the intrinsic parameters, the extrinsic parameters and the
distortion coefficients.

```py title="sample.urscript" linenums="1" hl_lines="8"  
# Define the safe zone around the robot
safe_zone = [x_min, x_max, y_min, y_max, z_min, z_max]
# Continuously check the robot's flange position
while True:
  flange_pose = get_forward_kin()
  x = flange_pose[0]
  y = flange_pose[1]
  z = flange_pose[2]
  # Check if the flange is within the safe zone
  if x < safe_zone[0] or x > safe_zone[1] or y < safe_zone[2] or y > safe_zone[3] or z < safe_zone[4] or z > safe_zone[5]:
    stop_robot()
    print("Robot's flange has left the safe zone. Movement stopped.")
  sleep(0.1)
```

Here is an example of how to use the get_forward_kin() function in UR Script for trajectory planning:
```py title="sample.urscript" linenums="1" hl_lines="8"  
# Define the desired joint angles for the trajectory
joint_angles = [0, 45, 90, 0, 0, 0]
# Calculate the flange pose for each joint angle
flange_poses = []
for angles in joint_angles:
  flange_pose = get_forward_kin(angles)
  flange_poses.append(flange_pose)
  # Move the robot along the trajectory
  movej(joint_angles, a=1, v=0.5)
  # Verify the robot's final flange pose
  final_flange_pose = get_forward_kin()
  if final_flange_pose == flange_poses[-1]:
    popup("Robot reached final flange pose as expected.")
  else:
    popup("Robot's final flange pose deviated from the expected.")

```

Here is an example of how to use the get_forward_kin() function for trajectory planning in UR Script:

```py title="sample.urscript" linenums="1" hl_lines="8"  
# Define a target position and orientation for the end effector
target_position = [1, 2, 3]
target_orientation = [0.7071, 0, 0.7071, 0]
# Define an array to store the planned trajectory points
trajectory_points = []
# Define the number of steps for the trajectory
num_steps = 50
# Define the current and next joint angles for the robot
current_joints = get_actual_joint_positions()
next_joints = current_joints
# Plan the trajectory by incrementally moving the robot's joints towards the target position
for i in range(num_steps):
  next_joints = [current_joints[i] + (target_position[i] - current_joints[i])/num_steps for i inrange(6)]
  movej(next_joints, a=1.2, v=0.5)
  # Append the current flange position and orientation to the trajectory points array
  flange_pose = get_forward_kin(next_joints)
  trajectory_points.append(flange_pose)
  # Print the planned trajectory points
  print(trajectory_points)
```

In this example, the program first defines a target position and orientation for the end effector. 
Next, it sets the current joint angles of the robot using the get_actual_joint_positions() function and the
next joint angles as a copy of the current joint angles. Then, the program plans the trajectory by
incrementally moving the robot's joints towards the target position over a specified number of steps.

At each step, it calls the movej() function to move the robot to the next joint angles, and it uses the
get_forward_kin() function to calculate the current flange position and orientation, which is then
appended to the trajectory_points array. Finally, the program prints the planned trajectory points.
This example shows how to use the get_forward_kin() function in conjunction with the robot's
current joint angles and the movej() function to plan and execute a precise trajectory.

It is important to note that this example is just a simple example, and in real-world scenarios,
trajectory planning is a complex task that may involve sophisticated algorithms and techniques to
ensure the accuracy, safety and efficiency of the robot's movement.

Here's an example of how to use the get_forward_kin() function for position control in UR Script:

```py title="sample.urscript" linenums="1" hl_lines="8"
# Define the target flange position and orientation
target_pos = [0.5, 0.2, 0.3] # x, y, z
target_rot = [0, 0, 1.57] # roll, pitch, yaw
# Continuously check the robot's current flange position and orientation
while True:
  current_pos, current_rot = get_forward_kin()
  # Compare the current flange position and orientation to the target
  pos_error = [target_pos[i] - current_pos[i] for i in range(3)]
  rot_error = [target_rot[i] - current_rot[i] for i in range(3)]
  # If the error is below a certain threshold, the robot has reached the target position
  if all(abs(error) < 0.01 for error in pos_error + rot_error):
    print("Target position reached.")
    break
  else:
    # Move the robot to reduce the error
    move_to_target(pos_error, rot_error)
```

#### 14.1.10 get_inverse_kin(x, qnear, maxPositionError =1e-10,maxOrientationError =1e-10,tcp=’active_tcp’)

What is Inverse Kinematics?

Inverse kinematics (IK) is a method used in robotics, computer graphics, and animation to find the
position and orientation of a robotic arm or a character's joints in order to reach a specific target or
goal. It is the inverse of forward kinematics, which calculates the end effector's position and
orientation based on the joints' angles. Inverse kinematics is used to calculate the angles of the
joints required to reach a desired position, which allows for more natural and realistic motion.

Inverse kinematics (IK) and forward kinematics (FK) are two methods used in robotics and
animation to calculate the position and orientation of a robotic arm or a character's joints.
Inverse kinematics (IK) is used to find the angles of the joints required to reach a desired position
and orientation of the end effector (e.g. the robotic arm's gripper or the character's hand). It is used
to calculate the motion of the joints that will result in a specific movement of the end effector.

On the other hand, forward kinematics (FK) is used to find the position and orientation of the end
effector based on the angles of the joints. It is used to calculate the position of the end effector based
on the known angles of the joints, and it is the inverse of the inverse kinematics.
In summary, Inverse kinematics is used to calculate the angles of the joints required to reach a
desired position and orientation, while forward kinematics is used to calculate the position and
orientation of the end effector based on the joints' angles.

Inverse kinematics (IK) and forward kinematics (FK) are used in robotic arm control for different
purposes and under different circumstances.

Inverse kinematics (IK) is used when the goal is to reach a specific target position and orientation of
the end effector, such as a robotic arm's gripper or a character's hand. This method calculates the
angles of the joints required to reach the desired position and orientation, allowing for natural and
realistic motion. IK is typically used when the robotic arm needs to interact with the environment or
perform tasks that require precise positioning, such as grasping or welding.

On the other hand, forward kinematics (FK) is used when the goal is to determine the position and
orientation of the end effector based on the angles of the joints. This method can be used to simulate
the movement of the robotic arm, to check for collisions with other objects, or to visualize the
robotic arm's motion for debugging purposes.


In summary, Inverse kinematics is used when the goal is to reach a specific position and orientation,
while forward kinematics is used to determine the position and orientation of the end effector based
on the angles of the joints and can be used for simulations, collision detection, and visualization.
Universal Robots (UR) is a popular brand of collaborative robots that uses inverse kinematics (IK)
to control the movement of its robotic arms. 

The specific method used by UR to calculate inverse
kinematics may vary depending on the specific model and version of the robot. However, in 
general, the following steps are commonly used:

The robot arm's forward kinematics are defined and modeled using a Denavit-Hartenberg (DH)
representation or another method. This allows the robot to calculate the position and orientation of
the end effector based on the angles of the joints.
The target position and orientation of the end effector are received as input from the user or from a
program.

The robot uses the inverse of the forward kinematics to calculate the angles of the joints required to
reach the target position and orientation. This is typically done using an iterative algorithm, such as
the Newton-Raphson method, which starts with an initial guess of the joint angles and then
repeatedly adjusts them until the end effector reaches the target position and orientation within a
certain tolerance.

Once the inverse kinematics solution is found, the robot sends commands to the motors to move the
joints to the calculated angles.
It's worth noting that some of the UR robots use the Modified DH representation which is a
variation of the DH representation, and it's a bit more efficient than the traditional DH
representation.

In summary, Universal Robots (UR) uses inverse kinematics (IK) to calculate the angles of the
joints required to reach a specific target position and orientation of the end effector. This is done by
using the inverse of the forward kinematics and iterative methods such as the Newton-Raphson
method.


Calculate the inverse kinematic transformation (tool space -> joint space). If qnear is defined, the
solution closest to qnear is returned.
Otherwise, the solution closest to the current joint positions is returned. If no tcp is provided the
currently active tcp of the controller is used.
Parameters

***x:*** tool pose

***qnear:*** list of joint positions (Optional)

***maxPositionError:*** the maximum allowed position error (Optional)

***tcp:*** tcp offset pose (Optional)

**Return Value**

joint positions

***Example command:*** ```get_inverse_kin(p[.1,.2,.2,0,3.14,0], [0.,3.14,1.57,.785,0,0])```

***Example Parameters:***

- x = p[.1,.2,.2,0,3.14,0] -> pose with position of x=100mm, y=200mm, z=200mm and

- rotation vector of rx=0 deg., ry=180 deg, rz=0 deg.

- qnear = [0.,3.14,1.57,.785,0,0] -> solution should be near to joint angles of j0=0 deg.

- j1=180 deg, j2=90 deg, j3=45 deg, j4=0 deg, j5=0 deg.

- maxPositionError is by default 1e-10 m

- maxOrientationError is by default 1e-10 rad

#### 14.1.11 get_inverse_kin_has_solution(pose, qnear, maxPositionError=1E-10,maxOrientationError=1e-10, tcp="active_tcp")

Check if get_inverse_kin has a solution and return boolean (True) or (False).
This can be used to avoid the runtime exception of get_inverse_kin when no solution exists.

**Parameters**

***pose:*** tool pose

***qnear:*** list of joint positions (Optional)

***maxPositionError:*** the maximum allowed position error (Optional)

***maxOrientationError:*** the maximum allowed orientation error (Optional)

***tcp:*** tcp offset pose (Optional)

**Return Value**

True if ```get_inverse_kin has a solution```, False otherwise (bool)
The ```get_inverse_kin_has_solution()``` function in URScript is a variation of the ```get_inverse_kin()```
function, which is used to check if a given pose (position and orientation) of the end effector is
reachable by the robotic arm, and whether it has a solution within a given tolerance.
Here are some examples of the use cases of the ```et_inverse_kin_has_solution()``` function in
URScript:

- Reachability check: The function can be used to check if a specific target position and
  orientation is reachable by the robotic arm without hitting any obstacles or reaching any
  limits.

- Redundant solutions: If a robotic arm has more than one solution for a specific target
  position and orientation, this function can be used to check for redundant solutions.


- Error handling: The function can be used to check if there is an error in the robotic arm's
  position or orientation or if the desired position and orientation is not reachable.

- Tolerance check: The function allows to set a tolerance for position and orientation error,
  which can be used to check if the robotic arm is able to reach the target position and
  orientation within the desired precision.

- Tcp selection: The function allows to specify the tcp (tool center point) to be used for the
  inverse kinematics calculation, which can be useful for different types of end-effectors.
  These are some examples of use cases of the get_inverse_kin_has_solution() function in URScript,
  but it can be used in many other applications where inverse kinematics is needed and there is a need
  to check for a solution within a given tolerance.

**Reachability check:**
```py title="sample.urscript" linenums="1" hl_lines="8"
pose = [0.5, 0.2, 0.3, 0, 0, 0]

qnear = [0, 0, 0, 0, 0, 0]

maxPositionError = 1E-10
maxOrientationError = 1e-10
tcp = "active_tcp"
if get_inverse_kin_has_solution(pose, qnear, maxPositionError, maxOrientationError, tcp):
  print("The desired pose is reachable.")
else:
  print("The desired pose is not reachable.")
```
Path planning: In this example, a robotic arm is required to follow a specific path while avoiding
obstacles. The get_inverse_kin_has_solution() function can be used to check if the robotic arm can
reach each point along the path without hitting any obstacles.

```py title="sample.urscript" linenums="1" hl_lines="8"
path = [[0.5, 0.3, 0.2, 0, 0, 1], [0.6, 0.4, 0.2, 0, 0, 1], [0.7, 0.5, 0.2, 0, 0, 1]]
for pose in path:
# Check if the target pose is reachable without hitting obstacles

if get_inverse_kin_has_solution(pose,get_actual_tcp_pose()):
# Move the robotic arm to the target pose
  movej(pose)
else:
  print("Target pose is not reachable without hitting obstacles")
```
#### 14.1.12 get_joint_temp(j)
Returns the temperature of joint j

The temperature of the joint house of joint j, counting from zero. j=0 is the base joint, and j=5 is
the last joint before the tool flange.

**Parameters**

j: The joint number (int)
Return Value
A temperature in degrees Celcius (float)

#### 14.1.13 get_joint_torques()

Returns the torques of all joints
The torque on the joints, corrected by the torque needed to move the robot itself (gravity, friction,
etc.), returned as a vector of length 6.

**Return Value**

The joint torque vector in Nm: [Base, Shoulder, Elbow, Wrist1, Wrist2, Wrist3]

Get the current torque values of the joints
```py title="sample.urscript" linenums="1" hl_lines="8"
torques = get_joint_torques()

Print the torque values for each joint
for i in range(len(torques)):
print("Joint", i+1, "torque:", torques[i])
```

The get_joint_torques() function in URScript is used to get the torque values of the joints of a UR
robotic arm. This function can be useful in various applications such as:

- Monitoring the robotic arm's performance: The torque values can be used to monitor the
  robotic arm's performance and detect any potential issues or malfunctions.


- Identifying the cause of errors: The torque values can be used to identify the cause of errors,
  such as when the arm is unable to move to a specific position or when it experiences a
  sudden stop.

- Controlling the robotic arm: The torque values can be used to control the robotic arm, by
  providing feedback to the controller and adjusting the control parameters accordingly.

- Safety: The torque values can be used to ensure the safety of the robotic arm and its
  environment, by detecting and preventing unsafe conditions.

- Predictive maintenance: The torque values can be used for predictive maintenance, by
  monitoring the wear and tear of the robotic arm's joints and scheduling maintenance before
  any critical failures occur.

- Adaptive control: The torque values can be used for adaptive control, by continuously
  monitoring the robotic arm's torque and adjusting the control parameters accordingly to
  improve the robotic arm's performance.


Monitoring the robotic arm's performance: 
In this example, the torque values of the robotic arm's joints are read and printed to the console every second.

```py title="sample.urscript" linenums="1" hl_lines="8"
while True:
torques = get_joint_torques()
print("Joint torques: ", torques)
sleep(1)
Identifying the cause of errors: In this example, the torque values of the robotic arm's joints are read
and compared to a threshold value. If the torque exceeds the threshold, an error message is printed.
threshold = [20, 20, 20, 20, 20, 20]
while True:
torques = get_joint_torques()
for i in range(6):
if abs(torques[i]) > threshold[i]:
print("Error: Joint", i+1, "has exceeded the torque threshold")
sleep(1)
Controlling the robotic arm: In this example, the torque values of the robotic arm's joints are read
and used to adjust the control parameters.
5.12UR Script Usage Page: 103/169
Kp = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
Ki = [0.01, 0.01, 0.01, 0.01, 0.01, 0.01]
while True:
torques = get_joint_torques()
for i in range(6):
control_parameter = Kp[i]*torques[i] + Ki[i]*integral_error[i]
sleep(1)

```
**Safety:**
  In this example, the torque values of the robotic arm's joints are read and compared to a
threshold value. If the torque exceeds the threshold, the robotic arm is stopped to prevent any unsafe
conditions.

```py title="sample.urscript" linenums="1" hl_lines="8"
threshold = [20, 20, 20, 20, 20, 20]
while True:
torques = get_joint_torques()
for i in range(6):
if abs(torques[i]) > threshold[i]:
stopj(0)
print("Robotic arm stopped due to high torque on joint", i+1)
sleep(1)

```

**Predictive maintenance:**
 In this example, the torque values of the robotic arm's joints are read and
logged to a file. The data is analyzed to detect any patterns or trends that may indicate wear and tear
on the joints, so maintenance can be scheduled before any critical failures occur.

```py title="sample.urscript" linenums="1" hl_lines="8"
while True:
  torques = get_joint_torques()
  textmsg("torques:", torques)
  sleep(60)
```

#### 14.1.14 get_steptime()

Returns the duration of the robot time step in seconds.
In every time step, the robot controller will receive measured joint positions and velocities from
the robot, and send desired joint positions and velocities back to the robot. This happens with a
predetermined frequency, in regular intervals. This interval length is the robot time step.

**Return Value**

duration of the robot step in seconds
The get_steptime() function in URScript is used to get the current steptime value of a UR robotic
arm. The steptime is the time it takes for the robotic arm to execute one control cycle. It is typically
set as the time it takes for the control system to receive new sensor data, process it, and produce
new control signals to the motors. A lower steptime value means that the control system is running
at a faster rate and can respond more quickly to changes in the environment, but it also requires
more computational resources.

#### 14.1.15 get_target_joint_positions()

In UR Script, get_joint_position() and get_target_joint_position() are both used to retrieve the
current or target joint positions of a UR robot, but they return slightly different information.
get_joint_position() returns the current joint positions of the robot, which are the positions that the
robot is currently at. These positions are measured by the robot's encoders and updated at a high
frequency (typically several times per second).

get_target_joint_position() returns the target joint positions of the robot, which are the positions that
the robot is currently moving towards. These positions are typically set by a higher-level controller
or program, and may be different from the current joint positions if the robot is in motion.
In short, get_joint_position() gives you the current position of the robot, while
get_target_joint_position() gives you the position that the robot is moving towards.

In UR Script, the get_target_joint_positions() function is used to retrieve the target joint positions of
a UR robot. This can be useful in a variety of ways, such as:

- Monitoring the robot's motion: By periodically reading the target joint positions, a program can
  track the robot's motion and ensure that it is moving as expected.

- Trajectory tracking: By comparing the target joint positions to a desired trajectory, a program can
  implement closed-loop control to keep the robot on track.

- Safety monitoring: By monitoring the target joint positions, a program can detect when the robot is
  about to reach a limit or a singularity, and take appropriate action to prevent damage or injury.

- Debugging: By reading the target joint positions, a program can diagnose problems with the robot's
  motion, and help identify the cause of errors or unexpected behavior.


```py title="sample.urscript" linenums="1" hl_lines="8"
movej([1.57, -1.57, 1.57, -1.57, 1.57, 0], a=1.2, v=1.05)
joint_target_positions = get_target_joint_position()
popup(joint_target_positions)

```
This will move the robot to the target joint positions [1.57, -1.57, 1.57, -1.57, 1.57, 0] with an
acceleration of 1.2 and a velocity of 1.05, and then retrieve the target joint positions and print them
to the console.

It is important to note that these examples are simplified and in real-world scenarios, you would
need to handle error cases, check if the robot is safe to move and check if the robot has reach the
target positions before retrieving the target positions.

#### 14.1.16 get_target_joint_speeds()

Returns the desired angular velocities of all joints
The angular target velocities are expressed in radians pr. second and returned as a vector of
length 6. Note that the output might differ from the output of get_actual_joint_speeds(),
especially during acceleration and heavy loads.

Return Value

The current target joint angular velocity vector in rad/s: [Base, Shoulder, Elbow, Wrist1, Wrist2,
Wrist3]

#### 14.1.17 get_target_payload()

Returns the weight of the active payload
Return Value
The weight of the current payload in kilograms

#### 14.1.18 get_target_payload_cog()

Retrieve the Center Of Gravity (COG) coordinates of the active payload.
This scripts returns the COG coordinates of the active payload, with respect to the tool flange
Return Value

The 3d coordinates of the COG [CoGx, CoGy, CoGz] in meters

In UR Script, the get_target_payload_cog() function is used to retrieve the target
payload center of gravity (CoG) of a UR robot. The payload is the object that the robot is carrying
or manipulating. The CoG is the point in the object where its weight is evenly distributed.

Here are some possible use cases for get_target_payload_cog():

1. Inverse Kinematics: The target payload CoG can be used as an input for the inverse
  kinematics algorithm to calculate the required target joint positions for the robot to reach a
  certain position of the object.

2. Collision Detection: By monitoring the target payload CoG, a program can detect when the
  robot is about to collide with an obstacle or another object, and take appropriate action to
  avoid the collision.

3. Dynamic Balancing: By monitoring the target payload CoG, a program can implement
  dynamic balancing control to keep the object stable while it is being manipulated.

4. Force Control: By monitoring the target payload CoG, a program can implement force
  control to apply a force to a specific point on the object.

5. Debugging: By reading the target payload CoG, a program can diagnose problems with the
  robot's motion, and help identify the cause of errors or unexpected behavior.
  It is important to note that in order to use this function, you need to have an accurate payload
  model, as well as the mass and the offset of the payload with respect to the robot flange.

Dynamic balancing is a control strategy used to keep an object stable while it is being manipulated
by a robot. The key idea behind dynamic balancing is to continuously adjust the robot's motion to
counterbalance the forces acting on the object, such as gravity, friction, and external disturbances.

To achieve dynamic balancing, the robot must have accurate information about the object's center
of gravity (CoG) and mass. This information can be obtained using a sensor, such as a load cell, or
by measuring the object's dimensions and calculating its CoG.

Once the object's CoG and mass are known, the robot can use this information to calculate the
forces acting on the object. The robot can then adjust its motion in real-time to counterbalance
these forces and keep the object stable.

For example, if the CoG of the object is located at the center, the robot can move the object in such
a way that the CoG always remains at the same point, effectively canceling out the effect of
gravity. In contrast, if the CoG is offset, the robot needs to compensate for that offset.
Dynamic balancing is particularly useful in tasks where the robot needs to manipulate objects with
varying shapes and sizes, or in environments with disturbances that could affect the stability of the
object.

It's worth noting that dynamic balancing is a complex task that requires precise control of the
robot's motion and a good understanding of the physics of the problem. It is also important to have
a good estimate of the object's mass and CoG which is why measuring them accurately is
important.

A load cell is a transducer, or sensor, that converts a force applied to it into an electrical signal.
Load cells are commonly used in industrial, research, and medical applications to measure weight,
force, or pressure. They are typically used in conjunction with a scale, force gauge, or other
measuring device to provide an accurate measurement of the applied force.

There are many different types of load cells available, each designed to measure specific types of
forces and/or in specific environments. Some common types of load cells include:

- Beam load cells: These load cells are designed to measure compression or tension forces.
  They consist of a small beam that deflects under load, and strain gauges that are attached to
  the beam to measure the deflection.

- S-type load cells: These load cells are also designed to measure compression or tension
  forces, but they have a slightly different design than beam load cells. They consist of a bent
  metal rod or "S" shape that deflects under load, and strain gauges that are attached to the
  rod to measure the deflection.

- Single-point load cells: These load cells are designed to measure compression forces only.
  They consist of a single point of contact between the load and the sensor.

- Compression load cells: These load cells are designed to measure compression forces only.
  They are often used in industrial and medical applications to measure the weight of an
  object.


- Tension load cells: These load cells are designed to measure tension forces only. They are
  often used in industrial and research applications to measure the force of a pulling or
  stretching action.

- Canister load cells: These load cells are designed to measure both compression and tension
  forces. They are often used in industrial and research applications to measure the force of
  both pushing and pulling actions.

It's worth noting that load cells are also available in different capacities, ranges and environments
such as waterproof, high temperature, and high accuracy.

In summary, load cells are devices that convert a force into an electrical signal, allowing for
accurate measurement of weight, force, or pressure. Different types of load cells are available for
specific types of forces and environments.

Universal Robots (UR) robots use a six-axis force-torque sensor (FTS) that is mounted at the
flange of the robot. This sensor measures the forces and torques acting on the robot in all six
degrees of freedom (x, y, z, roll, pitch, yaw).

The FTS used by UR robots is a type of load cell known as a "multi-axis load cell" or "force-
torque sensor" which is designed to measure forces and torques in multiple directions. It is an
integrated sensor, which means it directly interfaces with the robot's control system, providing
real-time data for the robot's control algorithm.

The FTS used by UR robots typically uses a combination of strain gauges and accelerometers to
measure forces and torques. The strain gauges are used to measure the bending or deflection of the
sensor's structure, while the accelerometers are used to measure the linear and angular acceleration
of the sensor.

The data from the FTS is used by the robot's internal controller to calculate the required torques to
counterbalance the payload's weight and keep it stable during the motion, as well as to perform
advanced motion control such as force control and impedance control.

In summary, Universal Robots use a six-axis force-torque sensor (FTS) that is mounted at the
flange of the robot. This sensor is a type of multi-axis load cell that uses a combination of strain
gauges and accelerometers to measure forces and torques in multiple directions. This sensor can be
used to measure forces and torques acting on the payload and also to perform advanced motion
control methods.

#### 14.1.19 get_target_payload_inertia()

Returns the most recently set payload inertia matrix.
This script function returns the inertia matrix of the active payload in tool flange coordinates, with
origin at the CoG.

**Return Value**

The six dimensional coordinates of the payload inertia matrix [Ixx, Iyy, Izz, Ixy, Ixz, Iyz]
expressed in kg*m^2.

Inertia is a measure of an object's resistance to changes in its motion. It is a fundamental concept in
physics that describes how an object will tend to remain at rest or in motion with a constant velocity
unless acted upon by an external force.

Inertia is determined by an object's mass. The greater an object's mass, the greater its inertia. Inertia
can be thought of as an object's "mass-related resistance to motion."
There are two types of inertia: linear and angular.

- Linear inertia is the resistance of an object to changes in its linear motion, such as
  acceleration or deceleration. This can be thought of as an object's resistance to changes in its
  velocity.

- Angular inertia is the resistance of an object to changes in its angular motion, such as
  rotation or angular acceleration. This can be thought of as an object's resistance to changes
  in its angular velocity.
  It's worth noting that the inertia of an object not only depends on its mass but also its shape and
  orientation. An object with the same mass can have different inertia depending on how its mass is
  distributed.

In summary, Inertia is a measure of an object's resistance to changes in its motion. Linear inertia is
the resistance of an object to changes in its linear motion and angular inertia is the resistance of an
object to changes in its angular motion. Inertia depends on mass, shape, and orientation of an object.

# The rest of the function explanations are to be added!

#### 14.1.20 get_target_tcp_pose()
#### 14.1.21 get_target_tcp_speed()
#### 14.1.22 get_target_waypoint()
#### 14.1.23 get_tcp_force()
#### 14.1.24 get_tcp_offset()
#### 14.1.25 get_tool_accelerometer_reading()
#### 14.1.26 get_tool_current()
#### 14.1.27 is_steady()
#### 14.1.28 is_within_safety_limits(pose, qnear)
#### 14.1.29 popup(s, title=’Popup’, warning=False, error=False, blocking=False)
#### 14.1.30 powerdown()
#### 14.1.31 set_gravity(d)
#### 14.1.32 set_payload(m, cog)
#### 14.1.33 set_payload_cog(CoG)
#### 14.1.34 set_payload_mass(m)
#### 14.1.35 set_target_payload(m, cog, inertia=[0, 0, 0, 0, 0, 0])
#### 14.1.36 set_tcp(pose)
#### 14.1.37 sleep(t)
#### 14.1.38 str_at(src, index)
#### 14.1.39 str_cat(op1, op2)
#### 14.1.40 str_empty(str)
#### 14.1.41 str_find(src, target, start_from=0)
#### 14.1.42 str_len(str)
#### 14.1.43 str_sub(src, index, len)
#### 14.1.44 sync()
#### 14.1.45 textmsg(s1, s2=’’)
#### 14.1.46 to_num(str )
#### 14.1.47 to_str(val)
#### 14.1.48 tool_contact(direction)
#### 14.1.49 tool_contact_examples()

## 15. Module urmath
### 15.1. Functions
#### 15.1.1. acos(f )
#### 15.1.2. asin(f )
#### 15.1.3. tan(f )
#### 15.1.4. atan2(x, y)
#### 15.1.5. binary_list_to_integer(l)
#### 15.1.6. ceil(f )
#### 15.1.7. cos(f )
#### 15.1.8. d2r(d)
#### 15.1.9. loor(f )
#### 15.1.10. get_list_length(v)
#### 15.1.11. nteger_to_binary_list(x)
#### 15.1.12. interpolate_pose(p_from, p_to, alpha)
#### 15.1.13. nv(m)
#### 15.1.14. ength(v)
#### 15.1.15. og(b, f )
#### 15.1.16. norm(a)
#### 15.1.17. normalize(v)
#### 15.1.18. point_dist(p_from, p_to)
#### 15.1.19. pose_add(p_1, p_2)
#### 15.1.20. pose_inv(p_from)
#### 15.1.21. pose_sub(p_to, p_from)
#### 15.1.22. pose_trans(p_from, p_from_to)
#### 15.1.23. pow(base, exponent)
#### 15.1.24. 2d(r)
#### 15.1.25. random()
#### 15.1.26. otvec2rpy(rotation_vector)
#### 15.1.27. rpy2rotvec(rpy_vector)
#### 15.1.28. sin(f )
#### 15.1.29. size(v)
#### 15.1.30. sqrt(f )
#### 15.1.31. tan(f)
#### 15.1.32. transpose(m)
#### 15.1.33. wrench_trans(T_from_to, w_from)

## 16. Module Interfaces
### 16.1 Functions
#### 16.1.1 enable_external_ft_sensor(enable, sensor_mass=0.0, sensor_measuring_offset=[0.0,0.0, 0.0], sensor_cog=[0.0, 0.0, 0.0])
### 16.1.2 ft_rtde_input_enable(enable, sensor_mass=0.0, sensor_measuring_offset=[0.0,0.0, 0.0], sensor_cog=[0.0, 0.0, 0.0]) 
#### 16.1.3 get_analog_in(n)
#### 16.1.4 get_analog_out(n)
#### 16.1.5 get_configurable_digital_in(n)
#### 16.1.6 get_configurable_digital_out(n)
#### 16.1.7 get_digital_in(n)
#### 16.1.8 get_digital_out(n)
#### 16.1.9 get_flag(n)
#### 16.1.10 get_standard_analog_in(n)
#### 16.1.11 get_standard_analog_out(n)
#### 16.1.12 get_standard_digital_in(n)
#### 16.1.13 get_standard_digital_out(n)
#### 16.1.14 get_tool_analog_in(n)
#### 16.1.15 get_tool_digital_in(n)
#### 16.1.16 get_tool_digital_out(n)
#### 16.1.17 modbus_add_signal(IP, slave_number, signal_address, signal_type, signal_name, sequential_mode=False)
#### 16.1.18 modbus_delete_signal(signal_name)
#### 16.1.19 modbus_get_signal_status(signal_name, is_secondary_program)
#### 16.1.20 modbus_send_custom_command(IP, slave_number, function_code, data)
#### 16.1.21 modbus_set_digital_input_action(signal_name, action)
#### 16.1.22 modbus_set_output_register(signal_name, register_value, is_secondary_program)
#### 16.1.23 modbus_set_output_signal(signal_name, digital_value, is_secondary_program)
#### 16.1.24 modbus_set_signal_update_frequency(signal_name, update_frequency)
#### 16.1.25 read_input_boolean_register(address)
#### 16.1.26 read_input_float_register(address)
#### 16.1.27 read_input_integer_register(address)
#### 16.1.28 read_output_boolean_register(address)
#### 16.1.29 read_output_float_register(address)
#### 16.1.30 read_output_integer_register(address)
#### 16.1.31 read_port_bit(address)
#### 16.1.32 read_port_register(address)
#### 16.1.33 rpc_factory(type, url)
#### 16.1.34 rtde_set_watchdog(variable_name, min_frequency, action=’pause’)
#### 16.1.35 set_analog_inputrange(port, range)
#### 16.1.36 set_analog_out(n, f )
#### 16.1.37 set_configurable_digital_out(n, b)
#### 16.1.38 set_digital_out(n, b)
#### 16.1.39 set_flag(n, b)
#### 16.1.40 set_standard_analog_out(n, f)
#### 16.1.41 set_standard_digital_out(n, b)
#### 16.1.42 set_tool_digital_out(n, b)
#### 16.1.43 set_tool_communication(enabled, baud_rate, parity, stop_bits, )
#### 16.1.44 set_tool_voltage(voltage)
#### 16.1.45 socket_close(socket_name=’socket_0’)
#### 16.1.46 socket_get_var(name, socket_name=’socket_0’)
#### 16.1.47 socket_open(address, port, socket_name=’socket_0’)
#### 16.1.48 socket_read_ascii_float(number, socket_name=’socket_0’, timeout=2)
#### 16.1.49 socket_read_binary_integer(number, socket_name=’socket_0’, timeout=2)
#### 16.1.50 socket_read_byte_list(number, socket_name=’socket_0’, timeout=2)
#### 16.1.socket_read_line(socket_name=’socket_0’, timeout=2)
#### 16.1.51 socket_read_string(socket_name=’socket_0’, prefix =’’, suffix =’’, interpret_escape=’False’, timeout=2)
#### 16.1.52 socket_send_byte(value, socket_name=’socket_0’)
#### 16.1.53 socket_send_int(value, socket_name=’socket_0’)
#### 16.1.54 socket_send_line(str, socket_name=’socket_0’)
#### 16.1.55 socket_send_string(str, socket_name=’socket_0’)
#### 16.1.56 socket_set_var(name, value, socket_name=’socket_0’)
#### 16.1.57 write_output_boolean_register(address, value)
#### 16.1.58 write_output_float_register(address, value)
#### 16.1.59 write_output_integer_register(address, value)
#### 16.1.60 write_port_bit(address, value)
#### 16.1.61 write_port_register(address, value)
#### 16.1.62 zero_ftsensor()

## 17. Module IO Configuration
### 17.1 Functions
#### 17.1.1 modbus_set_runstate_dependent_choice(signal_name, runstate_choice)
#### 17.1.2 set_analog_outputdomain(port, domain)
#### 17.1.3 set_configurable_digital_input_action(port, action)
#### 17.1.4 set_gp_boolean_input_action(port, action)
#### 17.1.5 set_input_actions_to_default()
#### 17.1.6 set_runstate_configurable_digital_output_to_value(outputId, state)
#### 17.1.7 set_runstate_gp_boolean_output_to_value(outputId, state)
#### 17.1.8 set_runstate_standard_analog_output_to_value(outputId, state)
#### 17.1.9 set_runstate_standard_digital_output_to_value(outputId, state)
#### 17.1.10 set_runstate_tool_digital_output_to_value(outputId, state)
#### 17.1.11 set_standard_analog_input_domain(port, domain)
#### 17.1.12 set_standard_digital_input_action(port, action)
#### 17.1.13 set_tool_analog_input_domain(port, domain)
#### 17.1.14 set_tool_digital_input_action(port, action)
