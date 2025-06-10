# F1 Task Commander

## Overview

Uses navigation2 package API's and functionality to enable automated task creation for vehicles.


## API

See its [API Guide Page](https://navigation.ros.org/commander_api/index.html) for additional parameter descriptions.



| F1 Navigator Method            | Description                                                                |
| --------------------------------- | -------------------------------------------------------------------------- |
| setInitialPose(initial_pose)      | Sets the initial pose (`PoseStamped`) of the robot to localization.        |
| goThroughPoses(poses, behavior_tree='') | Requests the robot to drive through a set of poses (list of `PoseStamped`).|
| goToPose(pose, behavior_tree='')  | Requests the robot to drive to a pose (`PoseStamped`).                     |
| followWaypoints(poses)            | Requests the robot to follow a set of waypoints (list of `PoseStamped`). This will execute the specific `TaskExecutor` at each pose.   |
| followPath(path, controller_id='', goal_checker_id='') | Requests the robot to follow a path from a starting to a goal `PoseStamped`, `nav_msgs/Path`.     |
| spin(spin_dist=1.57, time_allowance=10)   | Requests the robot to performs an in-place rotation by a given angle.      |
| backup(backup_dist=0.15, backup_speed=0.025, time_allowance=10) | Requests the robot to back up by a given distance.         |
| cancelTask()                       | Cancel an ongoing task request.|
| isTaskComplete()                   | Checks if task is complete yet, times out at `100ms`.  Returns `True` if completed and `False` if still going.                  |
| getFeedback()                     | Gets feedback from task, returns action server feedback object. |
| getResult()				        | Gets final result of task, to be called after `isTaskComplete` returns `True`. Returns action server result object. |
| getPath(start, goal, planner_id='', use_start=False) | Gets a path from a starting to a goal `PoseStamped`, `nav_msgs/Path`.      |
| getPathThroughPoses(start, goals, planner_id='', use_start=False) | Gets a path through a starting to a set of goals, a list of `PoseStamped`, `nav_msgs/Path`. |
| smoothPath(path, smoother_id='', max_duration=2.0, check_for_collision=False) | Smooths a given `nav_msgs/msg/Path` path. |
| changeMap(map_filepath)           | Requests a change from the current map to `map_filepath`'s yaml.           |
| clearAllCostmaps()                | Clears both the global and local costmaps.                                 |
| clearLocalCostmap()               | Clears the local costmap.                                                  |
| clearGlobalCostmap()              | Clears the global costmap.                                                 |
| getGlobalCostmap()                | Returns the global costmap, `nav2_msgs/Costmap`                            |
| getLocalCostmap()                 | Returns the local costmap, `nav2_msgs/Costmap`                             |
| waitUntilNav2Active(navigator='bt_navigator, localizer='amcl') | Blocks until Nav2 is completely online and lifecycle nodes are in the active state. To be used in conjunction with autostart or external lifecycle bringup. Custom navigator and localizer nodes can be specified  |
| lifecycleStartup()                | Sends a request to all lifecycle management servers to bring them into the active state, to be used if autostart is `false` and you want this program to control Nav2's lifecycle. |
| lifecycleShutdown()               | Sends a request to all lifecycle management servers to shut them down.     |
| destroyNode()                     | Releases the resources used by the object.                                 |

## Executables:
- `nav_through_poses.py` - Demonstrates the navigate through poses capabilities of the navigator, as well as a number of auxiliary methods.
- `waypoint_follower.py` - Demonstrates the waypoint following capabilities of the navigator, as well as a number of auxiliary methods.

## Launch Files:
- nav_through_poses_launch.py
- waypoint_follower_launch.py
