# xbot_navigoals
## Summary
This ros package is a tool for xbot navigation. The navigoals node could help user point out a serial of navigation goals for xbot pursue. 
The package is usually used for two cases:
* for testing xbot in many running loops among a set of goals.
* used in places like museum or gallery that xbot will lead visitors to look over every part of exhibitions and explain with TTS voice.

## Dependence
* move_base
* rviz
* [xbot](https://github.com/yowlings/xbot)
* gmapping/map_server


## Usage
1. rosrun 

Once the navigation with move_base launched, just run:
>rosrun xbot_navigoals navigoals

then in command line you will see the hint message for leading you use the tool.

2. roslaunch
add the following fragment to your launch file which already included the move_base package:
~~~
<node name="navi_goals" pkg="xbot_navigoals" type="navigoals" output="screen">
</node>
~~~

## API
### publish topics
1. /xbot_navigoals/navi_plans(nav_msgs/Path)
2. /move_base_simple/goal(geometry_msgs/PoseStamped)
3. /xbot_navigoals/marker_goals(visualization_msgs/MarkerArray)
### subscribe topics
1. /goal(geometry_msgs/PoseStamped)
2. /move_base/result(actionlib_msgs/GoalStatusArray)

### subscribe services
1. ~make_plan (nav_msgs/GetPlan)

## References
[kobuke_keyop](https://github.com/yujinrobot/kobuki/tree/devel/kobuki_keyop)




