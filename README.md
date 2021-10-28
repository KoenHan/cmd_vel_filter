# cmd_vel_filter

next_waypointの/area_typeが3だったらcmd_velのlinear.xをparamのslowdown_velにする.

- sub

  ~~~
  /cmd_vel(move_base)
  /area_type(cirkit_waypoint_navigator)
  ~~~

- pub

  ~~~
  /cmd_vel_filter(ypspur_ros)
  ~~~

### Usage

~~~
$ roslaunch cmd_vel_filter cmd_vel_filter.launch
~~~

~~~xml
<arg name="slowdown_vel" default="0.3"/>
~~~



