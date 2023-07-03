CRSS platform communication procedure.



启动

```
<launch>
    <node name="can_node"       pkg="crss_chassis_driver"   type="can_node_nx"  output="screen">
        <param name="vc"            value="100.00"/>      <!--velocity factor-->
        <param name="vel_max"       value="5.00"/>      <!--m/s-->
        <param name="ang_max"       value="0.61"/>      <!--radian-->
    </node>
</launch>
```



底盘控制

```
vd@vd:~$ rostopic pub -r 10 /chassis/ctrl_motion geometry_msgs/Twist "linear:
  x: 0.6
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.3"
```



底盘状态

```
header: 
  seq: 9126
  stamp: 
    secs: 0
    nsecs:         0
  frame_id: ''
WARMIGM: 0
ERROR: 0
MODE: 1
Vol: 6350
Curr: 0
Quantity: 0
ENCODER: 24364
CURR_STEER: 0
CURR_WHEEL_L: 16
CURR_WHEEL_R: 4
VEL_WHEEL_L: 0
VEL_WHEEL_R: 0
ENC_WHEEL_L: 185
ENC_WHEEL_R: 712
UL: [1032, 1181, 1498, 1341, 358, 789, 709, 2421]
```

