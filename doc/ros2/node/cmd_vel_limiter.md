# [d2_controller/](../../../README.md)[ros2/](../../ros2.md)[node/](../node.md)pure_pursuit

## about
cmd_velの座標系を変換する．

vel_limitからできる楕円体内にcmd_velのベクトルが収まるように速度を制限する．
加速度も同様．

例:
- 速度limit: { 1, 1, 1 }
- 元の速度: { 1, 1, 0 }

-> 制限後速度: { 0.707, 0.707, 0 }

速度を制限後に加速度を制限する(加速度が優先)．

## topics
| name | type | in/out | description |
| - | - | - | - |
| cmd_vel_nav | geometry_msgs/msg/Twist | in | 元の速度司令(必須) |
| vel_limit | geometry_msgs/msg/Twist | in | 速度制限値(オプション) |
| accel_limit | geometry_msgs/msg/Accel | in | 加速度性現値(オプション) |
| cmd_vel | geometry_msgs/msg/Twist | out | 制限後速度司令<br>時間でトリガー |

## parameters
| name | default | description |
| - | - | - |
| cmd_vel_timeout | 10.0 | cmd_vel_navを最後に受信してからcmd_velをpublishしなくするまでのtimeout[s] |
| vel_limit.linear.x | 1.0 | x方向並進移動速度制限値 |
| vel_limit.linear.y | 1.0 | y方向並進移動速度制限値 |
| vel_limit.linear.z | 1.0 | z方向並進移動速度制限値 |
| vel_limit.angular.x | 1.0 | x軸回転速度制限値 |
| vel_limit.angular.y | 1.0 | y軸回転速度制限値 |
| vel_limit.angular.z | 1.0 | z軸回転速度制限値 |
| accel_limit.linear.x | 1.0 | x方向並進移動加速度制限値 |
| accel_limit.linear.y | 1.0 | y方向並進移動加速度制限値 |
| accel_limit.linear.z | 1.0 | z方向並進移動加速度制限値 |
| accel_limit.angular.x | 1.0 | x軸回転加速度制限値 |
| accel_limit.angular.y | 1.0 | y軸回転加速度制限値 |
| accel_limit.angular.z | 1.0 | z軸回転加速度制限値 |
| publish_rate | 60.0 | 速度司令のpublish rate |
