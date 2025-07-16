# [d2_controller/](../../../README.md)[ros2/](../../ros2.md)[node/](../node.md)pure_pursuit

## about
cmd_velの座標系を変換する

## topics
| name | type | in/out | description |
| - | - | - | - |
| cmd_vel/stamped | geometry_msgs/msg/TwistStamped | in | 座標系付き速度司令(必須) |
| cmd_vel | geometry_msgs/msg/Twist | out | 座標系変換後速度司令<br>cmd_vel/stampedでトリガー |

## parameters
| name | default | description |
| - | - | - |
| base_footprint_frame_id | "base_footprint" | 変換後のframe_id |
