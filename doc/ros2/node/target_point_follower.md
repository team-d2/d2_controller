# [d2_controller/](../../../README.md)[ros2/](../../ros2.md)[node/](../node.md)pure_pursuit

## about
現在の姿勢と目的地から速度司令を作成する

| 並進速度 | = | 現在地 - 目的地 | * cmd_vel_distance_rate

## topics
| name | type | in/out | description |
| - | - | - | - |
| pose | geometry_msgs/msg/PoseStamped | in | 現在の姿勢(必須) |
| target_point | geometry_msgs/msg/PointStamped | in | 目的地(必須) |
| cmd_vel/stamped | geometry_msgs/msg/TwistStamped | out | poseの座標系の速度司令<br>poseとtarget_pointの入力でトリガー |

## parameters
| name | default | description |
| - | - | - |
| default_frame_id | "map" | pathのフレームidがセットされていなかったときに使用するframe_id |
| cmd_vel_distance_rate | 0.2 | 距離差から速度に変換する時の係数 |
