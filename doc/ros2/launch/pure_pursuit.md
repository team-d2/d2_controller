# [d2_controller/](../../../README.md)[ros2/](../../ros2.md)[launch/](../launch.md)pure_pursuit

## about
pure pursuitで経路追従を行う．

## topics
| name | type | in/out | description |
| - | - | - | - |
| global_plan | nav_msgs/msg/Path | in | 追従する経路(必須) |
| pose | geometry_msgs/msg/PoseStamped | in | 現在のロボットの姿勢(必須) |
| vel_limit | geometry_msgs/msg/Twist | in | ロボットの最大速度(オプション) |
| accel_limit | geometry_msgs/msg/Accel | in | ロボットの最大加速度(オプション) |
| cmd_vel | geometry_msgs/msg/Twist | out | ロボット司令速度 |

## argments
| name | default | description |
| - | - | - |
| namespace | "" | namespace |
| container_name | "d2_controller_container" | composable container name |
| thread_num | 2 | thread number of container |
| params_file | ["$(find-pkg-share d2_controller)/config/pure_pursuit.param.yaml"](../../../config/pure_pursuit.param.yaml) | parameter file path |
| use_sim_time | false | whether to use sim time |
| use_pose_transformer | true | pose入力時にframe_idを変換するか |
| use_global_plan_transformer | true | global_plan入力時にframe_idを変換するか |
| controller_name | "d2_controller" | controllerの名前空間 |
| pose_topic_name | "$(var namespace)/pose" | 入力するposeのtopic名 |
| global_plan_topic_name | "$(var namespace)/global_plan" | 入力するglobal_planのtopic名 |
| vel_limit_topic_name | "$(var namespace)/vel_limit" | 入力するvel_limitのtopic名 |
| accel_limit_topic_name | "$(var namespace)/accel_limit" | 入力するaccel_limitのtopic名 |
| cmd_vel_topic_name | "$(var namespace)/cmd_vel" | 出力するcmd_velのtopic名 |

## nodes
| name | type | description |
| - | - | - |
| global_plan_transformer | geometry_transformer/path_transformer | global_planのframe_idを変更 |
| pose_transformer | geometry_transformer/pose_transformer | poseのframe_idを変更 |
| local_planner | [d2_controller/local_plannner](../node/local_plannner.md) | global_planと自己位置から自己位置を始点とするplanを生成 | 
| pure_pursuit | [d2_controller/pure_pursuit](../node/pure_pursuit.md) | 経路始点から一定距離進んだ地点を計算 | 
| target_point_follower | [d2_controller/target_point_follower](../node/target_point_follower.md) | 点追従 | 
| cmd_vel_limiter | [d2_controller/cmd_vel_limiter](../node/cmd_vel_limiter.md) | 速度司令制限 | 
