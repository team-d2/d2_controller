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
| params_file | ["$(var project_share)/config/pure_pursuit.param.yaml"](../../config/pure_pursuit.param.yaml) | parameter file path |
| use_sim_time | false | whether to use A |

## nodes
| name | type | description |
| - | - | - |
| d2_local_planner | [d2_controller/local_plannner](../node/local_plannner.md) | global_planと自己位置から自己位置を始点とするplanを生成 | 
| d2_pure_pursuit | [d2_controller/pure_pursuit](../node/pure_pursuit.md) | 経路始点から一定距離進んだ地点を計算 | 
| d2_target_point_follower | [d2_controller/target_point_follower](../node/target_point_follower.md) | 点追従 | 
| d2_cmd_vel_transformer | [d2_controller/cmd_vel_transformer](../node/cmd_vel_transformer.md) | 速度司令のtf変換 | 
| d2_cmd_vel_limiter | [d2_controller/cmd_vel_limiter](../node/cmd_vel_limiter.md) | 速度司令制限 | 
