# [d2_controller/](../../../README.md)[ros2/](../../ros2.md)[node/](../node.md)local_plannner

## about
global_planと現在地から，現在地を始点とする経路を作成する．
現在地からパスまでの最短経路を計算->最短経路とglobal_planの交点の次の点から始まる経路を生成->始点に現在地を追加

## topics
| name | type | in/out | description |
| - | - | - | - |
| global_plan | nav_msgs/msg/Path | in | 元の経路(必須) |
| pose | geometry_msgs/msg/PoseStamped | in | 現在地(必須) |
| local_plan | nav_msgs/msg/Path | out | 生成された経路<br>global_planとposeの入力でトリガー |

## parameters
| name | default | description |
| - | - | - |
| default_frame_id | "map" | pathのフレームidがセットされていなかったときに使用するframe_id |
