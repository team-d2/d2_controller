# [d2_controller/](../../../README.md)[ros2/](../../ros2.md)[node/](../node.md)pure_pursuit

## about
pure pursuitで経路追従を行う．
パスの始点からオフセット距離分勧めた位置を計算する．

## topics
| name | type | in/out | description |
| - | - | - | - |
| local_plan | nav_msgs/msg/Path | in | 追従する経路(必須)<br>経路点数 >= 1<br>始点以外の姿勢は無視される |
| target_point | geometry_msgs/msg/PoseStamped | out | 計算された位置<br>local_planの入力でトリガー |

## parameters
| name | default | description |
| - | - | - |
| default_frame_id | "map" | pathのフレームidがセットされていなかったときに使用するframe_id |
| lookahead_distance | 1.0 | オフセット距離[m] |
