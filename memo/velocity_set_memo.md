# velocity_setのメモ

## file
+ velocity_set.cpp : main
+ velocity_set_path.cpp : 速度の再計画用？
+ velocity_set_info.cpp : パラメータの読み込み？


## 流れ

mainの中でループ
```cpp

// はじめにcrosswalkなどの設定をする

// 

EControl detection_result = obstacleDetection(closest_waypoint, vs_path.getPrevWaypoints(), crosswalk, vs_info, detection_range_pub, obstacle_pub, &obstacle_waypoint);

// detection_resultに応じてwaypointの速度を変更する
changeWaypoints(vs_info, detection_result, closest_waypoint,
                obstacle_waypoint, final_waypoints_pub, &vs_path);
```

`changeWaypoints`の中で、`Econtrol`のモードに応じて以下が呼ばれる

```cpp
(1) vs_path->changeWaypointsForStopping() // 停止時のみ
(2) vs_path->changeWaypointsForDeceleration() // 減速時のみ
(3) vs_path->avoidSuddenAcceleration() // 常に
(4) vs_path->avoidSuddenDeceleration() // 常に
```
の4つ。終わったらpub.
それぞれの関数に与える引数は

+ 減速度：STOPLINEの場合 `vs_info.getDecelerationStopline()`とそれ以外 `vs_info.getDecelerationObstacle()`の2種類




### (1) changeWaypointsForStopping()

+ `STOP`か`STOPLINE`の時に呼ばれる
+ 停止位置（`stop_waypoint`）から今の位置(`index`)まで、一定減速度で減速した時の速度を計算
+ 減速度は引数で受け取っており、STOPLINEの時は`vs_info.getDecelerationStopline()`、それ以外（KEEP, STOP, DECELATE, OTHERS）は`vs_info.getDecelerationObstacle()`で減速。
+ オリジナルのwaypointの速度は絶対に超えない
+ `stop_waypoint`から障害物まですべて速度0で埋める

### (2) changeWaypointsForDeceleration()

+ `DECELATE`のときに呼ばれる
+ 障害物位置(obstacle_waypoint)+4から現在位置(`closest_waypoint`)まで、最低速度(`decelerate_vel_min_`)に向かって一定減速度で減速した時の速度を計算
+ 減速度は引数で `vs_info.getDecelerationObstacle()`で与えられている
+ オリジナルのwaypointの速度は絶対に超えない

### (3) avoidSuddenAcceleration()

+ すべての状況で呼ばれる
+ 現在速度(`current_vel_`)から一定加速度 `deceleration`で加速した速度+オフセット `velocity_offset_`(rosparam)を指令速度として、自己位置から先の経路の速度を上書き
+ 経路の終端まで上書きするか、最大速度がオリジナル速度を上回ったら終了


### (4) avoidSuddenDeceleration()

+ すべての状況で呼ばれる
+ 自己位置のwaypoint速度と現在速度の速度差が`velocity_change_limit`未満であれば何もせずreturn
+ 現在車速から一定減速度 `deceleration`で減速した速度+オフセット `velocity_change_limit`を速度指令値として、自己位置から速度を上書き（しているつもりだが、計算式が少しおかしい）
+ 目標速度が負の値になったら終了
