<!--
 * @Autor: wei.canming
 * @Version: 1.0
 * @Date: 2026-03-24 16:45:57
 * @LastEditors: wei.canming
 * @LastEditTime: 2026-04-09 09:03:57
 * @Description: 
-->
colcon build \
  --cmake-args \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_TESTING=OFF \
  -DCMAKE_CXX_FLAGS="-Wno-error -Wno-unused-parameter"


### 清除路径 
service /api/routing/clear_route

# 待做

1. vehicle_cmd_gate 去除打印输出

2. path_smoother 优化打印输出

3. 查看autoware是怎么输出转向灯指令

