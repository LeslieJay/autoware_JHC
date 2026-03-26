<!--
 * @Autor: wei.canming
 * @Version: 1.0
 * @Date: 2026-03-24 16:45:57
 * @LastEditors: wei.canming
 * @LastEditTime: 2026-03-25 08:55:46
 * @Description: 
-->
colcon build \
  --cmake-args \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_TESTING=OFF \
  -DCMAKE_CXX_FLAGS="-Wno-error -Wno-unused-parameter"