<!--
 * @Autor: wei.canming
 * @Version: 1.0
 * @Date: 2026-03-25 09:42:42
 * @LastEditors: wei.canming
 * @LastEditTime: 2026-03-25 09:49:08
 * @Description: 
-->
### 关键信息

1. 终点不允许修改（哪怕允许修改到终点附件，工程上最好也别用），所以必须使用 fixed_goal_planner

2. fixed_goal_planner 的触发条件：
- Route is set with `allow_goal_modification=false`. This is the default.
- The goal is set on `road` lanes.

3. fixed_goal_planner 的特点
没有"goal search", "collision check", "safety check", etc等功能.

### 整体流程

1. 初始化 goal_searcher
2. 生成 goal candidates
3. 判断是否触发规划线程
4. 如果模块未激活 → 退出
5. 重置路径
6. 安全检测（静态+动态障碍物）
7. 状态机更新（核心）
8. 同步子线程（lane / freespace）
9. 更新 context_data（核心数据）
10. 决策完成 → 处理速度
11. 如果已激活 → 推进路径执行

