# 完整 SVPROJ 障碍地图

`svproj_course.xml` 是从工作空间根目录 `robot.svproj` 转换的完整场地，包含全部 28 个视觉实体、327 块凸分解碰撞体、地面、灯光和两个场地观察相机。所有地图网格的顶点和三角形都嵌入 XML，文件约 2.8 MB；单独复制这个 XML 即可在 MuJoCo 中加载地图，无需复制地图 OBJ/STL 文件。

坐标单位为米、Z 向上：`x=cad_x/1000`，`y=-cad_z/1000`，`z=(cad_y+75)/1000`。场地边界为 X `[-2,12]`、Y `[-5,2]`，出生点为 `(0.5,0.5,0)`。碰撞来自原有 CoACD 凸分解，为近似几何；框架开口、轮胎孔洞和 0.30 米墙顶已通过实体落球检查。

在工作空间根目录执行，仅查看地图：

```bash
.venv/bin/python -m mujoco.viewer \
  --mjcf=src/wheel_leg_mujoco/models/terrains/svproj_course.xml
```

机器人与地图的组合入口：

- Go2W：`../go2w/urdf/scene_svproj.xml`
- HUSTDOG v11：`../hustdog_v11/urdf/scene_svproj.xml`

两个入口都引用同一地图文件，使用各自现有机器人网格。地图的相机名为 `svproj_overview` 和 `svproj_top`。场景由 `tools/build_workspace_map.py` 生成，修正资源目录、出生位置和重复地面；机器人关节、惯性、约束、传感器和执行器沿用各自源模型。

查看 HUSTDOG 场景：

```bash
.venv/bin/python -m mujoco.viewer \
  --mjcf=src/wheel_leg_mujoco/models/hustdog_v11/urdf/scene_svproj.xml
```

Go2W 可以接入现有 ROS 启动脚本：

```bash
MODEL_XML="$PWD/src/wheel_leg_mujoco/models/go2w/urdf/scene_svproj.xml" ./start_sim.sh
```

现有策略不保证通过这张地图。HUSTDOG 场景可由 MuJoCo 独立加载和预览；现有 ROS 后端仍使用 Go2W 的关节映射及力矩接口，尚不能通过替换 `MODEL_XML` 控制 HUSTDOG。

重新生成与验证，在工作空间根目录执行：

```bash
python3 tools/build_workspace_map.py
MUJOCO_GL=egl .venv/bin/python tools/check_workspace_map.py
source /opt/ros/humble/setup.bash
colcon build --packages-select wheel_leg_mujoco
```

`svproj_course.json` 记录源文件校验值、坐标变换、建议导航点和几何尺寸；XML 运行时不依赖此 JSON。验证报告及预览图位于 `docs/svproj_workspace_validation/`。
