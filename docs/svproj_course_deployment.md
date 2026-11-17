# robot.svproj 场地转换与部署

`robot.svproj` 是 STEP Viewer 0.0.3 的场地工程，含 28 个实体；工程内只有一个没有分配实体、没有惯性参数的空 `base_link`，关节数为 0。本次导出的是静态障碍场地，训练机器人使用这两个现有项目中的 Go2W，16 个动作。不把场地当成有腿的机器人导出。

源文件 SHA256：`032b27feec3524a7f78d8532872d86ea573e05852e8020f1c7c8acf8a85d71d4`。源 STEP SHA256：`fc60d737467330500b6b855874782214f9091472e6272ea22e26c1324c41c45c`。

## 已部署的位置

| 用途 | 文件或目录 |
| --- | --- |
| 原始导出资产 | `/home/wangmx/hustrc2027dog/assets/svproj_course/` |
| 转换脚本 | `/home/wangmx/hustrc2027dog/tools/convert_svproj_course.py` |
| 部署脚本 | `/home/wangmx/hustrc2027dog/tools/deploy_svproj_course.py` |
| MuJoCo 独立场地 | `/home/wangmx/dogMujoco2027/fields/svproj_course/scene.xml` |
| MuJoCo Go2W + 场地 | `/home/wangmx/dogMujoco2027/unitree_robots/go2_w/urdf/scene_svproj.xml` |
| MuJoCo 站立控制/查看入口 | `/home/wangmx/dogMujoco2027/fields/svproj_course/run.py` |
| HIMLoco | `/home/wangmx/_PaperWorks/HIMLoco/`，任务 `him_go2w_svproj` |
| Extreme Parkour | `/home/wangmx/_PaperWorks/extreme-parkour/`，任务 `go2w_svproj` |

两个训练项目均增加了 `legged_gym/legged_gym/envs/svproj/course.py`、`legged_gym/legged_gym/utils/svproj_terrain.py` 和 `legged_gym/resources/terrains/svproj_course/`；在各自任务注册文件末尾注册新任务。原有任务继续使用各自原有实现。HIMLoco 部署到与 `himloco` conda 环境绑定的 `HIMLoco`，没有部署到 `HIMLoco_full` 或 `HIMLoco-for-Go2W`。

## 坐标、碰撞与观测

转换采用米制、Z 向上：`x=cad_x/1000`，`y=-cad_z/1000`，`z=(cad_y+75)/1000`。该旋转保持右手系。场地补充了地面。单块场地边界为 X `[-2,12]` 米，Y `[-5,2]` 米，尺寸 14×7 米。

安全出生点位于源场地坐标 `(0.5,0.5,0)`，机器人基座另加其初始高度。原先的 `(0,0)` 在起点框架的拐角处，会产生初始障碍接触，因此不用作默认出生点。MuJoCo 的 home keyframe 与两个训练任务的出生位置一致。

MuJoCo 保留 28 个视觉网格，碰撞使用 327 块 CoACD 凸分解网格，分组 3，视觉网格分组 2。凸分解是近似碰撞，不能认为每个凹形细节都与 CAD 完全相同；已通过框架开口、轮胎中心孔、0.30 米墙顶的实体落球验证。直接把整个轮胎交给一个凸包会堵住孔洞，因此没有采用这种方式。

Isaac Gym/PhysX 使用原始实体的三角网格和补充地面，共 20,832 个顶点、41,594 个三角形。策略高度扫描单独存储于 `terrain.npz`，水平分辨率 0.025 米、垂直单位 0.001 米；扫描取每个 XY 位置的最高表面。原始碰撞保留孔洞及架空结构，但高度扫描无法表达同一 XY 的上下两层通行空间。

默认训练配置为 64 个并行环境、2×2 块完整场地、每回合 120 秒。同块场地内多个机器人通过 Isaac Gym 的环境碰撞分组隔离。拼接场地时将源坐标平移到正坐标，并对三角网格、高度扫描、出生点和目标点使用同一个平移。越过本块场地边界会重置。

10 个路线目标是本次人为设置的建议路线，记录在 `manifest.json` 的 `goals`，并标记 `goals_are_authored=true`；它们并非原 CAD 工程中定义的路线。HIMLoco 将路线方向转换为局部速度与转向指令，Extreme Parkour 使用自己的逐目标跟踪逻辑。若竞赛要求固定绕杆、钻架或特定行走顺序，需据规则重新设置路线及奖励。

## 打开 MuJoCo

```bash
/home/wangmx/hustrc2027dog/.venv/bin/python \
  /home/wangmx/dogMujoco2027/fields/svproj_course/run.py --viewer --seconds 600
```

此入口对腿关节施加站立 PD 控制，轮子目标速度为零，便于检查模型和场地。它没有加载越障策略。无窗口复查：

```bash
/home/wangmx/hustrc2027dog/.venv/bin/python \
  /home/wangmx/dogMujoco2027/fields/svproj_course/run.py --seconds 10
```

## 启动训练

HIMLoco：

```bash
cd /home/wangmx/_PaperWorks/HIMLoco
LD_LIBRARY_PATH="/home/wangmx/miniconda3/envs/himloco/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" \
/home/wangmx/miniconda3/envs/himloco/bin/python \
  legged_gym/legged_gym/scripts/train.py \
  --task him_go2w_svproj --headless --num_envs 64 \
  --max_iterations 20000 --run_name svproj_course
```

Extreme Parkour 高度扫描教师策略：

```bash
cd /home/wangmx/_PaperWorks/extreme-parkour
LD_LIBRARY_PATH="/home/wangmx/miniconda3/envs/parkour/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" \
/home/wangmx/miniconda3/envs/parkour/bin/python \
  legged_gym/legged_gym/scripts/train.py \
  --task go2w_svproj --headless --num_envs 64 --max_iterations 20000 \
  --proj_name parkour_go2w_svproj --exptid svproj_course --no_wandb
```

`--no_wandb` 让训练在本地记录。默认关闭深度相机。本次没有训练或验证视觉学生蒸馏流程。正式训练前可以调整 `envs/svproj/course.py` 的环境数、速度、奖励、回合长度和地形块数；几何分辨率与单块尺寸应与 `terrain.npz` 保持一致，否则加载器会报错。

## 重复转换与部署

当前 `.venv` 已包含转换、仿真和渲染依赖，验证版本保存在 `tools/requirements_svproj.txt`。部署脚本使用 Python 3.10；训练项目继续使用各自的 Python 3.8 环境。

```bash
cd /home/wangmx/hustrc2027dog
.venv/bin/python tools/convert_svproj_course.py robot.svproj --output assets/svproj_course
python3 tools/deploy_svproj_course.py
```

转换会重新运行凸分解。部署可以重复执行，会更新这些场地资产、新增任务和生成的 MuJoCo 入口。项目根目录可用 `--mujoco-root`、`--him-root`、`--parkour-root` 参数指定。源 `assets/svproj_course/course.xml` 是给部署脚本使用的包含文件；应通过部署后生成的 `scene.xml` 或 `scene_svproj.xml` 打开。

## 验证

MuJoCo 的验证会核验源文件哈希、网格顶点与面、实体落球碰撞、Go2W 初始接触、10 秒站立仿真及渲染：

```bash
cd /home/wangmx/hustrc2027dog
MUJOCO_GL=egl .venv/bin/python tools/test_svproj_mujoco.py
```

两个项目的仿真检查分别使用对应环境运行 `tools/check_svproj.py --task him_go2w_svproj --headless --num_envs 16` 和 `tools/check_svproj.py --task go2w_svproj --headless --num_envs 16`，并设置与训练命令相同的 `LD_LIBRARY_PATH`。检查包括 4 块场地的网格与目标平移、10 个目标转换、终点重置、越界重置、300 步向量环境仿真和观测/奖励有限值。

验证产物位于 `/home/wangmx/dogMujoco2027/fields/svproj_course/validation/`，包含预览图 `overview.png`、MuJoCo 报告、两个项目的训练和仿真日志以及总报告 `deployment_report.json`。训练项目自身也保存 `legged_gym/resources/terrains/svproj_course/validation/simulation_report.json`。PPO 训练检查使用 64 环境、2 次迭代，检查点位置记在 `training_report.json`。

以上验证说明模型、场地、目标、重置和 PPO 更新可以运行。验证时保存的检查点仅用于启动与接口检查，不能作为已学会完整越障的策略。尚未验证完整路线通过率、策略收敛、视觉学生蒸馏或训练策略在 MuJoCo 中的迁移效果。

## 当前 ROS 工作空间的 XML 入口

完整场地现已导入本工作空间：`src/wheel_leg_mujoco/models/terrains/svproj_course.xml`。全部地图视觉及碰撞网格已嵌入 XML，单独复制此文件即可打开地图。Go2W 与 HUSTDOG 的组合入口分别为 `models/go2w/urdf/scene_svproj.xml`、`models/hustdog_v11/urdf/scene_svproj.xml`；相对于 `src/wheel_leg_mujoco/`。详细启动方式见 `src/wheel_leg_mujoco/models/terrains/README.md`。

使用 `python3 tools/build_workspace_map.py` 可根据本地资产与机器人源文件重新生成，使用 `MUJOCO_GL=egl .venv/bin/python tools/check_workspace_map.py` 可复验。HUSTDOG 保留原有位置执行器及基座贴地的零位预览姿态，只验证了独立模型加载和预览，没有适配 Go2W 的 ROS 控制接口。
