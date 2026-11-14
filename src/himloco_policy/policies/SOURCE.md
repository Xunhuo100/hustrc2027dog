启动文件默认选择 parkour；用 policy_profile:=flat 回到原来的 flat 配置。
输入均为 [1,342]，输出 [1,16]，包含 HIM estimator.encoder 与 actor。

parkour.pt 来自本机：
_PaperWorks/HIMLoco/legged_gym/logs/him_go2w_parkour/exported/policies/policy.pt
已逐项核对 actor 和 estimator 的全部参数，与以下 checkpoint 完全一致：
_PaperWorks/HIMLoco/legged_gym/logs/him_go2w_parkour/Oct24_15-45-13_/model_13500.pt
SHA256 前 16 位：925945b770662439。
对应 HIMGo2WParkourCfg（继承 HIMGo2WRoughCfg）：
hip=0、thigh=0.8、calf=-1.5；腿 kp=40、kd=1，轮 kd=0.5。
参数位于 config/parkour.yaml，同时应用于 Stand、策略和仿真初始姿态。

flat.pt 与兼容旧入口的 policy.pt 内容一致，来自：
_PaperWorks/HIMLoco/legged_gym/logs/him_go2w_flat/exported/policies/policy.pt
SHA256 前 16 位：c2dd4935389fafaa。
对应 HIMGo2WFlatCfg，参数位于 config/flat.yaml。

使用自定义 model_path 时仍需选择匹配的 policy_profile；输入输出维度相同不代表站姿和 PD 相同。
两种任务共用 config/himloco.yaml 的 57 维观测、6 帧历史、动作缩放与周期。
