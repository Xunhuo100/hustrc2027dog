原来的 12 自由度 unitree_guide 源码和构建配置保存在这里，便于查阅。
当前 CMakeLists.txt 只编译新的 16 自由度控制器，此目录不参加编译。
旧 Trotting、Balance 等状态的运动学只包含腿关节，不能直接用于轮腿机器人。
