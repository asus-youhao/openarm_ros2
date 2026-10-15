# Robot Description files for OpenArm

This package contains description files to generate OpenArm URDFs (Universal Robot Description Files). See [documentation](https://docs.openarm.dev/software/description) for details.

這個 package 原本在 `asus-youhao/openArm_leapHand_urdf`（`src/openarm_description/`），已整合進 openarm_ros2，
只保留 URDF 需要的檔案（xacro、用到的 mesh、config、launch、rviz）。安裝、fake 模式與 u22 / u24 的差異見
[openarm_ros2 的 README](../README.md)；工作規則與完整測試見 [CLAUDE.md](CLAUDE.md)；每個修正的紀錄見 [docs/](docs/README.md)。

```bash
ros2 launch openarm_description display_openarm.launch.py    # 只看模型（RViz + joint_state_publisher_gui）
```

## Related links

- 📚 Read the [documentation](https://docs.openarm.dev/software/description)
- 💬 Join the community on [Discord](https://discord.gg/FsZaZ4z3We)
- 📬 Contact us through <openarm@enactic.ai>

## License

[Apache License 2.0](LICENSE.txt)

Copyright 2025 Enactic, Inc.

## Code of Conduct

All participation in the OpenArm project is governed by our
[Code of Conduct](../CODE_OF_CONDUCT.md).
