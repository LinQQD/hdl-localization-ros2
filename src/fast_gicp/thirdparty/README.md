# thirdparty

**默认构建（BUILD_VGICP_CUDA=OFF）不需要此目录。**

仅当开启 CUDA（`-DBUILD_VGICP_CUDA=ON`）时需要 **nvbio**，由脚本拉取：

```bash
/home/qulingjun/hdl_ws_ros2/scripts/setup_fast_gicp_deps.sh
```

Eigen 始终使用系统包 `libeigen3-dev`，不放在 thirdparty。
