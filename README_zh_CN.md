# LIOKit

LIO-SAM 引擎，打包为跨平台 SDK，支持 Linux 和 Windows。

## 项目简介

LIOKit 是一个基于 LIO-SAM (Lidar-Inertial Odometry and Mapping with Smoothing and Mapping) 的 SLAM 引擎实现。该项目将 LIO-SAM 打包为易于使用的 SDK，支持多种平台。

## 主要特性

- 基于激光雷达和惯性测量单元的里程计和建图
- 平滑和映射优化
- 跨平台支持 (Linux/Windows)
- 使用 vcpkg 管理依赖

## 依赖项

项目使用 vcpkg 管理依赖，主要依赖包括：

- Boost
- Eigen
- GTSAM
- TBB
- 其他 Boost 组件

## 构建说明

### 环境要求

- CMake 3.16+
- C++17 编译器
- vcpkg

### 构建步骤

1. 克隆项目并初始化子模块（如果有）。

2. 安装依赖：

   ```bash
   vcpkg install --triplet x64-linux
   ```

   或使用提供的 vcpkg 配置：

   ```bash
   vcpkg export --output-dir=. --output=vcpkg_export --raw
   ```

3. 构建项目：

   使用提供的构建脚本：

   ```bash
   python scripts/build.py          # 默认 Release 模式
   python scripts/build.py --debug  # Debug 模式
   python scripts/build.py --release # Release 模式
   ```

   或手动使用 CMake：

   ```bash
   cmake -B build -S . \
     -DCMAKE_TOOLCHAIN_FILE=/path/to/vcpkg/scripts/buildsystems/vcpkg.cmake \
     -DVCPKG_INSTALLED_DIR=/path/to/vcpkg_installed
   cmake --build build -- -j$(nproc)
   ```

## 使用说明

### 作为 SDK 使用

LIOKit 提供 C++ API，可用于集成到其他项目中。

### 示例代码

```cpp
#include <LIOKit/LIOKit.h>

// 示例使用
```

## 测试

运行测试：

```bash
cd build
ctest
```

## 贡献

欢迎提交 Issue 和 Pull Request。

## 许可证

请查看 LICENSE 文件。

## 联系方式

如有问题，请通过 GitHub Issues 联系。
