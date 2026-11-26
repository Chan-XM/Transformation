# RobotPose / Transformation

三维空间中**位姿**（X/Y/Z + Rx/Ry/Rz）与**旋转矩阵**、**齐次矩阵**、**Rodrigues 旋转向量**之间的相互转换，
以及齐次矩阵求逆、位姿复合、位姿插值。提供 Eigen 与 OpenCV 两套实现，接口一一对应。

## 目录结构

```
Transformation/
├── CMakeLists.txt                  顶层构建脚本（生成 Visual Studio 工程到 build/）
├── CMakePresets.json               预设：生成器、依赖路径、Debug/Release、测试配置
├── src/                            核心库 TransformationCore（静态库）
│   ├── common.h                    公共定义：旋转顺序、角度单位、位姿结构体、容差常量
│   ├── transformation_eg.{h,cpp}   Eigen 实现
│   └── transformation_cv.{h,cpp}   OpenCV 实现
├── demo/                           使用示例 TransformationDemo
│   └── main.cpp
├── tests/                          回归测试 TransformationTest（10021 条断言）
│   ├── transformation_test.cpp
│   └── README.md                   用例清单与约定说明
└── build/                          构建产物（CMake 生成，已被 .gitignore 忽略）
    ├── Transformation.slnx          Visual Studio 解决方案（VS2026 生成新版 .slnx 格式）
    ├── bin/<Config>/               可执行文件 + 随构建复制的 OpenCV dll
    └── lib/<Config>/               TransformationCore.lib
```

## 依赖

| 依赖 | 版本 | 默认位置 |
| --- | --- | --- |
| Eigen | 3.4.0 | `E:/Package/Eigen/eigen-3.4.0`（仅需头文件，无需安装） |
| OpenCV | 4.7.0 预编译包 | `E:/installed/OpenCV/opencv_4.7.0_vc16/build` |
| CMake | ≥ 3.25 | 已在 4.3.2 上验证 |
| Visual Studio | 2026 | 生成器 `Visual Studio 18 2026`，平台 x64 |

换机器时只需改 `CMakePresets.json` 里的 `EIGEN3_INCLUDE_DIR` 与 `OpenCV_DIR`，
或在命令行传 `-DEIGEN3_INCLUDE_DIR=... -DOpenCV_DIR=...`。

## 构建与测试

```powershell
cmake --preset windows-x64        # 生成工程文件到 build/
cmake --build --preset debug      # Debug 构建；换成 --preset release 即 Release
ctest --preset debug              # 运行回归测试；换成 --preset release 即 Release
```

用 Visual Studio 开发时，直接打开 `build/Transformation.slnx`。
构建后 `build/bin/<Config>/` 下已自动复制对应版本的 OpenCV dll，不需要再设 `PATH`。

## CMake 选项

| 变量 | 默认值 | 说明 |
| --- | --- | --- |
| `EIGEN3_INCLUDE_DIR` | `E:/Package/Eigen/eigen-3.4.0` | Eigen 头文件目录（含 `Eigen/Dense` 的那一层） |
| `OpenCV_DIR` | `E:/installed/OpenCV/opencv_4.7.0_vc16/build` | OpenCV 的 build 目录（内含 `OpenCVConfig.cmake`） |
| `OpenCV_RUNTIME` / `OpenCV_ARCH` | `vc16` / `x64` | 显式指定预编译包标签，绕开 OpenCVConfig 按 MSVC 版本反推 |
| `TRANSFORMATION_COPY_OPENCV_DLL` | `ON` | 是否把 OpenCV dll 复制到可执行文件目录 |
| `BUILD_TESTING` | `ON` | 是否构建测试工程并注册到 CTest |

> `OpenCV_RUNTIME` 之所以要显式指定：本机预编译包由 VS2019(vc16) 构建，而默认工具集版本更新，
> OpenCV 的 `OpenCVConfig.cmake` 会按 MSVC 版本反推目录名并判定为「找不到 OpenCV」。
> 若将来换成与工具集匹配的 OpenCV 包，删掉 `CMakeLists.txt` 里那两行 `set(... CACHE ...)` 即可。

## 已知限制

见 `tests/README.md` 的「约定与已知限制」：容差常量的两类语义、近 π 的轴向符号约定与跨 π 跳变、
插值路径的唯一性、万向锁、欧拉角在 ±180° 处的等价性等。
