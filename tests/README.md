# 位姿变换回归测试

`TransformationTest` 工程用于验证 `CTransformation_EG` 与 `CTransformation_CV` 的
位姿 / 旋转矩阵 / 齐次矩阵 / Rodrigues 旋转向量之间的转换是否正确，并保留已修复缺陷的回归用例。

## 运行方式

在 Visual Studio 中把 `TransformationTest` 设为启动项目后按 F5，或直接运行：

```powershell
# 需要 OpenCV 的 dll 在 PATH 中
$env:PATH = "E:\installed\OpenCV\opencv_4.7.0_vc16\build\x64\vc16\bin;$env:PATH"
.\x64\Debug\TransformationTest.exe
```

命令行构建：

```powershell
msbuild Transformation.sln /p:Configuration=Debug /p:Platform=x64
```

退出码为失败用例个数，0 表示全部通过，便于接入 CI。

## 用例清单

| 分组 | 内容 |
| --- | --- |
| 容差常量 | `CHECK_ORTHOGONAL` 与 `BRANCH_ZERO_ANGLE` 的大小关系、零角度阈值不得掩盖真实小角度 |
| EG 欧拉角 | XYZ / ZYX 两种顺序各 2000 组随机位姿的矩阵 ↔ 旋转角往返 |
| EG 万向锁 | `ry = ±90°` 时解算结果可重建原矩阵 |
| EG 位姿往返 | 位姿 ↔ 齐次矩阵 |
| EG Rodrigues | 正反变换往返、近 π 两种分支、极小角度不被抹零 |
| EG 健壮性 | 非正交矩阵不产生 NaN、非法输入可被 `bool` 重载识别 |
| EG 角度单位 | `S_POSE` 版 Rodrigues 的 `angleType` 正确透传 |
| CV 同项 | 欧拉角、万向锁、位姿往返、Rodrigues 往返、角度单位透传 |
| CV 输入类型 | `CV_32F` / 4x4 / 非旋转矩阵不再崩溃且能被识别 |
| 一致性 | EG 与 CV 对同一输入的齐次矩阵与位姿必须一致 |

## 已知限制

`rotMatrix2RotRadian_Rodrigues()` 在旋转角接近 π 时存在**轴向符号二义性**：
`R + I = 2·k·kᵀ` 对 `k` 与 `-k` 不可区分，返回的旋转向量方向可能与输入相反。
两者代表同一旋转（重建后的矩阵一致），但若要对旋转向量做插值、滤波或误差计算，
需要先按业务约定统一符号。
