# 位姿变换回归测试

`TransformationTest` 工程用于验证 `CTransformation_EG` 与 `CTransformation_CV` 的
位姿 / 旋转矩阵 / 齐次矩阵 / Rodrigues 旋转向量之间的转换、求逆、复合与插值是否正确，
并保留已修复缺陷的回归用例。

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
| 容差常量 | `CHECK_ORTHOGONAL` 与 `BRANCH_ZERO_ANGLE` 的量级关系、零角度阈值不得掩盖真实小角度 |
| EG 欧拉角 | XYZ / ZYX 两种顺序各 2000 组随机位姿的矩阵 ↔ 旋转角往返 |
| EG 万向锁 | `ry = ±90°` 时解算结果可重建原矩阵 |
| EG 位姿往返 | 位姿 ↔ 齐次矩阵 |
| EG Rodrigues | 正反变换往返、近 π 两种分支、极小角度不被抹零 |
| EG 健壮性 | 非正交矩阵不产生 NaN、非法输入可被 `bool` 重载识别 |
| EG 角度单位 | `S_POSE` 版 Rodrigues 的 `angleType` 正确透传 |
| CV 同项 | 欧拉角、万向锁、位姿往返、Rodrigues 往返、角度单位透传 |
| CV 输入类型 | `CV_32F` / 4x4 / 非旋转矩阵不再崩溃且能被识别 |
| 一致性 | EG 与 CV 对同一输入的齐次矩阵与位姿必须一致 |
| 输入校验 | 含缩放的旋转矩阵、镜像矩阵、最后一行为非 `[0 0 0 1]` 的齐次矩阵均应被拒绝，且失败时输出被清零 |
| 近 π 轴向 | 返回的旋转向量须满足「绝对值最大的分量为正」，且 EG 与 CV 给出同一轴向 |
| 求逆 | `inv(T) * T = I`、与通用矩阵求逆一致、逆两次回原位姿、非法矩阵被拒绝 |
| 复合 | `compose(A, B) = A * B`、`A * A⁻¹ = I`、位姿级与矩阵级一致、非法输入被拒绝 |
| 插值 | 两端点精确复现、纯旋转与纯平移的中点正确、外插符合测地线定义、EG 与 CV 一致 |

## 约定与已知限制

- **输入校验容差**：旋转矩阵需满足正交且行列式为 1，容差为 `TRANSFORM_TOL::CHECK_ORTHOGONAL`（1e-6）；
  齐次矩阵另需最后一行为 `[0 0 0 1]`。数值分支阈值（`BRANCH_*`）与该校验容差是**分开**的，不要混用。
- **近 π 的轴向符号**：`rotMatrix2RotRadian_Rodrigues()` 在旋转角接近 π 时，`R + I = 2·k·kᵀ`
  对 `k` 与 `-k` 不可区分。当前实现统一固定为「绝对值最大的分量为正」，两套实现约定一致。
  但要注意：旋转角跨越 π 时轴角表示本身就会跳变（`(θ, k)` 与 `(2π−θ, −k)` 等价），
  若需要对旋转做插值或滤波，请改用 `interpolatePose()` 这类基于相对旋转的实现。
- **插值路径**：`interpolatePose()` 沿测地线（最短路径）插值，`t` 超出 `[0, 1]` 表示外插；
  相对旋转恰为 π 时插值路径本身不唯一，由 `Eigen::AngleAxisd` / `cv::Rodrigues` 的实现决定。
- **万向锁**：`ry = ±90°` 时 `rz` 被置零，`rx` 与 `rz` 的组合退化，此时只能保证重建的矩阵一致。
- **欧拉角在 ±180° 处的等价性**：断言请比较旋转矩阵而非角度分量。
