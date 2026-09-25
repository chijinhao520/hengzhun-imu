# 衡准 IMU · HengZhun IMU

**六面标定 · 双向旋转校准 · 航向误差分析**

A reproducible IMU calibration workflow with real sensor samples and a Python fitting tool.

衡准 IMU 从车载平衡滚球工程的传感器调试中整理而来：把采集、数据检查、参数拟合和接入验证连接起来，用可复算记录解释加速度计零偏、比例误差与航向漂移的不同来源。

“衡准”是本项目标定方案与工具的名称。姿态融合来自 **x-io Fusion**，原嵌入式 IMU 驱动基于 **逐飞科技**代码移植；这里不将第三方算法更名为自研成果。

> **禁止商业使用，商业用途须另行授权。** 自有代码采用 PolyForm Noncommercial 1.0.0，自有文档与数据采用 CC BY-NC 4.0。完整范围见 [LICENSE](LICENSE)，署名见 [NOTICE](NOTICE)。这是非商业许可的源码公开项目。

## 先跑一个真实样例

需要 Python 3.10 或更高版本，**只使用标准库，无需安装第三方 Python 包，也无需连接硬件**。在仓库根目录运行：

```bash
python tools/imu_calib_fit.py sixface examples/data/six-face.csv
python tools/imu_calib_fit.py yawscale examples/data/yaw-three-turns.csv:+3,-3
python -m unittest discover -s tests -v
```

工具只读取日志并打印结果，不会自动改固件、连接串口或烧录。

## 这个方案包含什么

| 问题 | 做法 | 产物 |
|---|---|---|
| 六个静止朝向读数不对称、模长不一致 | 检查六面静止窗口，分别拟合每轴 bias / scale | 可回填的加速度计标定参数 |
| 已知转三圈，yaw 变化不等于 1080° | 用顺、逆方向旋转与首尾静止平台拟合 Z 比例系数 | 双向系数、段间差和交叉检查 |
| 对齐错误或启动阶段干扰使拟合失真 | 圈数一致性、平台数量、启动期积分交叉检查 | 拒收原因或明确提示 |
| 固定 50 Hz 假设被丢帧打破 | 要求连续 20 ms 时间戳，拒绝缺帧、重复和复位拼接 | 输入质量检查 |
| 参数正确但装车后方向不一致 | 分清驱动系、车体系和调平矩阵的作用顺序 | [接入说明](docs/integration.md) |

## 可复算结果

样例来自 IMU660RB（ST LSM6DSR），加速度计 ±8g、陀螺 ±2000°/s；历史采集记录标注为 2026-07-19。仓库包含筛选出的全部合法数值记录与 [SHA256 清单](examples/data/manifest.json)。

| 样例 | 结果 | 能说明什么 |
|---|---|---|
| 六面，2343 条记录 | bias：−8.9 / −40.5 / −78.1 LSB；scale：1.00784 / 1.00061 / 1.00490 | 当前样机各轴的零偏与比例修正 |
| 六面拟合后自检 | 最大模长残差 0.46% | 同一拟合样本内的模长残差，不是姿态角精度 |
| 双向各三圈，4225 条记录 | Z 比例系数 1.00230；两段系数之差 0.096% | 双向标定的一致程度，不是独立验收或置信区间 |

完整输出：[六面](examples/results/six-face.txt)、[双向转圈](examples/results/yaw-three-turns.txt)。这些参数只属于原样机，不能当作其他模块的通用系数。

## 推荐阅读顺序

1. [采集流程](docs/acquisition.md)：恢复恒等校准、静置、六面动作与角对角参考。
2. [日志格式与数据来源](docs/data-format.md)：字段、单位、时间基准和数值导出方法。
3. [误差来源与能力边界](docs/error-sources.md)：比例、零偏、温漂、安装误差分别如何处理。
4. [MSPM0 / Fusion 接入](docs/integration.md)：标定参数到姿态链路的衔接，以及需补的硬件防护。
5. [来源与许可](THIRD_PARTY_NOTICES.md)：逐飞、Fusion 与本项目自有内容的边界。

本地 12 项离线回归已通过，适用范围见 [首版验证记录](docs/validation.md)。

```text
tools/                 标定拟合 CLI，纯 Python 标准库
tests/                 样例复算、断行解析及异常输入回归
examples/data/         两份真实传感器数值样例及来源哈希
examples/results/      同一工具生成的预期输出
docs/                  采集、格式、误差与接入说明
LICENSES/              完整软件及文档/数据许可
```

## 当前范围

首版聚焦离线标定流程，包含 `sixface`、`yawscale` 和实验性的 `gyroscale`。后者尚无真实 X/Y 轴标定样例随仓验证。没有打包逐飞驱动、Fusion、TI SDK 或可直接烧录的工程；这些组件需按各自许可从上游获取。

六轴 IMU 没有磁力计时不能仅靠该方案保证绝对航向长期无漂移。目前没有建立独立温度补偿曲线，也没有把历史标定样例当作校准后实板验收。下一步重点是补充独立静置漂移与闭合误差测试，而不是扩大精度承诺。

## 与电赛工程的关系

电赛整机使用了本方案的历史标定流程和参数。后续电赛仓库会引用本项目的固定版本，同时保留比赛固件与配置快照；当前不提供尚未发布的电赛仓库链接。

## 作者与致谢

作者：**池金壕**。项目整理和研发中使用 AI 辅助；标定工具、数据链路和工程结论以可检查的实现与记录为依据。感谢逐飞科技的 IMU 驱动与 x-io Technologies 的 Fusion 姿态库，具体贡献与许可见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。
