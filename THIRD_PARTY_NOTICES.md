# 来源、署名与许可边界

本仓库软件为从作者电赛项目中整理的离线标定工具；随仓数据为作者项目的传感器采集数值。未包含逐飞驱动、Fusion 源码、TI SDK、模型权重或外部二进制。

## 逐飞科技 SEEKFREE

- 上游：[MSPM0G3507_Library](https://gitee.com/seekfree/MSPM0G3507_Library)。
- 相关文件：`SeekFree_MSPM0G3507_Opensource_Library/libraries/zf_device/zf_device_imu660rb.c`。
- 本次核对的上游提交：`a3c9473defdca3093e7224030fdd8d734c3e1520`（2026-09-25 读取到的版本；不是对历史移植版本的追溯证明）。
- [本次核对的驱动文件](https://gitee.com/seekfree/MSPM0G3507_Library/blob/a3c9473defdca3093e7224030fdd8d734c3e1520/SeekFree_MSPM0G3507_Opensource_Library/libraries/zf_device/zf_device_imu660rb.c)；[上游 LICENSE](https://gitee.com/seekfree/MSPM0G3507_Library/blob/master/LICENSE)。
- 原整机的 `imu660rb.c/h` 依据逐飞 IMU660RB 驱动移植，保留了 SEEKFREE 来源及 GPL3.0 声明。本次直接读取上游驱动头：Copyright (c) 2022 SEEKFREE 逐飞科技，允许 GPL 第 3 版或任何后续版本（GPL-3.0-or-later）。同时获取上游 GPLv3 全文；不能追加非商业限制。

本仓库未复制该驱动。若后续公开包含此驱动的固件，必须保留版权与适用 GPL 声明并核对整个组合程序的发布条件；不得把本项目的非商业许可覆盖到逐飞代码上。

## x-io Technologies Fusion

- 上游：[xioTechnologies/Fusion](https://github.com/xioTechnologies/Fusion)。
- [MIT 许可](https://github.com/xioTechnologies/Fusion/blob/main/LICENSE.md)，Copyright (c) 2021 x-io Technologies。
- 原整机使用 FusionAhrs 和 FusionBias 完成姿态融合、偏置估计，本仓库的方案文档说明它们与标定参数的关系。

本次获取上游 MIT 许可并与本机项目中保存的声明核对。Fusion 算法属于第三方；“衡准 IMU”命名指本项目的采集、拟合和验证方案，不改变 Fusion 的作者或许可。本仓库未复制 Fusion 源码。

## 作者自有内容

Copyright (c) 2026 池金壕。

| 内容 | 许可 |
|---|---|
| `tools/`、`tests/` 内自有 Python 代码，`.github/` 内自有工作流配置 | [PolyForm Noncommercial 1.0.0](LICENSES/PolyForm-Noncommercial-1.0.0.txt) |
| README、文档、样例数值数据、清单与结果文件 | [CC BY-NC 4.0](LICENSES/CC-BY-NC-4.0.txt) |
| 第三方许可原文、名称和被引用内容 | 仍按相应原有权利与条款处理 |

未经另行授权，禁止商业使用作者自有内容。非商业使用仍需遵守完整许可，保留“池金壕 — 衡准 IMU / HengZhun IMU”署名、许可与修改说明。这里不把版权许可扩展为对一般数学方法、传感器原理或第三方算法的排他所有权。

完整许可原文来源：[PolyForm](https://polyformproject.org/licenses/noncommercial/1.0.0.txt)、[Creative Commons](https://creativecommons.org/licenses/by-nc/4.0/legalcode.txt)。本次随仓保存原文，不通过自拟短句替代其条款。
