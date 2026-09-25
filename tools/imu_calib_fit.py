# -*- coding: utf-8 -*-
# Copyright (c) 2026 池金壕
# SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
"""衡准 IMU：IMU660RB 标定拟合工具（采集流程见 docs/acquisition.md）

用法：
  python imu_calib_fit.py sixface  六面.log [更多文件...]
  python imu_calib_fit.py yawscale 逆3圈.log:+3 [顺3圈.log:-3 ...]
  python imu_calib_fit.py gyroscale 绕X3圈.log:x:+3 [绕Y3圈.log:y:+3 ...]

日志格式：串口工具原始保存即可——允许 "[hh:mm:ss.mmm] Rx: " 前缀、断行断在
数字/记录中间；记录为 11 字段 CSV：
  ms,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,yaw10,pitch10,roll10,temp_c10

⚠️ 拟合前提：采数据时 imu660rb_calib.h 必须是恒等值（BIAS=0/SCALE=1）。
"""

import re
import sys
import statistics
import math

ACC_LSB_PER_G    = 4098.36          # ±8g，0.244 mg/LSB
GYRO_LSB_PER_DPS = 1000.0 / 70.0    # ±2000dps，70 mdps/LSB
NFIELD           = 11
FS_HZ            = 50.0             # CSV 遥测帧率
STATIC_WIN       = 50               # 静止窗口长度（1s）
ACC_STD_LIMIT    = 25.0             # 静止判据：加计各轴 std 上限（LSB）
GYRO_ABS_LIMIT   = 30.0             # 静止判据：陀螺各轴均值绝对值上限（LSB）

# ---------------------------------------------------------------- 解析

_RX_PREFIX = re.compile(r'^\[\d{2}:\d{2}:\d{2}\.\d{3}\]\s*Rx:\s?')
_INTS      = re.compile(r'-?\d+(,-?\d+)*$')
_PARTIAL   = re.compile(r'-?\d+(,-?\d+)*(,-?|-)?$')   # 允许尾随 , 或 -（断行断在数字/符号中间）


def _payload_lines(path):
    with open(path, 'r', encoding='utf-8', errors='replace') as fh:
        return [_RX_PREFIX.sub('', ln).strip() for ln in fh.read().splitlines()]


def _to_ints(s):
    if not s or not _INTS.match(s):
        return None
    try:
        return [int(x) for x in s.split(',')]
    except ValueError:
        return None


def _fields_ok(f, prev_ms):
    if len(f) != NFIELD:
        return False
    if f[0] % 20 != 0 or not (0 <= f[0] < 10 ** 8):
        return False
    if not (150 <= f[10] <= 500):                     # temp_c10 合理区间 15~50°C
        return False
    if any(abs(v) > 40000 for v in f[1:10]):
        return False
    if prev_ms is not None:
        d = f[0] - prev_ms
        fresh_boot = f[0] < 60000                     # 设备中途重启：ms 归零重来
        if not (0 < d <= 600000) and not fresh_boot:  # 容忍 ≤10min 的日志暂停
            return False
    return True


def parse_records(path):
    """断行重组解析：缓冲区凑不满 11 个合法字段就与下一行无缝拼接（断行可断在
    数字/逗号/负号中间），凑满且通过 sanity 检查即产出一条记录；拼坏则重新同步。"""
    recs, dropped = [], 0
    prev_ms, buf = None, ''
    lines = _payload_lines(path)
    i = 0
    while True:
        f = _to_ints(buf)
        if f is not None and _fields_ok(f, prev_ms):
            recs.append(f)
            prev_ms = f[0]
            buf = ''
            continue
        if f is not None and len(f) > NFIELD:          # 拼过头（罕见边界），作废重同步
            dropped += 1
            buf = ''
            continue
        if i >= len(lines):
            if buf:
                dropped += 1
            break
        nxt = lines[i]
        i += 1
        if not nxt:
            continue
        cand = buf + nxt
        if _PARTIAL.match(cand):
            buf = cand
        else:
            if buf:
                dropped += 1
            buf = nxt if _PARTIAL.match(nxt) else ''
    return recs, dropped


def _load(path):
    recs, dropped = parse_records(path)
    if not recs:
        sys.exit(f'[错误] {path}: 未解析出任何记录，检查文件格式')
    if dropped:
        sys.exit(f'[错误] {path}: 存在 {dropped} 段无法重组的数据；请检查原始日志后重采或重新导出')
    for previous, current in zip(recs, recs[1:]):
        if current[0] - previous[0] != 20:
            sys.exit(f'[错误] {path}: 时间戳必须连续递增 20ms；'
                     f'发现 {previous[0]} -> {current[0]}，请检查丢帧、重复或设备复位')
    span = (recs[-1][0] - recs[0][0]) / 1000.0
    print(f'[解析] {path}: {len(recs)} 条记录, 跨度 {span:.1f}s, 重同步丢弃 {dropped} 段')
    return recs

# ---------------------------------------------------------------- 静止窗口

def _static_windows(recs):
    """返回 [(起始下标, acc均值[3], gyro均值[3]), ...]，窗口不重叠。"""
    out = []
    i = 0
    while i + STATIC_WIN <= len(recs):
        win = recs[i:i + STATIC_WIN]
        acc = list(zip(*[(r[1], r[2], r[3]) for r in win]))
        gyr = list(zip(*[(r[4], r[5], r[6]) for r in win]))
        if (max(statistics.pstdev(a) for a in acc) < ACC_STD_LIMIT
                and max(abs(statistics.fmean(g)) for g in gyr) < GYRO_ABS_LIMIT
                and max(statistics.pstdev(g) for g in gyr) < ACC_STD_LIMIT):
            out.append((i, [statistics.fmean(a) for a in acc],
                        [statistics.fmean(g) for g in gyr]))
            i += STATIC_WIN
        else:
            i += 5
    return out

# ---------------------------------------------------------------- 六面拟合

_AXIS = 'XYZ'


def _turns(value):
    try:
        turns = float(value)
    except ValueError:
        sys.exit(f'[错误] 圈数必须是非零有限数值，收到 {value}')
    if not math.isfinite(turns) or turns == 0:
        sys.exit(f'[错误] 圈数必须是非零有限数值，收到 {value}')
    return turns


def cmd_sixface(paths):
    faces = {}                                        # (轴号, ±1) -> [各窗口该轴均值向量]
    for p in paths:
        recs = _load(p)
        for _, acc, _g in _static_windows(recs):
            norm = sum(a * a for a in acc) ** 0.5
            if not (0.9 * ACC_LSB_PER_G < norm < 1.1 * ACC_LSB_PER_G):
                continue                              # 模长离 1g 太远（运动/振动残留），弃
            for ax in range(3):
                if abs(acc[ax]) / norm > 0.94:        # 主导轴（倾斜 < ~20°）
                    faces.setdefault((ax, 1 if acc[ax] > 0 else -1), []).append(acc)
    missing = [f'{"+-"[s < 0]}{_AXIS[ax]}' for ax in range(3) for s in (1, -1)
               if (ax, s) not in faces]
    if missing:
        sys.exit(f'[错误] 缺少朝向 {missing} 的静止数据，补采后重跑')

    print('\n各朝向静止窗口数:', {f'{"+-"[s < 0]}{_AXIS[ax]}': len(v)
                                  for (ax, s), v in sorted(faces.items())})
    bias, scale = [0.0] * 3, [1.0] * 3
    for ax in range(3):
        up = statistics.fmean(m[ax] for m in faces[(ax, 1)])
        dn = statistics.fmean(m[ax] for m in faces[(ax, -1)])
        bias[ax] = (up + dn) / 2.0
        scale[ax] = 2.0 * ACC_LSB_PER_G / (up - dn)
        print(f'  {_AXIS[ax]}: +1g读数 {up:8.1f}  -1g读数 {dn:8.1f}  '
              f'bias {bias[ax]:+7.1f} LSB  scale {scale[ax]:.5f}')

    print('\n// ==== 粘贴到 imu660rb_calib.h（并更新标定履历日期）====')
    for ax in range(3):
        print(f'#define IMU660RB_ACC_BIAS_{_AXIS[ax]}     ( {bias[ax]:.1f}f )')
    for ax in range(3):
        print(f'#define IMU660RB_ACC_SCALE_{_AXIS[ax]}    ( {scale[ax]:.5f}f )')

    # 残差自检：套用拟合值后各面模长应≈1g
    errs = []
    for (ax, s), ms in faces.items():
        for m in ms:
            c = [(m[k] - bias[k]) * scale[k] for k in range(3)]
            errs.append(abs(sum(v * v for v in c) ** 0.5 / ACC_LSB_PER_G - 1.0))
    print(f'// 自检：拟合后模长残差 max {max(errs) * 100:.2f}% '
          f'(修正前 z 轴约 2%)，>0.5% 说明采集面不稳，建议重采')

# ---------------------------------------------------------------- yaw 刻度（用解算 yaw，200Hz 内部积分）

def _parse_spec(spec, want_axis):
    # 从右侧拆分，避免 Windows 盘符冒号（C:/...）干扰
    if want_axis:
        parts = spec.rsplit(':', 2)
        if len(parts) != 3 or parts[1].lower() not in ('x', 'y', 'z'):
            sys.exit(f'[错误] 参数格式应为 文件:轴:圈数，如 log.txt:x:+3，收到 {spec}')
        return parts[0], 'xyz'.index(parts[1].lower()), _turns(parts[2])
    parts = spec.rsplit(':', 1)
    if len(parts) != 2:
        sys.exit(f'[错误] 参数格式应为 文件:圈数列表，如 log.txt:+3 或 log.txt:+3,-3，收到 {spec}')
    try:
        return parts[0], None, [_turns(x) for x in parts[1].split(',')]
    except ValueError:
        sys.exit(f'[错误] 圈数解析失败：{spec}（应如 log.txt:+3 或 log.txt:+3,-3）')


def _end_statics(recs):
    wins = _static_windows(recs)
    if len(wins) < 2 or wins[0][0] > len(recs) // 3 or wins[-1][0] < len(recs) * 2 // 3:
        sys.exit('[错误] 找不到首尾静止段——采集时动作前后各静置 3s 以上')
    return wins[0], wins[-1]


def _plateaus(recs):
    """把静止窗口聚成平台：[(首窗下标, 末窗下标, yaw 均值, gz 均值), ...]。
    相邻窗口时间接近且 yaw 相近 → 同一平台。"""
    groups = []
    for i, _acc, g in _static_windows(recs):
        yaw = statistics.fmean(r[7] for r in recs[i:i + STATIC_WIN]) / 10.0
        if groups and i - groups[-1][-1][0] <= STATIC_WIN * 3 and abs(yaw - groups[-1][-1][1]) < 3.0:
            groups[-1].append((i, yaw, g[2]))
        else:
            groups.append([(i, yaw, g[2])])
    # 平台锚点：进平台取首窗、出平台取末窗（旋转段两端各自最近的静止值）
    return [(g[0][0], g[-1][0], statistics.fmean(y for _, y, _z in g),
             statistics.fmean(z for _, _y, z in g)) for g in groups]


def cmd_yawscale(specs):
    scales = []
    for spec in specs:
        path, _, turns = _parse_spec(spec, want_axis=False)
        recs = _load(path)
        plat = _plateaus(recs)
        if len(plat) != len(turns) + 1:
            sys.exit(f'[错误] {path}: 找到 {len(plat)} 个静止平台（yaw: '
                     f'{[f"{p[2]:+.1f}" for p in plat]}），{len(turns)} 段旋转需要 '
                     f'{len(turns) + 1} 个——每段旋转前后都要贴参考静置 ≥3s')
        for k, tr in enumerate(turns):
            meas = plat[k + 1][2] - plat[k][2]
            true = 360.0 * tr
            # 半圈误计检测：矩形板贴直边每 180° 就贴齐一次，数圈常差半圈
            half_turns = round(meas / 180.0) / 2.0
            if abs(half_turns - tr) > 0.01:
                sys.exit(f'[错误] {path} 第{k + 1}段: 实测 {meas:+.1f}° ≈ {half_turns:+.1f} 圈，'
                         f'与标称 {tr:+.0f} 圈不符——若差半圈=贴边参考混淆（板转180°后边仍贴齐），'
                         f'请用「角对角」标记消除歧义后重采；确认实际就是 {half_turns:+.1f} 圈'
                         f'则以其为圈数重跑')
            # 启动窗污染检测：融合 yaw 与去零偏原始积分应一致（上电 ~5s 内旋转会被
            # Fusion 启动高增益踢坏 yaw，见 FusionAhrs startup/rampedGain）。
            # 零偏取两端平台全长均值——单窗估计噪声 ±0.3dps × 段长会造成 ±10° 量级假警报
            a, b = plat[k][1], plat[k + 1][0]
            bias = (plat[k][3] + plat[k + 1][3]) / 2.0
            integ = sum(r[6] - bias for r in recs[a:b]) * (1.0 / FS_HZ) / GYRO_LSB_PER_DPS
            if abs(meas - integ) > max(10.0, abs(true) * 0.012):
                if recs[a][0] < 8000:                     # 段起点在上电 8s 内才可能是启动窗污染
                    sys.exit(f'[错误] {path} 第{k + 1}段: 融合yaw Δ={meas:+.1f}° 与去零偏原始积分 '
                             f'{integ:+.1f}° 相差 {meas - integ:+.1f}°，且旋转始于上电 '
                             f'{recs[a][0] / 1000.0:.1f}s——Fusion 启动高增益窗污染 yaw，'
                             f'重采时先静置 ≥5s 再动')
                print(f'  [警告] 第{k + 1}段融合与原始积分差 {meas - integ:+.1f}°'
                      f'（50Hz 原始积分欠采样/零偏漂移所致，拟合以融合值为准）')
            s = true / meas
            scales.append(s)
            print(f'  {path} 第{k + 1}段: 实测 {meas:+9.2f}°  应为 {true:+.0f}°  scale {s:.5f}  '
                  f'(闭合误差 {meas - true:+.2f}° = {(meas - true) / tr:+.2f}°/圈; '
                  f'原始积分交叉验证差 {meas - integ:+.1f}°)')
    k = statistics.fmean(scales)
    spread = (max(scales) - min(scales)) if len(scales) > 1 else 0.0
    print(f'\n// ==== 粘贴到 imu660rb_calib.h ====')
    print(f'#define IMU660RB_GYRO_SCALE_Z   ( {k:.5f}f )')
    print(f'// 各段散布 {spread * 100:.3f}%（>0.3% 说明对齐参考不稳，建议重采）')

# ---------------------------------------------------------------- 陀螺 x/y 刻度（原始积分，扣首段零偏）

def cmd_gyroscale(specs):
    result = {}
    for spec in specs:
        path, ax, turns = _parse_spec(spec, want_axis=True)
        recs = _load(path)
        (i0, _, g0), _ = _end_statics(recs)
        bias = g0[ax]
        integ = sum(r[4 + ax] - bias for r in recs) / FS_HZ / GYRO_LSB_PER_DPS
        if abs(integ) < 1e-6 or integ * turns <= 0:
            sys.exit('[错误] 积分角度为零或与标称旋转方向不一致，请核对轴向和采集动作')
        k = 360.0 * turns / integ
        result.setdefault(ax, []).append(k)
        print(f'  {path}: {_AXIS[ax]} 轴积分 {integ:+9.2f}°  应为 {360 * turns:+.0f}°  scale {k:.5f}')
    print(f'\n// ==== 粘贴到 imu660rb_calib.h ====')
    for ax, ks in sorted(result.items()):
        print(f'#define IMU660RB_GYRO_SCALE_{_AXIS[ax]}   ( {statistics.fmean(ks):.5f}f )')
    print('// 注意：CSV 50Hz 积分对快速旋转有欠采样误差，采集时须缓慢匀速（≤90°/s）')

# ---------------------------------------------------------------- 入口

if __name__ == '__main__':
    if sys.argv[1:] in (['--help'], ['-h']):
        print(__doc__)
        sys.exit(0)
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(1)
    cmd, args = sys.argv[1], sys.argv[2:]
    if cmd == 'sixface':
        cmd_sixface(args)
    elif cmd == 'yawscale':
        cmd_yawscale(args)
    elif cmd == 'gyroscale':
        cmd_gyroscale(args)
    else:
        print(__doc__)
        sys.exit(f'[错误] 未知命令 {cmd}')
