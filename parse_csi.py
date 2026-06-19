#!/usr/bin/env python3
"""
解析 ESP32-S3 CSI 二进制导出文件 (csi_dump.bin)

每条记录格式:
┌──────────────┬────────────────┬──────────┬──────────────┬───────────┬──────────────┐
│ magic (4B)   │ timestamp (8B) │ rssi(1B) │ src_mac (6B) │ len (2B)  │ csi_buf (nB) │
│ 0x43534921   │ uint64_t (us)  │ int8_t   │              │ uint16_t  │ raw bytes    │
└──────────────┴────────────────┴──────────┴──────────────┴───────────┴──────────────┘

CSI buf 内结构 (ltf_merge_en=true):
  - Header: 24 字节 (wifi_csi_header_t)
  - 子载波: 每对 2x int16 (real, imag) = 4 字节

用法:
  python parse_csi.py csi_dump.bin                    # 统计所有记录
  python parse_csi.py csi_dump.bin --csv out.csv      # 导出 CSV
  python parse_csi.py csi_dump.bin --plot             # 绘制第一条记录的幅值
  python parse_csi.py csi_dump.bin --plot 5           # 绘制第 5 条记录
"""

import struct
import argparse
import os
import sys

# ─── 常量 ─────────────────────────────────────────────
CSI_RECORD_MAGIC  = 0x43534921
CSI_HEADER_SIZE   = 24          # CSI buffer 头部大小 (字节)
SUBCARRIER_SIZE   = 4           # 每个子载波 4 字节 (real int16 + imag int16)
RECORD_HEADER_FMT = "<IQb6sH"   # magic, timestamp, rssi, mac, len
RECORD_HEADER_SZ  = struct.calcsize(RECORD_HEADER_FMT)  # 21 字节


def find_all_records(filepath):
    """扫描文件，返回所有记录的 (offset, header_dict) 列表"""
    with open(filepath, "rb") as f:
        data = f.read()

    file_size = len(data)
    records = []
    offset = 0

    while offset + RECORD_HEADER_SZ <= file_size:
        # 搜索 magic
        magic = struct.unpack_from("<I", data, offset)[0]
        if magic != CSI_RECORD_MAGIC:
            offset += 1
            continue

        # 解析头部
        hdr_raw = data[offset : offset + RECORD_HEADER_SZ]
        magic_v, ts_us, rssi, mac_raw, csi_len = struct.unpack(
            RECORD_HEADER_FMT, hdr_raw
        )

        # 简化长度检查 (len 可能为 0 表示异常)
        if csi_len == 0:
            offset += RECORD_HEADER_SZ
            continue

        if offset + RECORD_HEADER_SZ + csi_len > file_size:
            break  # 数据不完整，结束

        mac_str = ":".join(f"{b:02X}" for b in mac_raw)

        records.append({
            "offset":   offset,
            "ts_us":    ts_us,
            "ts_s":     ts_us / 1_000_000.0,
            "rssi":     rssi,
            "mac":      mac_str,
            "csi_len":  csi_len,
        })
        offset += RECORD_HEADER_SZ + csi_len

    return records, data


def extract_subcarriers(data, record):
    """从 record 起始位置提取子载波 I/Q 数据"""
    csi_start = record["offset"] + RECORD_HEADER_SZ
    csi_end   = csi_start + record["csi_len"]
    csi_raw   = data[csi_start:csi_end]

    # 跳过 24 字节 header
    payload = csi_raw[CSI_HEADER_SIZE:]
    n_sub = len(payload) // SUBCARRIER_SIZE

    subcarriers = []
    for i in range(n_sub):
        real, imag = struct.unpack_from("<hh", payload, i * SUBCARRIER_SIZE)
        subcarriers.append((real, imag))
    return subcarriers


def subcarrier_to_amplitude(sub):
    """计算幅值列表"""
    import math
    return [math.sqrt(r * r + i * i) for r, i in sub]


def subcarrier_to_phase(sub):
    """计算相位列表 (弧度, 缠绕在 [-π, π])"""
    import math
    return [math.atan2(i, r) for r, i in sub]


def phase_unwrap(phases):
    """相位解缠绕：消除 2π 跳变, 使相位连续"""
    import math
    unwrapped = list(phases)
    for i in range(1, len(unwrapped)):
        diff = unwrapped[i] - unwrapped[i - 1]
        # 将 diff 规整到 [-π, π] 的跳变倍数
        unwrapped[i] -= round(diff / (2 * math.pi)) * (2 * math.pi)
    return unwrapped


# ─── CSV 导出 ─────────────────────────────────────────
def export_csv(records, data, outpath):
    """导出所有记录的子载波数据到 CSV (含幅值、缠绕相位、解缠相位)"""
    with open(outpath, "w", encoding="utf-8") as f:
        for idx, rec in enumerate(records):
            sub = extract_subcarriers(data, rec)
            amps = subcarrier_to_amplitude(sub)
            phases = subcarrier_to_phase(sub)
            phases_uw = phase_unwrap(phases)
            if idx == 0:
                header = ["index", "ts_us", "ts_s", "rssi", "mac", "n_sub"]
                for i in range(len(amps)):
                    header += [f"amp_{i}", f"phase_w_{i}", f"phase_uw_{i}"]
                f.write(",".join(header) + "\n")
            row = [str(idx), str(rec["ts_us"]), f"{rec['ts_s']:.6f}",
                   str(rec["rssi"]), rec["mac"], str(len(sub))]
            for i in range(len(amps)):
                row += [f"{amps[i]:.2f}", f"{phases[i]:.4f}", f"{phases_uw[i]:.4f}"]
            f.write(",".join(row) + "\n")
    print(f"Exported {len(records)} records → {outpath}")


# ─── 绘图 ─────────────────────────────────────────────
def plot_record(records, data, index):
    """绘制指定索引记录的幅值 & 相位"""
    try:
        import matplotlib.pyplot as plt
        import numpy as np
    except ImportError:
        print("需要 matplotlib: pip install matplotlib")
        return

    if index < 0 or index >= len(records):
        print(f"记录索引 {index} 超出范围 (0 ~ {len(records)-1})")
        return

    rec = records[index]
    sub = extract_subcarriers(data, rec)
    amps = subcarrier_to_amplitude(sub)
    phases = subcarrier_to_phase(sub)
    phases_unwrapped = phase_unwrap(phases)
    x = np.arange(len(amps))

    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

    ax1.stem(x, amps, basefmt=" ", linefmt="C0-", markerfmt="C0o")
    ax1.set_ylabel("Amplitude")
    ax1.set_title(f"CSI Record #{index}  |  MAC={rec['mac']}  "
                  f"RSSI={rec['rssi']}dBm  n_sub={len(sub)}  "
                  f"ts={rec['ts_s']:.3f}s")
    ax1.grid(True, alpha=0.3)

    ax2.stem(x, phases, basefmt=" ", linefmt="C1-", markerfmt="C1o")
    ax2.set_ylabel("Phase (rad)\nWrapped")
    ax2.set_ylim(-np.pi - 0.5, np.pi + 0.5)
    ax2.grid(True, alpha=0.3)

    ax3.stem(x, phases_unwrapped, basefmt=" ", linefmt="C3-", markerfmt="C3o")
    ax3.set_ylabel("Phase (rad)\nUnwrapped")
    ax3.set_xlabel("Subcarrier Index")
    ax3.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()


# ─── 统计概要 ─────────────────────────────────────────
def print_summary(records, data):
    """打印所有记录的统计信息"""
    if not records:
        print("No CSI records found.")
        return

    rssi_list = [r["rssi"] for r in records]
    ts_first  = records[0]["ts_s"]
    ts_last   = records[-1]["ts_s"]
    mac_set   = set(r["mac"] for r in records)

    print(f"=== CSI Dump Summary ===")
    print(f"Total records:    {len(records)}")
    print(f"Time span:        {ts_last - ts_first:.2f} s")
    print(f"RSSI range:       {min(rssi_list)} ~ {max(rssi_list)} dBm")
    print(f"RSSI mean:        {sum(rssi_list)/len(rssi_list):.1f} dBm")
    print(f"Unique MACs:      {len(mac_set)}")
    for mac in sorted(mac_set):
        cnt = sum(1 for r in records if r["mac"] == mac)
        print(f"  {mac}: {cnt} packets")

    # 子载波数量采样
    sub = extract_subcarriers(data, records[0])
    print(f"Subcarriers/rec:  {len(sub)} (from first record)")
    print(f"Data size/rec:    {records[0]['csi_len']} bytes")
    print()


# ─── 主入口 ───────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(
        description="ESP32-S3 CSI binary dump parser")
    parser.add_argument("file", help="CSI dump binary file (csi_dump.bin)")
    parser.add_argument("--csv", metavar="OUT.csv", help="Export to CSV")
    parser.add_argument("--plot", nargs="?", const=0, type=int,
                        metavar="INDEX", help="Plot record by index (default: 0)")
    args = parser.parse_args()

    if not os.path.exists(args.file):
        print(f"File not found: {args.file}")
        sys.exit(1)

    records, raw_data = find_all_records(args.file)
    print_summary(records, raw_data)

    if args.csv:
        export_csv(records, raw_data, args.csv)

    if args.plot is not None:
        plot_record(records, raw_data, args.plot)


if __name__ == "__main__":
    main()
