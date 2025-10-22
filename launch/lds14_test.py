#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LD14(LDS-14) LiDAR 간단 테스트 (angle, distance 출력 전용)
 - Ubuntu 전용 (Python3)
 - 포트: /dev/ttyUSB0, Baud: 230400
"""

import serial
import struct
import time

PORT = '/dev/ttyUSB0'
BAUD = 115200
TIMEOUT = 1.0

HEADER = b'\x54\x2C'  # LD14 Typical header
FRAME_LEN = 47        # 일반적인 프레임 길이

def find_header(buf):
    """버퍼에서 헤더(0x54 0x2C)를 찾음"""
    return buf.find(HEADER)

def parse_frame(frame):
    """프레임 파싱: 시작각, 끝각, 거리 12개를 단순 추출"""
    if len(frame) != FRAME_LEN:
        return None

    try:
        start_angle_raw = struct.unpack_from('<H', frame, 4)[0]
        start_angle = start_angle_raw / 100.0
        end_angle_raw = struct.unpack_from('<H', frame, 42)[0]
        end_angle = end_angle_raw / 100.0
    except struct.error:
        return None

    N = 12
    distances = []
    offset = 6
    try:
        for i in range(N):
            d = struct.unpack_from('<H', frame, offset + 2*i)[0]
            distances.append(d / 1000.0)
    except struct.error:
        return None

    # 각도 보간
    if end_angle >= start_angle:
        step = (end_angle - start_angle) / max(N - 1, 1)
    else:
        wrapped = (end_angle + 360.0) - start_angle
        step = wrapped / max(N - 1, 1)

    angles = [(start_angle + step*i) % 360.0 for i in range(N)]
    return list(zip(angles, distances))

def main():
    print(f"🔌 Opening {PORT} @ {BAUD} ...")
    try:
        ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT)
    except Exception as e:
        print(f"[ERROR] Serial open failed: {e}")
        return

    time.sleep(0.2)
    ser.reset_input_buffer()
    buf = bytearray()

    try:
        while True:
            data = ser.read(512)
            if data:
                buf.extend(data)

            idx = find_header(buf)
            if idx >= 0 and len(buf) >= idx + FRAME_LEN:
                frame = bytes(buf[idx:idx+FRAME_LEN])
                del buf[:idx+FRAME_LEN]
                parsed = parse_frame(frame)
                if parsed:
                    for a, d in parsed:
                        print(f"{a:6.2f}°, {d:5.3f} m")

    except KeyboardInterrupt:
        print("\n[STOP] Interrupted by user.")
    finally:
        try:
            ser.close()
        except:
            pass

if __name__ == '__main__':
    main()
