#!/usr/bin/env python3
"""Serial protocol integration tool for rm_serial_driver."""

import argparse
import json
import struct
import sys
import time
from typing import Dict, Optional


CMD_PACKET_HEADER = 0xA5
FEEDBACK_PACKET_HEADER = 0xA6
CMD_PACKET_LENGTH = 32
FEEDBACK_PACKET_LENGTH = 32

CMD_STRUCT = struct.Struct("<BBBfffffBB5sH")
FEEDBACK_STRUCT = struct.Struct("<BBBfffffBBB4sH")

MODE_MAP = {
    "noop": 0,
    "joint": 1,
    "gripper": 2,
    "homing": 3,
}

ACTION_MAP = {
    "none": 0,
    "execute": 1,
    "pause": 2,
    "resume": 3,
    "clear_fault": 4,
    "estop": 5,
}


def crc16(frame_without_crc: bytes) -> int:
    """CRC16 implementation aligned with rm_serial_driver/crc.cpp."""
    crc = 0xFFFF
    for byte in frame_without_crc:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0x8408
            else:
                crc >>= 1
    return crc & 0xFFFF


def append_crc(frame: bytearray) -> None:
    checksum = crc16(frame[:-2])
    frame[-2] = checksum & 0xFF
    frame[-1] = (checksum >> 8) & 0xFF


def verify_crc(frame: bytes) -> bool:
    if len(frame) < 2:
        return False
    expected = crc16(frame[:-2])
    actual = frame[-2] | (frame[-1] << 8)
    return actual == expected


def build_cmd_packet(
    seq: int,
    mode: int,
    cmd_action: int,
    j1: float,
    j2: float,
    j3: float,
    j4: float,
    gripper: float,
    inject_bad_length: bool,
    inject_bad_crc: bool,
) -> bytes:
    length = CMD_PACKET_LENGTH if not inject_bad_length else (CMD_PACKET_LENGTH - 1)
    frame = bytearray(
        CMD_STRUCT.pack(
            CMD_PACKET_HEADER,
            length,
            seq & 0xFF,
            float(j1),
            float(j2),
            float(j3),
            float(j4),
            float(gripper),
            mode & 0xFF,
            cmd_action & 0xFF,
            b"\x00" * 5,
            0,
        )
    )
    append_crc(frame)
    if inject_bad_crc:
        frame[-2] ^= 0x01
    return bytes(frame)


def decode_feedback_packet(frame: bytes) -> Dict[str, object]:
    (
        header,
        length,
        seq,
        j1,
        j2,
        j3,
        j4,
        gripper,
        online,
        status,
        error_code,
        _reserved,
        checksum,
    ) = FEEDBACK_STRUCT.unpack(frame)
    return {
        "header": f"0x{header:02X}",
        "length": length,
        "seq": seq,
        "j1": j1,
        "j2": j2,
        "j3": j3,
        "j4": j4,
        "gripper": gripper,
        "online": online,
        "status": status,
        "error_code": error_code,
        "checksum": checksum,
    }


def read_feedback_packet(port, timeout_sec: float) -> Optional[bytes]:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        b = port.read(1)
        if not b:
            continue
        if b[0] != FEEDBACK_PACKET_HEADER:
            continue
        tail = port.read(FEEDBACK_PACKET_LENGTH - 1)
        if len(tail) != FEEDBACK_PACKET_LENGTH - 1:
            continue
        frame = b + tail
        if frame[1] != FEEDBACK_PACKET_LENGTH:
            print(
                f"[RX] drop frame: length invalid expect={FEEDBACK_PACKET_LENGTH} got={frame[1]}",
                file=sys.stderr,
            )
            continue
        if not verify_crc(frame):
            print("[RX] drop frame: CRC invalid", file=sys.stderr)
            continue
        return frame
    return None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="rm_serial_driver serial integration tool")
    parser.add_argument(
        "--port",
        required=True,
        help="Serial port, e.g. /dev/ttyACM0 or /tmp/ttyV1",
    )
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    parser.add_argument(
        "--serial-timeout",
        type=float,
        default=0.02,
        help="Serial read timeout (seconds)",
    )

    parser.add_argument("--tx", action="store_true", help="Send CmdPacket")
    parser.add_argument("--rx", action="store_true", help="Receive FeedbackPacket")
    parser.add_argument(
        "--tx-count",
        type=int,
        default=1,
        help="Number of CmdPackets to send",
    )
    parser.add_argument(
        "--rx-count",
        type=int,
        default=10,
        help="Number of FeedbackPackets to read",
    )
    parser.add_argument(
        "--tx-interval",
        type=float,
        default=0.10,
        help="Interval between TX packets (seconds)",
    )
    parser.add_argument(
        "--rx-timeout",
        type=float,
        default=1.0,
        help="Per-frame RX timeout (seconds)",
    )
    parser.add_argument("--seq-start", type=int, default=0, help="Start sequence number [0,255]")

    parser.add_argument("--mode", choices=sorted(MODE_MAP.keys()), default="joint")
    parser.add_argument("--cmd-action", choices=sorted(ACTION_MAP.keys()), default="execute")
    parser.add_argument("--j1", type=float, default=0.0, help="Joint 1 target (rad)")
    parser.add_argument("--j2", type=float, default=0.0, help="Joint 2 target (rad)")
    parser.add_argument("--j3", type=float, default=0.0, help="Joint 3 target (rad)")
    parser.add_argument("--j4", type=float, default=0.0, help="Joint 4 target (rad)")
    parser.add_argument("--gripper", type=float, default=0.0, help="Gripper target (m)")

    parser.add_argument(
        "--inject-bad-length",
        action="store_true",
        help="Send length!=32 but CRC still valid",
    )
    parser.add_argument("--inject-bad-crc", action="store_true", help="Send bad CRC")
    parser.add_argument("--hex", action="store_true", help="Print raw frame hex for TX/RX")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        import serial
    except ImportError as exc:  # pragma: no cover - runtime dependency check
        raise SystemExit(
            "pyserial is required. Install with: sudo apt install python3-serial"
        ) from exc

    do_tx = args.tx
    do_rx = args.rx
    if not do_tx and not do_rx:
        do_tx = True
        do_rx = True

    try:
        with serial.Serial(args.port, args.baud, timeout=args.serial_timeout) as port:
            mode = MODE_MAP[args.mode]
            cmd_action = ACTION_MAP[args.cmd_action]

            if do_tx:
                for i in range(max(0, args.tx_count)):
                    seq = (args.seq_start + i) & 0xFF
                    frame = build_cmd_packet(
                        seq=seq,
                        mode=mode,
                        cmd_action=cmd_action,
                        j1=args.j1,
                        j2=args.j2,
                        j3=args.j3,
                        j4=args.j4,
                        gripper=args.gripper,
                        inject_bad_length=args.inject_bad_length,
                        inject_bad_crc=args.inject_bad_crc,
                    )
                    port.write(frame)
                    tx_log = {
                        "seq": seq,
                        "mode": args.mode,
                        "cmd_action": args.cmd_action,
                        "j1": args.j1,
                        "j2": args.j2,
                        "j3": args.j3,
                        "j4": args.j4,
                        "gripper": args.gripper,
                        "inject_bad_length": args.inject_bad_length,
                        "inject_bad_crc": args.inject_bad_crc,
                    }
                    print("[TX]", json.dumps(tx_log, ensure_ascii=False))
                    if args.hex:
                        print("[TX][HEX]", frame.hex())
                    if i + 1 < args.tx_count:
                        time.sleep(max(0.0, args.tx_interval))

            if do_rx:
                for _ in range(max(0, args.rx_count)):
                    frame = read_feedback_packet(port, args.rx_timeout)
                    if frame is None:
                        print("[RX] timeout")
                        continue
                    decoded = decode_feedback_packet(frame)
                    print("[RX]", json.dumps(decoded, ensure_ascii=False))
                    if args.hex:
                        print("[RX][HEX]", frame.hex())
    except serial.SerialException as exc:
        print(f"Serial error: {exc}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
