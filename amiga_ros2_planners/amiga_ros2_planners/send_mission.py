#!/usr/bin/env python3
"""
send_mission.py

Sends a mission XML file to tcp_demux_node over TCP -- the automated
equivalent of the `nc 0.0.0.0 12346 < mission.xml` step
amiga_ros2_behavior_tree/README.md's own "Quick demo" describes by
hand. Used by run_problog_problem.launch.py so bringing up a problem
folder needs no manual step at all.

Retries the connection for up to `--timeout` seconds (`tcp_demux_node`
takes a little while to come up after the rest of the launch tree) --
`nc` has no equivalent retry of its own, which is why this script
exists rather than an `ExecuteProcess(cmd=["nc", ...])` action.

Defaults to `payload_length_included=false` framing (a single raw XML
frame, no length prefix, no second JSON frame) -- the simplest mode
that `bt.launch.py`'s own Quick demo uses for exactly this "hand-feed a
mission" case; matches `run_problog_problem.launch.py`'s own
`expect_json:=false payload_length_included:=false` bt.launch.py
arguments. Pass `--length-prefixed` if launching against a bt.launch.py
started with the default (length-prefixed) framing instead.

    ros2 run amiga_ros2_planners send_mission -- \
        --port 12346 --file /tmp/problem0_adapted.xml
"""
import argparse
import socket
import struct
import sys
import time


def send(host, port, xml_bytes, length_prefixed, timeout):
    deadline = time.monotonic() + timeout
    last_error = None
    while time.monotonic() < deadline:
        try:
            with socket.create_connection((host, port), timeout=2.0) as sock:
                if length_prefixed:
                    sock.sendall(struct.pack(">I", len(xml_bytes)))
                sock.sendall(xml_bytes)
            return True
        except (ConnectionRefusedError, OSError) as exc:
            last_error = exc
            time.sleep(1.0)
    print(f"send_mission: giving up after {timeout}s, last error: {last_error}",
          file=sys.stderr)
    return False


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, required=True)
    ap.add_argument("--file", required=True, help="Mission XML file to send")
    ap.add_argument(
        "--length-prefixed", action="store_true",
        help="Prefix the frame with a 4-byte big-endian length, matching "
        "tcp_demux_node's default payload_length_included:=true.")
    ap.add_argument(
        "--timeout", type=float, default=60.0,
        help="How long to keep retrying the connection, seconds.")
    args = ap.parse_args()

    with open(args.file, "rb") as f:
        xml_bytes = f.read()

    ok = send(args.host, args.port, xml_bytes, args.length_prefixed, args.timeout)
    if ok:
        print(f"send_mission: sent {len(xml_bytes)} bytes to "
              f"{args.host}:{args.port}")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
