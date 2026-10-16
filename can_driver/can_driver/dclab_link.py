#!/usr/bin/env python3
"""DCLab Link: use the robot's CAN-USB from the Super App over SSH.

The robot needs no internet: the configuration computer opens the Super App
(online) and reaches this relay through an SSH tunnel. The relay carries the
same slcan text the page speaks to a CANable, and puts the frames on the
robot's SocketCAN interface (a CANable with candleLight firmware is `can0`
through the kernel gs_usb driver; an slcan CANable works after
`slcand -o -s8 /dev/ttyACM0 can0`). Shared with ROS or anything else on can0.

The ROS 2 can_driver starts it by itself (parameter superapp_link, default
on), as a separate low-priority process so it never slows the driver; then
the configuration computer only needs the tunnel:

    ssh -N -L 48791:127.0.0.1:48791 USER@ROBOT

Run from the configuration computer (one command, stops when SSH closes):

    ssh -L 48791:127.0.0.1:48791 USER@ROBOT python3 dclab_link.py

then choose "Robot over SSH" in the Super App and Connect.

Safe by design: it listens on 127.0.0.1 only (reachable only through SSH),
accepts browser pages only from the Super App's own site, and opens the CAN
interface only while a page is connected - idle, it sends nothing. The kernel
hands it only the boards' answers to the Super App (DCLab management frames),
not the robot's own traffic, and it runs at low CPU priority. One page
at a time; a newer one takes over. Python 3.6+, standard library only.
"""
import argparse
import base64
import errno
import hashlib
import os
import select
import socket
import struct
import sys
import threading
import time

VERSION = "2"
DEFAULT_ORIGINS = ("https://superapp-v2.dclab.live",)
WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
CAN_FRAME = struct.Struct("=IB3x8s")
CAN_EFF_FLAG, CAN_RTR_FLAG, CAN_ERR_FLAG = 0x80000000, 0x40000000, 0x20000000
CAN_ERR_MASK = 0x1FFFFFFF
SOL_CAN_RAW = 101
CAN_RAW_FILTER, CAN_RAW_ERR_FILTER = 1, 2
# What the page uses (slcan.js canableSoftwareFilter): extended DCLab
# management frames from a board to the host (ID 0x1F8xxxxx, direction bit
# 20 set). Everything else stays in the kernel.
MGMT_FROM_BOARD = 0x1F800000 | (1 << 20)
MGMT_FILTER = struct.pack("=II", CAN_EFF_FLAG | MGMT_FROM_BOARD,
                          CAN_EFF_FLAG | CAN_RTR_FLAG | MGMT_FROM_BOARD)


def log(message):
    sys.stderr.write(time.strftime("%H:%M:%S ") + message + "\n")
    sys.stderr.flush()


# --- slcan text <-> CAN frames (as the page's CANable ports) ---

def frame_to_line(can_id, data):
    extended = bool(can_id & CAN_EFF_FLAG)
    remote = bool(can_id & CAN_RTR_FLAG)
    ident = can_id & (CAN_ERR_MASK if extended else 0x7FF)
    kind = ("R" if remote else "T") if extended else ("r" if remote else "t")
    line = kind + ("%08X" if extended else "%03X") % ident + "%X" % len(data)
    if not remote:
        line += data.hex().upper()
    return line + "\r"


def line_to_frame(line):
    """slcan t/T/r/R line -> (can_id with flags, data) or None."""
    kind = line[:1]
    extended = kind in ("T", "R")
    width = 8 if extended else 3
    try:
        ident = int(line[1:1 + width], 16)
        dlc = int(line[1 + width], 16)
        if dlc > 8 or ident > (CAN_ERR_MASK if extended else 0x7FF):
            return None
        data = b""
        if kind in ("t", "T"):
            body = line[2 + width:2 + width + 2 * dlc]
            if len(body) != 2 * dlc:
                return None
            data = bytes.fromhex(body)
    except (ValueError, IndexError):
        return None
    can_id = ident | (CAN_EFF_FLAG if extended else 0) | (CAN_RTR_FLAG if kind in ("r", "R") else 0)
    return can_id, data if kind in ("t", "T") else bytes(dlc)


# --- minimal WebSocket server (RFC 6455, text messages) ---

class WebSocket:
    def __init__(self, sock):
        self.sock = sock
        self.send_lock = threading.Lock()
        self.closed = False

    def send_text(self, text):
        payload = text.encode("ascii")
        n = len(payload)
        if n < 126:
            header = struct.pack("!BB", 0x81, n)
        elif n < 65536:
            header = struct.pack("!BBH", 0x81, 126, n)
        else:
            header = struct.pack("!BBQ", 0x81, 127, n)
        self._send(header + payload)

    def _send(self, data):
        with self.send_lock:
            if self.closed:
                return
            try:
                self.sock.sendall(data)
            except OSError:
                self.closed = True

    def _recv_exact(self, n):
        out = b""
        while len(out) < n:
            chunk = self.sock.recv(n - len(out))
            if not chunk:
                raise ConnectionError("closed")
            out += chunk
        return out

    def messages(self):
        """Yield text messages until the page closes the connection."""
        parts = []
        while True:
            b0, b1 = self._recv_exact(2)
            opcode, length = b0 & 0x0F, b1 & 0x7F
            if length == 126:
                length = struct.unpack("!H", self._recv_exact(2))[0]
            elif length == 127:
                length = struct.unpack("!Q", self._recv_exact(8))[0]
            mask = self._recv_exact(4) if b1 & 0x80 else b"\0\0\0\0"
            body = bytearray(self._recv_exact(length))
            for i in range(length):
                body[i] ^= mask[i % 4]
            if opcode == 0x8:
                self._send(b"\x88\x00")
                return
            if opcode == 0x9:
                self._send(struct.pack("!BB", 0x8A, len(body)) + bytes(body[:125]))
                continue
            if opcode in (0x0, 0x1, 0x2):
                parts.append(bytes(body))
                if b0 & 0x80:
                    yield b"".join(parts).decode("ascii", "replace")
                    parts = []

    def close(self):
        self._send(b"\x88\x00")
        self.closed = True
        try:
            self.sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        self.sock.close()


def handshake(sock, origins):
    """Read the HTTP upgrade; returns True when accepted."""
    request = b""
    while b"\r\n\r\n" not in request:
        chunk = sock.recv(4096)
        if not chunk or len(request) > 16384:
            return False
        request += chunk
    lines = request.split(b"\r\n\r\n")[0].decode("latin-1").split("\r\n")
    headers = {}
    for line in lines[1:]:
        if ":" in line:
            key, value = line.split(":", 1)
            headers[key.strip().lower()] = value.strip()
    origin = headers.get("origin")
    key = headers.get("sec-websocket-key")
    if not key or headers.get("upgrade", "").lower() != "websocket":
        sock.sendall(b"HTTP/1.1 400 Bad Request\r\nContent-Length: 0\r\n\r\n")
        return False
    # Browsers always send Origin: only the Super App's own pages may drive
    # the robot's bus (another site open in the same browser must not).
    if origin is not None and origin not in origins:
        log("refused a page from %s (allowed: %s)" % (origin, ", ".join(origins)))
        sock.sendall(b"HTTP/1.1 403 Forbidden\r\nContent-Length: 0\r\n\r\n")
        return False
    accept = base64.b64encode(hashlib.sha1((key + WS_GUID).encode()).digest()).decode()
    sock.sendall(("HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\n"
                  "Connection: Upgrade\r\nSec-WebSocket-Accept: %s\r\n\r\n" % accept).encode())
    return True


# --- one page session: WebSocket <-> SocketCAN ---

class Session:
    def __init__(self, ws, interface):
        self.ws = ws
        self.interface = interface
        self.can = None
        self.stop = threading.Event()

    def run(self):
        try:
            self.can = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            self.can.setsockopt(SOL_CAN_RAW, CAN_RAW_FILTER, MGMT_FILTER)
            self.can.setsockopt(SOL_CAN_RAW, CAN_RAW_ERR_FILTER, struct.pack("=I", CAN_ERR_MASK))
            self.can.bind((self.interface,))
        except OSError as error:
            log("cannot open %s: %s" % (self.interface, error))
            self.ws.send_text("#X%s cannot be opened on the robot: %s\r" % (self.interface, error))
            self.ws.close()
            return
        reader = threading.Thread(target=self._can_to_page, daemon=True)
        reader.start()
        pending = ""
        try:
            for message in self.ws.messages():
                pending += message
                while "\r" in pending:
                    line, pending = pending.split("\r", 1)
                    self._command(line)
        except (ConnectionError, OSError):
            pass
        finally:
            self.stop.set()
            reader.join(1.0)
            self.can.close()
            if not self.ws.closed:
                self.ws.close()

    def _command(self, line):
        kind = line[:1]
        if kind in ("t", "T", "r", "R"):
            frame = line_to_frame(line)
            if frame is None:
                self.ws.send_text("\x07")
                return
            self._send_frame(*frame)
        elif kind in ("V", "v"):
            self.ws.send_text("V0101\r")
        elif kind:
            # S (bitrate), O/L (open), C (close): the robot's interface runs
            # at the bitrate it was set up with; nothing to change here.
            self.ws.send_text("\r")

    def _send_frame(self, can_id, data):
        packet = CAN_FRAME.pack(can_id, len(data), data.ljust(8, b"\0"))
        deadline = time.monotonic() + 1.0
        while True:
            try:
                self.can.send(packet)
                return
            except OSError as error:
                # The interface queue is full (a burst): wait for room.
                if error.errno not in (errno.ENOBUFS, errno.EAGAIN) or time.monotonic() > deadline:
                    self.ws.send_text("\x07")
                    return
                time.sleep(0.001)

    def _can_to_page(self):
        while not self.stop.is_set():
            ready, _, _ = select.select([self.can], [], [], 0.2)
            if not ready:
                continue
            lines = []
            # Many frames per message on a busy bus.
            while len(lines) < 256:
                try:
                    packet = self.can.recv(CAN_FRAME.size, socket.MSG_DONTWAIT)
                except (BlockingIOError, InterruptedError):
                    break
                except OSError:
                    self.stop.set()
                    break
                can_id, dlc, data = CAN_FRAME.unpack(packet)
                data = data[:min(dlc, 8)]
                if can_id & CAN_ERR_FLAG:
                    lines.append("#E%08X:%s\r" % (can_id & CAN_ERR_MASK, data.hex().upper()))
                else:
                    lines.append(frame_to_line(can_id, data))
            if lines:
                self.ws.send_text("".join(lines))
                if self.ws.closed:
                    self.stop.set()


def interface_state(interface):
    try:
        with open("/sys/class/net/%s/operstate" % interface) as f:
            return f.read().strip()
    except OSError:
        return None


def main():
    parser = argparse.ArgumentParser(description="DCLab Link: the robot's CAN for the Super App over SSH.")
    parser.add_argument("--can", default="can0", help="SocketCAN interface (default can0)")
    parser.add_argument("--port", type=int, default=48791, help="local port (default 48791)")
    parser.add_argument("--allow-origin", action="append", default=[],
                        help="another Super App site allowed to connect (repeatable)")
    parser.add_argument("--parent", type=int, default=0, metavar="PID",
                        help="exit when this process (the can_driver node that started it) exits")
    args = parser.parse_args()
    try:
        os.nice(10)  # the robot's own control always comes first
    except OSError:
        pass
    if args.parent:
        def watch_parent():
            # the parent's PID is passed in: had it died before this line,
            # getppid() would already show the new parent
            while os.getppid() == args.parent:
                time.sleep(1.0)
            os._exit(0)

        threading.Thread(target=watch_parent, daemon=True).start()
    origins = tuple(DEFAULT_ORIGINS) + tuple(args.allow_origin)

    state = interface_state(args.can)
    if state is None:
        sys.exit("%s not found. Plug in the CANable (candleLight shows up as can0), or for slcan firmware: "
                 "sudo slcand -o -s8 /dev/ttyACM0 can0" % args.can)
    if state == "down":
        sys.exit("%s is down. Bring it up at 1 Mbit/s: sudo ip link set %s up type can bitrate 1000000"
                 % (args.can, args.can))

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        server.bind(("127.0.0.1", args.port))
    except OSError as error:
        sys.exit("port %d is busy (%s): is DCLab Link already running?" % (args.port, error))
    server.listen(4)
    log("DCLab Link %s: %s on 127.0.0.1:%d, waiting for the Super App (Ctrl-C to stop)"
        % (VERSION, args.can, args.port))

    current = {"session": None}
    lock = threading.Lock()

    def serve(client, address):
        client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        try:
            if not handshake(client, origins):
                client.close()
                return
        except OSError:
            client.close()
            return
        session = Session(WebSocket(client), args.can)
        with lock:
            previous, current["session"] = current["session"], session
        if previous is not None:
            log("a newer page took over")
            previous.ws.close()
        log("page connected")
        session.run()
        with lock:
            if current["session"] is session:
                current["session"] = None
                log("page disconnected; %s idle" % args.can)

    try:
        while True:
            client, address = server.accept()
            threading.Thread(target=serve, args=(client, address), daemon=True).start()
    except KeyboardInterrupt:
        pass
    finally:
        server.close()


if __name__ == "__main__":
    os.environ.setdefault("PYTHONUNBUFFERED", "1")
    main()
