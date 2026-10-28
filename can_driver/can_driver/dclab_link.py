#!/usr/bin/env python3
"""DCLab Link: use the robot's CAN-USB from the Super App.

The robot needs no internet: the configuration computer opens the Super App
(online). The relay carries the same slcan text the page speaks to a CANable,
and puts the frames on the robot's SocketCAN interface (a CANable with
candleLight firmware is `can0` through the kernel gs_usb driver; an slcan
CANable works after `slcand -o -s8 /dev/ttyACM0 can0`). Shared with ROS or
anything else on can0.

The ROS 2 can_driver starts it by itself (parameter superapp_link, default
on), as a separate low-priority process so it never slows the driver.

Connect from the Super App (nothing to install on the computer; owner
2026-10-17): choose "Robot", type the robot's address, Connect. A small
window from this relay asks for the robot computer's login (the account that
runs it, checked by the robot's own system as for SSH) and links the page to
can0 while it stays open. Or through an SSH tunnel, as before:

    ssh -N -L 48791:127.0.0.1:48791 USER@ROBOT

Safe by design: CAN only after the robot's login (or through SSH), browser
pages only from the Super App's site or this relay's own window, can0 opened
only while a page is connected - idle, it sends nothing. The kernel hands it
only the boards' answers to the Super App (DCLab management frames), not the
robot's own traffic, and it runs at low CPU priority. One page at a time; a
newer one takes over. Python 3.6+, standard library only.
"""
import argparse
import base64
import errno
import hashlib
import html
import os
import select
import socket
import struct
import sys
import threading
import time

VERSION = "3.4"
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
        # largest message: a login line before the login (review #3: a
        # multi-GB frame from anyone on the network ran the robot out of
        # memory), a burst of slcan lines after
        self.max_message = 1024

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
            if length + sum(len(p) for p in parts) > self.max_message or (opcode >= 0x8 and length > 125):
                raise ConnectionError("message too long")
            mask = self._recv_exact(4) if b1 & 0x80 else b"\0\0\0\0"
            raw = self._recv_exact(length)
            key = int.from_bytes((mask * (length // 4 + 1))[:length], "big")
            body = (int.from_bytes(raw, "big") ^ key).to_bytes(length, "big")
            if opcode == 0x8:
                self._send(b"\x88\x00")
                return
            if opcode == 0x9:
                self._send(struct.pack("!BB", 0x8A, len(body)) + body)
                continue
            if opcode in (0x0, 0x1, 0x2):
                parts.append(body)
                if b0 & 0x80:
                    # UTF-8: a password may hold any character (review #3 L8)
                    yield b"".join(parts).decode("utf-8", "replace")
                    parts = []

    def close(self):
        self._send(b"\x88\x00")
        self.closed = True
        try:
            self.sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        self.sock.close()


def read_request(sock):
    """(request line, headers) of one HTTP request, or None."""
    request = b""
    while b"\r\n\r\n" not in request:
        chunk = sock.recv(4096)
        if not chunk or len(request) > 16384:
            return None
        request += chunk
    lines = request.split(b"\r\n\r\n")[0].decode("latin-1").split("\r\n")
    headers = {}
    for line in lines[1:]:
        if ":" in line:
            key, value = line.split(":", 1)
            headers[key.strip().lower()] = value.strip()
    return lines[0], headers


def own_host(host):
    """The Host header names this computer: an IP address, its name or
    name.local, localhost. Anything else is a DNS rebinding (a website's name
    pointed at the robot, review #3 L3)."""
    name = host.rsplit("]", 1)[0].lstrip("[") if host.startswith("[") else host.rsplit(":", 1)[0]
    name = name.lower().rstrip(".")
    try:
        socket.inet_pton(socket.AF_INET6 if ":" in name else socket.AF_INET, name)
        return True
    except (OSError, ValueError):
        pass
    me = socket.gethostname().lower()
    return name in (me, me + ".local", me.split(".")[0] + ".local", "localhost")


def handshake(sock, origins, loopback, allow_login=True):
    """Serve the login window, or accept a WebSocket. Returns None (done),
    "tunnel" (the Super App through SSH: the robot's login was SSH's) or
    "login" (this relay's own window: the robot's login comes first)."""
    parsed = read_request(sock)
    if parsed is None:
        return None
    request_line, headers = parsed
    parts = request_line.split(" ")
    path = parts[1] if len(parts) > 1 else "/"
    origin = headers.get("origin")
    own = "http://" + headers.get("host", "")
    key = headers.get("sec-websocket-key")
    if headers.get("upgrade", "").lower() != "websocket":
        if parts[0] == "GET" and path.split("?")[0] in ("/", "/link"):
            body = window_page(origins)
            sock.sendall(("HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\n"
                          "Content-Length: %d\r\nCache-Control: no-store\r\nX-Frame-Options: DENY\r\n"
                          "Referrer-Policy: no-referrer\r\nConnection: close\r\n\r\n" % len(body)).encode() + body)
        else:
            sock.sendall(b"HTTP/1.1 404 Not Found\r\nContent-Length: 0\r\nConnection: close\r\n\r\n")
        return None
    # Browsers always send Origin. The Super App's site only through the SSH
    # tunnel (loopback); this relay's own window from anywhere, after the
    # robot's login; nothing else (another site in the same browser must not
    # drive the robot's bus).
    if not key:
        sock.sendall(b"HTTP/1.1 400 Bad Request\r\nContent-Length: 0\r\n\r\n")
        return None
    if loopback and (origin is None or origin in origins):
        kind = "tunnel"  # (no Origin: a program on this computer, not a browser)
    elif allow_login and origin == own and path.split("?")[0] == "/link" and own_host(headers.get("host", "")):
        kind = "login"
    else:
        log("refused a page from %s" % origin)
        sock.sendall(b"HTTP/1.1 403 Forbidden\r\nContent-Length: 0\r\n\r\n")
        return None
    accept = base64.b64encode(hashlib.sha1((key + WS_GUID).encode()).digest()).decode()
    sock.sendall(("HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\n"
                  "Connection: Upgrade\r\nSec-WebSocket-Accept: %s\r\n\r\n" % accept).encode())
    return kind


# --- the robot computer's own login (PAM, as SSH checks it) ---

def running_account():
    """The account this relay runs as; without root only its password can be checked."""
    try:
        import pwd
        return pwd.getpwuid(os.geteuid()).pw_name
    except (ImportError, KeyError):
        return os.environ.get("USER", "")


def login_refusal(user):
    """What the window shows for a refused login (2026-10-22: a tester typed a
    different Raspberry Pi account and only saw the form again)."""
    account = running_account()
    if os.geteuid() != 0 and account and user != account:
        return ("Wrong username: this robot accepts only %s "
                "(the account running can_driver)" % account)
    return "Wrong username or password"


def check_login(user, password, rhost=""):
    """True when PAM accepts user/password and the account may log in now.
    Without root only the account running this relay can be checked
    (pam_unix's unix_chkpwd). Never an empty password (review #3 L6: nullok
    let a password-less account in), the caller's address for the logs."""
    if not password:
        return False
    import ctypes
    import ctypes.util
    names = (ctypes.util.find_library("pam"), "libpam.so.0")
    libpam = None
    for name in names:
        try:
            libpam = ctypes.CDLL(name) if name else None
        except OSError:
            libpam = None
        if libpam is not None:
            break
    libc = ctypes.CDLL(ctypes.util.find_library("c") or "libc.so.6")
    if libpam is None:
        log("PAM is not available on this computer: logins cannot be checked")
        return False

    class Message(ctypes.Structure):
        _fields_ = [("msg_style", ctypes.c_int), ("msg", ctypes.c_char_p)]

    class Response(ctypes.Structure):
        _fields_ = [("resp", ctypes.c_void_p), ("resp_retcode", ctypes.c_int)]

    CONV = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_int, ctypes.POINTER(ctypes.POINTER(Message)),
                            ctypes.POINTER(ctypes.POINTER(Response)), ctypes.c_void_p)

    class Conv(ctypes.Structure):
        _fields_ = [("conv", CONV), ("appdata_ptr", ctypes.c_void_p)]

    libc.calloc.restype = ctypes.c_void_p
    libc.calloc.argtypes = (ctypes.c_size_t, ctypes.c_size_t)
    libc.strdup.restype = ctypes.c_void_p
    libc.strdup.argtypes = (ctypes.c_char_p,)
    secret = password.encode("utf-8")

    def conversation(count, messages, responses, _data):
        # PAM frees what it is given: allocated with the C library
        block = libc.calloc(count, ctypes.sizeof(Response))
        if not block:
            return 5  # PAM_BUF_ERR
        answers = ctypes.cast(block, ctypes.POINTER(Response))
        for i in range(count):
            if messages[i].contents.msg_style == 1:  # PAM_PROMPT_ECHO_OFF: the password
                answers[i].resp = libc.strdup(secret)
        responses[0] = answers
        return 0

    handle = ctypes.c_void_p()
    conv = Conv(CONV(conversation), None)
    libpam.pam_start.argtypes = (ctypes.c_char_p, ctypes.c_char_p, ctypes.POINTER(Conv),
                                 ctypes.POINTER(ctypes.c_void_p))
    libpam.pam_authenticate.argtypes = (ctypes.c_void_p, ctypes.c_int)
    libpam.pam_acct_mgmt.argtypes = (ctypes.c_void_p, ctypes.c_int)
    libpam.pam_set_item.argtypes = (ctypes.c_void_p, ctypes.c_int, ctypes.c_void_p)
    libpam.pam_end.argtypes = (ctypes.c_void_p, ctypes.c_int)
    if libpam.pam_start(b"login", user.encode("utf-8"), ctypes.byref(conv), ctypes.byref(handle)) != 0:
        return False
    PAM_RHOST, PAM_DISALLOW_NULL_AUTHTOK = 4, 0x1
    host = ctypes.create_string_buffer(rhost.encode("ascii", "replace"))
    libpam.pam_set_item(handle, PAM_RHOST, ctypes.cast(host, ctypes.c_void_p))
    result = libpam.pam_authenticate(handle, PAM_DISALLOW_NULL_AUTHTOK)
    if result == 0:
        result = libpam.pam_acct_mgmt(handle, PAM_DISALLOW_NULL_AUTHTOK)  # expired, locked, pam_access
    libpam.pam_end(handle, result)
    return result == 0


class LoginThrottle:
    """Guessing over the network: one login check at a time (parallel
    connections all waited, then guessed together: review #3 L3), and after
    5 failures within a minute from one address its attempts wait (2 s more
    each, up to 30 s); other addresses are not held up."""

    def __init__(self):
        self.lock = threading.Lock()
        self.check = threading.Lock()
        self.failures = {}

    def run(self, address, attempt):
        with self.lock:
            now = time.monotonic()
            for key in list(self.failures):
                self.failures[key] = [t for t in self.failures[key] if now - t < 60.0]
                if not self.failures[key]:
                    del self.failures[key]
            extra = len(self.failures.get(address, ())) - 4
        if extra > 0:
            time.sleep(min(30.0, 2.0 * extra))
        with self.check:
            ok = attempt()
        if not ok:
            with self.lock:
                self.failures.setdefault(address, []).append(time.monotonic())
        return ok


WINDOW_PAGE = """<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>DCLab robot link</title>
<style>
:root{color-scheme:light dark;--bg:#f6f7f9;--fg:#1d2330;--muted:#5b6475;--line:#d5d9e0;--accent:#2563eb;--bad:#b42318}
@media (prefers-color-scheme:dark){:root{--bg:#14171c;--fg:#e7eaf0;--muted:#9aa3b2;--line:#323844;--accent:#6ea0ff;--bad:#ff8a80}}
body{margin:0;background:var(--bg);color:var(--fg);font:15px/1.4 system-ui,sans-serif}
main{max-width:340px;margin:0 auto;padding:20px 16px}
h1{font-size:17px;margin:0 0 4px}p{margin:6px 0;color:var(--muted)}
label{display:block;margin:10px 0 2px;font-size:13px;color:var(--muted)}
input{box-sizing:border-box;width:100%;padding:8px;border:1px solid var(--line);border-radius:6px;background:transparent;color:var(--fg);font:inherit}
button{margin-top:14px;width:100%;padding:9px;border:0;border-radius:6px;background:var(--accent);color:#fff;font:inherit;cursor:pointer}
#status.bad{color:var(--bad)}.hidden{display:none}
#error{margin:10px 0;padding:10px 12px;border-radius:6px;border:1px solid var(--bad);color:var(--bad);font-weight:600}
</style></head><body><main>
<h1>Robot link: HOSTNAME</h1>
<div id="error" role="alert" class="hidden"></div>
<form id="login">
<p>Log in with ACCOUNT_HINT.</p>
<label for="user">Username</label><input id="user" autocomplete="username" autocapitalize="off" spellcheck="false" required>
<label for="password">Password</label><input id="password" type="password" autocomplete="current-password" required>
<button type="submit">Connect</button>
</form>
<p id="status" role="status"></p>
</main><script>
"use strict";
const ORIGINS = ORIGIN_LIST;
let ws = null, ready = false, opener = null, login = null, retries = 0, connectedAt = 0, ended = null, acceptedUser = "", prefilled = false;
const status = document.getElementById("status");
const say = (text, bad) => { status.textContent = text; status.className = bad ? "bad" : ""; };
const post = (message) => { if (opener) window.opener.postMessage(Object.assign({dclabLink: 1}, message), opener); };
if (!window.opener) say("Open this from the Super App: Robot, then Connect.", true);
window.addEventListener("message", (event) => {
  if (event.source !== window.opener || !ORIGINS.includes(event.origin)) return;
  const m = event.data;
  if (!m || m.dclabLink !== 1) return;
  opener = event.origin;
  if (m.type === "hello") {
    /* the page's saved username for this robot: only the password is typed */
    const user = document.getElementById("user");
    if (!prefilled && typeof m.user === "string" && m.user && !user.value && !login) { user.value = m.user; document.getElementById("password").focus(); }
    prefilled = true;
    post(ready ? {type: "ready", user: acceptedUser} : {type: "waiting"});
  }
  else if (m.type === "data" && ready && ws && ws.readyState === 1) ws.send(String(m.text));
  else if (m.type === "close") { ready = false; if (ws) ws.close(); window.close(); }
});
function connect() {
  ws = new WebSocket("ws://" + location.host + "/link");
  let accepted = false, buffer = "";
  ws.onopen = () => ws.send("#L" + login.user + "\\t" + login.password + "\\r");
  ws.onmessage = (event) => {
    let text = String(event.data);
    if (!accepted) {
      buffer += text;
      const end = buffer.indexOf("\\r");
      if (end < 0) return;
      const first = buffer.slice(0, end); text = buffer.slice(end + 1); buffer = "";
      if (first !== "#A") {
        login = null; ws.onclose = null; ws.close();
        const reason = first.startsWith("#X") ? first.slice(2) : "The robot refused the login.";
        say("");
        const box = document.getElementById("error");
        box.textContent = reason; box.classList.remove("hidden");
        document.getElementById("login").classList.remove("hidden");
        document.getElementById("password").focus();
        if (ready) { ready = false; post({type: "fatal", message: "The robot refused the login"}); }
        else post({type: "refused", message: reason});
        return;
      }
      accepted = true; ready = true; ended = null; connectedAt = Date.now(); acceptedUser = login.user;
      document.getElementById("error").classList.add("hidden");
      document.getElementById("login").classList.add("hidden");
      say("Connected. Keep this window open (you can minimise it); closing it ends the link.");
      post({type: "ready", user: acceptedUser});
      if (!text) return;
    }
    /* the relay's last word (taken over by another page, the adapter lost):
     * final, no logging in again (review #3 L1: two windows took the link
     * from each other forever) */
    const fatal = /(?:^|\\r)#X([^\\r]*)\\r/.exec(text);
    if (fatal) { ended = fatal[1]; login = null; }
    post({type: "data", text});
  };
  ws.onclose = () => {
    if (!accepted) { if (!ready) say("Cannot reach DCLab Link on this robot.", true); return; }
    /* a dropped connection is tried again; the count starts over only after
     * the link has run a while */
    if (Date.now() - connectedAt > 10000) retries = 0;
    if (login && !ended && retries < 5) { retries += 1; say("Reconnecting to the robot..."); setTimeout(connect, 500 * retries); return; }
    const reason = ended || "The robot link closed";
    ready = false; post({type: "fatal", message: reason});
    say(reason, true);
  };
}
document.getElementById("login").addEventListener("submit", (event) => {
  event.preventDefault();
  login = {user: document.getElementById("user").value.trim(), password: document.getElementById("password").value};
  document.getElementById("password").value = "";
  document.getElementById("error").classList.add("hidden");
  say("Checking the login...");
  connect();
});
window.addEventListener("pagehide", () => post({type: "fatal", message: "The robot window was closed"}));
</script></body></html>
"""


def window_page(origins):
    account = running_account()
    if os.geteuid() == 0 or not account:
        hint = "an account of this computer"
    else:
        hint = ("<b>%s</b>, the account running can_driver here "
                "(only its password can be checked)" % html.escape(account))
    page = WINDOW_PAGE.replace("HOSTNAME", socket.gethostname()).replace("ACCOUNT_HINT", hint)
    return page.replace("ORIGIN_LIST", "[" + ", ".join('"%s"' % o for o in origins) + "]").encode("utf-8")


# --- one page session: WebSocket <-> SocketCAN ---

class Session:
    def __init__(self, ws, interface):
        self.ws = ws
        self.interface = interface
        self.can = None
        self.stop = threading.Event()

    def open(self):
        """The CAN interface, before the page is told it is in (review #3 L1:
        a login window kept logging in again to an interface that failed)."""
        try:
            self.can = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            self.can.setsockopt(SOL_CAN_RAW, CAN_RAW_FILTER, MGMT_FILTER)
            self.can.setsockopt(SOL_CAN_RAW, CAN_RAW_ERR_FILTER, struct.pack("=I", CAN_ERR_MASK))
            self.can.bind((self.interface,))
            return True
        except OSError as error:
            log("cannot open %s: %s" % (self.interface, error))
            self.ws.send_text("#X%s cannot be opened on the robot: %s\r" % (self.interface, error))
            self.ws.close()
            return False

    def run(self, messages):
        reader = threading.Thread(target=self._can_to_page, daemon=True)
        reader.start()
        pending = ""
        try:
            for message in messages:
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
                except OSError as error:
                    # the adapter is gone (unplugged): say so and end, or the
                    # page only saw every frame refused (review #3 L5)
                    log("%s lost: %s" % (self.interface, error))
                    self.ws.send_text("".join(lines) + "#X%s was lost on the robot: %s\r" % (self.interface, error))
                    self.stop.set()
                    self.ws.close()
                    return
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


def local_addresses():
    """This computer's IPv4 addresses on its networks (for the log)."""
    found = []
    try:
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        probe.connect(("192.0.2.1", 9))  # no packet is sent: picks the outgoing address
        found.append(probe.getsockname()[0])
        probe.close()
    except OSError:
        pass
    return ", ".join(found + [socket.gethostname() + ".local"])


def main():
    parser = argparse.ArgumentParser(description="DCLab Link: the robot's CAN for the Super App over SSH.")
    parser.add_argument("--can", default="can0", help="SocketCAN interface (default can0)")
    parser.add_argument("--port", type=int, default=48791, help="port (default 48791)")
    parser.add_argument("--listen", default="0.0.0.0",
                        help="address to listen on (default all: the Super App's robot window; "
                             "127.0.0.1 = through an SSH tunnel only)")
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
        server.bind((args.listen, args.port))
    except OSError as error:
        sys.exit("port %d is busy (%s): is DCLab Link already running?" % (args.port, error))
    server.listen(4)
    log("DCLab Link %s: %s on %s:%d, waiting for the Super App (Ctrl-C to stop)"
        % (VERSION, args.can, args.listen, args.port))
    if args.listen != "127.0.0.1":
        log("in the Super App choose Robot and enter this computer's address (%s)" % local_addresses())
    throttle = LoginThrottle()
    # the login path only where the window is served (review #3 L9: through
    # the tunnel a website could rebind its name to 127.0.0.1 and guess)
    allow_login = args.listen not in ("127.0.0.1", "::1", "localhost")
    # connections at once (review #3 L4: ~1000 idle ones ended the relay)
    slots = threading.BoundedSemaphore(8)

    current = {"session": None}
    lock = threading.Lock()

    def serve(client, address):
        try:
            serve_one(client, address)
        except (ConnectionError, OSError, ValueError, struct.error):
            try:
                client.close()
            except OSError:
                pass
        finally:
            slots.release()

    def serve_one(client, address):
        client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        client.settimeout(10.0)  # the request and the login line; then none
        try:
            kind = handshake(client, origins, address[0] in ("127.0.0.1", "::1"), allow_login)
        except OSError:
            kind = None
        if kind is None:
            client.close()
            return
        ws = WebSocket(client)
        messages = ws.messages()
        if kind == "login":
            # the robot computer's login first; only then may it take over
            try:
                first = next(messages).split("\r", 1)[0]
            except (StopIteration, ConnectionError, OSError):
                ws.close()
                return
            user, _, password = first[2:].partition("\t")
            if not first.startswith("#L") or not user or not throttle.run(
                    address[0], lambda: check_login(user, password, address[0])):
                log("refused a login as %r from %s" % (user[:32], address[0]))
                ws.send_text("#X%s\r" % login_refusal(user))
                ws.close()
                return
            log("%s logged in from %s" % (user, address[0]))
        client.settimeout(None)
        ws.max_message = 65536
        session = Session(ws, args.can)
        if not session.open():
            return
        if kind == "login":
            ws.send_text("#A\r")
        with lock:
            previous, current["session"] = current["session"], session
        if previous is not None:
            log("a newer page took over")
            try:  # said first, or that page reconnects and takes it back
                previous.ws.send_text("#XThe robot link was taken over by another page\r")
            except OSError:
                pass
            previous.ws.close()
        log("page connected")
        session.run(messages)
        with lock:
            if current["session"] is session:
                current["session"] = None
                log("page disconnected; %s idle" % args.can)

    try:
        while True:
            try:
                client, address = server.accept()
            except OSError as error:  # out of file descriptors: wait, go on
                log("accept failed: %s" % error)
                time.sleep(0.5)
                continue
            if not slots.acquire(blocking=False):
                client.close()  # busy: 8 connections already
                continue
            try:
                threading.Thread(target=serve, args=(client, address), daemon=True).start()
            except RuntimeError:
                slots.release()
                client.close()
    except KeyboardInterrupt:
        pass
    finally:
        server.close()


if __name__ == "__main__":
    os.environ.setdefault("PYTHONUNBUFFERED", "1")
    main()
