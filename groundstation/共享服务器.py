# -*- coding: utf-8 -*-
"""
S4Fly 数传地面站 —— 手机共享中转服务器
======================================
用途：电脑上的 Chrome 地面站读到串口遥测后，把数据推到这里，
      手机连同一 WiFi 打开 http://<电脑IP>:8787 即可实时查看。

零第三方依赖（仅 Python 标准库），双击「启动共享服务器.bat」运行。

路由：
  GET  /           手机版页面（手机共享页面.html）
  GET  /api        最新一帧遥测 JSON
  GET  /ip         本机局域网 IP（地面站页面用来显示手机访问地址）
  POST /data       接收电脑地面站推送的遥测（带 CORS）
"""
import http.server
import socketserver
import socket

PORT = 8787
LATEST = b""


def lan_ip():
    """取本机局域网 IP（不产生真实流量）。"""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "127.0.0.1"


class Handler(http.server.BaseHTTPRequestHandler):

    def _cors(self):
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET,POST,OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")

    def _ok(self, body, ctype="text/plain; charset=utf-8"):
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self._cors()
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_OPTIONS(self):
        self.send_response(200)
        self._cors()
        self.send_header("Content-Length", "0")
        self.end_headers()

    def do_GET(self):
        global LATEST
        p = self.path.split("?")[0]
        if p == "/api":
            self._ok(LATEST, "application/json; charset=utf-8")
        elif p == "/ip":
            self._ok(lan_ip().encode("utf-8"))
        elif p in ("/", "/index.html"):
            self._serve_file("手机共享页面.html", "text/html; charset=utf-8")
        else:
            self.send_response(404)
            self._cors()
            self.send_header("Content-Length", "0")
            self.end_headers()

    def do_POST(self):
        global LATEST
        if self.path.split("?")[0] == "/data":
            try:
                n = int(self.headers.get("Content-Length", 0))
                if n > 0:
                    LATEST = self.rfile.read(n)
            except Exception:
                pass
            self._ok(b"ok")
        else:
            self.send_response(404)
            self._cors()
            self.send_header("Content-Length", "0")
            self.end_headers()

    def _serve_file(self, name, ctype):
        import os
        base = os.path.dirname(os.path.abspath(__file__))
        try:
            with open(os.path.join(base, name), "rb") as f:
                data = f.read()
            self._ok(data, ctype)
        except Exception:
            self.send_response(500)
            self.send_header("Content-Length", "0")
            self.end_headers()

    def log_message(self, *a):
        pass


def main():
    ip = lan_ip()
    print("=" * 48)
    print("  S4Fly 手机共享服务器 已启动")
    print("  手机连同一 WiFi，打开:  http://%s:%d" % (ip, PORT))
    print("  本机测试:  http://localhost:%d" % PORT)
    print("  电脑地面站点「共享到手机」即可推送数据")
    print("  按 Ctrl+C 停止")
    print("=" * 48)
    with socketserver.ThreadingTCPServer(("0.0.0.0", PORT), Handler) as srv:
        srv.daemon_threads = True
        srv.serve_forever()


if __name__ == "__main__":
    main()
