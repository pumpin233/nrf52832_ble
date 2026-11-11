"""电脑自身蓝牙连接开发板；运行后打开浏览器收发窗口。"""
import asyncio
import codecs
import json
import queue
import secrets
import threading
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from bleak import BleakClient, BleakScanner

NAME = "BLE_PC_Link"
RX = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # 电脑写入此特征
TX = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # 电脑订阅此特征，接收开发板通知
TOKEN = secrets.token_hex(16)


class Bluetooth:
    def __init__(self):
        self.loop = asyncio.new_event_loop()
        self.client = None
        self.events = queue.Queue()
        self.ready = False
        self.decoder = codecs.getincrementaldecoder("utf-8")(errors="replace")
        threading.Thread(target=self.loop.run_forever, daemon=True).start()
        # 所有蓝牙操作都在同一个后台事件循环执行，避免阻塞窗口。
        self.lock = None

    def notify(self, _, data):
        self.events.put({"kind": "rx", "text": self.decoder.decode(bytes(data)),
                         "hex": bytes(data).hex(" ")})

    def disconnected(self, client):
        if client is self.client:
            self.ready = False
            self.events.put({"kind": "log", "text": "蓝牙已断开"})

    async def action(self, command, text="", hex_mode=False):
        if self.lock is None:
            self.lock = asyncio.Lock()
        async with self.lock:
            if command == "connect":
                if self.ready:
                    return "已经连接"
                if self.client and self.client.is_connected:
                    await self.client.disconnect()
                device = await BleakScanner.find_device_by_filter(
                    lambda d, adv: adv.local_name == NAME or d.name == NAME,
                    timeout=10,
                )
                if device is None:
                    raise RuntimeError("找不到 BLE_PC_Link：请先烧录新固件，开启电脑蓝牙，并断开手机连接。")
                client = BleakClient(device, disconnected_callback=self.disconnected, timeout=15)
                self.client = client
                try:
                    await client.connect()
                    self.decoder.reset()
                    # 必须先订阅通知，开发板才可以向电脑发送数据。
                    await client.start_notify(TX, self.notify)
                    self.ready = True
                except Exception:
                    await client.disconnect()
                    raise
                return "已连接 BLE_PC_Link，可发送数据"
            if command == "disconnect":
                if self.client:
                    await self.client.disconnect()
                self.ready = False
                return "已断开"
            if command == "send":
                if not self.ready or not self.client.is_connected:
                    raise RuntimeError("请先连接开发板")
                data = bytes.fromhex(text) if hex_mode else text.encode("utf-8")
                if not data:
                    raise ValueError("请输入要发送的内容")
                if len(data) > 4096:
                    raise ValueError("每次请发送不超过4096字节")
                # 固件当前MTU为23，每次写入最多20字节；无需加回车。
                for offset in range(0, len(data), 20):
                    await self.client.write_gatt_char(RX, data[offset:offset + 20], response=True)
                    await asyncio.sleep(0.02)
                return f"已发送 {len(data)} 字节：{data.hex(' ')}"
            raise ValueError("未知操作")

    def run(self, command, text="", hex_mode=False):
        future = asyncio.run_coroutine_threadsafe(self.action(command, text, hex_mode), self.loop)
        try:
            return future.result(timeout=60)
        except TimeoutError:
            future.cancel()
            raise RuntimeError("蓝牙操作超时，请重新连接")


def main():
    bluetooth = Bluetooth()
    page = Path(__file__).with_name("index.html").read_text(encoding="utf-8").replace("__TOKEN__", TOKEN)

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def reply(self, data, status=200):
            body = json.dumps(data, ensure_ascii=False).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path == "/":
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.end_headers()
                self.wfile.write(page.encode("utf-8"))
            elif self.path == "/state" and self.headers.get("X-Token") == TOKEN:
                events = []
                while True:
                    try:
                        events.append(bluetooth.events.get_nowait())
                    except queue.Empty:
                        break
                self.reply({"ready": bluetooth.ready, "events": events})
            else:
                self.reply({"error": "Not found"}, 404)

        def do_POST(self):
            if self.path != "/action" or self.headers.get("X-Token") != TOKEN:
                self.reply({"error": "Forbidden"}, 403)
                return
            try:
                length = int(self.headers.get("Content-Length", 0))
                if not 0 < length <= 65536:
                    raise ValueError("请求长度不正确")
                data = json.loads(self.rfile.read(length))
                message = bluetooth.run(data["command"], data.get("text", ""), data.get("hex", False))
                self.reply({"message": message})
            except Exception as error:
                self.reply({"error": str(error)}, 400)

    server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    url = f"http://127.0.0.1:{server.server_port}"
    print(f"BLE电脑收发窗口：{url}\n请保持此窗口打开，按 Ctrl+C 退出。", flush=True)
    webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            bluetooth.run("disconnect")
        finally:
            server.server_close()
            bluetooth.loop.call_soon_threadsafe(bluetooth.loop.stop)


if __name__ == "__main__":
    main()
