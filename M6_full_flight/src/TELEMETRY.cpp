/*
 * ============================================================================
 * TELEMETRY.cpp —— WiFi 数据看板实现
 * 页面放在 PROGMEM（不占 RAM），数据流与 LOGGER 共用 RAM 环形缓冲
 * ============================================================================
 */
#include "TELEMETRY.h"
#include "LOGGER.h"
#include <WiFi.h>
#include <WebServer.h>
#include <LittleFS.h>

static WebServer server(80);

/* ---- 看板页面（PROGMEM）---- */
static const char TEL_PAGE[] PROGMEM = R"HTML(<!DOCTYPE html>
<html lang="zh">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>S4Fly 遥测</title>
<style>
  body{font-family:system-ui,-apple-system,'Microsoft YaHei',sans-serif;margin:0;background:#10151c;color:#dbe4f0;padding:12px;}
  h1{font-size:18px;margin:0 0 4px;}
  .sub{font-size:12px;color:#8aa0b8;margin-bottom:10px;}
  .bar{display:flex;gap:8px;margin-bottom:10px;flex-wrap:wrap;}
  button{flex:1;min-width:120px;padding:10px;border:none;border-radius:8px;background:#2563eb;color:#fff;font-size:14px;}
  button.off{background:#475569;}
  #log{background:#0b0f16;border:1px solid #22303f;border-radius:8px;padding:8px;
       height:62vh;overflow-y:auto;font-family:Consolas,Menlo,monospace;font-size:12px;white-space:pre-wrap;word-break:break-all;}
  #log .new{color:#7dd3fc;}
  .st{font-size:12px;color:#94a3b8;margin-bottom:6px;}
</style>
</head>
<body>
<h1>S4Fly 飞行遥测</h1>
<div class="sub">热点 SSID: <b>S4Fly-Telemetry</b> · 飞行中每秒更新 · 断开 USB 也能看</div>
<div class="st" id="stat">等待数据…</div>
<div class="bar">
  <button id="btnPause" onclick="togglePause()">暂停滚动</button>
  <button onclick="download()">下载记录</button>
</div>
<div id="log"></div>
<script>
var lastSeq = 0, paused = false, all = [];
var logEl = document.getElementById('log');
var statEl = document.getElementById('stat');
function append(body){
  var d = document.createElement('div');
  d.textContent = body;
  logEl.appendChild(d);
  while (logEl.childNodes.length > 400) logEl.removeChild(logEl.firstChild);
  if (!paused) logEl.scrollTop = logEl.scrollHeight;
}
async function poll(){
  try{
    var r = await fetch('/feed?since=' + lastSeq);
    var txt = await r.text();
    if (!txt) return;
    var lines = txt.split('\n');
    for (var i=0;i<lines.length;i++){
      var ln = lines[i].trim();
      if (!ln) continue;
      var t = ln.indexOf('\t');
      if (t < 0) continue;
      var s = parseInt(ln.slice(0,t),10);
      var body = ln.slice(t+1);
      if (s > lastSeq){ lastSeq = s; all.push(body); append(body); }
    }
    statEl.textContent = '已接收 ' + all.length + ' 行 · 最新行号 ' + lastSeq;
  }catch(e){}
}
setInterval(poll, 1000);
poll();
function togglePause(){ paused = !paused; document.getElementById('btnPause').textContent = paused ? '继续滚动' : '暂停滚动'; document.getElementById('btnPause').className = paused ? 'off' : ''; }
function download(){
  if (!all.length){ alert('暂无数据'); return; }
  var blob = new Blob([all.join('\n') + '\n'], {type:'text/plain'});
  var a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = 's4fly_log_' + Date.now() + '.txt';
  a.click();
  setTimeout(function(){ URL.revokeObjectURL(a.href); }, 1000);
}
</script>
</body>
</html>
)HTML";

/* ---- 路由：看板页面 ---- */
static void handle_root() {
  server.sendHeader("Cache-Control", "no-store");
  server.send_P(200, "text/html; charset=utf-8", TEL_PAGE);
}

/* ---- 路由：增量日志 ---- */
static void handle_feed() {
  String sinceS = server.arg("since");
  uint32_t since = sinceS.length() ? (uint32_t)sinceS.toInt() : 0;
  uint32_t cur = logger_seq();
  if (cur == 0 || since >= cur) { server.send(200, "text/plain", ""); return; }

  uint32_t from = since + 1;
  if (cur - from + 1 > 100) from = cur - 100 + 1;   // 单次最多 100 行

  String out;
  out.reserve(100 * LOG_LINE_LEN);
  char buf[LOG_LINE_LEN];
  for (uint32_t s = from; s <= cur; s++) {
    if (logger_line_at(s, buf)) {
      out += String(s);
      out += '\t';
      out += buf;
    }
  }
  server.send(200, "text/plain", out);
}

/* ---- 路由：下载完整记录（优先 Flash 黑匣子）---- */
static void handle_download() {
  if (LittleFS.exists("/log.txt")) {
    File f = LittleFS.open("/log.txt", "r");
    if (f) {
      server.sendHeader("Content-Disposition", "attachment; filename=s4fly_log.txt");
      server.streamFile(f, "text/plain");
      f.close();
      return;
    }
  }
  // 兜底：RAM 环形
  String out;
  char buf[LOG_LINE_LEN];
  uint32_t cur = logger_seq();
  for (uint32_t s = 1; s <= cur; s++) {
    if (logger_line_at(s, buf)) out += buf;
  }
  server.sendHeader("Content-Disposition", "attachment; filename=s4fly_log.txt");
  server.send(200, "text/plain", out);
}

void telemetry_begin() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(TEL_SSID, TEL_PASS);
  delay(100);
  server.on("/", handle_root);
  server.on("/feed", handle_feed);
  server.on("/download", handle_download);
  server.begin();
  Serial.printf("[TEL] WiFi 看板: SSID=%s PASS=%s → 手机连热点开 http://192.168.4.1\n",
                TEL_SSID, TEL_PASS);
}

void telemetry_poll() {
  // 主 loop 为 1kHz，这里限流到 100Hz（10ms 节拍）：
  // 同步 WebServer 的 handleClient 每环空转纯属浪费 CPU，
  // 且浏览器是 1 秒轮询一次 /feed，10ms 粒度完全够用。
  static uint32_t tTel = 0;
  uint32_t now = millis();
  if (now - tTel < 10) return;
  tTel = now;
  server.handleClient();
}
