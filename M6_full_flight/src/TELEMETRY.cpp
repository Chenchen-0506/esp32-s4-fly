/*
 * ============================================================================
 * TELEMETRY.cpp —— WiFi 仪表盘实现
 * 页面放在 PROGMEM（不占 RAM），/api 输出结构化 JSON，
 * 数据流与 LOGGER 共用 RAM 环形缓冲。
 * ============================================================================
 */
#include "TELEMETRY.h"
#include "LOGGER.h"
#include <WiFi.h>
#include <WebServer.h>
#include <LittleFS.h>

static WebServer server(80);

tel_status_t tel_status = {0};

/* ---- 仪表盘页面（PROGMEM）---- */
static const char TEL_PAGE[] PROGMEM = R"HTML(<!DOCTYPE html>
<html lang="zh">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>S4Fly 飞行遥测</title>
<style>
  *{box-sizing:border-box;}
  body{font-family:system-ui,-apple-system,'Microsoft YaHei',sans-serif;margin:0;background:#10151c;color:#dbe4f0;padding:10px;}
  h1{font-size:16px;margin:0;}
  .sub{font-size:11px;color:#8aa0b8;}
  header{display:flex;justify-content:space-between;align-items:center;margin-bottom:10px;flex-wrap:wrap;gap:6px;}
  .st{display:flex;gap:8px;align-items:center;font-size:12px;}
  .dot{width:10px;height:10px;border-radius:50%;display:inline-block;background:#475569;}
  .dot.on{background:#22c55e;box-shadow:0 0 8px #22c55e;}
  .dot.warn{background:#f59e0b;}
  .badge{padding:2px 8px;border-radius:6px;background:#1e293b;font-size:12px;font-weight:600;}
  .badge.armed{background:#16a34a;}
  .badge.rtl{background:#f59e0b;}
  .grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(160px,1fr));gap:8px;}
  .card{background:#1e293b;border-radius:10px;padding:10px;}
  .card h3{font-size:11px;color:#8aa0b8;margin:0 0 6px;letter-spacing:.5px;text-transform:uppercase;}
  .big{font-size:clamp(22px,7vw,34px);font-weight:700;font-variant-numeric:tabular-nums;line-height:1;}
  .unit{font-size:12px;color:#8aa0b8;font-weight:400;margin-left:3px;}
  .pair{display:flex;justify-content:space-between;font-size:12px;color:#94a3b8;margin-top:4px;}
  .ai{width:100%;display:block;}
  .bar{height:8px;background:#0b0f16;border-radius:4px;position:relative;margin:4px 0 2px;}
  .bar i{position:absolute;left:0;top:0;bottom:0;border-radius:4px;background:#2563eb;}
  .bar .mid{position:absolute;left:50%;top:-2px;bottom:-2px;width:1px;background:#64748b;}
  .row{display:flex;align-items:center;gap:6px;font-size:11px;margin-bottom:3px;}
  .row .lbl{width:26px;color:#8aa0b8;font-weight:600;}
  .row .val{width:38px;text-align:right;font-variant-numeric:tabular-nums;color:#7dd3fc;}
  .row .bar{flex:1;}
  .gpsline{font-size:12px;font-variant-numeric:tabular-nums;color:#94a3b8;margin-top:2px;}
  .altbar{position:relative;height:90px;background:linear-gradient(180deg,#0b0f16,#12202e);border-radius:6px;overflow:hidden;margin-top:6px;}
  .altbar .fill{position:absolute;bottom:0;left:0;right:0;background:linear-gradient(0deg,#0ea5e9,#22c55e);}
  .altbar .zero{position:absolute;left:0;right:0;height:1px;background:#64748b;}
  #log{background:#0b0f16;border:1px solid #22303f;border-radius:10px;padding:8px;margin-top:8px;
       height:140px;overflow-y:auto;font-family:Consolas,Menlo,monospace;font-size:11px;white-space:pre-wrap;word-break:break-all;display:none;}
  .logbar{display:flex;gap:8px;margin-top:8px;flex-wrap:wrap;}
  .logbar button{flex:1;min-width:110px;padding:8px;border:none;border-radius:8px;background:#2563eb;color:#fff;font-size:12px;}
  .logbar button.off{background:#475569;}
  .logbar button.alt{background:#1e293b;color:#cbd5e1;}
  .altwrap{display:none;}
</style>
</head>
<body>
<header>
  <div>
    <h1>S4Fly 飞行遥测</h1>
    <div class="sub">SSID: S4Fly-Telemetry · 断开 USB 也能看</div>
  </div>
  <div class="st">
    <span id="armDot" class="dot"></span><span id="armTxt">上锁</span>
    <span id="modeBadge" class="badge">MANU</span>
    <span id="gpsDot" class="dot"></span><span id="gpsTxt">无定位</span>
  </div>
</header>

<div class="grid">
  <div class="card" style="grid-column:span 2;">
    <h3>姿态</h3>
    <svg id="ai" class="ai" viewBox="0 0 220 180">
      <defs><clipPath id="aiClip"><circle cx="110" cy="90" r="82"/></clipPath></defs>
      <circle cx="110" cy="90" r="82" fill="#0b0f16" stroke="#22303f" stroke-width="2"/>
      <g clip-path="url(#aiClip)">
        <g id="aiRoll" transform="translate(110,90)">
          <g id="aiPitch">
            <rect id="sky" x="-200" y="-240" width="400" height="240" fill="#0e2a4a"/>
            <rect id="gnd" x="-200" y="0" width="400" height="240" fill="#1c2936"/>
            <line x1="-200" y1="0" x2="200" y2="0" stroke="#7dd3fc" stroke-width="2"/>
            <line x1="-200" y1="-30" x2="200" y2="-30" stroke="#33475e" stroke-width="1" stroke-dasharray="5 9"/>
            <line x1="-200" y1="30" x2="200" y2="30" stroke="#33475e" stroke-width="1" stroke-dasharray="5 9"/>
          </g>
        </g>
      </g>
      <g stroke="#94a3b8" stroke-width="1.4">
        <line x1="110" y1="2" x2="110" y2="13"/><line x1="110" y1="167" x2="110" y2="178"/>
        <line x1="16" y1="90" x2="27" y2="90"/><line x1="193" y1="90" x2="204" y2="90"/>
        <path d="M110 6 L117 13 L103 13 Z" fill="#94a3b8" stroke="none"/>
      </g>
      <path id="ship" d="M110 74 L121 88 L99 88 Z" fill="none" stroke="#fbbf24" stroke-width="2"/>
      <circle cx="110" cy="90" r="2.6" fill="#fbbf24"/>
    </svg>
    <div class="pair"><span>横滚 <b id="vRoll" style="color:#7dd3fc">0.0</b>°</span><span>俯仰 <b id="vPitch" style="color:#7dd3fc">0.0</b>°</span><span>偏航 <b id="vYaw" style="color:#7dd3fc">0.0</b>°</span></div>
  </div>

  <div class="card">
    <h3>高度</h3>
    <div class="big"><span id="vAlt">0.0</span><span class="unit">m</span></div>
    <div class="altbar">
      <div class="zero" id="altZero" style="bottom:0"></div>
      <div class="fill" id="altFill"></div>
    </div>
  </div>

  <div class="card">
    <h3>航向</h3>
    <div class="big"><span id="vHdg">--</span><span class="unit">°</span></div>
    <div class="pair"><span id="hdgDir">--</span><span>HOME <b id="vHome" style="color:#7dd3fc">--</b></span></div>
  </div>

  <div class="card">
    <h3>GPS</h3>
    <div class="pair"><span>星数</span><b id="vSats" style="color:#7dd3fc">--</b></div>
    <div class="gpsline" id="vLat">纬度 --</div>
    <div class="gpsline" id="vLon">经度 --</div>
  </div>

  <div class="card" style="grid-column:span 2;">
    <h3>遥控通道 CH1~CH7</h3>
    <div id="rcRows"></div>
  </div>

  <div class="card" style="grid-column:span 2;">
    <h3>电机油门</h3>
    <div id="motRows"></div>
  </div>
</div>

<div class="logbar">
  <button id="btnLog" onclick="toggleLog()">显示日志</button>
  <button id="btnPause" onclick="togglePause()" class="alt">暂停日志</button>
  <button onclick="download()">下载记录</button>
</div>
<div id="log"></div>

<script>
(function(){
var armDot=document.getElementById('armDot'),armTxt=document.getElementById('armTxt'),
    modeB=document.getElementById('modeBadge'),gpsDot=document.getElementById('gpsDot'),gpsTxt=document.getElementById('gpsTxt'),
    aiRoll=document.getElementById('aiRoll'),aiPitch=document.getElementById('aiPitch'),
    vRoll=document.getElementById('vRoll'),vPitch=document.getElementById('vPitch'),vYaw=document.getElementById('vYaw'),
    vAlt=document.getElementById('vAlt'),altFill=document.getElementById('altFill'),
    vHdg=document.getElementById('vHdg'),hdgDir=document.getElementById('hdgDir'),vHome=document.getElementById('vHome'),
    vSats=document.getElementById('vSats'),vLat=document.getElementById('vLat'),vLon=document.getElementById('vLon'),
    rcRows=document.getElementById('rcRows'),motRows=document.getElementById('motRows');
var lastSeq=0,paused=false,logOn=false,all=[];
var logEl=document.getElementById('log');
var RC_LBL=['ROLL','PITCH','THR','YAW','ARM','RTL','POS'];
var DIRS=['N','NE','E','SE','S','SW','W','NW'];
function barHtml(v){
  var p=Math.max(0,Math.min(100,(v-1000)/10));
  return '<div class="bar"><i style="width:'+p+'%"></i><span class="mid"></span></div>';
}
function render(d){
  armDot.className='dot'+(d.armed?' on':'');
  armTxt.textContent=d.armed?'解锁':'上锁';
  modeB.textContent=d.mode;
  modeB.className='badge'+(d.mode.indexOf('RTL')===0||d.mode==='MAGCAL'?' rtl':(d.armed?' armed':''));
  var fix=d.fix&&d.sats>0;
  gpsDot.className='dot'+(fix?' on':(d.home?' warn':''));
  gpsTxt.textContent=fix?'已定位':'无定位';
  var r=isFinite(d.roll)?d.roll:0,p=isFinite(d.pitch)?d.pitch:0;
  var pr=Math.max(-60,Math.min(60,-p*1.8));
  aiRoll.setAttribute('transform','translate(110,90) rotate('+r+')');
  aiPitch.setAttribute('transform','translate(0,'+pr+')');
  vRoll.textContent=r.toFixed(1);vPitch.textContent=p.toFixed(1);vYaw.textContent=isFinite(d.yaw)?d.yaw.toFixed(1):'--';
  var alt=isFinite(d.alt)?d.alt:0;
  vAlt.textContent=alt.toFixed(1);
  var ap=Math.max(0,Math.min(100,alt/1.2));
  altFill.style.height=ap+'%';
  var h=isFinite(d.hdg)?d.hdg:-1;
  if(h<0){vHdg.textContent='--';hdgDir.textContent='罗盘离线';}
  else{vHdg.textContent=h.toFixed(0);hdgDir.textContent=DIRS[Math.floor(((h+22.5)%360)/45)];}
  vHome.textContent=d.home?'已设':'未设';
  vSats.textContent=d.sats;
  vLat.textContent='纬度 '+(isFinite(d.lat)?d.lat.toFixed(5):'--');
  vLon.textContent='经度 '+(isFinite(d.lon)?d.lon.toFixed(5):'--');
  rcRows.innerHTML='';
  for(var i=0;i<7;i++){
    var row=document.createElement('div');row.className='row';
    row.innerHTML='<span class="lbl">'+RC_LBL[i]+'</span>'+barHtml(d.rc[i])+'<span class="val">'+d.rc[i]+'</span>';
    rcRows.appendChild(row);
  }
  motRows.innerHTML='';
  for(var j=0;j<4;j++){
    var m=document.createElement('div');m.className='row';
    var mp=Math.max(0,Math.min(100,(d.mot[j]-1000)/10));
    m.innerHTML='<span class="lbl">M'+(j+1)+'</span><div class="bar"><i style="width:'+mp+'%"></i><span class="mid"></span></div><span class="val">'+d.mot[j]+'</span>';
    motRows.appendChild(m);
  }
}
async function tick(){
  try{var r=await fetch('/api');var d=await r.json();render(d);}catch(e){}
}
function appendLog(body){
  var d=document.createElement('div');d.textContent=body;
  logEl.appendChild(d);
  while(logEl.childNodes.length>300)logEl.removeChild(logEl.firstChild);
  if(!paused&&logOn)logEl.scrollTop=logEl.scrollHeight;
}
async function pollLog(){
  if(!logOn)return;
  try{
    var r=await fetch('/feed?since='+lastSeq);var txt=await r.text();
    if(!txt)return;
    var lines=txt.split('\n');
    for(var i=0;i<lines.length;i++){
      var ln=lines[i].trim();if(!ln)continue;
      var t=ln.indexOf('\t');if(t<0)continue;
      var s=parseInt(ln.slice(0,t),10);var body=ln.slice(t+1);
      if(s>lastSeq){lastSeq=s;all.push(body);appendLog(body);}
    }
  }catch(e){}
}
window.toggleLog=function(){logOn=!logOn;document.getElementById('log').style.display=logOn?'block':'none';
  document.getElementById('btnLog').textContent=logOn?'隐藏日志':'显示日志';
  document.getElementById('btnLog').className=logOn?'off':'';pollLog();};
window.togglePause=function(){paused=!paused;document.getElementById('btnPause').textContent=paused?'继续日志':'暂停日志';};
window.download=function(){
  if(!all.length){alert('暂无日志数据');return;}
  var blob=new Blob([all.join('\n')+'\n'],{type:'text/plain'});
  var a=document.createElement('a');a.href=URL.createObjectURL(blob);a.download='s4fly_log_'+Date.now()+'.txt';a.click();
  setTimeout(function(){URL.revokeObjectURL(a.href);},1000);
};
setInterval(tick,500);
tick();
setInterval(pollLog,1000);
})();
</script>
</body>
</html>
)HTML";

/* ---- 路由：仪表盘页面 ---- */
static void handle_root() {
  server.sendHeader("Cache-Control", "no-store");
  server.send_P(200, "text/html; charset=utf-8", TEL_PAGE);
}

/* ---- 路由：最新状态 JSON ---- */
static void handle_api() {
  String j;
  j.reserve(300);
  j = "{\"t\":"; j += (uint32_t)millis();
  j += ",\"armed\":"; j += tel_status.armed;
  j += ",\"mode\":\""; j += tel_status.mode; j += "\"";
  j += ",\"roll\":"; j += String(tel_status.roll, 1);
  j += ",\"pitch\":"; j += String(tel_status.pitch, 1);
  j += ",\"yaw\":"; j += String(tel_status.yaw, 1);
  j += ",\"alt\":"; j += String(tel_status.alt, 1);
  j += ",\"rc\":[";
  for (int i = 0; i < 7; i++) { if (i) j += ','; j += tel_status.rc[i]; }
  j += "],\"fix\":"; j += tel_status.gps_fix;
  j += ",\"sats\":"; j += tel_status.gps_sats;
  j += ",\"lat\":"; j += String(tel_status.gps_lat, 5);
  j += ",\"lon\":"; j += String(tel_status.gps_lon, 5);
  j += ",\"home\":"; j += tel_status.home_valid;
  j += ",\"hdg\":"; j += String(tel_status.hdg, 1);
  j += ",\"mot\":[";
  for (int i = 0; i < 4; i++) { if (i) j += ','; j += tel_status.mot[i]; }
  j += "]}";
  server.send(200, "application/json", j);
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
  server.on("/api", handle_api);
  server.on("/feed", handle_feed);
  server.on("/download", handle_download);
  server.begin();
  Serial.printf("[TEL] WiFi 仪表盘: SSID=%s PASS=%s → 手机连热点开 http://192.168.4.1\n",
                TEL_SSID, TEL_PASS);
}

void telemetry_poll() {
  // 主 loop 为 1kHz，这里限流到 100Hz（10ms 节拍）：
  // 同步 WebServer 的 handleClient 每环空转纯属浪费 CPU，
  // 且浏览器是 500ms~1s 轮询，10ms 粒度完全够用。
  static uint32_t tTel = 0;
  uint32_t now = millis();
  if (now - tTel < 10) return;
  tTel = now;
  server.handleClient();
}
