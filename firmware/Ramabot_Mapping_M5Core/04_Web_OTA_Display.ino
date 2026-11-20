#include "config.h"

bool imuSerialDebug = false;

// ================= HALAMAN WEB GRAPH TUNING, CSV LOGGER & WEB OTA =================
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>RAMABOT MAPPING — Heading PID Graph Tuning, Logger & OTA</title>
<style>
  body{font-family:system-ui,sans-serif;background:#111827;color:#f3f4f6;margin:0;padding:16px}
  h2{margin:0 0 12px;color:#38bdf8;font-size:1.2rem}
  .grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(135px,1fr));gap:10px;margin-bottom:14px}
  .card{background:#1f2937;padding:10px 12px;border-radius:8px;border:1px solid #374151}
  .lbl{font-size:0.75rem;color:#9ca3af}
  .val{font-size:1.18rem;font-weight:700;margin-top:2px}
  .toolbar{display:flex;flex-wrap:wrap;gap:8px;margin-bottom:12px;align-items:center}
  button{background:#2563eb;color:#fff;border:none;padding:8px 13px;border-radius:6px;font-weight:600;cursor:pointer}
  button:hover{background:#1d4ed8}
  .btn-rec{background:#dc2626}.btn-rec.active{background:#16a34a}
  .btn-step{background:#d97706}
  canvas{width:100%;height:260px;background:#0f172a;border:1px solid #334155;border-radius:8px;display:block;margin-bottom:14px}
  .tune{display:grid;grid-template-columns:repeat(auto-fit,minmax(115px,1fr));gap:8px;background:#1f2937;padding:12px;border-radius:8px;margin-bottom:14px}
  .tune label{font-size:0.75rem;color:#cbd5e1;display:block}
  .tune input{width:100%;box-sizing:border-box;background:#111827;color:#fff;border:1px solid #4b5563;padding:6px;border-radius:4px;margin-top:3px}
  .ota-box{background:#1f2937;padding:12px;border-radius:8px;border:1px solid #374151}
  .ota-row{display:flex;flex-wrap:wrap;gap:10px;align-items:center;margin-top:8px}
  progress{width:220px;height:18px}
</style>
</head>
<body>
<h2>RAMABOT MAPPING — Heading PID Graph Tuning, CSV Logger & OTA</h2>

<div class="toolbar">
  <button id="recBtn" class="btn-rec" onclick="toggleRec()">Mulai Rekam CSV</button>
  <button onclick="downloadCSV()">Download CSV (<span id="cnt">0</span> baris)</button>
  <button onclick="clearData()">Clear Grafik</button>
  <button class="btn-step" onclick="runStepTest(0.30, 3000)">Uji Step Lurus 0.30 m/s (3 dtk)</button>
  <button class="btn-step" style="background:#0284c7" onclick="runStepTurn(90)">Uji Step Putar +90°</button>
  <button style="background:#0d9488" onclick="toggleLock()">Toggle Heading Lock</button>
  <button style="background:#4b5563" onclick="fetch('/reset')">Reset Odo & Heading</button>
  <button style="background:#b45309" onclick="resetDef()">Reset Default PID</button>
</div>

<div class="grid">
  <div class="card"><div class="lbl">Target / Actual Heading</div><div class="val" style="color:#38bdf8" id="hdg">0.0° / 0.0°</div></div>
  <div class="card"><div class="lbl">Yaw PID Output</div><div class="val" style="color:#facc15" id="yawOut">0.0</div></div>
  <div class="card"><div class="lbl">PWM (L / R)</div><div class="val" style="color:#4ade80" id="pwm">0 / 0</div></div>
  <div class="card"><div class="lbl">Est. Vel (Vx)</div><div class="val" id="vx">0.00 m/s</div></div>
  <div class="card"><div class="lbl">Odometry (X, Y)</div><div class="val" id="odo">0.00, 0.00</div></div>
</div>

<canvas id="chart" width="900" height="260"></canvas>

<div class="tune">
  <div><label>Yaw Kp</label><input id="kpy" type="number" step="0.05"></div>
  <div><label>Yaw Ki</label><input id="kiy" type="number" step="0.01"></div>
  <div><label>Yaw Kd</label><input id="kdy" type="number" step="0.01"></div>
  <div><label>Max Vel (m/s)</label><input id="maxvel" type="number" step="0.01"></div>
  <div><label>Norm Speed (0-1)</label><input id="normpct" type="number" step="0.05" min="0.2" max="1.0"></div>
  <div><label>Linear Scale</label><input id="linscale" type="number" step="0.01"></div>
  <div style="display:flex;align-items:end"><button style="width:100%;background:#059669" onclick="savePID()">Simpan PID</button></div>
</div>

<div class="ota-box">
  <div style="font-weight:700;color:#fbbf24">Firmware Update OTA (File .bin dari Sketch &gt; Export Compiled Binary)</div>
  <div class="ota-row">
    <input type="file" id="binFile" accept=".bin">
    <button style="background:#7c3aed" onclick="uploadOTA()">Upload Firmware OTA</button>
    <progress id="otaProg" value="0" max="100"></progress>
    <span id="otaStat" style="font-size:0.85rem;color:#cbd5e1">Siap</span>
  </div>
</div>

<script>
let recording = false, t0 = 0, otaBusy = false;
let logs = [["time_ms","tgtHeading_deg","heading_deg","yawPid","pwmL","pwmR","estVx","odoX","odoY","odoTheta_deg"]];
let hist = {tgtHdg:[], hdg:[], yawOut:[]};
const MAX_PTS = 180;

function loadParams(){
  fetch('/params').then(r=>r.json()).then(p=>{
    document.getElementById('kpy').value = p.kpy;
    document.getElementById('kiy').value = p.kiy;
    document.getElementById('kdy').value = p.kdy;
    document.getElementById('maxvel').value = p.maxvel;
    document.getElementById('normpct').value = p.normpct;
    document.getElementById('linscale').value = p.linscale;
  });
}
loadParams();

function toggleRec(){
  recording = !recording;
  const b = document.getElementById('recBtn');
  if(recording){ t0 = performance.now(); b.textContent = "Stop Rekam"; b.classList.add('active'); }
  else { b.textContent = "Mulai Rekam CSV"; b.classList.remove('active'); }
}

function clearData(){
  hist = {tgtHdg:[], hdg:[], yawOut:[]};
  logs = [logs[0]];
  document.getElementById('cnt').textContent = 0;
}

function runStepTest(vel, dur){
  if(!recording) toggleRec();
  fetch(`/step?vel=${vel}&dur=${dur}`);
}

function runStepTurn(deg){
  if(!recording) toggleRec();
  fetch(`/stepturn?deg=${deg}`);
}

function savePID(){
  const q = `kpy=${kpy.value}&kiy=${kiy.value}&kdy=${kdy.value}&maxvel=${maxvel.value}&normpct=${normpct.value}&linscale=${linscale.value}`;
  fetch('/set?' + q).then(()=>alert('Parameter PID tersimpan di Flash M5Stack!'));
}

function resetDef(){
  fetch('/resetdef').then(()=>{ loadParams(); alert('Parameter PID dikembalikan ke default stabil!'); });
}

function downloadCSV(){
  const csv = logs.map(r=>r.join(",")).join("\n");
  const blob = new Blob([csv], {type:"text/csv"});
  const a = document.createElement("a");
  a.href = URL.createObjectURL(blob);
  a.download = "ramabot_mapping_pid_" + Date.now() + ".csv";
  a.click();
}

function uploadOTA(){
  const fileInput = document.getElementById('binFile');
  if(!fileInput.files.length){ alert('Pilih file firmware .bin terlebih dahulu!'); return; }
  otaBusy = true;
  const formData = new FormData();
  formData.append('update', fileInput.files[0]);
  const xhr = new XMLHttpRequest();
  xhr.open('POST', '/update', true);
  xhr.upload.onprogress = (e) => {
    if(e.lengthComputable){
      const pct = Math.round((e.loaded / e.total) * 100);
      document.getElementById('otaProg').value = pct;
      document.getElementById('otaStat').textContent = `Mengunggah: ${pct}%`;
    }
  };
  xhr.onload = () => {
    if(xhr.status === 200 && xhr.responseText === 'OK'){
      document.getElementById('otaStat').textContent = 'Sukses! M5Stack rebooting...';
      setTimeout(()=>location.reload(), 4000);
    } else {
      document.getElementById('otaStat').textContent = 'Gagal Update OTA!';
      otaBusy = false;
    }
  };
  xhr.onerror = () => { document.getElementById('otaStat').textContent = 'Koneksi terputus!'; otaBusy = false; };
  xhr.send(formData);
}

function drawChart(){
  const c = document.getElementById('chart'), ctx = c.getContext('2d');
  const W = c.width, H = c.height;
  ctx.clearRect(0,0,W,H);
  ctx.strokeStyle = '#1e293b'; ctx.lineWidth = 1;
  for(let y=0;y<=H;y+=H/4){ ctx.beginPath(); ctx.moveTo(0,y); ctx.lineTo(W,y); ctx.stroke(); }
  const midY = H/2, scale = (H/2 - 15)/180;
  ctx.strokeStyle = '#475569'; ctx.beginPath(); ctx.moveTo(0,midY); ctx.lineTo(W,midY); ctx.stroke();

  const plot = (arr, color) => {
    if(arr.length < 2) return;
    ctx.strokeStyle = color; ctx.lineWidth = 2; ctx.beginPath();
    arr.forEach((v,i)=>{
      const x = (i/(MAX_PTS-1))*W;
      const y = midY - Math.max(-180,Math.min(180,v))*scale;
      i===0 ? ctx.moveTo(x,y) : ctx.lineTo(x,y);
    });
    ctx.stroke();
  };
  plot(hist.tgtHdg, '#f87171');
  plot(hist.hdg, '#4ade80');
  plot(hist.yawOut, '#38bdf8');

  ctx.fillStyle='#94a3b8'; ctx.font='12px sans-serif';
  ctx.fillText('Merah: Target Heading (°) | Hijau: Actual Heading (°) | Biru: Yaw PID PWM Output (Skala -180 s/d +180)', 10, 18);
}

setInterval(async ()=>{
  if(otaBusy) return;
  try{
    const r = await fetch('/data');
    const d = await r.json();
    document.getElementById('hdg').textContent = `${d.tgtHdg.toFixed(1)}° / ${d.hdg.toFixed(1)}°`;
    document.getElementById('yawOut').textContent = d.yawOut.toFixed(1);
    document.getElementById('pwm').textContent = `${d.pwmL.toFixed(0)} / ${d.pwmR.toFixed(0)}`;
    document.getElementById('vx').textContent = `${d.vx.toFixed(2)} m/s`;
    document.getElementById('odo').textContent = `${d.x.toFixed(2)}, ${d.y.toFixed(2)}`;

    hist.tgtHdg.push(d.tgtHdg); hist.hdg.push(d.hdg); hist.yawOut.push(d.yawOut);
    if(hist.tgtHdg.length > MAX_PTS){ hist.tgtHdg.shift(); hist.hdg.shift(); hist.yawOut.shift(); }
    drawChart();

    if(recording){
      const dt = Math.round(performance.now() - t0);
      logs.push([dt, d.tgtHdg.toFixed(2), d.hdg.toFixed(2), d.yawOut.toFixed(2),
                 d.pwmL.toFixed(1), d.pwmR.toFixed(1), d.vx.toFixed(3),
                 d.x.toFixed(4), d.y.toFixed(4), d.th.toFixed(2)]);
      document.getElementById('cnt').textContent = logs.length - 1;
    }
  }catch(e){}
}, 50);

function toggleLock(){
  fetch('/togglelock').then(r=>r.text()).then(s=>alert('Heading Lock: ' + s));
}

</script>
</body>
</html>
)rawliteral";

// ================= SETUP ARDUINO OTA =================
void setupArduinoOTA() {
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.setMdnsEnabled(true);

  ArduinoOTA.onStart([]() {
    otaInProgress = true;
    motorRunning = false;
    webStepActive = false;
    driveMotors(0, 0);

    M5.Display.clear();
    M5.Display.setCursor(0, 20);
    M5.Display.setTextColor(YELLOW, BLACK);
    M5.Display.println("ARDUINO OTA UPDATE");
    M5.Display.println("Jangan matikan daya!");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    unsigned int pct = (progress / (total / 100));
    M5.Display.setCursor(0, 80);
    M5.Display.setTextColor(CYAN, BLACK);
    M5.Display.printf("Progress: %u%%   ", pct);
  });

  ArduinoOTA.onEnd([]() {
    M5.Display.setCursor(0, 110);
    M5.Display.setTextColor(GREEN, BLACK);
    M5.Display.println("SELESAI! REBOOT...");
  });

  ArduinoOTA.onError([](ota_error_t error) {
    otaInProgress = false;
    M5.Display.setCursor(0, 110);
    M5.Display.setTextColor(RED, BLACK);
    M5.Display.printf("OTA ERROR [%u]", error);
  });

  ArduinoOTA.begin();
}

// ================= SETUP WEB SERVER & ENDPOINTS =================
void setupWebServer() {
  server.on("/", HTTP_GET, []() {
    server.send_P(200, "text/html", INDEX_HTML);
  });

  server.on("/data", HTTP_GET, []() {
    portENTER_CRITICAL(&stateMux);
    float pL = outputPWM_A, pR = outputPWM_B;
    float hdg = headingDeg, thdg = targetHeading, yOut = yawPidOut;
    float vx = estVx, x = odoX, y = odoY, th = odoTheta * RAD_TO_DEG;
    portEXIT_CRITICAL(&stateMux);

    char json[260];
    snprintf(json, sizeof(json),
             "{\"pwmL\":%.1f,\"pwmR\":%.1f,\"hdg\":%.2f,\"tgtHdg\":%.2f,"
             "\"yawOut\":%.2f,\"vx\":%.3f,\"x\":%.4f,\"y\":%.4f,\"th\":%.2f}",
             pL, pR, hdg, thdg, yOut, vx, x, y, th);
    server.send(200, "application/json", json);
  });

  server.on("/params", HTTP_GET, []() {
    char json[180];
    snprintf(json, sizeof(json),
             "{\"kpy\":%.3f,\"kiy\":%.3f,\"kdy\":%.3f,\"maxvel\":%.3f,\"normpct\":%.2f,\"linscale\":%.3f}",
             Kp_yaw, Ki_yaw, Kd_yaw, maxLinearVel, normalSpeedPct, linearScale);
    server.send(200, "application/json", json);
  });

  server.on("/set", HTTP_GET, []() {
    if (server.hasArg("kpy"))      Kp_yaw = max(0.0f, server.arg("kpy").toFloat());
    if (server.hasArg("kiy"))      Ki_yaw = max(0.0f, server.arg("kiy").toFloat());
    if (server.hasArg("kdy"))      Kd_yaw = max(0.0f, server.arg("kdy").toFloat());
    if (server.hasArg("maxvel"))   maxLinearVel = constrain(server.arg("maxvel").toFloat(), 0.1f, 2.0f);
    if (server.hasArg("normpct"))  normalSpeedPct = constrain(server.arg("normpct").toFloat(), 0.2f, 1.0f);
    if (server.hasArg("linscale")) linearScale = constrain(server.arg("linscale").toFloat(), 0.3f, 2.5f);
    savePreferences();
    server.send(200, "text/plain", "OK");
  });

  server.on("/step", HTTP_GET, []() {
    float v = server.hasArg("vel") ? server.arg("vel").toFloat() : 0.30f;
    unsigned long dur = server.hasArg("dur") ? server.arg("dur").toInt() : 3000;
    portENTER_CRITICAL(&stateMux);
    webStepTargetVel = constrain(v, -maxLinearVel, maxLinearVel);
    webStepEndTime = millis() + dur;
    webStepActive = true;
    motorRunning = true;
    eStopActive = false;
    portEXIT_CRITICAL(&stateMux);
    server.send(200, "text/plain", "STEP STARTED");
  });

  server.on("/stepturn", HTTP_GET, []() {
    float deg = server.hasArg("deg") ? server.arg("deg").toFloat() : 90.0f;
    portENTER_CRITICAL(&stateMux);
    targetHeading = normalizeAngleDeg(targetHeading + deg);
    yawErrSum = 0.0f;
    lastYawErr = 0.0f;
    motorRunning = true;
    eStopActive = false;
    portEXIT_CRITICAL(&stateMux);
    server.send(200, "text/plain", "STEP TURN STARTED");
  });

  server.on("/reset", HTTP_GET, []() {
    resetPoseAndOdometry();
    server.send(200, "text/plain", "RESET OK");
  });

  server.on("/togglelock", HTTP_GET, []() {
    portENTER_CRITICAL(&stateMux);
    headingLockEnabled = !headingLockEnabled;
    targetHeading = headingDeg;
    yawErrSum = 0.0f;
    lastYawErr = 0.0f;
    portEXIT_CRITICAL(&stateMux);
    savePreferences();
    server.send(200, "text/plain", headingLockEnabled ? "ON" : "OFF");
  });

  server.on("/resetdef", HTTP_GET, []() {
    resetDefaultPreferences();
    server.send(200, "text/plain", "RESET DEFAULT OK");
  });

  server.on("/update", HTTP_POST, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", Update.hasError() ? "FAIL" : "OK");
    delay(500);
    ESP.restart();
  }, []() {
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
      otaInProgress = true;
      motorRunning = false;
      webStepActive = false;
      driveMotors(0, 0);
      M5.Display.clear();
      M5.Display.setCursor(0, 20);
      M5.Display.setTextColor(YELLOW, BLACK);
      M5.Display.println("WEB OTA UPDATE...");
      Update.begin(UPDATE_SIZE_UNKNOWN);
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      if (Update.write(upload.buf, upload.currentSize) == upload.currentSize) {
        M5.Display.setCursor(0, 80);
        M5.Display.setTextColor(CYAN, BLACK);
        M5.Display.printf("Diterima: %u KB  ", Update.progress() / 1024);
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) {
        M5.Display.setCursor(0, 110);
        M5.Display.setTextColor(GREEN, BLACK);
        M5.Display.println("SUKSES! REBOOT...");
      } else {
        otaInProgress = false;
        M5.Display.setCursor(0, 110);
        M5.Display.setTextColor(RED, BLACK);
        M5.Display.println("GAGAL WRITE FLASH!");
      }
    }
  });

  server.begin();
  MDNS.addService("http", "tcp", 80);
}

// ================= SIARAN UDP TELEMETRY (20Hz) =================
void sendUdpTelemetry() {
  portENTER_CRITICAL(&stateMux);
  float pL = outputPWM_A, pR = outputPWM_B, hdg = headingDeg, yOut = yawPidOut;
  float x = odoX, y = odoY;
  portEXIT_CRITICAL(&stateMux);

  char udpBuf[140];
  int n = snprintf(udpBuf, sizeof(udpBuf),
                   "%lu,%.1f,%.1f,%.2f,%.2f,%.4f,%.4f\n",
                   millis(), pL, pR, hdg, yOut, x, y);
  udp.beginPacket(IPAddress(255, 255, 255, 255), UDP_PORT);
  udp.write((uint8_t*)udpBuf, n);
  udp.endPacket();
}

// ================= GAMBAR KOMPAS DIGITAL (LANGSUNG KE M5.DISPLAY) =================
void drawHeadingCompass(int centerX, int centerY, int radius, float angleDeg) {
  M5.Display.fillCircle(centerX, centerY, radius + 12, BLACK);
  M5.Display.drawCircle(centerX, centerY, radius, WHITE);
  M5.Display.drawCircle(centerX, centerY, 2, WHITE);

  float rad = (-angleDeg) * DEG_TO_RAD;
  int xTarget = centerX + (int)((radius - 6) * sinf(rad));
  int yTarget = centerY - (int)((radius - 6) * cosf(rad));
  int xFront  = centerX + (int)((radius + 7) * sinf(rad));
  int yFront  = centerY - (int)((radius + 7) * cosf(rad));

  M5.Display.drawLine(centerX, centerY, xTarget, yTarget, GREEN);
  M5.Display.fillCircle(xTarget, yTarget, 3, RED);

  M5.Display.setTextSize(1);
  M5.Display.setTextColor(CYAN, BLACK);
  M5.Display.setCursor(xFront - 3, yFront - 3);
  M5.Display.print("F");
  M5.Display.setTextSize(2);
}

// ================= KONTROL TOMBOL FISIK M5STACK =================
void handlePhysicalButtons() {
  if (M5.BtnA.wasPressed()) {
    screenMode = (UIMode)((screenMode + 1) % UI_MODE_COUNT);
    M5.Display.clear();
  }

  if (M5.BtnB.wasPressed()) {
    if (screenMode == DASHBOARD) { headingLockEnabled = !headingLockEnabled; savePreferences(); }
    else if (screenMode == TUNE_YAW_KP) { Kp_yaw += 0.10f; savePreferences(); }
    else if (screenMode == TUNE_YAW_KI) { Ki_yaw += 0.01f; savePreferences(); }
    else if (screenMode == TUNE_YAW_KD) { Kd_yaw += 0.02f; savePreferences(); }
    else if (screenMode == TUNE_MAX_VEL) { maxLinearVel += 0.05f; savePreferences(); }
    else if (screenMode == TUNE_LIN_SCALE) { linearScale += 0.02f; savePreferences(); }
    else if (screenMode == IMU_VIEW) { imuSerialDebug = !imuSerialDebug; }
  }

  if (M5.BtnC.wasPressed()) {
    if (screenMode == DASHBOARD) { resetPoseAndOdometry(); }
    else if (screenMode == TUNE_YAW_KP) { Kp_yaw = max(0.0f, Kp_yaw - 0.10f); savePreferences(); }
    else if (screenMode == TUNE_YAW_KI) { Ki_yaw = max(0.0f, Ki_yaw - 0.01f); savePreferences(); }
    else if (screenMode == TUNE_YAW_KD) { Kd_yaw = max(0.0f, Kd_yaw - 0.02f); savePreferences(); }
    else if (screenMode == TUNE_MAX_VEL) { maxLinearVel = max(0.10f, maxLinearVel - 0.05f); savePreferences(); }
    else if (screenMode == TUNE_LIN_SCALE) { linearScale = max(0.30f, linearScale - 0.02f); savePreferences(); }
    else if (screenMode == IMU_VIEW) { calibrateGyroZ(); }
    else if (screenMode == ODOMETRY_VIEW) { resetPoseAndOdometry(); }
  }
}

// ================= RENDER HALAMAN LCD M5STACK =================
void renderLcdScreen() {
  M5.Display.setCursor(0, 0);
  if (screenMode == DASHBOARD) {
    M5.Display.setTextColor(WHITE, BLACK);
    M5.Display.printf("SPD :%s\n", boostActive ? "BOOST" : "NORM ");
    M5.Display.setTextColor(telemetryData.headingLock ? GREEN : YELLOW, BLACK);
    M5.Display.printf("LOCK:%s\n", telemetryData.headingLock ? "ON  " : "OFF ");
    M5.Display.setTextColor(CYAN, BLACK);
    M5.Display.printf("HDG :%+4.0f\n", telemetryData.headingDeg);

    drawHeadingCompass(255, 42, 28, telemetryData.headingDeg);

    M5.Display.setCursor(0, 86);
    M5.Display.setTextColor(WHITE, BLACK);
    M5.Display.println("--------------------");
    M5.Display.printf("TGT H:%+4.0f | PID:%+4.0f\n", telemetryData.targetHeading, telemetryData.pidOut);
    M5.Display.printf("PWM L:%+4.0f R:%+4.0f  \n", telemetryData.pwmL, telemetryData.pwmR);
    M5.Display.printf("ODO X:%+5.2f Y:%+5.2f\n", telemetryData.odoX, telemetryData.odoY);

    M5.Display.setCursor(0, 222);
    M5.Display.setTextSize(1);
    M5.Display.setTextColor(LIGHTGREY, BLACK);
    M5.Display.print("BtnA:Next | BtnB:Lock ON/OFF | BtnC:Reset 0");
    M5.Display.setTextSize(2);
  }
  else if (screenMode >= TUNE_YAW_KP && screenMode <= TUNE_YAW_KD) {
    M5.Display.setTextColor(WHITE, BLACK);
    M5.Display.println(" == PID HEADING ==");
    M5.Display.printf(" %c Kp_yaw: %.2f  \n", screenMode == TUNE_YAW_KP ? '>' : ' ', Kp_yaw);
    M5.Display.printf(" %c Ki_yaw: %.2f  \n", screenMode == TUNE_YAW_KI ? '>' : ' ', Ki_yaw);
    M5.Display.printf(" %c Kd_yaw: %.2f  \n", screenMode == TUNE_YAW_KD ? '>' : ' ', Kd_yaw);
    M5.Display.printf(" YawOut  : %+5.1f \n", telemetryData.pidOut);
    M5.Display.println("\nBtnB:+  BtnC:- (AutoSave)");
  }
  else if (screenMode == TUNE_MAX_VEL || screenMode == TUNE_LIN_SCALE) {
    M5.Display.setTextColor(WHITE, BLACK);
    M5.Display.println(" == TUNE KECEPATAN ==");
    M5.Display.printf(" %c MaxVel  : %.2f m/s\n", screenMode == TUNE_MAX_VEL ? '>' : ' ', maxLinearVel);
    M5.Display.printf(" %c LinScale: %.2f    \n", screenMode == TUNE_LIN_SCALE ? '>' : ' ', linearScale);
    M5.Display.println("\nBtnB:+  BtnC:- (AutoSave)");
  }
  else if (screenMode == IMU_VIEW) {
    float rateZ = GYRO_Z_SIGN * (gyroZ - gyroZOffset);
    const char* dirX = (accX < -0.15f) ? "X- F/MAJU" : ((accX > 0.15f) ? "X+ MUNDUR" : "NETRAL   ");
    const char* dirY = (accY >  0.15f) ? "Y+ KANAN " : ((accY < -0.15f) ? "Y- KIRI  " : "NETRAL   ");
    const char* dirZ = (accZ >  0.50f) ? "Z+ ATAS  " : ((accZ < -0.50f) ? "Z- BAWAH " : "MIRING   ");

    M5.Display.setTextColor(CYAN, BLACK);
    M5.Display.println(" == ORIENTASI IMU ==");
    M5.Display.setTextColor(WHITE, BLACK);
    M5.Display.printf("AccX:%+5.2f %s\n", accX, dirX);
    M5.Display.printf("AccY:%+5.2f %s\n", accY, dirY);
    M5.Display.printf("AccZ:%+5.2f %s\n", accZ, dirZ);
    M5.Display.println("----------------------");
    M5.Display.setTextColor(YELLOW, BLACK);
    M5.Display.printf("GyrZ:%+6.1f d/s\n", rateZ);
    M5.Display.setTextColor(GREEN, BLACK);
    M5.Display.printf("HDG :%+6.1f deg (%s)\n", telemetryData.headingDeg,
                      rateZ > 2.0f ? "KIRI+" : (rateZ < -2.0f ? "KANAN" : "DIAM "));

    M5.Display.setCursor(0, 222);
    M5.Display.setTextSize(1);
    M5.Display.setTextColor(LIGHTGREY, BLACK);
    M5.Display.printf("BtnA:Next | BtnB:Serial(%s) | BtnC:Calib", imuSerialDebug ? "ON" : "OFF");
    M5.Display.setTextSize(2);
  }
  else if (screenMode == ODOMETRY_VIEW) {
    M5.Display.setTextColor(WHITE, BLACK);
    M5.Display.println(" == IMU ODOMETRY ==");
    M5.Display.printf("X : %+6.3f m   \n", telemetryData.odoX);
    M5.Display.printf("Y : %+6.3f m   \n", telemetryData.odoY);
    M5.Display.printf("Th: %+6.1f deg \n", telemetryData.odoTheta * RAD_TO_DEG);
    M5.Display.println("\nBtnC: Reset Pose");
  }
  else if (screenMode == WIFI_INFO) {
    M5.Display.setTextColor(WHITE, BLACK);
    M5.Display.println(" == WIFI & OTA INFO ==");
    M5.Display.printf("SSID: %s (Ch:%d)\n", WIFI_SSID, (int)WiFi.channel());
    if (wifiConnected) {
      M5.Display.setTextColor(GREEN, BLACK);
      M5.Display.print("IP  : "); M5.Display.println(WiFi.localIP());
      M5.Display.setTextColor(CYAN, BLACK);
      M5.Display.println("Web : ramabot-map.local");
      M5.Display.printf("OTA : %s.local\n", OTA_HOSTNAME);
    } else {
      M5.Display.setTextColor(RED, BLACK);
      M5.Display.println("STAT: OFFLINE");
    }
  }
}

// ================= CORE 0: UI, WEB SERVER, OTA, ROS SERIAL & ESP-NOW =================
void taskUiAndComm(void *parameter) {
  unsigned long lastUiUpdate = 0;
  unsigned long lastUdpTx = 0;
  unsigned long lastRosSerialTx = 0;

  for (;;) {
    if (wifiConnected) {
      ArduinoOTA.handle();
      server.handleClient();
    }

    if (otaInProgress) {
      vTaskDelay(pdMS_TO_TICKS(2));
      continue;
    }

    M5.update();
    handlePhysicalButtons();

    if (millis() - lastRosSerialTx >= 20) {
      lastRosSerialTx = millis();
      if (imuSerialDebug) {
        float rateZ = GYRO_Z_SIGN * (gyroZ - gyroZOffset);
        Serial.printf("AccX:%.2f,AccY:%.2f,AccZ:%.2f,GyroZ:%.2f,Hdg:%.1f\n",
                      accX, accY, accZ, rateZ, headingDeg);
      } else {
        processRosSerial();
      }
    }

    if (wifiConnected && (millis() - lastUdpTx >= 50)) {
      lastUdpTx = millis();
      sendUdpTelemetry();
    }

    if (millis() - lastUiUpdate >= 100) {
      lastUiUpdate = millis();
      sendTelemetryESPNow();
      renderLcdScreen();
    }

    vTaskDelay(pdMS_TO_TICKS(5));
  }
}