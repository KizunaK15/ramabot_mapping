// ================= HALAMAN WEB DASHBOARD, CSV LOGGER & WEB OTA =================
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>RAMABOT MAPPING Telemetry, PID & OTA</title>
<style>
  body{font-family:system-ui,sans-serif;background:#111827;color:#f3f4f6;margin:0;padding:16px}
  h2{margin:0 0 12px;color:#38bdf8;font-size:1.25rem}
  .grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(130px,1fr));gap:10px;margin-bottom:14px}
  .card{background:#1f2937;padding:10px 12px;border-radius:8px;border:1px solid #374151}
  .lbl{font-size:0.75rem;color:#9ca3af}
  .val{font-size:1.2rem;font-weight:700;margin-top:2px}
  .toolbar{display:flex;flex-wrap:wrap;gap:8px;margin-bottom:12px;align-items:center}
  button{background:#2563eb;color:#fff;border:none;padding:8px 14px;border-radius:6px;font-weight:600;cursor:pointer}
  button:hover{background:#1d4ed8}
  .btn-rec{background:#dc2626}.btn-rec.active{background:#16a34a}
  .btn-step{background:#d97706}
  canvas{width:100%;height:250px;background:#0f172a;border:1px solid #334155;border-radius:8px;display:block;margin-bottom:14px}
  .tune{display:grid;grid-template-columns:repeat(auto-fit,minmax(110px,1fr));gap:8px;background:#1f2937;padding:12px;border-radius:8px;margin-bottom:14px}
  .tune label{font-size:0.75rem;color:#cbd5e1;display:block}
  .tune input{width:100%;box-sizing:border-box;background:#111827;color:#fff;border:1px solid #4b5563;padding:6px;border-radius:4px;margin-top:3px}
  .ota-box{background:#1f2937;padding:12px;border-radius:8px;border:1px solid #374151}
  .ota-row{display:flex;flex-wrap:wrap;gap:10px;align-items:center;margin-top:8px}
  progress{width:220px;height:18px}
</style>
</head>
<body>
<h2>RAMABOT MAPPING — Wireless Telemetry, PID Logger & OTA</h2>

<div class="toolbar">
  <button id="recBtn" class="btn-rec" onclick="toggleRec()">Mulai Rekam CSV</button>
  <button onclick="downloadCSV()">Download CSV (<span id="cnt">0</span> baris)</button>
  <button onclick="clearData()">Clear Grafik</button>
  <button class="btn-step" onclick="runStepTest(40,3000)">Uji Step 40 RPM (3 dtk)</button>
  <button style="background:#4b5563" onclick="fetch('/reset')">Reset Odo & Heading</button>
</div>

<div class="grid">
  <div class="card"><div class="lbl">Target RPM (L / R)</div><div class="val" id="tgt">0 / 0</div></div>
  <div class="card"><div class="lbl">Actual RPM L</div><div class="val" style="color:#4ade80" id="rpmL">0.0</div></div>
  <div class="card"><div class="lbl">Actual RPM R</div><div class="val" style="color:#38bdf8" id="rpmR">0.0</div></div>
  <div class="card"><div class="lbl">PWM (L / R)</div><div class="val" style="color:#facc15" id="pwm">0 / 0</div></div>
  <div class="card"><div class="lbl">Heading / YawOut</div><div class="val" id="hdg">0.0° / 0.0</div></div>
  <div class="card"><div class="lbl">Odometry (X, Y)</div><div class="val" id="odo">0.00, 0.00</div></div>
</div>

<canvas id="chart" width="900" height="250"></canvas>

<div class="tune">
  <div><label>Vel Kp</label><input id="kp" type="number" step="0.01"></div>
  <div><label>Vel Ki</label><input id="ki" type="number" step="0.005"></div>
  <div><label>Vel Kd</label><input id="kd" type="number" step="0.01"></div>
  <div><label>Yaw Kp</label><input id="kpy" type="number" step="0.05"></div>
  <div><label>Yaw Kd</label><input id="kdy" type="number" step="0.01"></div>
  <div><label>Max RPM</label><input id="maxrpm" type="number" step="5"></div>
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
let logs = [["time_ms","tgtL","tgtR","rpmL","rpmR","pwmL","pwmR","heading_deg","tgtHeading_deg","yawPid","odoX","odoY","odoTheta_deg"]];
let hist = {tgt:[], rpmL:[], rpmR:[]};
const MAX_PTS = 180;

fetch('/params').then(r=>r.json()).then(p=>{
  document.getElementById('kp').value = p.kp;
  document.getElementById('ki').value = p.ki;
  document.getElementById('kd').value = p.kd;
  document.getElementById('kpy').value = p.kpy;
  document.getElementById('kdy').value = p.kdy;
  document.getElementById('maxrpm').value = p.maxrpm;
});

function toggleRec(){
  recording = !recording;
  const b = document.getElementById('recBtn');
  if(recording){ t0 = performance.now(); b.textContent = "Stop Rekam"; b.classList.add('active'); }
  else { b.textContent = "Mulai Rekam CSV"; b.classList.remove('active'); }
}

function clearData(){
  hist = {tgt:[], rpmL:[], rpmR:[]};
  logs = [logs[0]];
  document.getElementById('cnt').textContent = 0;
}

function runStepTest(rpm, dur){
  if(!recording) toggleRec();
  fetch(`/step?rpm=${rpm}&dur=${dur}`);
}

function savePID(){
  const q = `kp=${kp.value}&ki=${ki.value}&kd=${kd.value}&kpy=${kpy.value}&kdy=${kdy.value}&maxrpm=${maxrpm.value}`;
  fetch('/set?' + q).then(()=>alert('Parameter PID tersimpan di Flash M5Stack!'));
}

function downloadCSV(){
  const csv = logs.map(r=>r.join(",")).join("\n");
  const blob = new Blob([csv], {type:"text/csv"});
  const a = document.createElement("a");
  a.href = URL.createObjectURL(blob);
  a.download = "ramabot_map_telemetry_" + Date.now() + ".csv";
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
  const midY = H/2, scale = (H/2 - 15)/100;
  ctx.strokeStyle = '#475569'; ctx.beginPath(); ctx.moveTo(0,midY); ctx.lineTo(W,midY); ctx.stroke();

  const plot = (arr, color) => {
    if(arr.length < 2) return;
    ctx.strokeStyle = color; ctx.lineWidth = 2; ctx.beginPath();
    arr.forEach((v,i)=>{
      const x = (i/(MAX_PTS-1))*W;
      const y = midY - Math.max(-100,Math.min(100,v))*scale;
      i===0 ? ctx.moveTo(x,y) : ctx.lineTo(x,y);
    });
    ctx.stroke();
  };
  plot(hist.tgt, '#f87171');
  plot(hist.rpmL, '#4ade80');
  plot(hist.rpmR, '#38bdf8');

  ctx.fillStyle='#94a3b8'; ctx.font='12px sans-serif';
  ctx.fillText('Merah: Target RPM | Hijau: RPM Kiri | Biru: RPM Kanan (Skala -100 s/d +100 RPM)', 10, 18);
}

setInterval(async ()=>{
  if(otaBusy) return;
  try{
    const r = await fetch('/data');
    const d = await r.json();
    document.getElementById('tgt').textContent = `${d.tgtL.toFixed(0)} / ${d.tgtR.toFixed(0)}`;
    document.getElementById('rpmL').textContent = d.rpmL.toFixed(1);
    document.getElementById('rpmR').textContent = d.rpmR.toFixed(1);
    document.getElementById('pwm').textContent = `${d.pwmL.toFixed(0)} / ${d.pwmR.toFixed(0)}`;
    document.getElementById('hdg').textContent = `${d.hdg.toFixed(1)}° / ${d.yawOut.toFixed(1)}`;
    document.getElementById('odo').textContent = `${d.x.toFixed(2)}, ${d.y.toFixed(2)}`;

    hist.tgt.push(d.tgtL); hist.rpmL.push(d.rpmL); hist.rpmR.push(d.rpmR);
    if(hist.tgt.length > MAX_PTS){ hist.tgt.shift(); hist.rpmL.shift(); hist.rpmR.shift(); }
    drawChart();

    if(recording){
      const dt = Math.round(performance.now() - t0);
      logs.push([dt, d.tgtL.toFixed(2), d.tgtR.toFixed(2), d.rpmL.toFixed(2), d.rpmR.toFixed(2),
                 d.pwmL.toFixed(1), d.pwmR.toFixed(1), d.hdg.toFixed(2), d.tgtHdg.toFixed(2),
                 d.yawOut.toFixed(2), d.x.toFixed(4), d.y.toFixed(4), d.th.toFixed(2)]);
      document.getElementById('cnt').textContent = logs.length - 1;
    }
  }catch(e){}
}, 50);
</script>
</body>
</html>
)rawliteral";
