"use strict";
/*
 * StampFly 統合管制 UI
 *
 * 契約: docs/ARCHITECTURE.md「pc_server API(UI⇔サーバ契約)」に厳密に従う。
 *  - UI⇔WebSocket の単位は deg / m(rad変換はサーバ側 session 層の責務)
 *  - UI→サーバ:  {"type":"command", ...} / {"type":"setpoint", ...} / {"type":"target", ...}
 *  - サーバ→UI:  {"type":"state","data":{drone, mocap, session}} (20Hz)
 *                {"type":"event", ...}(TLM_EVENT) / {"type":"log","origin","line"}
 */

/* ===================== UI定数(マジックナンバー集約) ===================== */
/* ARCHITECTURE.md「安全クランプ」の UI層: roll/pitch ±5°(既定)、高度 0.1–1.0m */
const UI = {
  WS_RECONNECT_MS: 1000,        // WebSocket再接続バックオフ
  SEND_THROTTLE_MS: 100,        // setpoint/target 送信スロットル(10Hz)
  ROLL_PITCH_LIMIT_DEG: 5.0,    // UI層クランプ(既定±5°)
  ALT_MIN_M: 0.1,
  ALT_MAX_M: 1.0,
  VOLT_WARN_V: 3.5,             // これ未満で警告(黄)
  VOLT_CRIT_V: 3.4,             // これ未満で危険(赤)
  CONSOLE_MAX_LINES: 200,       // コンソールのスクロールバック行数
  ECHO_SUPPRESS_MS: 1500,       // UI操作直後、20Hzのサーバecho上書きを抑制する猶予
  ATT_BAR_RANGE_DEG: 30,        // 姿勢バーのフルスケール(ファームクランプ±30°)
  YAW_BAR_RANGE_DEG: 180,       // ヨーバーのフルスケール
  ALT_BAR_MAX_M: 1.5,           // 高度バーのフルスケール(ファームクランプ上限)
  PLOT_RANGE_M: 2.0,            // XYプロットの表示半幅 [m]
  PLOT_GRID_M: 0.5,             // XYプロットのグリッド間隔 [m]
  TRAIL_MAX_POINTS: 600,        // 軌跡の保持点数(20Hz×30s)
  AF_DEFAULT_CHANNEL: 1,        // プロファイル編集「行追加」の既定チャネル
  AF_DEFAULT_ALT_M: 0.3,        // 同・既定初期高度 [m]
};

/* MAC未設定プロファイルのプルダウン表示サフィックス */
const MAC_UNSET_SUFFIX = " ⚠ MAC未設定";

/* PROTOCOL.md の enum 定義 */
const FLIGHT_STATES = [
  { name: "INIT",        jp: "初期化" },
  { name: "CALIBRATION", jp: "キャリブレーション" },
  { name: "WAIT",        jp: "待機" },
  { name: "TAKEOFF",     jp: "離陸" },
  { name: "HOVER",       jp: "ホバリング" },
  { name: "LANDING",     jp: "着陸" },
  { name: "COMPLETE",    jp: "完了(要RESET)" },
];
const STATE_COMPLETE = 6;
const REASONS = [
  "none", "start_cmd", "stop_cmd", "max_flight_time", "low_voltage",
  "start_rejected_low_voltage", "landed", "over_g", "link_loss", "reset_cmd",
  "start_rejected_not_ready",
];
/* TLM_STATE flags ビット定義 */
const FLAG_FLYING = 0x04; // bit2 = flying

/* ===================== DOM参照 ===================== */
const $ = (id) => document.getElementById(id);
const els = {
  portSelect: $("portSelect"), btnRefreshPorts: $("btnRefreshPorts"), btnConnect: $("btnConnect"),
  airframeSelect: $("airframeSelect"), btnEditAirframes: $("btnEditAirframes"),
  afEditor: $("afEditor"), afKnownMacs: $("afKnownMacs"), afTbody: $("afTbody"),
  btnAfAddRow: $("btnAfAddRow"), afEditorMsg: $("afEditorMsg"),
  btnAfCancel: $("btnAfCancel"), btnAfSave: $("btnAfSave"),
  linkSerial: $("linkSerial"), linkRelay: $("linkRelay"), linkDrone: $("linkDrone"),
  voltage: $("voltage"),
  tabPosture: $("tabPosture"), tabPosition: $("tabPosition"),
  panelPosture: $("panelPosture"), panelPosition: $("panelPosition"),
  rollSlider: $("rollSlider"), pitchSlider: $("pitchSlider"), altSlider: $("altSlider"),
  rollValue: $("rollValue"), pitchValue: $("pitchValue"), altValue: $("altValue"),
  btnCenter: $("btnCenter"), postureNote: $("postureNote"),
  targetX: $("targetX"), targetY: $("targetY"), targetZ: $("targetZ"),
  btnPresetHere: $("btnPresetHere"), btnPresetOrigin: $("btnPresetOrigin"),
  mocapStatus: $("mocapStatus"), mocapStatusText: $("mocapStatusText"), mocapCoords: $("mocapCoords"),
  xyCanvas: $("xyCanvas"), cmdRoll: $("cmdRoll"), cmdPitch: $("cmdPitch"),
  stateBadge: $("stateBadge"), phaseLabel: $("phaseLabel"), btnRearm: $("btnRearm"),
  attRollBar: $("attRollBar"), attPitchBar: $("attPitchBar"), attYawBar: $("attYawBar"),
  attRollNum: $("attRollNum"), attPitchNum: $("attPitchNum"), attYawNum: $("attYawNum"),
  altCurBar: $("altCurBar"), altRefMarker: $("altRefMarker"),
  altCurNum: $("altCurNum"), altRefNum: $("altRefNum"),
  dutyBars: { fr: $("dutyFR"), fl: $("dutyFL"), rr: $("dutyRR"), rl: $("dutyRL") },
  dutyNums: { fr: $("dutyFRNum"), fl: $("dutyFLNum"), rr: $("dutyRRNum"), rl: $("dutyRLNum") },
  latency: $("latency"), relayStats: $("relayStats"),
  logToggle: $("logToggle"), logFile: $("logFile"),
  consoleEl: $("consoleEl"), overlay: $("overlay"), spaceHint: $("spaceHint"),
};

/* ===================== 状態 ===================== */
let ws = null;
let wsOpen = false;
let uiMode = "posture";            // UI表示中のモード(サーバechoで同期)
let modeSentAt = -Infinity;        // set_mode送信時刻(echo抑制用)
let logToggleSentAt = -Infinity;   // set_logging送信時刻(echo抑制用)
let airframeSentAt = -Infinity;    // select_airframe送信時刻(echo抑制用)
let lastSession = null;            // 直近の session オブジェクト
let lastDrone = null;              // 直近の drone オブジェクト
let lastMocap = null;              // 直近の mocap オブジェクト
let airframes = [];                // /api/airframes の配列
let lastEventKey = null;           // TLM_EVENT 2Hz再送のコンソール重複抑制
const trail = [];                  // XYプロット軌跡 [{x,y}]

const now = () => performance.now();
const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));

/* TLM_STATE角度フィールドは「同名でdeg換算」が契約だが、*_deg別名にも耐性を持たせる */
function pick(obj, ...names) {
  if (!obj) return null;
  for (const n of names) {
    if (obj[n] !== undefined && obj[n] !== null) return obj[n];
  }
  return null;
}

/* ===================== WebSocket ===================== */
function wsConnect() {
  const proto = location.protocol === "https:" ? "wss:" : "ws:";
  ws = new WebSocket(`${proto}//${location.host}/ws`);

  ws.onopen = () => {
    wsOpen = true;
    els.overlay.classList.remove("visible");
    appendConsole("ui", "サーバーに接続しました");
  };
  ws.onclose = () => {
    if (wsOpen) appendConsole("ui", "サーバーとの接続が切断されました。再接続します…");
    wsOpen = false;
    els.overlay.classList.add("visible");
    renderConnectivityLost();
    setTimeout(wsConnect, UI.WS_RECONNECT_MS); // 1秒バックオフで自動再接続
  };
  ws.onerror = () => { /* onclose が後続するためここでは何もしない */ };
  ws.onmessage = (ev) => {
    let msg;
    try { msg = JSON.parse(ev.data); } catch { return; }
    if (msg.type === "state") onState(msg.data || {});
    else if (msg.type === "event") onEvent(msg.data !== undefined ? msg.data : msg);
    else if (msg.type === "log") onLog(msg);
  };
}

function wsSend(obj) {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify(obj));
    return true;
  }
  return false;
}

const sendCommand = (action, extra = {}) => wsSend({ type: "command", action, ...extra });

/* ===================== 送信スロットル(10Hz) ===================== */
function makeThrottledSender(sendFn) {
  let lastSent = -Infinity;
  let timer = null;
  return () => {
    const elapsed = now() - lastSent;
    if (elapsed >= UI.SEND_THROTTLE_MS) {
      lastSent = now();
      sendFn();
    } else if (timer === null) {
      // 末尾の値を確実に送るためトレーリング送信を予約
      timer = setTimeout(() => {
        timer = null;
        lastSent = now();
        sendFn();
      }, UI.SEND_THROTTLE_MS - elapsed);
    }
  };
}

function sendSetpointNow() {
  wsSend({
    type: "setpoint",
    roll_deg: clamp(parseFloat(els.rollSlider.value), -UI.ROLL_PITCH_LIMIT_DEG, UI.ROLL_PITCH_LIMIT_DEG),
    pitch_deg: clamp(parseFloat(els.pitchSlider.value), -UI.ROLL_PITCH_LIMIT_DEG, UI.ROLL_PITCH_LIMIT_DEG),
    alt_m: clamp(parseFloat(els.altSlider.value), UI.ALT_MIN_M, UI.ALT_MAX_M),
  });
}
function sendTargetNow() {
  wsSend({
    type: "target",
    x: parseFloat(els.targetX.value) || 0,
    y: parseFloat(els.targetY.value) || 0,
    z: clamp(parseFloat(els.targetZ.value) || UI.ALT_MIN_M, UI.ALT_MIN_M, UI.ALT_MAX_M),
  });
}
const sendSetpointThrottled = makeThrottledSender(sendSetpointNow);
const sendTargetThrottled = makeThrottledSender(sendTargetNow);

/* ===================== REST ===================== */
async function fetchPorts() {
  try {
    const res = await fetch("/api/ports");
    const ports = await res.json(); // [{device, description}]
    const prev = els.portSelect.value;
    els.portSelect.innerHTML = "";
    for (const p of ports) {
      const opt = document.createElement("option");
      opt.value = p.device;
      opt.textContent = p.description ? `${p.device} — ${p.description}` : p.device;
      els.portSelect.appendChild(opt);
    }
    if (prev && ports.some((p) => p.device === prev)) els.portSelect.value = prev;
    if (ports.length === 0) {
      const opt = document.createElement("option");
      opt.value = "";
      opt.textContent = "(ポートなし)";
      els.portSelect.appendChild(opt);
    }
  } catch {
    appendConsole("ui", "ポート一覧の取得に失敗しました (/api/ports)");
  }
}

function macIsSet(mac) {
  return typeof mac === "string" && mac.trim() !== "";
}

/* プルダウンを airframes 配列から再構築する(MAC未設定は ⚠ サフィックス付き)。
   現在の選択(なければサーバ側選択)が新リストに残っていれば維持する。 */
function renderAirframeOptions() {
  const prev = els.airframeSelect.value ||
               (lastSession && lastSession.airframe) || "";
  els.airframeSelect.innerHTML = "";
  for (const a of airframes) {
    const opt = document.createElement("option");
    opt.value = a.name;
    opt.textContent = macIsSet(a.mac) ? a.name : a.name + MAC_UNSET_SUFFIX;
    opt.title = a.notes || "";
    els.airframeSelect.appendChild(opt);
  }
  if (prev && airframes.some((a) => a.name === prev)) {
    els.airframeSelect.value = prev;
  }
}

async function fetchAirframes() {
  try {
    const res = await fetch("/api/airframes");
    const body = await res.json(); // airframes.json の内容: {"airframes":[...]}
    airframes = Array.isArray(body) ? body : (body.airframes || []);
    renderAirframeOptions();
  } catch {
    appendConsole("ui", "機体プロファイルの取得に失敗しました (/api/airframes)");
  }
}

/* ===================== 機体プロファイル編集 ===================== */
/* 契約: PUT /api/airframes {"airframes":[...]} → {"ok","error","airframes"}。
   検証(名前一意・MAC形式・チャネル/バイアス/高度範囲)はサーバが正。 */

let afRows = [];   // 編集中の行データ(airframes のディープコピー)

function afBlankRow() {
  return {
    name: "",
    mac: "",
    wifi_channel: UI.AF_DEFAULT_CHANNEL,
    roll_bias_deg: 0,
    pitch_bias_deg: 0,
    default_alt_m: UI.AF_DEFAULT_ALT_M,
    notes: "",
  };
}

function afMakeInput(row, field, type, opts = {}) {
  const input = document.createElement("input");
  input.type = type;
  if (opts.step !== undefined) input.step = String(opts.step);
  if (opts.placeholder) input.placeholder = opts.placeholder;
  if (opts.className) input.className = opts.className;
  input.value = row[field] === undefined || row[field] === null ? "" : String(row[field]);
  input.addEventListener("input", () => {
    if (type === "number") {
      const v = input.step && input.step.indexOf(".") < 0
        ? parseInt(input.value, 10) : parseFloat(input.value);
      row[field] = Number.isNaN(v) ? null : v;   // null はサーバ検証で弾かれる
    } else {
      row[field] = input.value;
    }
  });
  const td = document.createElement("td");
  td.appendChild(input);
  return td;
}

function renderAfEditorRows() {
  els.afTbody.innerHTML = "";
  for (const row of afRows) {
    const tr = document.createElement("tr");
    tr.appendChild(afMakeInput(row, "name", "text", { className: "af-name" }));
    tr.appendChild(afMakeInput(row, "mac", "text",
      { className: "af-mac mono", placeholder: "(未設定)" }));
    tr.appendChild(afMakeInput(row, "wifi_channel", "number",
      { step: 1, className: "af-ch" }));
    tr.appendChild(afMakeInput(row, "roll_bias_deg", "number",
      { step: 0.001, className: "af-num" }));
    tr.appendChild(afMakeInput(row, "pitch_bias_deg", "number",
      { step: 0.001, className: "af-num" }));
    tr.appendChild(afMakeInput(row, "default_alt_m", "number",
      { step: 0.05, className: "af-num" }));
    tr.appendChild(afMakeInput(row, "notes", "text", { className: "af-notes" }));

    const tdDel = document.createElement("td");
    const btnDel = document.createElement("button");
    btnDel.type = "button";
    btnDel.className = "btn btn-small";
    btnDel.textContent = "行削除";
    btnDel.addEventListener("click", () => {
      afRows.splice(afRows.indexOf(row), 1);
      renderAfEditorRows();
    });
    tdDel.appendChild(btnDel);
    tr.appendChild(tdDel);
    els.afTbody.appendChild(tr);
  }
}

function setAfEditorMsg(text, isError) {
  els.afEditorMsg.textContent = text || "";
  els.afEditorMsg.classList.toggle("err", !!isError);
}

function openAirframeEditor() {
  afRows = airframes.map((a) => ({ ...a }));   // ディープコピー(1段で十分)
  // 既知の候補 MAC(設定済み MAC の一覧)をヒントに表示
  const known = [...new Set(airframes.map((a) => (a.mac || "").trim()).filter(Boolean))];
  els.afKnownMacs.textContent = known.length ? known.join(" / ") : "-";
  setAfEditorMsg("", false);
  renderAfEditorRows();
  els.afEditor.classList.add("visible");
}

function closeAirframeEditor() {
  els.afEditor.classList.remove("visible");
}

async function saveAirframes() {
  els.btnAfSave.disabled = true;
  setAfEditorMsg("保存中…", false);
  try {
    const res = await fetch("/api/airframes", {
      method: "PUT",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ airframes: afRows }),
    });
    const body = await res.json(); // {"ok","error","airframes"}
    if (body.ok) {
      airframes = body.airframes || [];
      renderAirframeOptions();
      appendConsole("ui", `機体プロファイルを保存しました(${airframes.length}件)`);
      closeAirframeEditor();
    } else {
      setAfEditorMsg(body.error || "保存に失敗しました", true);
    }
  } catch {
    setAfEditorMsg("サーバとの通信に失敗しました (/api/airframes)", true);
  } finally {
    els.btnAfSave.disabled = false;
  }
}

/* ===================== サーバ→UI: state(20Hz) ===================== */
function onState(data) {
  lastDrone = data.drone || null;
  lastMocap = data.mocap !== undefined ? data.mocap : null;
  lastSession = data.session || null;

  renderHeader();
  renderSession();
  renderDrone();
  renderMocap();
  if (uiMode === "position") drawPlot();
}

function onEvent(ev) {
  /* TLM_EVENT: {state, prev_state, reason, flags, voltage}。2Hz再送は重複表示しない */
  const state = pick(ev, "state");
  const prev = pick(ev, "prev_state");
  const reason = pick(ev, "reason");
  const voltage = pick(ev, "voltage");
  const key = `${prev}>${state}:${reason}`;
  if (key === lastEventKey) return;
  lastEventKey = key;

  const sName = (i) => (FLIGHT_STATES[i] ? FLIGHT_STATES[i].name : `?${i}`);
  const rName = REASONS[reason] !== undefined ? REASONS[reason] : `?${reason}`;
  const vStr = typeof voltage === "number" ? ` ${voltage.toFixed(2)}V` : "";
  appendConsole("event", `${sName(prev)} → ${sName(state)} (${rName})${vStr}`);
}

function onLog(msg) {
  /* {"type":"log","origin":0|1,"line":...}; origin 0=relay, 1=drone(文字列にも耐性) */
  const o = msg.origin;
  const tag = (o === 1 || o === "drone") ? "drone" : (o === 0 || o === "relay") ? "relay" : "ui";
  appendConsole(tag, String(msg.line !== undefined ? msg.line : ""));
}

/* ===================== 描画: ヘッダ ===================== */
/* on=点灯(緑)。warn=true なら警告色(黄)で点灯 — 「生きているが要注意」状態 */
function setLinkInd(el, on, warn = false) {
  el.classList.toggle("on", !!on && !warn);
  el.classList.toggle("warn", !!on && !!warn);
}

function renderHeader() {
  const s = lastSession;
  const serialOn = !!(s && s.serial_connected);
  setLinkInd(els.linkSerial, serialOn);

  // リレー鮮度: サーバが RLY_STATS(1Hz)の受信時刻から判定した relay_fresh を使う。
  // (counter の内容変化では判定しない — ターゲット未設定で上り転送を拒否中は
  //  全counterが静止し得るが、リレー自体は生きている)
  // リレー生存かつ ESP-NOW ターゲット未設定(relay_target_ok=false)は警告色で区別。
  const relayFresh = serialOn && !!(s && s.relay_fresh);
  const targetOk = !!(s && s.relay_target_ok);
  setLinkInd(els.linkRelay, relayFresh, !targetOk);
  els.linkRelay.title = (relayFresh && !targetOk)
    ? "リレー応答あり / ESP-NOWターゲット未設定(機体宛コマンドは転送されません)"
    : "";

  setLinkInd(els.linkDrone, !!(lastDrone && lastDrone.fresh));

  // 接続ボタンのトグル表示
  els.btnConnect.textContent = serialOn ? "切断" : "接続";
  els.btnConnect.classList.toggle("btn-danger", serialOn);
  els.btnConnect.classList.toggle("btn-primary", !serialOn);

  // 電圧(<3.5V 警告, <3.4V 危険)
  const v = lastDrone ? pick(lastDrone, "voltage") : null;
  if (typeof v === "number") {
    els.voltage.textContent = `${v.toFixed(2)} V`;
    els.voltage.className = "voltage " +
      (v < UI.VOLT_CRIT_V ? "v-crit" : v < UI.VOLT_WARN_V ? "v-warn" : "v-ok");
  } else {
    els.voltage.textContent = "--.- V";
    els.voltage.className = "voltage v-na";
  }
}

function renderConnectivityLost() {
  /* WS切断時はリンク表示を即時オフにし、操作系を安全側(無効)へ倒す */
  setLinkInd(els.linkSerial, false);
  setLinkInd(els.linkRelay, false);
  setLinkInd(els.linkDrone, false);
  for (const el of [els.rollSlider, els.pitchSlider, els.altSlider, els.btnCenter]) {
    el.disabled = true;
  }
  els.postureNote.textContent = "スライダはシリアル接続後に操作できます";
  els.postureNote.classList.remove("hidden");
}

/* ===================== 描画: セッション/モニタ ===================== */
function isFlying() {
  if (lastSession && lastSession.phase === "flying") return true;
  if (lastDrone && typeof lastDrone.flags === "number") return !!(lastDrone.flags & FLAG_FLYING);
  return false;
}

function renderSession() {
  const s = lastSession;
  if (!s) return;

  // サーバ側モードへタブを同期(自分の set_mode 直後は抑制)
  if (s.mode && s.mode !== uiMode && now() - modeSentAt > UI.ECHO_SUPPRESS_MS) {
    applyMode(s.mode, false);
  }

  els.phaseLabel.textContent = `phase: ${s.phase || "-"} / mode: ${s.mode || "-"} / 機体: ${s.airframe || "-"}`;

  // 機体プロファイルのサーバ側選択をドロップダウンへ反映(ユーザー操作直後は抑制)
  if (s.airframe && els.airframeSelect.value !== s.airframe &&
      now() - airframeSentAt > UI.ECHO_SUPPRESS_MS &&
      airframes.some((a) => a.name === s.airframe)) {
    els.airframeSelect.value = s.airframe;
  }

  // レイテンシ
  els.latency.textContent = typeof s.latency_ms === "number" ? `${s.latency_ms.toFixed(1)} ms` : "-- ms";

  // リレー統計(コンパクト表示)
  if (s.relay_stats) {
    const r = s.relay_stats;
    const e = (r.crc_errors || 0) + (r.cobs_errors || 0) + (r.espnow_send_fail || 0) + (r.overflow_drops || 0);
    els.relayStats.textContent = `relay ↑${r.up_frames ?? "-"} ↓${r.down_frames ?? "-"} err:${e}`;
    els.relayStats.classList.toggle("err", e > 0);
  } else {
    els.relayStats.textContent = "";
  }

  // ログ保存トグル+ファイル名(サーバが正。ユーザー操作直後のみecho上書きを抑制)
  if (now() - logToggleSentAt > UI.ECHO_SUPPRESS_MS) {
    els.logToggle.checked = !!s.logging;
  }
  els.logFile.textContent = s.log_file || "-";

  // Position: 指令roll/pitch表示(サーバが計算した適用中setpoint)
  if (s.setpoint) {
    els.cmdRoll.textContent = fmtDeg(s.setpoint.roll_deg);
    els.cmdPitch.textContent = fmtDeg(s.setpoint.pitch_deg);
  }

  // Posture スライダの有効/無効。
  // 高度: 接続中なら地上でも操作可。離陸前の alt_ref 更新は契約どおり有効
  //  (PROTOCOL.md CMD_SETPOINT flags bit0、ファームは WAIT 中も alt_ref を受理)で、
  //  離陸目標高度を START 前に選べるようにする。
  // roll/pitch(+中央戻し): 安全のため従来どおり飛行中のみ操作可。
  const altEnable = wsOpen && uiMode === "posture" && !!s.serial_connected;
  const rpEnable = altEnable && isFlying();
  els.altSlider.disabled = !altEnable;
  for (const el of [els.rollSlider, els.pitchSlider, els.btnCenter]) {
    el.disabled = !rpEnable;
  }
  els.postureNote.textContent = altEnable
    ? "Roll/Pitch は飛行中のみ操作できます(高度は離陸前から変更可)"
    : "スライダはシリアル接続後に操作できます";
  els.postureNote.classList.toggle("hidden", rpEnable);
}

function renderDrone() {
  const d = lastDrone;
  const state = d ? pick(d, "state") : null;

  // 飛行状態バッジ
  if (typeof state === "number" && FLIGHT_STATES[state]) {
    els.stateBadge.textContent = `${FLIGHT_STATES[state].name} — ${FLIGHT_STATES[state].jp}`;
    els.stateBadge.dataset.state = String(state);
  } else {
    els.stateBadge.textContent = "---";
    els.stateBadge.dataset.state = "-1";
  }

  // Re-arm(RESET)は COMPLETE のときのみ表示
  els.btnRearm.classList.toggle("hidden", state !== STATE_COMPLETE);

  // 姿勢(契約: TLM_STATE全フィールド・角度はdeg換算)
  const roll = pick(d, "roll", "roll_deg");
  const pitch = pick(d, "pitch", "pitch_deg");
  const yaw = pick(d, "yaw", "yaw_deg");
  setBipolarBar(els.attRollBar, roll, UI.ATT_BAR_RANGE_DEG);
  setBipolarBar(els.attPitchBar, pitch, UI.ATT_BAR_RANGE_DEG);
  setBipolarBar(els.attYawBar, yaw, UI.YAW_BAR_RANGE_DEG);
  els.attRollNum.textContent = fmtNum(roll, 1);
  els.attPitchNum.textContent = fmtNum(pitch, 1);
  els.attYawNum.textContent = fmtNum(yaw, 1);

  // 高度: 現在(カルマン推定) vs 目標
  const altEst = pick(d, "altitude_est");
  const altRef = pick(d, "alt_ref");
  els.altCurBar.style.width = `${pctOf(altEst, UI.ALT_BAR_MAX_M)}%`;
  els.altRefMarker.style.left = `${pctOf(altRef, UI.ALT_BAR_MAX_M)}%`;
  els.altRefMarker.style.display = typeof altRef === "number" ? "" : "none";
  els.altCurNum.textContent = fmtNum(altEst, 2);
  els.altRefNum.textContent = fmtNum(altRef, 2);

  // モータデューティ(0–1)
  for (const k of ["fr", "fl", "rr", "rl"]) {
    const duty = pick(d, `duty_${k}`);
    els.dutyBars[k].style.width = `${pctOf(duty, 1)}%`;
    els.dutyNums[k].textContent = typeof duty === "number" ? `${Math.round(duty * 100)}%` : "--%";
  }
}

function renderMocap() {
  const m = lastMocap;
  if (m) {
    setLinkInd(els.mocapStatus, !!m.fresh);
    els.mocapStatusText.textContent = m.fresh ? "受信中" : "途絶";
    const conf = typeof m.confidence === "number" ? ` conf: ${m.confidence.toFixed(2)}` : "";
    els.mocapCoords.textContent =
      `x: ${fmtNum(m.x, 2)} y: ${fmtNum(m.y, 2)} z: ${fmtNum(m.z, 2)} yaw: ${fmtNum(m.yaw_deg, 1)}°${conf}`;
    if (typeof m.x === "number" && typeof m.y === "number") {
      trail.push({ x: m.x, y: m.y });
      if (trail.length > UI.TRAIL_MAX_POINTS) trail.shift();
    }
  } else {
    setLinkInd(els.mocapStatus, false);
    els.mocapStatusText.textContent = "未受信";
    els.mocapCoords.textContent = "x: --.-- y: --.-- z: --.--";
  }
}

/* ===================== バー描画ヘルパ ===================== */
function pctOf(v, max) {
  return typeof v === "number" ? clamp(v / max, 0, 1) * 100 : 0;
}

/* 双極バー: 中央0で左右に伸びる */
function setBipolarBar(el, value, range) {
  if (typeof value !== "number") {
    el.style.left = "50%";
    el.style.width = "0%";
    return;
  }
  const half = clamp(value / range, -1, 1) * 50; // -50..+50 [%]
  if (half >= 0) {
    el.style.left = "50%";
    el.style.width = `${half}%`;
  } else {
    el.style.left = `${50 + half}%`;
    el.style.width = `${-half}%`;
  }
}

function fmtNum(v, digits) {
  return typeof v === "number" ? v.toFixed(digits) : "--." + "-".repeat(Math.max(digits, 1));
}
function fmtDeg(v) {
  return typeof v === "number" ? `${v >= 0 ? "+" : ""}${v.toFixed(1)}°` : "--.-°";
}

/* ===================== XYプロット ===================== */
const plotCtx = els.xyCanvas.getContext("2d");
(function setupCanvasDpr() {
  const dpr = window.devicePixelRatio || 1;
  const w = els.xyCanvas.width, h = els.xyCanvas.height;
  els.xyCanvas.style.width = `${w}px`;
  els.xyCanvas.style.height = `${h}px`;
  els.xyCanvas.width = w * dpr;
  els.xyCanvas.height = h * dpr;
  plotCtx.scale(dpr, dpr);
})();

function plotToPx(x, y) {
  /* ワールド座標[m] → キャンバス座標。+X右 / +Y上 */
  const w = parseFloat(els.xyCanvas.style.width);
  const h = parseFloat(els.xyCanvas.style.height);
  const sx = w / (2 * UI.PLOT_RANGE_M);
  const sy = h / (2 * UI.PLOT_RANGE_M);
  return [w / 2 + x * sx, h / 2 - y * sy];
}

function drawPlot() {
  const w = parseFloat(els.xyCanvas.style.width);
  const h = parseFloat(els.xyCanvas.style.height);
  const css = getComputedStyle(document.documentElement);
  const cGrid = css.getPropertyValue("--plot-grid").trim() || "#2a3140";
  const cAxis = css.getPropertyValue("--plot-axis").trim() || "#3d4860";
  const cTrail = css.getPropertyValue("--plot-trail").trim() || "#3b82f6";
  const cCur = css.getPropertyValue("--plot-current").trim() || "#60a5fa";
  const cTarget = css.getPropertyValue("--plot-target").trim() || "#f59e0b";
  const cStale = css.getPropertyValue("--plot-stale").trim() || "#6b7280";

  plotCtx.clearRect(0, 0, w, h);

  // グリッド
  plotCtx.lineWidth = 1;
  plotCtx.strokeStyle = cGrid;
  for (let g = -UI.PLOT_RANGE_M; g <= UI.PLOT_RANGE_M + 1e-9; g += UI.PLOT_GRID_M) {
    const [gx] = plotToPx(g, 0);
    const [, gy] = plotToPx(0, g);
    plotCtx.beginPath(); plotCtx.moveTo(gx, 0); plotCtx.lineTo(gx, h); plotCtx.stroke();
    plotCtx.beginPath(); plotCtx.moveTo(0, gy); plotCtx.lineTo(w, gy); plotCtx.stroke();
  }
  // 原点軸
  plotCtx.strokeStyle = cAxis;
  const [ox, oy] = plotToPx(0, 0);
  plotCtx.beginPath(); plotCtx.moveTo(ox, 0); plotCtx.lineTo(ox, h); plotCtx.stroke();
  plotCtx.beginPath(); plotCtx.moveTo(0, oy); plotCtx.lineTo(w, oy); plotCtx.stroke();

  // 軌跡
  if (trail.length > 1) {
    plotCtx.strokeStyle = cTrail;
    plotCtx.globalAlpha = 0.45;
    plotCtx.lineWidth = 1.5;
    plotCtx.beginPath();
    trail.forEach((p, i) => {
      const [px, py] = plotToPx(p.x, p.y);
      if (i === 0) plotCtx.moveTo(px, py); else plotCtx.lineTo(px, py);
    });
    plotCtx.stroke();
    plotCtx.globalAlpha = 1;
  }

  // 目標位置(サーバ側で保持している target を正とする)
  const t = lastSession && lastSession.target;
  if (t && typeof t.x === "number" && typeof t.y === "number") {
    const [tx, ty] = plotToPx(t.x, t.y);
    plotCtx.strokeStyle = cTarget;
    plotCtx.lineWidth = 1.5;
    const r = 7;
    plotCtx.beginPath(); plotCtx.moveTo(tx - r, ty); plotCtx.lineTo(tx + r, ty); plotCtx.stroke();
    plotCtx.beginPath(); plotCtx.moveTo(tx, ty - r); plotCtx.lineTo(tx, ty + r); plotCtx.stroke();
    plotCtx.beginPath(); plotCtx.arc(tx, ty, r - 2.5, 0, Math.PI * 2); plotCtx.stroke();
  }

  // 現在位置
  const m = lastMocap;
  if (m && typeof m.x === "number" && typeof m.y === "number") {
    const [cx, cy] = plotToPx(m.x, m.y);
    plotCtx.fillStyle = m.fresh ? cCur : cStale;
    plotCtx.beginPath(); plotCtx.arc(cx, cy, 5, 0, Math.PI * 2); plotCtx.fill();
  }
}

/* ===================== コンソール ===================== */
const CONSOLE_TAGS = {
  ui: "UI", relay: "RELAY", drone: "DRONE", event: "EVENT",
};

function appendConsole(tag, text) {
  const c = els.consoleEl;
  const nearBottom = c.scrollHeight - c.scrollTop - c.clientHeight < 30;

  const line = document.createElement("div");
  line.className = `line line-${tag}`;
  const t = new Date();
  const hh = String(t.getHours()).padStart(2, "0");
  const mm = String(t.getMinutes()).padStart(2, "0");
  const ss = String(t.getSeconds()).padStart(2, "0");

  const timeEl = document.createElement("span");
  timeEl.className = "time";
  timeEl.textContent = `${hh}:${mm}:${ss}`;
  const tagEl = document.createElement("span");
  tagEl.className = `tag tag-${tag}`;
  tagEl.textContent = CONSOLE_TAGS[tag] || tag;
  const msgEl = document.createElement("span");
  msgEl.className = "msg";
  msgEl.textContent = text;

  line.append(timeEl, tagEl, msgEl);
  c.appendChild(line);
  while (c.childElementCount > UI.CONSOLE_MAX_LINES) c.removeChild(c.firstElementChild);
  if (nearBottom) c.scrollTop = c.scrollHeight;
}

/* ===================== モード(タブ)切替 ===================== */
function applyMode(mode, sendToServer) {
  uiMode = mode;
  els.tabPosture.classList.toggle("active", mode === "posture");
  els.tabPosition.classList.toggle("active", mode === "position");
  els.panelPosture.classList.toggle("active", mode === "posture");
  els.panelPosition.classList.toggle("active", mode === "position");
  if (sendToServer) {
    modeSentAt = now();
    sendCommand("set_mode", { mode });
  }
  if (mode === "position") drawPlot();
}

/* ===================== STOP(緊急停止) ===================== */
function doStop() {
  sendCommand("stop");
  appendConsole("ui", "STOP 送信(着陸要求)");
  // 視覚フィードバック: フッタヒントを点滅
  els.spaceHint.classList.remove("flash");
  void els.spaceHint.offsetWidth; // reflowを挟んでアニメーションを再始動
  els.spaceHint.classList.add("flash");
}

/* ===================== イベント配線 ===================== */
function wireEvents() {
  // ヘッダ
  els.btnRefreshPorts.addEventListener("click", fetchPorts);
  els.btnConnect.addEventListener("click", () => {
    if (lastSession && lastSession.serial_connected) {
      sendCommand("disconnect");
    } else {
      const port = els.portSelect.value;
      if (!port) {
        appendConsole("ui", "シリアルポートを選択してください");
        return;
      }
      sendCommand("connect", { port });
    }
  });
  // 機体プロファイル編集モーダル
  els.btnEditAirframes.addEventListener("click", openAirframeEditor);
  els.btnAfAddRow.addEventListener("click", () => {
    afRows.push(afBlankRow());
    renderAfEditorRows();
  });
  els.btnAfCancel.addEventListener("click", closeAirframeEditor);
  els.btnAfSave.addEventListener("click", saveAirframes);

  els.airframeSelect.addEventListener("change", () => {
    const name = els.airframeSelect.value;
    airframeSentAt = now();
    sendCommand("select_airframe", { name });
    // 選択機体の既定高度をスライダ初期値へ反映(飛行中は触らない)
    const af = airframes.find((a) => a.name === name);
    if (af && typeof af.default_alt_m === "number" && !isFlying()) {
      const v = clamp(af.default_alt_m, UI.ALT_MIN_M, UI.ALT_MAX_M);
      els.altSlider.value = String(v);
      els.altValue.textContent = `${v.toFixed(2)} m`;
      els.targetZ.value = v.toFixed(2);
    }
  });

  // タブ
  for (const tab of [els.tabPosture, els.tabPosition]) {
    tab.addEventListener("click", () => {
      if (isFlying()) {
        appendConsole("ui", "飛行中はモードを切り替えできません");
        return;
      }
      if (tab.dataset.mode !== uiMode) applyMode(tab.dataset.mode, true);
    });
  }

  // START / STOP / RESET(両タブ共通: data-action で配線)
  for (const btn of document.querySelectorAll("[data-action=start]")) {
    btn.addEventListener("click", () => {
      const label = uiMode === "posture" ? "Posture(姿勢制御)" : "Position(位置制御)";
      if (window.confirm(`${label} モードで離陸を開始します。よろしいですか?`)) {
        sendCommand("start");
        appendConsole("ui", "START 送信");
      }
    });
  }
  for (const btn of document.querySelectorAll("[data-action=stop]")) {
    btn.addEventListener("click", doStop);
  }
  els.btnRearm.addEventListener("click", () => {
    if (window.confirm("Re-arm(RESET)します。機体が静止し高度0.15m未満であることを確認してください。")) {
      sendCommand("reset");
      appendConsole("ui", "RESET 送信");
    }
  });

  // Posture スライダ(10Hzスロットルで setpoint 送信)
  const onSliderInput = () => {
    els.rollValue.textContent = fmtDeg(parseFloat(els.rollSlider.value));
    els.pitchValue.textContent = fmtDeg(parseFloat(els.pitchSlider.value));
    els.altValue.textContent = `${parseFloat(els.altSlider.value).toFixed(2)} m`;
    sendSetpointThrottled();
  };
  for (const el of [els.rollSlider, els.pitchSlider, els.altSlider]) {
    el.addEventListener("input", onSliderInput);
  }
  els.btnCenter.addEventListener("click", () => {
    els.rollSlider.value = "0";
    els.pitchSlider.value = "0";
    onSliderInput();
  });

  // Position 目標入力+プリセット
  const onTargetChanged = () => sendTargetThrottled();
  for (const el of [els.targetX, els.targetY, els.targetZ]) {
    el.addEventListener("change", onTargetChanged);
  }
  els.btnPresetHere.addEventListener("click", () => {
    // この場で: 現在のMoCap位置XYを目標に(Zは現在の入力値を維持)
    if (!lastMocap || typeof lastMocap.x !== "number") {
      appendConsole("ui", "MoCap位置が未受信のため「この場で」を設定できません");
      return;
    }
    els.targetX.value = lastMocap.x.toFixed(2);
    els.targetY.value = lastMocap.y.toFixed(2);
    onTargetChanged();
  });
  els.btnPresetOrigin.addEventListener("click", () => {
    els.targetX.value = "0.00";
    els.targetY.value = "0.00";
    onTargetChanged();
  });

  // ログ保存トグル
  els.logToggle.addEventListener("change", () => {
    logToggleSentAt = now();
    sendCommand("set_logging", { enabled: els.logToggle.checked });
  });

  // SPACE = どこからでも緊急STOP
  // 例外: プロファイル編集モーダルのテキスト/数値入力中のみ通常入力を許す
  // (機体名やメモに空白を打てるようにする。モーダル外では従来どおり即STOP)
  document.addEventListener("keydown", (ev) => {
    if (ev.code === "Space" && !ev.repeat) {
      if (els.afEditor.classList.contains("visible") &&
          ev.target instanceof HTMLInputElement) {
        return;
      }
      ev.preventDefault(); // ボタンのSpace押下/スクロールを抑止し必ずSTOPにする
      doStop();
    }
  });
}

/* ===================== 起動 ===================== */
function init() {
  wireEvents();
  fetchPorts();
  fetchAirframes();
  applyMode("posture", false);
  els.overlay.classList.add("visible"); // 接続成功までオーバーレイ表示
  wsConnect();
  drawPlot();
}

init();
