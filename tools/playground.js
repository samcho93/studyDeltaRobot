// playground.js — Python Playground: CodeMirror editor + Pyodide worker + embedded simulator.
import { createEditor } from './playground-editor.js';
import { EXAMPLES, DEFAULT_CODE } from './playground-examples.js';

const $ = (id) => document.getElementById(id);
const LS_CODE = 'studydelta.playground.code.v1';
const LIB_BASE = new URL('../python/', location.href).href;
const lsGet = (k) => { try { return localStorage.getItem(k); } catch (e) { return null; } };
const lsSet = (k, v) => { try { localStorage.setItem(k, v); } catch (e) { /* private mode */ } };

// ------------------------------------------------------------------ output
const outEl = $('out');
function out(text, cls = '') {
  const near = outEl.scrollHeight - outEl.scrollTop - outEl.clientHeight < 40;
  const line = document.createElement('div');
  if (cls) line.className = 'ln-' + cls;
  line.textContent = text;
  outEl.appendChild(line);
  while (outEl.childElementCount > 4000) outEl.firstElementChild.remove();
  if (near) outEl.scrollTop = outEl.scrollHeight;
}
function setStatus(text, cls = '') { const s = $('pyStatus'); s.textContent = text; s.className = 'pg-status' + (cls ? ' ' + cls : ''); }

// ------------------------------------------------------------------ code sources
const b64url = (str) => { const b = new TextEncoder().encode(str); let s = ''; for (const c of b) s += String.fromCharCode(c); return btoa(s).replace(/\+/g, '-').replace(/\//g, '_').replace(/=+$/, ''); };
const unb64url = (s) => { let b = s.replace(/-/g, '+').replace(/_/g, '/'); while (b.length % 4) b += '='; return new TextDecoder().decode(Uint8Array.from(atob(b), (c) => c.charCodeAt(0))); };
let fromLesson = false;
function initialCode() {
  const m = location.hash.match(/(?:^#|&)code=([^&]+)/);
  if (m) {
    try {
      const code = unb64url(decodeURIComponent(m[1]));
      history.replaceState(null, '', location.pathname + location.search);
      fromLesson = true;
      return code;
    } catch (e) { out('링크의 코드를 해석하지 못했습니다: ' + e.message, 'err'); }
  }
  const ex = new URLSearchParams(location.search).get('example');
  const found = EXAMPLES.find((e) => e.id === ex);
  if (found) return found.code;
  const saved = lsGet(LS_CODE);
  return saved && saved.trim() ? saved : DEFAULT_CODE;
}

// ------------------------------------------------------------------ simulator (iframe)
const frame = $('simFrame');
let simReady = false;
let simHello = null;
let pendingTimeline = null;
window.addEventListener('message', (ev) => {
  if (ev.source !== frame.contentWindow) return;
  const m = ev.data || {};
  if (m.source !== 'delta-sim') return;
  if (m.type === 'sim-ready' || m.type === 'sim-design') {
    simReady = true;
    simHello = { design: m.design, scene: m.scene };
    if (pendingTimeline) { frame.contentWindow.postMessage({ type: 'timeline', timeline: pendingTimeline }, '*'); pendingTimeline = null; }
  } else if (m.type === 'sim-analysis') {
    paintSummary(m.eval, m.duration);
  } else if (m.type === 'sim-error') {
    out('시뮬레이터: ' + m.error, 'err');
  }
});
function sendTimeline(tl) {
  if (simReady) frame.contentWindow.postMessage({ type: 'timeline', timeline: tl }, '*');
  else pendingTimeline = tl;
}
function paintSummary(ev, duration) {
  if (!ev) { $('summary').innerHTML = `<span class="hint">총 ${Number(duration || 0).toFixed(2)} s 재생 중</span>`; return; }
  const li = (ok, label, v) => `<li class="${ok ? 'ok' : 'bad'}"><b>${ok ? '✔' : '✖'}</b>${label} <code>${v}</code></li>`;
  const pct = (v) => (v * 100).toFixed(0) + '%';
  $('summary').innerHTML = '<ul class="d-check">' +
    `<li class="ok"><b>⏱</b>총 시간 <code>${Number(duration).toFixed(2)} s</code></li>` +
    li(ev.peak_ratio <= 1, '피크 토크', pct(ev.peak_ratio)) +
    li(ev.rms_ratio <= 1, 'RMS 토크', pct(ev.rms_ratio)) +
    li(ev.speed_ratio <= 1, '모터 속도', pct(ev.speed_ratio)) +
    li(ev.curve_ratio <= 1, '속도-토크 곡선', pct(ev.curve_ratio)) + '</ul>' +
    (ev.problems.length ? `<div class="ln-err">${ev.problems.join(' · ')}</div>` : '');
}

// ------------------------------------------------------------------ worker
let worker = null, ready = false, running = false, runId = 0;
let slowTimer = null, loadT0 = 0;
function startWorker() {
  ready = false;
  loadT0 = performance.now();
  $('btnRun').disabled = false;          // clicking while loading queues the run
  out('파이썬(Pyodide)을 불러오는 중입니다 — 처음에는 10~30초 걸립니다. 실행을 먼저 눌러 두면 준비되는 대로 실행합니다.', 'info');
  clearTimeout(slowTimer);
  slowTimer = setTimeout(() => {
    if (!ready) out('아직 준비 중입니다. 인터넷(cdn.jsdelivr.net) 접속이 느리거나 학교·회사 방화벽에서 막혀 있을 수 있습니다. ' +
      '다른 네트워크에서 열거나, PC에 설치한 파이썬으로 같은 코드를 실행하세요 (pip install -e python → R03).', 'err');
  }, 40000);
  worker = new Worker('playground-worker.js');
  worker.onmessage = (ev) => {
    const m = ev.data;
    if (m.type === 'status') setStatus(m.text);
    else if (m.type === 'ready') {
      ready = true;
      $('pyInfo').textContent = `Pyodide ${m.pyodide} · Python ${m.python} · deltarobot ${m.lib}`;
      setStatus('준비됨 — Ctrl+Enter로 실행', 'ok');
      clearTimeout(slowTimer);
      $('btnRun').disabled = false;
      $('btnRun').innerHTML = '&#9654; 실행';
      out('✔ 파이썬 준비 완료 (' + ((performance.now() - loadT0) / 1000).toFixed(1) + ' s)', 'ok');
      if (pendingRun) { pendingRun = false; run(); }
    } else if (m.type === 'fatal') {
      clearTimeout(slowTimer);
      pendingRun = false;
      $('btnRun').innerHTML = '&#9654; 실행';
      setStatus('Pyodide 로드 실패', 'bad');
      out('Pyodide를 불러오지 못했습니다: ' + m.error + '\n(인터넷 연결이 필요합니다 — jsdelivr CDN)', 'err');
    } else if (m.type === 'stdout') out(m.text);
    else if (m.type === 'stderr') out(m.text, 'err');
    else if (m.type === 'done' && m.runId === runId) finish(m);
  };
  worker.onerror = (e) => { out('worker 오류: ' + (e.message || e), 'err'); };
  worker.postMessage({ type: 'init', base: LIB_BASE });
}
let t0 = 0;
let pendingRun = false;
function run() {
  if (running) return;
  if (!ready || !editor) {
    if (!pendingRun) {
      pendingRun = true;
      $('btnRun').innerHTML = '&#8987; 준비되면 실행';
      out('아직 불러오는 중입니다 — 준비가 끝나면 자동으로 실행합니다.', 'info');
    }
    return;
  }
  const src = editor.getValue();
  lsSet(LS_CODE, src);
  running = true;
  runId++;
  t0 = performance.now();
  $('btnRun').disabled = true;
  $('btnStop').disabled = false;
  setStatus('실행 중…');
  out('▶ 실행 (' + new Date().toLocaleTimeString() + ')', 'info');
  if (frame.contentWindow) frame.contentWindow.postMessage({ type: 'stop' }, '*');
  worker.postMessage({ type: 'run', code: src, hello: simHello || {}, runId });
}
function finish(m) {
  running = false;
  $('btnRun').disabled = false;
  $('btnStop').disabled = true;
  const sec = ((performance.now() - t0) / 1000).toFixed(2);
  if (m.status === 'ok') {
    setStatus('완료 (' + sec + ' s)', 'ok');
    if (m.timeline && m.timeline.frames && m.timeline.frames.length > 1) {
      out(`■ 완료 — 기록된 동작 ${m.timeline.duration.toFixed(2)} s (프레임 ${m.timeline.frames.length}개) → 시뮬레이터에서 재생`, 'ok');
      sendTimeline(m.timeline);
    } else {
      out('■ 완료 (로봇 동작 없음)', 'ok');
    }
  } else {
    setStatus('오류', 'bad');
    if (m.timeline && m.timeline.frames && m.timeline.frames.length > 1) {
      out('오류가 나기 전까지 기록된 동작을 재생합니다.', 'info');
      sendTimeline(m.timeline);
    }
  }
}
function stop() {
  if (!running) return;
  worker.terminate();
  running = false;
  out('■ 정지 — Python을 다시 시작합니다 (몇 초 걸립니다)', 'err');
  $('btnStop').disabled = true;
  startWorker();
}

// ------------------------------------------------------------------ UI
// Start Python and wire the buttons FIRST — the CodeMirror bundle can take a while to download,
// and nothing should wait for it (clicks before the editor/Python are ready are queued).
let editor = null;
const doc0 = initialCode();
const code = () => (editor ? editor.getValue() : doc0);
$('btnRun').addEventListener('click', run);
$('btnStop').addEventListener('click', stop);
$('btnClear').addEventListener('click', () => { outEl.textContent = ''; });
$('btnShare').addEventListener('click', async () => {
  const url = location.origin + location.pathname + '#code=' + b64url(code());
  try { await navigator.clipboard.writeText(url); out('링크를 복사했습니다.', 'info'); } catch (e) { prompt('링크를 복사하세요', url); }
});
$('btnDownload').addEventListener('click', () => {
  const a = document.createElement('a');
  a.href = URL.createObjectURL(new Blob([code()], { type: 'text/x-python' }));
  a.download = 'main.py';
  a.click();
});
document.addEventListener('keydown', (e) => {
  if ((e.ctrlKey || e.metaKey) && e.key === 'Enter') { e.preventDefault(); run(); }
});
startWorker();
if (fromLesson) run();

editor = await createEditor($('editor'), {
  doc: doc0,
  onRun: run,
  onChange: (v) => lsSet(LS_CODE, v),
});
if (editor.loadError) out('코드 편집기(CodeMirror)를 불러오지 못해 기본 입력창을 씁니다.', 'info');
if (pendingRun && ready) { pendingRun = false; run(); }

const sel = $('examples');
sel.innerHTML += EXAMPLES.map((e) => `<option value="${e.id}">${e.title}</option>`).join('');
sel.addEventListener('change', () => {
  const ex = EXAMPLES.find((e) => e.id === sel.value);
  if (ex && (editor.getValue() === ex.code || confirm('지금 코드를 예제로 바꿀까요? (현재 코드는 사라집니다)'))) editor.setValue(ex.code);
  sel.value = '';
});
