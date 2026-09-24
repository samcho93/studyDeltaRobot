/* playground-worker.js — Pyodide + deltarobot in a Web Worker.
 *
 * The student's script runs synchronously here (a `while True:` cannot freeze the page).
 * The record backend keeps a virtual clock, so time.sleep() and robot.wait() return at once;
 * when the script ends the recorded timeline is posted to the page, which plays it in the
 * embedded simulator. "정지" terminates this worker (the page starts a fresh one).
 *
 * main -> worker : {type:'init', base}  {type:'run', code, hello, runId}
 * worker -> main : status | ready | fatal | stdout | stderr | done {runId, status, timeline}
 */
'use strict';

const PYODIDE_VERSION = '0.29.5';
const PYODIDE_BASE = 'https://cdn.jsdelivr.net/pyodide/v' + PYODIDE_VERSION + '/full/';
const MOUNT = '/home/pyodide/delta';

let pyodide = null;
let runner = null;
const post = (m) => self.postMessage(m);

const RUNNER = `
import json, sys, traceback
from deltarobot.backends import record
from deltarobot import playground

playground.install()
FILENAME = "main.py"

def run(source, hello_json):
    record.reset()
    record.DEFAULT_HELLO.clear()
    record.DEFAULT_HELLO.update(json.loads(hello_json) or {})
    status = "ok"
    try:
        code = compile(source, FILENAME, "exec")
        exec(code, {"__name__": "__main__", "__file__": FILENAME})
    except SystemExit as e:
        status = "ok" if e.code in (None, 0) else "error"
    except BaseException as exc:
        tb = [f for f in traceback.extract_tb(exc.__traceback__) if f.filename == FILENAME]
        lines = ["Traceback (most recent call last):\\n"] + traceback.format_list(tb)
        lines += traceback.format_exception_only(type(exc), exc)
        sys.stderr.write("".join(lines))
        status = "error"
    sys.stdout.flush()
    return json.dumps({"status": status, "timeline": json.loads(record.export_last_json())})
`;

async function init(base) {
  post({ type: 'status', text: 'Pyodide ' + PYODIDE_VERSION + ' 내려받는 중… (처음 한 번 약 10 MB)' });
  importScripts(PYODIDE_BASE + 'pyodide.js');
  pyodide = await self.loadPyodide({ indexURL: PYODIDE_BASE });
  pyodide.setStdout({ batched: (s) => post({ type: 'stdout', text: s }) });
  pyodide.setStderr({ batched: (s) => post({ type: 'stderr', text: s }) });
  pyodide.setStdin({ stdin: () => undefined });

  post({ type: 'status', text: 'deltarobot 라이브러리 마운트 중…' });
  const manifest = await (await fetch(base + 'manifest.json', { cache: 'no-cache' })).json();
  const files = await Promise.all(manifest.files.map(async (rel) => {
    const res = await fetch(base + rel + '?v=' + manifest.version);
    if (!res.ok) throw new Error(rel + ' → HTTP ' + res.status);
    return [rel, await res.text()];
  }));
  for (const [rel, text] of files) {
    const path = MOUNT + '/' + rel;
    pyodide.FS.mkdirTree(path.slice(0, path.lastIndexOf('/')));
    pyodide.FS.writeFile(path, text);
  }
  pyodide.runPython('import sys\nsys.path.insert(0, ' + JSON.stringify(MOUNT) + ')\n');
  pyodide.runPython(RUNNER);
  runner = pyodide.globals.get('run');
  post({
    type: 'ready', pyodide: PYODIDE_VERSION, lib: manifest.version,
    python: pyodide.runPython('import sys; sys.version.split()[0]'),
  });
}

async function run(msg) {
  let res;
  try {
    try { await pyodide.loadPackagesFromImports(msg.code, { messageCallback: () => {} }); } catch (e) { /* surfaces below */ }
    res = JSON.parse(runner(msg.code, JSON.stringify(msg.hello || {})));
  } catch (e) {
    post({ type: 'stderr', text: String(e && e.message ? e.message : e) });
    res = { status: 'error', timeline: null };
  }
  post({ type: 'done', runId: msg.runId, status: res.status, timeline: res.timeline });
}

self.onmessage = (ev) => {
  const m = ev.data || {};
  if (m.type === 'init') init(m.base).catch((e) => post({ type: 'fatal', error: String(e && e.message ? e.message : e) }));
  else if (m.type === 'run') run(m);
};
