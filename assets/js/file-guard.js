/*
 * file-guard.js — warn when a tool page is opened straight from disk (file://).
 * Browsers refuse ES modules, fetch() and WASM on file:// URLs, so the 3D viewer, simulator and
 * Playground cannot start. Classic (non-module) script so it still runs in that situation.
 */
(function () {
  'use strict';
  if (location.protocol !== 'file:') return;

  function show() {
    if (document.getElementById('fileGuard')) return;
    var box = document.createElement('div');
    box.id = 'fileGuard';
    box.setAttribute('role', 'alert');
    box.style.cssText = [
      'position:fixed', 'left:50%', 'top:16px', 'transform:translateX(-50%)', 'z-index:10000',
      'max-width:min(640px,calc(100vw - 32px))', 'padding:16px 18px', 'border-radius:10px',
      'background:#7f1d1d', 'color:#fff', 'font:14px/1.6 system-ui,"Malgun Gothic",sans-serif',
      'box-shadow:0 8px 30px rgba(0,0,0,.4)'
    ].join(';');
    box.innerHTML =
      '<b style="font-size:15px">파일을 직접 열어서(file://) 실행할 수 없습니다</b><br>' +
      '브라우저 보안 정책상 <code>file://</code> 주소에서는 ES 모듈·WASM·fetch가 차단되어 ' +
      '3D 뷰어·시뮬레이터·Playground가 동작하지 않습니다. 저장소 폴더에서 로컬 서버를 켜고 접속하세요.' +
      '<pre style="margin:10px 0 6px;padding:8px 10px;background:rgba(0,0,0,.35);border-radius:6px;' +
      'font:13px/1.5 Consolas,monospace;white-space:pre-wrap">python build.py --serve</pre>' +
      '그다음 <b>http://localhost:8000/</b> 으로 접속합니다. ' +
      '(배포본: <a style="color:#fecaca" href="https://samcho93.github.io/studyDeltaRobot/">samcho93.github.io/studyDeltaRobot</a>)';
    document.body.appendChild(box);
  }

  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', show);
  else show();
})();
