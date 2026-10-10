/*
 * Operator notification banner, served at /tune/notify.js.
 *
 * Any page on the Robot Controller's web server can include it with
 *   <script src="/tune/notify.js"></script>
 * and a host session's `mechtune.py notify "..."` appears on top of that page. Open notes stay
 * pinned until the operator answers them (or, for await=shot:<wheel>, until the Mechanism Tuner
 * detects the shot). Self-contained: injects its own styles, needs nothing from the host page.
 *
 * window.TuneNotes.subscribe(fn) gives a page the full note list on every poll.
 */
(function () {
  if (window.TuneNotes) return;
  var POLL_MS = 1000;
  var notes = [];
  var seen = {};
  var subscribers = [];
  var flashUntil = {};
  var audio = null;
  var titleBase = document.title;
  var titleTimer = null;

  var css = '' +
    '#tn-wrap{position:fixed;top:10px;left:50%;transform:translateX(-50%);z-index:9999;' +
    'width:min(720px,calc(100vw - 32px));display:flex;flex-direction:column;gap:8px;' +
    'font:14px/1.45 -apple-system,"Segoe UI",Roboto,sans-serif;pointer-events:none}' +
    '#tn-wrap.docked{position:static;transform:none;width:auto;margin-top:14px;z-index:auto}' +
    '#tn-wrap.docked:empty{display:none}' +
    '.tn{pointer-events:auto;background:#1f2731;color:#e6edf3;border:1px solid #2b3542;' +
    'border-left:5px solid #4da3ff;border-radius:10px;padding:12px 14px;' +
    'box-shadow:0 8px 28px rgba(0,0,0,.45)}' +
    '.tn.action{border-left-color:#d29922;background:#2a2416}' +
    '.tn.warn{border-left-color:#f85149;background:#2a1616}' +
    '.tn.done{border-left-color:#3fb950;background:#123122}' +
    '.tn.pulse{animation:tnpulse 1.2s ease-in-out 3}' +
    '@keyframes tnpulse{50%{box-shadow:0 0 0 4px #d29922aa}}' +
    '.tn-top{display:flex;gap:10px;align-items:baseline}' +
    '.tn-text{flex:1;font-size:16px;font-weight:600;white-space:pre-wrap}' +
    '.tn-meta{color:#8b98a8;font-size:12px;white-space:nowrap}' +
    '.tn-row{display:flex;gap:8px;margin-top:9px;flex-wrap:wrap;align-items:center}' +
    '.tn-row input{flex:1;min-width:140px;font:inherit;font-size:13px;background:#0c1015;' +
    'color:#e6edf3;border:1px solid #2b3542;border-radius:6px;padding:6px 8px}' +
    '.tn-row button{font:inherit;color:#e6edf3;background:#14395f;border:1px solid #1f5c99;' +
    'border-radius:7px;padding:6px 14px;cursor:pointer}' +
    '.tn-row button.tn-skip{background:#1f2731;border-color:#2b3542}' +
    '.tn-wait{color:#d29922;font-size:13px}';

  function el(tag, cls, text) {
    var e = document.createElement(tag);
    if (cls) e.className = cls;
    if (text != null) e.textContent = text;
    return e;
  }

  function beep() {
    try {
      audio = audio || new (window.AudioContext || window.webkitAudioContext)();
      [0, 0.18].forEach(function (at) {
        var o = audio.createOscillator(), g = audio.createGain();
        o.frequency.value = 880;
        g.gain.setValueAtTime(0.0001, audio.currentTime + at);
        g.gain.exponentialRampToValueAtTime(0.25, audio.currentTime + at + 0.02);
        g.gain.exponentialRampToValueAtTime(0.0001, audio.currentTime + at + 0.15);
        o.connect(g); g.connect(audio.destination);
        o.start(audio.currentTime + at); o.stop(audio.currentTime + at + 0.16);
      });
    } catch (e) { /* audio is a nicety */ }
  }

  function flashTitle(on) {
    if (on && !titleTimer) {
      titleBase = document.title.replace(/^\(!\) /, '');
      var t = false;
      titleTimer = setInterval(function () {
        t = !t;
        document.title = (t ? '(!) ' : '') + titleBase;
      }, 800);
    } else if (!on && titleTimer) {
      clearInterval(titleTimer);
      titleTimer = null;
      document.title = titleBase;
    }
  }

  function ago(ms) {
    var s = Math.max(0, Math.round(ms / 1000));
    return s < 60 ? s + ' s ago' : Math.round(s / 60) + ' min ago';
  }

  function ack(id, reply, resolution) {
    var q = '/tune/ack?id=' + id + '&resolution=' + encodeURIComponent(resolution) +
      '&reply=' + encodeURIComponent(reply || '');
    fetch(q, {cache: 'no-store'}).then(poll, poll);
  }

  function render(now) {
    var wrap = document.getElementById('tn-wrap');
    if (!wrap) {
      var style = el('style'); style.textContent = css; document.head.appendChild(style);
      wrap = el('div'); wrap.id = 'tn-wrap';
      // A page can reserve a spot (id="tn-dock") so banners push content down instead of
      // covering its controls; anywhere else they float over the top of the page.
      var dock = document.getElementById('tn-dock');
      if (dock) { wrap.className = 'docked'; dock.appendChild(wrap); } else { document.body.appendChild(wrap); }
    }
    // Keep a half-typed reply across re-renders.
    var drafts = {};
    Array.prototype.forEach.call(wrap.querySelectorAll('input[data-id]'), function (i) {
      drafts[i.getAttribute('data-id')] = {v: i.value, f: document.activeElement === i};
    });
    wrap.innerHTML = '';
    var openCount = 0;
    notes.forEach(function (n) {
      var showDone = n.resolved && flashUntil[n.id] && Date.now() < flashUntil[n.id];
      if (n.resolved && !showDone) return;
      if (!n.resolved) openCount++;
      var box = el('div', 'tn ' + (n.resolved ? 'done' : n.level) +
        (!n.resolved && n.level === 'action' && Date.now() - seen[n.id] < 4000 ? ' pulse' : ''));
      var top = el('div', 'tn-top');
      top.appendChild(el('div', 'tn-text', n.text));
      top.appendChild(el('div', 'tn-meta', (n.from || '') + ' · ' + ago(now - n.postedMs)));
      box.appendChild(top);
      var row = el('div', 'tn-row');
      if (n.resolved) {
        row.appendChild(el('div', 'tn-meta', '✓ ' + (n.resolution || 'done') +
          (n.reply ? ' — "' + n.reply + '"' : '')));
      } else {
        if (n.await && n.await.indexOf('shot:') === 0) {
          row.appendChild(el('div', 'tn-wait', '⏳ waiting for the ' + n.await.slice(5) +
            ' flywheel to see the shot…'));
        }
        var input = el('input');
        input.placeholder = 'Reply (optional) — e.g. volts, what you saw';
        input.setAttribute('data-id', n.id);
        var d = drafts[n.id];
        if (d) input.value = d.v;
        var done = el('button', null, n.await === 'ack' ? 'Done' : 'Done anyway');
        var skip = el('button', 'tn-skip', "Can't / skip");
        done.onclick = function () { ack(n.id, input.value, 'done'); };
        skip.onclick = function () { ack(n.id, input.value, 'skipped'); };
        input.onkeydown = function (e) { if (e.key === 'Enter') done.onclick(); };
        row.appendChild(input); row.appendChild(done); row.appendChild(skip);
        if (d && d.f) setTimeout(function () { input.focus(); }, 0);
      }
      box.appendChild(row);
      wrap.appendChild(box);
    });
    flashTitle(openCount > 0);
  }

  function poll() {
    fetch('/tune/notes', {cache: 'no-store'}).then(function (r) { return r.json(); })
      .then(function (j) {
        var fresh = false;
        j.notes.forEach(function (n) {
          var prev = notes.filter(function (p) { return p.id === n.id; })[0];
          if (!(n.id in seen)) {
            seen[n.id] = Date.now();
            if (!n.resolved) fresh = true;
          }
          if (n.resolved && prev && !prev.resolved) flashUntil[n.id] = Date.now() + 5000;
        });
        notes = j.notes;
        if (fresh) beep();
        render(j.now);
        subscribers.forEach(function (fn) { try { fn(notes, j.now); } catch (e) {} });
      })
      .catch(function () { /* robot offline: keep the last banner */ });
  }

  window.TuneNotes = {
    subscribe: function (fn) { subscribers.push(fn); },
    ack: ack,
    poll: poll
  };
  // Browsers only allow audio after a user gesture; unlock it on the first click.
  document.addEventListener('click', function () {
    try { audio = audio || new (window.AudioContext || window.webkitAudioContext)(); audio.resume(); } catch (e) {}
  }, {once: true});
  function boot() {
    // Everything already on the board when the page opens is old news: no beep for it.
    fetch('/tune/notes', {cache: 'no-store'}).then(function (r) { return r.json(); })
      .then(function (j) { j.notes.forEach(function (n) { seen[n.id] = 0; }); })
      .catch(function () {})
      .then(function () { poll(); setInterval(poll, POLL_MS); });
  }
  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', boot);
  else boot();
})();
