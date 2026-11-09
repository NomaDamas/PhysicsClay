/* PhysicsClay presentation: narrated, auto-advancing 6-slide pitch.
   Params: ?slide=N (1-based, static preview)  ?auto=1 (skip overlay)  ?lang=ko
           ?next=/?demo=full (where to go after the close)  ?go=0 (don't auto-navigate)
           ?rate=1.0  ?mute=1 (no audio, timed advance) */
(function () {
  const q = new URLSearchParams(location.search);
  const LANG = q.get('lang') === 'ko' ? 'ko' : 'en';
  const NEXT = q.get('next') || '/?demo=full';
  const AUTO_GO = q.get('go') !== '0';
  const RATE = parseFloat(q.get('rate') || '1') || 1;
  const MUTE = q.get('mute') === '1' || !('speechSynthesis' in window);
  const GAP = 600;

  const NARR = {
    en: [
      "PhysicsClay. A new generation of interactive prompt for Physical AI.",
      "Some tasks are nearly impossible to give a robot in text alone. Where exactly? Which path? How high? What not to touch? Physical AI needs its own prompt. A spatial one.",
      "Some motions you can't describe. A Taekwondo kick, a sprinter's stride, a fighter's stance. Showing beats describing.",
      "So you show, say, point, and shape the route. Gemini understands. Physics verifies.",
      "Teach it like a friend. It asks back, practices in simulation, and you correct it.",
      "The playground is free in the browser. Robot makers, integrators and labs pay per robot.",
      "What if teaching a robot were as natural as teaching a friend? Let's see it live."
    ],
    ko: [
      "피직스클레이. 피지컬 AI를 위한 새로운 세대의 인터랙티브 프롬프트입니다.",
      "어떤 일은 글만으로 로봇에게 시키기가 거의 불가능합니다. 정확히 어디로, 어떤 길로, 얼마나 높이, 무엇을 피해야 하는지. 피지컬 AI에는 공간으로 된 프롬프트가 필요합니다.",
      "말로 설명할 수 없는 동작도 있습니다. 태권도 발차기, 단거리 선수의 보폭, 격투가의 자세. 보여주는 것이 설명보다 낫습니다.",
      "그래서 보여주고, 말하고, 가리키고, 경로를 그립니다. 제미나이가 이해하고, 물리 엔진이 검증합니다.",
      "친구에게 가르치듯 가르치세요. 로봇이 되묻고, 시뮬레이션에서 연습하고, 당신이 고쳐줍니다.",
      "브라우저 플레이그라운드는 무료이고, 로봇 제조사와 통합 업체, 연구실이 로봇 단위로 비용을 냅니다.",
      "로봇을 가르치는 일이 친구를 가르치는 것만큼 자연스럽다면 어떨까요? 라이브 데모로 보시죠."
    ]
  };
  const lines = NARR[LANG];

  const slides = Array.from(document.querySelectorAll('.slide'));
  const N = slides.length;
  const stage = document.getElementById('stage');
  const bar = document.getElementById('bar');
  const pager = document.getElementById('pager');
  const overlay = document.getElementById('overlay');
  const pausedEl = document.getElementById('paused');
  const countEl = document.getElementById('count');

  // hide video tiles whose file is missing
  document.querySelectorAll('.vtile video').forEach(v => {
    const hide = () => v.closest('.vtile').classList.add('gone');
    v.addEventListener('error', hide);
    const src = v.getAttribute('src');
    if (location.protocol.indexOf('http') === 0) fetch(src, { method: 'HEAD' }).then(r => { if (!r.ok) hide(); }).catch(() => {});
  });

  // fit the 1920x1080 stage to the window
  function fit() {
    const s = Math.min(innerWidth / 1920, innerHeight / 1080);
    stage.style.transform = 'scale(' + s + ')';
  }
  addEventListener('resize', fit); fit();

  // voice selection
  let voice = null;
  function pickVoice() {
    if (MUTE) return;
    const vs = speechSynthesis.getVoices();
    const prefs = LANG === 'ko'
      ? ['Yuna', 'Google 한국의', 'Sora', 'Microsoft SunHi', 'Jian']
      : ['Samantha', 'Google US English', 'Ava', 'Allison', 'Microsoft Aria', 'Microsoft Jenny', 'Alex'];
    const tag = LANG === 'ko' ? 'ko' : 'en-US';
    const norm = v => v.lang.replace('_', '-');
    for (const p of prefs) {
      const v = vs.find(v => v.name.indexOf(p) === 0 && norm(v).indexOf(tag) === 0) ||
                vs.find(v => v.name.indexOf(p) === 0);
      if (v) { voice = v; return; }
    }
    voice = vs.find(v => norm(v).indexOf(tag) === 0) || null;
  }
  if (!MUTE) { pickVoice(); speechSynthesis.onvoiceschanged = pickVoice; }

  // state
  let idx = 0, started = false, paused = false, token = 0;
  let elapsed = 0, estimate = 4000, spokenDone = false, doneAt = 0, silent = MUTE, finished = false;

  function estimateMs(t) {
    const units = LANG === 'ko' ? t.replace(/\s/g, '').length / 6.5 : t.split(/\s+/).length / 2.6;
    return Math.max(2500, units * 1000 / RATE);
  }

  function show(i) {
    idx = Math.max(0, Math.min(N - 1, i));
    slides.forEach((s, k) => s.classList.toggle('on', k === idx));
    pager.textContent = (idx + 1) + ' / ' + N;
    countEl.textContent = '';
    setBar();
  }
  function setBar() {
    const f = started ? Math.min(1, elapsed / (estimate + GAP)) : 0;
    bar.style.width = ((idx + f) / N * 100) + '%';
  }

  function narrate() {
    const my = ++token;
    elapsed = 0; spokenDone = false; doneAt = 0; finished = false;
    const text = lines[idx];
    estimate = estimateMs(text);
    if (silent) return;
    if (LANG !== 'ko' && !window.__noServerTTS) {
      try { if (window.__pa) window.__pa.pause(); } catch (e) {}
      const a = new Audio('/tts?voice=alloy&text=' + encodeURIComponent(text)); window.__pa = a;
      a.onended = () => { if (my === token && !spokenDone) { spokenDone = true; doneAt = elapsed; } };
      a.onerror = () => { window.__noServerTTS = true; if (my === token) narrate(); };
      a.play().catch((e) => { if (my !== token) return; if (String(e).includes('NotAllowed')) silent = true; else { window.__noServerTTS = true; narrate(); } });
      return;
    }
    try { speechSynthesis.cancel(); } catch (e) {}
    const u = new SpeechSynthesisUtterance(text);
    u.lang = LANG === 'ko' ? 'ko-KR' : 'en-US';
    if (voice) u.voice = voice;
    u.rate = RATE; u.pitch = 1;
    u.onend = () => { if (my === token && !spokenDone) { spokenDone = true; doneAt = elapsed; } };
    u.onerror = (e) => {
      if (my !== token) return;
      if (e.error === 'not-allowed') silent = true; // no user gesture: fall back to timed advance
      else if (e.error !== 'interrupted' && e.error !== 'canceled') { spokenDone = true; doneAt = elapsed; }
    };
    setTimeout(() => { if (my === token) speechSynthesis.speak(u); }, 250);
  }

  function go(i) { show(i); if (started) narrate(); }

  function onSlideDone() {
    if (idx < N - 1) { go(idx + 1); return; }
    if (finished) return;
    finished = true; bar.style.width = '100%';
    if (!AUTO_GO) return;
    let n = 3;
    const tick = () => {
      if (!finished) return;
      if (paused) { setTimeout(tick, 300); return; }
      if (n <= 0) { location.href = NEXT; return; }
      countEl.textContent = (LANG === 'ko' ? '라이브 데모 시작 ' : 'Starting live demo in ') + n + '…';
      n--; setTimeout(tick, 1000);
    };
    tick();
  }

  // ticker
  let last = performance.now();
  function loop(now) {
    const dt = now - last; last = now;
    if (started && !paused && !finished) {
      elapsed += dt;
      if (silent && !spokenDone && elapsed >= estimate) { spokenDone = true; doneAt = elapsed; }
      // safety: some engines never fire onend
      if (!spokenDone && elapsed > estimate * 2 + 4000) { spokenDone = true; doneAt = elapsed; }
      setBar();
      if (spokenDone && elapsed - doneAt >= GAP) onSlideDone();
    }
    requestAnimationFrame(loop);
  }
  requestAnimationFrame(loop);

  function start() {
    if (started) return;
    started = true;
    overlay.classList.add('hide');
    if (!MUTE) pickVoice();
    narrate();
  }
  function togglePause() {
    if (!started) { start(); return; }
    paused = !paused;
    pausedEl.classList.toggle('on', paused);
    if (!silent) { try { paused ? speechSynthesis.pause() : speechSynthesis.resume(); } catch (e) {} }
    const pa = window.__pa; // server /tts narration is an <audio>, not speechSynthesis
    if (pa && !silent) { try { if (paused) pa.pause(); else if (!pa.ended && pa.currentTime > 0) pa.play().catch(() => {}); } catch (e) {} }
  }

  overlay.addEventListener('click', start);
  document.getElementById('cta').addEventListener('click', (e) => { e.stopPropagation(); location.href = NEXT; });
  addEventListener('keydown', (e) => {
    if (e.key === 'ArrowRight' || e.key === 'PageDown') { e.preventDefault(); if (!started) start(); finished = false; go(idx + 1); }
    else if (e.key === 'ArrowLeft' || e.key === 'PageUp') { e.preventDefault(); finished = false; go(idx - 1); }
    else if (e.key === ' ' || e.key === 'Spacebar') { e.preventDefault(); togglePause(); }
    else if (e.key === 'Enter' && !started) start();
    else if (e.key === 'Escape') { finished = false; countEl.textContent = ''; }
  });

  // boot
  const sp = parseInt(q.get('slide') || '', 10);
  if (sp >= 1) { document.body.classList.add('static'); show(sp - 1); overlay.classList.add('hide'); } // static preview frame
  else {
    show(0);
    if (q.get('auto') === '1') setTimeout(start, 400);
  }
})();
