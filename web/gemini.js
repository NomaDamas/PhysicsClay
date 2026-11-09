/* PCGemini - dependency-free browser client for the Gemini Live API (raw WebSocket)
 * and Gemini Robotics-ER pointing (REST). Classic script; defines window.PCGemini.
 * Contract: docs/PROTOCOL.md "Browser-only (web/gemini.js) contract".
 *
 * Live model ids (verified 2026-11 on aistudio.google.com/docs/models):
 *   gemini-3.8-live                                 (default, native audio)
 *   gemini-3.8-live-extended-thinking               (async fn calling required)
 *   gemini-3.1-flash-live-preview                   (legacy preview)
 *   gemini-2.5-flash-native-audio-preview-12-2025   (limited to existing users)
 * ER model ids (aistudio.google.com/docs/robotics-overview):
 *   gemini-robotics-er-2-preview (default), gemini-robotics-er-2-streaming-preview,
 *   gemini-robotics-er-1.6-preview (deprecated 2026-08-31)
 */
(function () {
  "use strict";

  var DEFAULT_LIVE_MODEL = "gemini-3.8-live";
  var DEFAULT_ER_MODEL = "gemini-robotics-er-2-preview";
  var POINT_FALLBACK_MODELS = ["gemini-3.6-flash", "gemini-2.5-flash"];
  var WS_URL = "wss://generativelanguage.googleapis.com/ws/google.ai.generativelanguage.v1beta.GenerativeService.BidiGenerateContent";
  var REST_BASE = "https://generativelanguage.googleapis.com/v1beta/models/";
  var TOOL_TIMEOUT_MS = 8000;
  var SETUP_TIMEOUT_MS = 15000;
  var IN_RATE = 16000;
  var OUT_RATE = 24000;
  var CHUNK_SAMPLES = 1280; // 80 ms @ 16 kHz

  // ---------- helpers ----------
  function stripModel(m) { return String(m || "").replace(/^models\//, "").trim(); }
  function stripDataUrl(b64) {
    b64 = String(b64 || "");
    var i = b64.indexOf("base64,");
    return i >= 0 ? b64.slice(i + 7) : b64;
  }
  function mimeFromDataUrl(b64, dflt) {
    var m = /^data:([^;,]+)[;,]/.exec(String(b64 || ""));
    return m ? m[1] : dflt;
  }
  function bytesToB64(bytes) {
    var s = "", CH = 0x8000;
    for (var i = 0; i < bytes.length; i += CH) {
      s += String.fromCharCode.apply(null, bytes.subarray(i, i + CH));
    }
    return btoa(s);
  }
  function b64ToBytes(b64) {
    var s = atob(b64), out = new Uint8Array(s.length);
    for (var i = 0; i < s.length; i++) out[i] = s.charCodeAt(i);
    return out;
  }
  function safe(fn) {
    return function () {
      if (typeof fn !== "function") return undefined;
      try { return fn.apply(null, arguments); } catch (e) { /* UI handler error: ignore */ }
    };
  }
  function errMsg(e) { return (e && (e.message || e.reason)) || String(e); }

  // AudioWorklet source: downsample to 16 kHz mono PCM16, post ~80 ms chunks + RMS.
  var WORKLET_SRC = [
    "class PCMic extends AudioWorkletProcessor {",
    "  constructor(o){super();this.ratio=sampleRate/" + IN_RATE + ";this.pos=0;this.prev=0;",
    "    this.buf=new Int16Array(" + CHUNK_SAMPLES + ");this.n=0;this.acc=0;this.cnt=0;}",
    "  process(inputs){const inp=inputs[0];if(!inp||!inp[0])return true;",
    "    const ch=inp[0],len=ch.length;let mono=ch;",
    "    if(inp.length>1){mono=new Float32Array(len);for(let c=0;c<inp.length;c++){const d=inp[c];for(let i=0;i<len;i++)mono[i]+=d[i]/inp.length;}}",
    "    for(let i=0;i<len;i++){const x=mono[i];this.acc+=x*x;this.cnt++;}",
    // linear-interpolation resampler; pos is fractional index relative to current block (prev = sample -1)
    "    while(this.pos<len){const i0=Math.floor(this.pos),f=this.pos-i0;",
    "      const a=i0-1<0?this.prev:mono[i0-1],b=mono[i0];let s=a+(b-a)*f;",
    "      s=s<-1?-1:s>1?1:s;this.buf[this.n++]=s<0?s*0x8000:s*0x7fff;",
    "      if(this.n===this.buf.length){const rms=Math.sqrt(this.acc/Math.max(1,this.cnt));",
    "        this.port.postMessage({pcm:this.buf.buffer,rms:rms},[this.buf.buffer]);",
    "        this.buf=new Int16Array(" + CHUNK_SAMPLES + ");this.n=0;this.acc=0;this.cnt=0;}",
    "      this.pos+=this.ratio;}",
    "    this.pos-=len;this.prev=mono[len-1];return true;}",
    "}",
    "registerProcessor('pc-mic',PCMic);"
  ].join("\n");

  // ---------- Live ----------
  function connectLive(opts) {
    opts = opts || {};
    var apiKey = String(opts.apiKey || "").trim();
    var model = stripModel(opts.model) || DEFAULT_LIVE_MODEL;
    var onStatus = safe(opts.onStatus);
    var onTranscript = safe(opts.onTranscript);
    var onAudioLevel = safe(opts.onAudioLevel);
    var onToolCall = opts.onToolCall;

    if (!apiKey && !opts.proxy) {
      onStatus("error: missing API key");
      return Promise.reject(new Error("missing API key"));
    }

    var ws = null, closed = false, ready = false, lastStatus = "";
    var micOn = false, micStream = null, micCtx = null, micNode = null, micSrc = null, micSink = null;
    var outCtx = null, playHead = 0, sources = [], speakingTimer = null;
    var lastImageAt = 0;
    var userText = "", modelText = "";

    function status(s) {
      if (s === lastStatus) return;
      lastStatus = s;
      onStatus(s);
    }
    function idleStatus() { status(micOn ? "listening" : "live"); }

    function send(obj) {
      if (!ws || ws.readyState !== 1) return false;
      try { ws.send(JSON.stringify(obj)); return true; } catch (e) { return false; }
    }

    // ----- playback (24 kHz PCM16) -----
    function ensureOut() {
      if (!outCtx) {
        var AC = window.AudioContext || window.webkitAudioContext;
        outCtx = new AC();
      }
      if (outCtx.state === "suspended") outCtx.resume().catch(function () {});
      return outCtx;
    }
    function playPcm(b64) {
      var ctx = ensureOut();
      var bytes = b64ToBytes(b64);
      var n = bytes.length >> 1;
      if (!n) return;
      var view = new DataView(bytes.buffer, bytes.byteOffset, n * 2);
      var buf = ctx.createBuffer(1, n, OUT_RATE);
      var ch = buf.getChannelData(0);
      for (var i = 0; i < n; i++) ch[i] = view.getInt16(i * 2, true) / 32768;
      var src = ctx.createBufferSource();
      src.buffer = buf;
      src.connect(ctx.destination);
      var now = ctx.currentTime;
      if (playHead < now + 0.02) playHead = now + 0.05; // small jitter buffer after a gap
      src.start(playHead);
      playHead += buf.duration;
      sources.push(src);
      src.onended = function () {
        var k = sources.indexOf(src);
        if (k >= 0) sources.splice(k, 1);
        if (!sources.length) scheduleIdle();
      };
      if (speakingTimer) { clearTimeout(speakingTimer); speakingTimer = null; }
      status("speaking");
    }
    function scheduleIdle() {
      if (speakingTimer) clearTimeout(speakingTimer);
      speakingTimer = setTimeout(function () {
        speakingTimer = null;
        if (!sources.length && !closed) idleStatus();
      }, 150);
    }
    function flushPlayback() {
      var list = sources.slice();
      sources = [];
      for (var i = 0; i < list.length; i++) {
        try { list[i].onended = null; list[i].stop(); } catch (e) {}
      }
      playHead = 0;
      if (!closed) idleStatus();
    }

    // ----- transcripts -----
    function finalizeTurn() {
      if (userText) onTranscript({ role: "user", text: userText, final: true });
      if (modelText) onTranscript({ role: "model", text: modelText, final: true });
      userText = ""; modelText = "";
    }

    // ----- tool calls -----
    function runTool(fc) {
      var p;
      try {
        p = typeof onToolCall === "function"
          ? Promise.resolve(onToolCall({ id: fc.id, name: fc.name, args: fc.args || {} }))
          : Promise.resolve({ ok: false, message: "no tool handler" });
      } catch (e) { p = Promise.reject(e); }
      var timeout = new Promise(function (res) {
        setTimeout(function () { res({ ok: false, message: "timed out" }); }, TOOL_TIMEOUT_MS);
      });
      return Promise.race([p, timeout]).then(function (r) {
        if (r === undefined || r === null) r = { ok: true };
        else if (typeof r !== "object" || Array.isArray(r)) r = { result: r };
        return r;
      }, function (e) {
        return { ok: false, message: errMsg(e) };
      }).then(function (r) {
        return { id: fc.id, name: fc.name, response: r };
      });
    }
    function handleToolCall(tc) {
      var calls = (tc && tc.functionCalls) || [];
      if (!calls.length) return;
      Promise.all(calls.map(runTool)).then(function (responses) {
        if (!closed) send({ toolResponse: { functionResponses: responses } });
      });
    }

    // ----- server messages -----
    function handleMessage(msg, resolveSetup) {
      if (msg.setupComplete) {
        ready = true;
        idleStatus();
        resolveSetup();
        return;
      }
      if (msg.toolCall) handleToolCall(msg.toolCall);
      var sc = msg.serverContent;
      if (sc) {
        if (sc.interrupted) {
          flushPlayback();
          if (modelText) { onTranscript({ role: "model", text: modelText, final: true }); modelText = ""; }
        }
        if (sc.inputTranscription && sc.inputTranscription.text) {
          userText += sc.inputTranscription.text;
          onTranscript({ role: "user", text: userText, final: false });
        }
        var parts = (sc.modelTurn && sc.modelTurn.parts) || [];
        var gotTextPart = false;
        for (var i = 0; i < parts.length; i++) {
          var p = parts[i];
          if (p.inlineData && p.inlineData.data && /^audio\//.test(p.inlineData.mimeType || "audio/pcm")) {
            try { playPcm(p.inlineData.data); } catch (e) {}
          } else if (typeof p.text === "string" && p.text && !p.thought) {
            modelText += p.text; gotTextPart = true;
          }
        }
        if (sc.outputTranscription && sc.outputTranscription.text) {
          modelText += sc.outputTranscription.text; gotTextPart = true;
        }
        if (gotTextPart) {
          // the model has started answering: freeze the user's line
          if (userText) { onTranscript({ role: "user", text: userText, final: true }); userText = ""; }
          onTranscript({ role: "model", text: modelText, final: false });
        }
        if (sc.turnComplete) finalizeTurn();
      }
      if (msg.goAway) {
        var tl = msg.goAway.timeLeft ? " (" + msg.goAway.timeLeft + ")" : "";
        status("error: server closing session soon" + tl + ", reconnect");
      }
    }

    // ----- mic -----
    function startMic() {
      if (micOn) return Promise.resolve();
      if (closed) return Promise.reject(new Error("session closed"));
      if (!navigator.mediaDevices || !navigator.mediaDevices.getUserMedia) {
        status("error: microphone needs a secure context (https or localhost)");
        return Promise.reject(new Error("getUserMedia unavailable"));
      }
      micOn = true;
      ensureOut(); // unlock playback inside the user gesture
      var AC = window.AudioContext || window.webkitAudioContext;
      var url = null;
      return navigator.mediaDevices.getUserMedia({
        audio: { channelCount: 1, echoCancellation: true, noiseSuppression: true, autoGainControl: true }
      }).then(function (stream) {
        micStream = stream;
        micCtx = new AC();
        url = URL.createObjectURL(new Blob([WORKLET_SRC], { type: "application/javascript" }));
        return micCtx.audioWorklet.addModule(url);
      }).then(function () {
        if (url) URL.revokeObjectURL(url);
        if (!micOn) throw new Error("mic stopped");
        micSrc = micCtx.createMediaStreamSource(micStream);
        micNode = new AudioWorkletNode(micCtx, "pc-mic", { numberOfInputs: 1, numberOfOutputs: 1, channelCount: 1, channelCountMode: "explicit", channelInterpretation: "speakers" });
        micNode.port.onmessage = function (ev) {
          var d = ev.data;
          onAudioLevel(Math.min(1, d.rms * 4));
          if (!micOn || !ready) return;
          send({ realtimeInput: { audio: { data: bytesToB64(new Uint8Array(d.pcm)), mimeType: "audio/pcm;rate=" + IN_RATE } } });
        };
        micSink = micCtx.createGain();
        micSink.gain.value = 0;
        micSrc.connect(micNode);
        micNode.connect(micSink);
        micSink.connect(micCtx.destination); // keep the graph pulled; silent
        if (micCtx.state === "suspended") micCtx.resume().catch(function () {});
        if (sources.length) status("speaking"); else idleStatus();
      }).catch(function (e) {
        var wasOn = micOn;
        teardownMic(false);
        if (wasOn) status("error: microphone: " + errMsg(e));
        throw e;
      });
    }
    function teardownMic(sendEnd) {
      var wasOn = micOn;
      micOn = false;
      try { if (micNode) { micNode.port.onmessage = null; micNode.disconnect(); } } catch (e) {}
      try { if (micSrc) micSrc.disconnect(); } catch (e) {}
      try { if (micSink) micSink.disconnect(); } catch (e) {}
      if (micStream) micStream.getTracks().forEach(function (t) { try { t.stop(); } catch (e) {} });
      if (micCtx) micCtx.close().catch(function () {});
      micStream = micCtx = micNode = micSrc = micSink = null;
      onAudioLevel(0);
      if (wasOn && sendEnd && ready) send({ realtimeInput: { audioStreamEnd: true } });
    }
    function stopMic() {
      teardownMic(true);
      if (!closed && !sources.length) idleStatus();
    }

    // ----- session api -----
    function sendImage(b64, mime) {
      if (!ready || closed || !b64) return false;
      var now = Date.now();
      if (now - lastImageAt < 1000) return false;
      lastImageAt = now;
      var m = mime || mimeFromDataUrl(b64, "image/jpeg");
      return send({ realtimeInput: { video: { data: stripDataUrl(b64), mimeType: m } } });
    }
    function sendText(text) {
      text = String(text || "").trim();
      if (!text || !ready || closed) return false;
      // realtimeInput.text is the documented path for 3.x Live models (also accepted by 2.5).
      var ok = send({ realtimeInput: { text: text } });
      if (ok) {
        if (userText) finalizeTurn();
        onTranscript({ role: "user", text: text, final: true });
      }
      return ok;
    }
    function close() {
      if (closed) return;
      closed = true;
      teardownMic(false);
      flushPlayback();
      if (outCtx) { outCtx.close().catch(function () {}); outCtx = null; }
      try { if (ws) ws.close(1000, "client closed"); } catch (e) {}
      status("closed");
    }

    var session = {
      model: model,
      startMic: startMic,
      stopMic: stopMic,
      sendImage: sendImage,
      sendText: sendText,
      close: close,
      get isMicOn() { return micOn; },
      get isOpen() { return ready && !closed; }
    };

    // ----- setup message -----
    var setup = {
      model: "models/" + model,
      generationConfig: { responseModalities: ["AUDIO"] },
      inputAudioTranscription: {},
      outputAudioTranscription: {},
      realtimeInputConfig: {
        automaticActivityDetection: { disabled: false, silenceDurationMs: 600, prefixPaddingMs: 100 }
      }
    };
    if (opts.voiceName) {
      setup.generationConfig.speechConfig = { voiceConfig: { prebuiltVoiceConfig: { voiceName: opts.voiceName } } };
    }
    if (opts.systemInstruction) {
      setup.systemInstruction = typeof opts.systemInstruction === "string"
        ? { parts: [{ text: opts.systemInstruction }] }
        : opts.systemInstruction;
    }
    if (opts.tools && opts.tools.length) setup.tools = opts.tools;

    status("connecting");
    return new Promise(function (resolve, reject) {
      var settled = false;
      function ok() { if (!settled) { settled = true; clearTimeout(t); resolve(session); } }
      function fail(m) {
        if (!settled) { settled = true; clearTimeout(t); reject(new Error(m)); }
      }
      var t = setTimeout(function () {
        if (!ready) { status("error: setup timed out"); fail("setup timed out"); try { ws.close(); } catch (e) {} }
      }, SETUP_TIMEOUT_MS);

      try {
        ws = (opts.proxy || !apiKey) ? new WebSocket((location.protocol === "https:" ? "wss://" : "ws://") + location.host + "/gemini/live") : new WebSocket(WS_URL + "?key=" + encodeURIComponent(apiKey));
      } catch (e) {
        status("error: " + errMsg(e));
        fail(errMsg(e));
        return;
      }
      ws.onopen = function () { send({ setup: setup }); };
      ws.onmessage = function (ev) {
        var d = ev.data;
        var p = typeof d === "string" ? Promise.resolve(d)
          : (d && typeof d.text === "function") ? d.text()
          : Promise.resolve(new TextDecoder().decode(d));
        p.then(function (txt) {
          var msg;
          try { msg = JSON.parse(txt); } catch (e) { return; }
          if (msg.error) { status("error: " + (msg.error.message || JSON.stringify(msg.error))); return; }
          handleMessage(msg, ok);
        });
      };
      ws.onerror = function () { /* details arrive in onclose */ };
      ws.onclose = function (ev) {
        var wasClosed = closed;
        closed = true;
        ready = false;
        teardownMic(false);
        flushPlayback();
        var reason = (ev && ev.reason) || "";
        var code = ev ? ev.code : 0;
        if (!wasClosed) {
          if (code === 1000 && !reason) status("closed");
          else status("error: " + (reason || "connection closed") + " (code " + code + ")");
        }
        fail(reason ? reason + " (code " + code + ")" : "connection closed (code " + code + ")");
      };
    });
  }

  // ---------- Robotics-ER pointing ----------
  function buildPointPrompt(query) {
    var q = String(query || "").trim();
    var target = q ? "Point to: " + q + ". If there are several matching items, point to each (no more than 10)."
                   : "Point to no more than 10 items in the image.";
    return target +
      " The label returned should be an identifying name for the object detected." +
      ' The answer should follow the json format: [{"point": <point>, "label": <label1>}, ...].' +
      " The points are in [y, x] format normalized to 0-1000." +
      " If nothing matches, return []. Output only the JSON.";
  }
  function parsePoints(text) {
    var s = String(text || "").trim();
    s = s.replace(/```(?:json)?/gi, "").trim();
    var a = s.indexOf("["), b = s.lastIndexOf("]");
    var o1 = s.indexOf("{"), o2 = s.lastIndexOf("}");
    var arr;
    try {
      if (a >= 0 && b > a && (o1 < 0 || a < o1)) arr = JSON.parse(s.slice(a, b + 1));
      else if (o1 >= 0 && o2 > o1) arr = JSON.parse(s.slice(o1, o2 + 1));
      else arr = JSON.parse(s);
    } catch (e) {
      throw new Error("could not parse points: " + s.slice(0, 120));
    }
    if (!Array.isArray(arr)) arr = arr && (arr.points || [arr]) || [];
    var out = [];
    for (var i = 0; i < arr.length; i++) {
      var it = arr[i] || {};
      var pt = it.point || it.points || it.yx;
      if (Array.isArray(pt) && Array.isArray(pt[0])) pt = pt[0];
      if (!Array.isArray(pt) || pt.length < 2) continue;
      var y = Number(pt[0]), x = Number(pt[1]);
      if (!isFinite(x) || !isFinite(y)) continue;
      out.push({
        label: String(it.label || it.name || ""),
        u: Math.max(0, Math.min(1, x / 1000)),
        v: Math.max(0, Math.min(1, y / 1000))
      });
    }
    return out;
  }
  function generate(apiKey, model, body) {
    var url = apiKey ? REST_BASE + encodeURIComponent(model) + ":generateContent?key=" + encodeURIComponent(apiKey)
                     : "/gemini/rest/" + encodeURIComponent(model) + ":generateContent";
    return fetch(url, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body)
    }).then(function (r) {
      return r.text().then(function (t) {
        var j = null;
        try { j = JSON.parse(t); } catch (e) {}
        if (!r.ok) {
          var e = new Error((j && j.error && j.error.message) || ("HTTP " + r.status));
          e.status = r.status;
          throw e;
        }
        var parts = (j && j.candidates && j.candidates[0] && j.candidates[0].content && j.candidates[0].content.parts) || [];
        var txt = parts.filter(function (p) { return p.text && !p.thought; }).map(function (p) { return p.text; }).join("");
        if (!txt) throw new Error("empty response" + (j && j.promptFeedback ? ": " + JSON.stringify(j.promptFeedback) : ""));
        return txt;
      });
    });
  }
  function point(opts) {
    opts = opts || {};
    var apiKey = String(opts.apiKey || "").trim();
    if (!apiKey && !opts.proxy) return Promise.reject(new Error("missing API key"));
    if (!opts.imageBase64) return Promise.reject(new Error("missing image"));
    var model = stripModel(opts.model) || DEFAULT_ER_MODEL;
    var mime = opts.mime || mimeFromDataUrl(opts.imageBase64, "image/jpeg");
    var contents = [{
      role: "user",
      parts: [
        { inlineData: { mimeType: mime, data: stripDataUrl(opts.imageBase64) } },
        { text: buildPointPrompt(opts.query) }
      ]
    }];
    function thinkingFor(m) {
      // 2.x / ER 1.x use thinkingBudget; 3.x / ER 2 use thinkingLevel.
      return /(^|-)(1\.\d|2\.\d)(-|$)/.test(m) ? { thinkingBudget: 0 } : { thinkingLevel: "low" };
    }
    var attempts = [{ model: model, thinking: true }, { model: model, thinking: false }];
    POINT_FALLBACK_MODELS.forEach(function (m) { if (m !== model) attempts.push({ model: m, thinking: true }); });

    var firstErr = null;
    function tryAt(i) {
      if (i >= attempts.length) return Promise.reject(firstErr || new Error("pointing failed"));
      var a = attempts[i];
      var gc = { temperature: 0.5 };
      if (a.thinking) gc.thinkingConfig = thinkingFor(a.model);
      return generate(apiKey, a.model, { contents: contents, generationConfig: gc })
        .then(parsePoints)
        .catch(function (e) {
          if (!firstErr) firstErr = e;
          if (e && (e.status === 401 || e.status === 403) && /key/i.test(e.message)) throw e; // bad key: no point retrying
          // a thinking-config rejection retries the same model without it; anything else moves on
          var next = i + 1;
          if (a.thinking && !(e && e.status === 400)) {
            while (next < attempts.length && attempts[next].model === a.model) next++;
          }
          return tryAt(next);
        });
    }
    return tryAt(0);
  }

  window.PCGemini = {
    connectLive: connectLive,
    point: point,
    parsePoints: parsePoints,
    DEFAULT_LIVE_MODEL: DEFAULT_LIVE_MODEL,
    DEFAULT_ER_MODEL: DEFAULT_ER_MODEL
  };
})();
