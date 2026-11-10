"""Optional VLM summary of what the person is doing ("squatting, rep 4, knees caving in"), every few seconds.

Gemini (GEMINI_API_KEY) first, then OpenAI vision (OPENAI_API_KEY), both from the environment / .env through the same
helpers as the rest of the server (pps/spatial.py), so `--byok` (PC_BYOK=1) means no server key and no summary.
It is opt-in (`{type: "watch", op: "summary", on: true}`), runs only while a watch mode is on and a real camera
(phone or webcam) is the source, never with the synthetic person, the mock brain or PC_OFFLINE=1, and at most once
per INTERVAL seconds in a background thread (the robot loop never waits on it). The frame goes to the provider; the
rule-based reading is sent along so the model only has to add what the rules can't see.
"""
import base64
import json
import os
import ssl
import threading
import time
import urllib.request

import certifi

from ..spatial import gemini_key, openai_key

INTERVAL = 8.0
GEMINI_MODELS = ("gemini-3.8-flash", "gemini-3.6-flash")
OPENAI_MODEL = os.environ.get("PC_VLM_OPENAI_MODEL", "gpt-4.1-mini")
PROMPT = ("You watch a person through a camera so a robot can help them. In at most 12 words, say what they are doing: "
          "the activity, the rep count if exercising, any form problem, and a tool in their hand. No preamble. "
          "A rule-based reading of their pose says: {rules}.")
SSL_CTX = ssl.create_default_context(cafile=certifi.where())


def offline():
    return os.environ.get("PC_OFFLINE") == "1"


class Summarizer:
    def __init__(self):
        self.enabled = False
        self.busy, self.last_t, self.last = False, 0.0, None     # last: (t, text, provider) | error
        self.error = None

    def allowed(self, src, mock=False):
        return self.enabled and not offline() and not mock and src in ("phone", "webcam") and (gemini_key() or openai_key())

    def maybe_run(self, frame_bgr, rules_text, src, mock=False, now=None):
        now = time.time() if now is None else now
        if self.busy or now - self.last_t < INTERVAL or frame_bgr is None or not self.allowed(src, mock):
            return False
        self.busy, self.last_t = True, now
        threading.Thread(target=self._run, args=(frame_bgr.copy(), rules_text), daemon=True, name="watch-vlm").start()
        return True

    def _run(self, frame, rules_text):
        try:
            import cv2
            h, w = frame.shape[:2]
            small = cv2.resize(frame, (480, int(h * 480 / w)))
            jpg = base64.b64encode(cv2.imencode(".jpg", small, [cv2.IMWRITE_JPEG_QUALITY, 70])[1].tobytes()).decode()
            prompt = PROMPT.format(rules=rules_text or "unknown")
            text, who = None, None
            gk = gemini_key()
            if gk:
                text, who = self._gemini(gk, jpg, prompt), "gemini"
            if not text:
                ok = openai_key()
                if ok:
                    text, who = self._openai(ok, jpg, prompt), "openai"
            if text:
                self.last = (time.time(), " ".join(text.split())[:140], who)
                self.error = None
        except Exception as e:          # never break the robot loop; never log the key (errors don't carry it)
            self.error = str(e)[:100]
        finally:
            self.busy = False

    @staticmethod
    def _post(url, body, headers):
        req = urllib.request.Request(url, json.dumps(body).encode(), dict(headers, **{"Content-Type": "application/json",
                                                                                       "User-Agent": "PhysicsClay/1.0"}))
        with urllib.request.urlopen(req, timeout=20, context=SSL_CTX) as r:
            return json.loads(r.read())

    def _gemini(self, key, jpg, prompt):
        body = {"contents": [{"parts": [{"inline_data": {"mime_type": "image/jpeg", "data": jpg}}, {"text": prompt}]}],
                "generationConfig": {"temperature": 0.2, "maxOutputTokens": 60}}
        for m in GEMINI_MODELS:
            try:
                d = self._post(f"https://generativelanguage.googleapis.com/v1beta/models/{m}:generateContent", body, {"x-goog-api-key": key})
                return "".join(p.get("text", "") for p in d["candidates"][0]["content"]["parts"]).strip()
            except Exception:
                continue
        return None

    def _openai(self, key, jpg, prompt):
        body = {"model": OPENAI_MODEL, "max_tokens": 60,
                "messages": [{"role": "user", "content": [{"type": "text", "text": prompt},
                                                          {"type": "image_url", "image_url": {"url": "data:image/jpeg;base64," + jpg}}]}]}
        try:
            d = self._post("https://api.openai.com/v1/chat/completions", body, {"Authorization": "Bearer " + key})
            return d["choices"][0]["message"]["content"].strip()
        except Exception:
            return None
