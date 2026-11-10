"""Background jobs (one photo import or table scan each): created by the HTTP upload or a websocket message, run in a
thread (one heavy job at a time, so the robot loop keeps its frame rate), read by the feature's state and the
preview endpoints. Only small summaries go into the state frame; meshes and images stay here."""
import threading
import time
import traceback
import uuid
from collections import OrderedDict

from .base import ReconError

MAX_JOBS = 10
_jobs = OrderedDict()
_lock = threading.Lock()
_heavy = threading.Semaphore(1)
STEPS = {
    "object": [("recognize", "Recognizing it"), ("depth", "Estimating depth"), ("outline", "Finding the outline"),
               ("shape", "Fitting a 3D shape"), ("scale", "Setting the real size"), ("physics", "Making it physical")],
    "space": [("recognize", "Finding the objects"), ("depth", "Estimating depth"), ("surface", "Finding the table"),
              ("layout", "Measuring the layout"), ("shapes", "Shaping the objects")],
}


class Job:
    def __init__(self, kind, tag=None, auto=None):
        self.id = uuid.uuid4().hex[:10]
        self.kind, self.tag, self.auto = kind, tag, auto          # auto: {"add": True, ...} (the agent path)
        self.status, self.step, self.progress, self.message = "queued", None, 0.0, "Waiting…"
        self.created, self.t0, self.seconds = time.time(), None, None
        self.steps = [{"id": k, "label": v, "state": "todo"} for k, v in STEPS[kind]]
        self.summary, self.error, self.data = None, None, {}       # data: meshes, images, layout (not JSON)
        self.added = None                                         # what "Add to scene" did: {"lib", "name"}

    def at(self, step, frac=None, message=None):
        """Now on `step` (earlier steps done unless skipped); frac: progress within it."""
        ids = [s["id"] for s in self.steps]
        i = ids.index(step) if step in ids else 0
        for k, s in enumerate(self.steps):
            if s["state"] != "skipped":
                s["state"] = "done" if k < i else "active" if k == i else "todo"
        self.step = step
        self.progress = round((i + (frac or 0.0)) / len(self.steps), 3)
        if message:
            self.message = message
        else:
            self.message = next((s["label"] for s in self.steps if s["id"] == step), self.message) + "…"

    def skip(self, step):
        for s in self.steps:
            if s["id"] == step:
                s["state"] = "skipped"

    def public(self, full=False):
        out = {"id": self.id, "kind": self.kind, "tag": self.tag, "status": self.status, "step": self.step, "progress": self.progress,
               "message": self.message, "error": self.error, "steps": self.steps, "seconds": self.seconds, "added": self.added,
               "created": round(self.created, 1)}
        if self.summary is not None:
            s = self.summary
            out["summary"] = s if full else {k: s.get(k) for k in ("label", "kind", "provider", "shape", "size_cm", "mass_g", "confidence",
                                                                    "scale", "count", "surface", "objects_brief")}
        return out


def new(kind, tag=None, auto=None):
    j = Job(kind, tag, auto)
    with _lock:
        _jobs[j.id] = j
        while len(_jobs) > MAX_JOBS:
            old = next(iter(_jobs))
            if _jobs[old].status in ("running", "queued"):
                break
            _jobs.pop(old)
    return j


def get(i):
    with _lock:
        return _jobs.get(str(i or ""))


def recent(n=4):
    with _lock:
        js = list(_jobs.values())[-n:]
    return [j.public() for j in js]


def busy():
    with _lock:
        return any(j.status in ("queued", "running") for j in _jobs.values())


def start(job, fn):
    """Run fn(job) in a background thread; fn sets job.summary / job.data. Errors become a calm job.error."""
    def go():
        with _heavy:
            job.status, job.t0 = "running", time.time()
            try:
                fn(job)
                for s in job.steps:
                    if s["state"] == "active":
                        s["state"] = "done"
                job.status, job.progress = "ready", 1.0
                job.message = job.message if job.message and not job.message.endswith("…") else "Ready."
            except ReconError as e:
                job.status, job.error, job.message = "failed", str(e), str(e)
            except Exception as e:                       # a bug: say so plainly, keep the server alive
                traceback.print_exc()
                job.status, job.error = "failed", f"Something went wrong while reconstructing ({str(e)[:100]})."
                job.message = job.error
            finally:
                for s in job.steps:
                    if s["state"] == "active" and job.status == "failed":
                        s["state"] = "failed"
                job.seconds = round(time.time() - job.t0, 1)
    job.status = "queued"                                # synchronously: nobody sees a stale "ready" in between
    threading.Thread(target=go, daemon=True, name=f"recon-{job.id}").start()
    return job
