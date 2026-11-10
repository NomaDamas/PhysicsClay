#!/usr/bin/env bash
# PhysicsClay browser HUD: one process (camera + pose + IK + sim render + web server).
# Restarting stops only the server this script started on the same --port (pid file), or a PhysicsClay server
# still listening on that port; other PhysicsClay servers on the machine (a live demo on another port) keep running.
cd "$(dirname "$0")/.."
PORT=8777
prev=""
for a in "$@"; do
  case "$a" in --port=*) PORT="${a#--port=}" ;; esac
  [ "$prev" = "--port" ] && PORT="$a"
  prev="$a"
done
PIDFILE="${TMPDIR:-/tmp}/physicsclay-web-$PORT.pid"

is_ours() { [ -n "$1" ] && ps -p "$1" -o command= 2>/dev/null | grep -q "pps\.app"; }
stop() {
  kill "$1" 2>/dev/null
  for _ in $(seq 20); do kill -0 "$1" 2>/dev/null || return 0; sleep 0.25; done
  kill -9 "$1" 2>/dev/null
}

if [ -f "$PIDFILE" ]; then
  old="$(cat "$PIDFILE" 2>/dev/null)"
  is_ours "$old" && stop "$old"          # the pid may have been reused by something else: only a pps.app process
  rm -f "$PIDFILE"
fi
if command -v lsof >/dev/null 2>&1; then
  for pid in $(lsof -nP -iTCP:"$PORT" -sTCP:LISTEN -t 2>/dev/null); do
    is_ours "$pid" && stop "$pid"        # started by hand on our port
  done
fi

( sleep 3; open "http://127.0.0.1:$PORT" 2>/dev/null ) &
echo $$ > "$PIDFILE"                    # exec keeps this pid: the file names the server itself
exec .venv/bin/python -m pps.app "$@"
