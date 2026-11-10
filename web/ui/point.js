// Point (UX 2.0): with the tool on, click the table in the 3D stage to drop a marker (A-F), then say "put it there".
// On the Robot-eye view, tapping already drops a marker; this is the same thing in 3D. Server: pps/features/point.py.
"use strict";
(function () {
  const U = window.PCUI; if (!U) return;
  let release = null;
  U.tool({
    id: "point", label: "Point", icon: "point", title: "Point at a spot on the table, then say “put it there”",
    onSelect() {
      if (window.PC && window.PC.setStageView) window.PC.setStageView("3d");
      U.with3D((v) => {
        if (release) return;
        release = v.capture(({ type, hit }) => {
          if (type === "down") return !!hit;              // claim the click only when it lands on the table
          if (type !== "up" || !hit) return false;
          const pos = [hit.x, hit.y, v.TABLE.z].map((n) => Math.round(n * 1000) / 1000);
          if (U.send({ type: "point3d", pos })) U.say("sys", "Marker placed where you pointed. Now say what to put there.");
          else U.toast("Not connected to the robot.");
          return true;
        });
      });
      U.toast("Click a spot on the table.");
    },
    onDeselect() { if (release) { release(); release = null; } },
  });
})();
