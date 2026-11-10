// Unit tests for web/qr.js (the dependency-free QR encoder behind the laptop's "scan to open on your phone" code).
// The structural checks read the symbol back the way a scanner would (ISO/IEC 18004); the golden hash pins a matrix
// that was verified to decode with OpenCV's QRCodeDetector. Run: node --test tests/qr.test.mjs (or tests/*.mjs)
import test from "node:test";
import assert from "node:assert/strict";
import { createRequire } from "node:module";
import { createHash } from "node:crypto";
import { readFileSync } from "node:fs";
import vm from "node:vm";

const require = createRequire(import.meta.url);
const QR = require("../web/qr.js");

const LEVELS = ["L", "M", "Q", "H"];
const SAMPLES = ["https://192.168.0.6:8781/phone", "https://10.0.0.123:8443/phone?x=1", "x", "로봇 카메라",
  "https://192.168.100.200:8781/phone?session=" + "0123456789abcdef".repeat(6)];
// ISO/IEC 18004 table 7: byte-mode capacity (bytes) of versions 1-10 per level.
const CAP = {
  L: [17, 32, 53, 78, 106, 134, 154, 192, 230, 271],
  M: [14, 26, 42, 62, 84, 106, 122, 152, 180, 213],
  Q: [11, 20, 32, 46, 60, 74, 86, 108, 130, 151],
  H: [7, 14, 24, 34, 44, 58, 64, 84, 98, 119],
};
// Annex E: alignment-pattern centre coordinates, versions 2-10.
const ALIGN = { 2: [6, 18], 3: [6, 22], 4: [6, 26], 5: [6, 30], 6: [6, 34], 7: [6, 22, 38], 8: [6, 24, 42], 9: [6, 26, 46], 10: [6, 28, 50] };
const FORMAT_ECL = { 1: "L", 0: "M", 3: "Q", 2: "H" };

const forEachSymbol = (fn) => { for (const t of SAMPLES) for (const e of LEVELS) fn(QR.matrix(t, e), t, e); };
const bchRemainder = (word, gen, deg) => { for (let i = 30; i >= deg; i--) if ((word >>> i) & 1) word ^= gen << (i - deg); return word; };
const bits = (q, cells) => cells.reduce((acc, [x, y], i) => acc | ((q.get(x, y) ? 1 : 0) << i), 0);

function readFormat(q) {  // both 15-bit copies, bit 0 first
  const n = q.size;
  const a = [[8, 0], [8, 1], [8, 2], [8, 3], [8, 4], [8, 5], [8, 7], [8, 8], [7, 8], [5, 8], [4, 8], [3, 8], [2, 8], [1, 8], [0, 8]];
  const b = [];
  for (let i = 0; i < 8; i++) b.push([n - 1 - i, 8]);
  for (let i = 8; i < 15; i++) b.push([8, n - 15 + i]);
  return [bits(q, a), bits(q, b)];
}

test("size is 17 + 4·version, and the smallest version that fits is chosen (spec capacities, v1-v10)", () => {
  assert.equal(QR.matrix("x").size, 21);
  assert.equal(QR.matrix("x").version, 1);
  for (const e of LEVELS) {
    CAP[e].forEach((cap, i) => {
      const v = i + 1, full = QR.matrix("a".repeat(cap), e), over = QR.matrix("a".repeat(cap + 1), e);
      assert.equal(full.version, v, `${e}: ${cap} bytes should fit version ${v}`);
      assert.equal(full.size, 17 + 4 * v);
      assert.equal(over.version, v + 1, `${e}: ${cap + 1} bytes should need version ${v + 1}`);
      assert.equal(over.size, 17 + 4 * (v + 1));
      assert.equal(full.modules.length, full.size);
      assert.ok(full.modules.every((row) => row.length === full.size && row.every((d) => typeof d === "boolean")));
    });
  }
  assert.equal(QR.matrix("https://192.168.0.6:8781/phone", "M").size, 29);  // version 3
  assert.equal(QR.matrix("a".repeat(2331), "M").size, 177);  // version 40
});

test("finder patterns with light separators sit in three corners", () => {
  forEachSymbol((q) => {
    const n = q.size;
    for (const [ox, oy] of [[0, 0], [n - 7, 0], [0, n - 7]]) {
      for (let dy = -1; dy <= 7; dy++) {
        for (let dx = -1; dx <= 7; dx++) {
          const x = ox + dx, y = oy + dy;
          if (x < 0 || y < 0 || x >= n || y >= n) continue;
          const ring = Math.max(Math.abs(dx - 3), Math.abs(dy - 3));  // 0-1 core, 2 light, 3 dark, 4 separator
          assert.equal(q.get(x, y), ring !== 2 && ring !== 4, `finder at (${ox},${oy}) wrong at (${x},${y})`);
        }
      }
    }
  });
});

test("timing patterns alternate, the dark module is set, alignment patterns are at the spec positions", () => {
  forEachSymbol((q) => {
    const n = q.size, v = q.version;
    for (let i = 8; i < n - 8; i++) {
      assert.equal(q.get(i, 6), i % 2 === 0, `row-6 timing at x=${i}`);
      assert.equal(q.get(6, i), i % 2 === 0, `column-6 timing at y=${i}`);
    }
    assert.equal(q.get(8, n - 8), true, "dark module");
    const ps = ALIGN[v] || [];
    ps.forEach((cy, i) => ps.forEach((cx, j) => {
      if ((i === 0 && j === 0) || (i === 0 && j === ps.length - 1) || (i === ps.length - 1 && j === 0)) return;
      for (let dy = -2; dy <= 2; dy++) {
        for (let dx = -2; dx <= 2; dx++) assert.equal(q.get(cx + dx, cy + dy), Math.max(Math.abs(dx), Math.abs(dy)) !== 1, `v${v} alignment (${cx},${cy})`);
      }
    }));
  });
});

test("format information: both copies agree, are BCH-valid and name the requested level and a mask 0-7", () => {
  const seen = new Set();
  forEachSymbol((q, t, e) => {
    const [a, b] = readFormat(q);
    assert.equal(a, b, "the two format copies differ");
    const word = a ^ 0x5412;
    assert.equal(bchRemainder(word, 0x537, 10), 0, `format word ${word.toString(2)} is not a BCH(15,5) codeword`);
    const ecl = FORMAT_ECL[(word >>> 13) & 3], mask = (word >>> 10) & 7;
    assert.equal(ecl, e);
    assert.equal(q.ecl, e);
    assert.equal(mask, q.mask);
    assert.ok(mask >= 0 && mask <= 7);
    seen.add(mask);
  });
  assert.ok(seen.size >= 3, `mask choice looks stuck: ${[...seen]}`);
  assert.equal(QR.matrix("x", "l").ecl, "L");  // level names are case-insensitive
});

test("version information (v >= 7) is BCH-valid in both blocks and names the version", () => {
  for (const [len, e] of [[CAP.M[5] + 1, "M"], [CAP.L[8], "L"], [CAP.H[9] + 1, "H"], [1000, "L"], [2331, "M"]]) {
    const q = QR.matrix("b".repeat(len), e), n = q.size;
    assert.ok(q.version >= 7);
    const tr = [], bl = [];
    for (let i = 0; i < 18; i++) { tr.push([n - 11 + (i % 3), Math.floor(i / 3)]); bl.push([Math.floor(i / 3), n - 11 + (i % 3)]); }
    const w = bits(q, tr);
    assert.equal(bits(q, bl), w, "the two version blocks differ");
    assert.equal(bchRemainder(w, 0x1f25, 12), 0, "not a BCH(18,6) codeword");
    assert.equal(w >>> 12, q.version);
    if (q.version === 7) assert.equal(w, 0x07c94);  // Annex D
  }
});

test("get() matches modules[y][x] and is false outside the symbol", () => {
  const q = QR.matrix("https://192.168.0.6:8781/phone");
  for (let y = 0; y < q.size; y++) for (let x = 0; x < q.size; x++) assert.equal(q.get(x, y), q.modules[y][x]);
  for (const [x, y] of [[-1, 0], [0, -1], [q.size, 0], [0, q.size], [-4, -4], [q.size + 3, q.size + 3]]) assert.equal(q.get(x, y), false);
});

test("svg() is a well-formed, scalable image of the same matrix", () => {
  const url = "https://192.168.0.6:8781/phone", q = QR.matrix(url, "M"), s = QR.svg(url, { title: "Open <PhysicsClay> & go" });
  assert.ok(s.startsWith("<svg ") && s.endsWith("</svg>"));
  assert.match(s, /viewBox="0 0 37 37"/);  // 29 modules + 2 × 4 border
  assert.match(s, /shape-rendering="crispEdges"/);
  assert.match(s, /role="img"/);
  assert.match(s, /aria-label="Open &lt;PhysicsClay&gt; &amp; go"/);
  assert.equal((s.match(/<path /g) || []).length, 1);
  assert.equal((s.match(/<rect /g) || []).length, 0, "transparent light colour -> no background rect");
  assert.match(s, /fill="currentColor"/);
  const d = s.match(/ d="([^"]*)"/)[1];
  assert.match(d, /^(M\d+ \d+h\d+v1h-\d+z)+$/);
  let painted = 0;
  for (const [, x, y, w] of d.matchAll(/M(\d+) (\d+)h(\d+)/g)) {
    for (let k = 0; k < +w; k++) { assert.equal(q.get(+x - 4 + k, +y - 4), true); painted++; }
  }
  assert.equal(painted, q.modules.flat().filter(Boolean).length, "every dark module painted exactly once");
  const t = QR.svg("x", { ecl: "H", border: 0, dark: "#000", light: "#fff" });
  assert.match(t, /viewBox="0 0 21 21"/);
  assert.match(t, /<rect width="21" height="21" fill="#fff"\/>/);
  assert.match(t, /aria-label="QR code"/);
  assert.equal((s.match(/</g) || []).length, (s.match(/>/g) || []).length);
});

test("too-long input and unknown levels throw clear errors", () => {
  assert.throws(() => QR.matrix("x".repeat(2332), "M"), /too long.*2332.*at most 2331/);
  assert.throws(() => QR.matrix("x".repeat(1274), "H"), /too long/);
  assert.throws(() => QR.matrix("x".repeat(2954), "L"), /too long/);
  assert.throws(() => QR.matrix("가".repeat(800), "M"), /2400 UTF-8 bytes/);  // 3 bytes each
  assert.throws(() => QR.svg("x".repeat(5000)), /too long/);
  assert.throws(() => QR.matrix("x", "X"), /error-correction level/);
  assert.equal(QR.matrix("x".repeat(1273), "H").version, 40);
});

test("golden: the phone link at level M (decoded by OpenCV when this hash was taken)", () => {
  const q = QR.matrix("https://192.168.0.6:8781/phone", "M");
  const s = q.modules.map((row) => row.map((d) => (d ? "1" : "0")).join("")).join("");
  assert.equal(q.version, 3);
  assert.equal(createHash("sha256").update(s).digest("hex"), "fc3f6716ce65085e8106d5f8b964f75685422241d9ba129947eb536e4f665846");
});

test("loads as a classic browser script and sets window.PCQR", () => {
  const ctx = { window: {} };
  vm.createContext(ctx);
  vm.runInContext(readFileSync(new URL("../web/qr.js", import.meta.url), "utf8"), ctx);
  const P = ctx.window.PCQR;
  assert.equal(typeof P.matrix, "function");
  assert.equal(typeof P.svg, "function");
  assert.equal(P.svg("https://192.168.0.6:8781/phone"), QR.svg("https://192.168.0.6:8781/phone"));
});
