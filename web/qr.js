// PhysicsClay QR: a small, dependency-free QR Code encoder (ISO/IEC 18004, model 2) so the laptop UI can show the
// phone link (`https://<lan-ip>:<port>/phone`) as a code to scan. Classic script: sets window.PCQR in a browser and
// module.exports under Node (tests/qr.test.mjs). Byte mode (UTF-8), levels L/M/Q/H, versions 1-40, all 8 masks.
//
//   PCQR.matrix(text, ecl = "M") -> { version, ecl, mask, size, modules, get(x, y) }
//     text is encoded as UTF-8 bytes; ecl is "L" | "M" | "Q" | "H" (~7 / 15 / 25 / 30 % of codewords recoverable).
//     The smallest version that fits is used (size = 17 + 4 * version modules); throws an Error if the text is too
//     long (at most 2953 / 2331 / 1663 / 1273 bytes for L / M / Q / H; a 30-char LAN URL is version 3, 29×29, at M).
//     modules is row-major: modules[y][x] === true for a dark module (x right, y down, no quiet zone included);
//     get(x, y) is the same lookup but returns false outside the symbol, so it can be read with any border.
//   PCQR.svg(text, { ecl = "M", border = 4, dark = "currentColor", light = "transparent", title = "" }) -> string
//     an <svg> with viewBox="0 0 n n" (one unit per module, n = size + 2 * border), shape-rendering="crispEdges",
//     role="img", aria-label=title (or "QR code" when empty), a background <rect> only when light isn't
//     transparent, and one <path> holding every dark module. Size it with CSS; keep border >= 4 (the quiet zone).
//
// Structure (tables, function-pattern layout, mask scoring) follows Project Nayuki's QR Code generator, MIT License.
(function () {
  "use strict";

  // Error-correction level -> [row in the tables below, 2-bit code in the format information].
  const ECL = { L: [0, 1], M: [1, 0], Q: [2, 3], H: [3, 2] };
  // ISO/IEC 18004 table 9, one row per level, indexed by version (index 0 unused):
  // error-correction codewords per block, and number of blocks.
  const ECC_PER_BLOCK = [
    [0, 7, 10, 15, 20, 26, 18, 20, 24, 30, 18, 20, 24, 26, 30, 22, 24, 28, 30, 28, 28, 28, 28, 30, 30, 26, 28, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30],
    [0, 10, 16, 26, 18, 24, 16, 18, 22, 22, 26, 30, 22, 22, 24, 24, 28, 28, 26, 26, 26, 26, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28],
    [0, 13, 22, 18, 26, 18, 24, 18, 22, 20, 24, 28, 26, 24, 20, 30, 24, 28, 28, 26, 30, 28, 30, 30, 30, 30, 28, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30],
    [0, 17, 28, 22, 16, 22, 28, 26, 26, 24, 28, 24, 28, 22, 24, 24, 30, 28, 28, 26, 28, 30, 24, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30],
  ];
  const BLOCKS = [
    [0, 1, 1, 1, 1, 1, 2, 2, 2, 2, 4, 4, 4, 4, 4, 6, 6, 6, 6, 7, 8, 8, 9, 9, 10, 12, 12, 12, 13, 14, 15, 16, 17, 18, 19, 19, 20, 21, 22, 24, 25],
    [0, 1, 1, 1, 2, 2, 4, 4, 4, 5, 5, 5, 8, 9, 9, 10, 10, 11, 13, 14, 16, 17, 17, 18, 20, 21, 23, 25, 26, 28, 29, 31, 33, 35, 37, 38, 40, 43, 45, 47, 49],
    [0, 1, 1, 2, 2, 4, 4, 6, 6, 8, 8, 8, 10, 12, 16, 12, 17, 16, 18, 21, 20, 23, 23, 25, 27, 29, 34, 34, 35, 38, 40, 43, 45, 48, 51, 53, 56, 59, 62, 65, 68],
    [0, 1, 1, 2, 4, 4, 4, 5, 6, 8, 8, 11, 11, 16, 16, 18, 16, 19, 21, 25, 25, 25, 34, 30, 32, 35, 37, 40, 42, 45, 48, 51, 54, 57, 60, 63, 66, 70, 74, 77, 81],
  ];

  // ---------------------------------------------------------------- GF(256) / Reed-Solomon (polynomial 0x11D, α = 2)
  const EXP = new Uint8Array(512), LOG = new Uint8Array(256);
  for (let i = 0, x = 1; i < 255; i++) { EXP[i] = EXP[i + 255] = x; LOG[x] = i; x = (x << 1) ^ (x & 0x80 ? 0x11d : 0); }
  const gmul = (a, b) => (a && b ? EXP[LOG[a] + LOG[b]] : 0);
  const generators = {};
  function generator(n) {  // coefficients of ∏_{i<n} (x - α^i), highest degree first (leading 1 included)
    if (!generators[n]) {
      let g = [1];
      for (let i = 0; i < n; i++) {
        const h = g.concat(0);
        for (let j = 0; j < g.length; j++) h[j + 1] ^= gmul(g[j], EXP[i]);
        g = h;
      }
      generators[n] = g;
    }
    return generators[n];
  }
  function rsRemainder(data, n) {  // the n error-correction codewords for one block
    const g = generator(n), r = new Array(n).fill(0);
    for (const b of data) {
      const f = b ^ r.shift();
      r.push(0);
      if (f) for (let i = 0; i < n; i++) r[i] ^= gmul(g[i + 1], f);
    }
    return r;
  }

  // ---------------------------------------------------------------- capacity and codewords
  function rawModules(v) {  // modules left for data + EC once the function patterns are drawn
    let r = (16 * v + 128) * v + 64;
    if (v >= 2) {
      const n = Math.floor(v / 7) + 2;  // alignment patterns per row / column
      r -= (25 * n - 10) * n - 55;
      if (v >= 7) r -= 36;  // two version-information blocks
    }
    return r;
  }
  const dataCodewords = (v, e) => Math.floor(rawModules(v) / 8) - ECC_PER_BLOCK[e][v] * BLOCKS[e][v];
  const countBits = (v) => (v < 10 ? 8 : 16);  // byte-mode character count indicator

  function utf8(s) {
    const b = [];
    for (const ch of s) {
      let c = ch.codePointAt(0);
      if (c >= 0xd800 && c <= 0xdfff) c = 0xfffd;  // lone surrogate
      if (c < 0x80) b.push(c);
      else if (c < 0x800) b.push(0xc0 | (c >> 6), 0x80 | (c & 63));
      else if (c < 0x10000) b.push(0xe0 | (c >> 12), 0x80 | ((c >> 6) & 63), 0x80 | (c & 63));
      else b.push(0xf0 | (c >> 18), 0x80 | ((c >> 12) & 63), 0x80 | ((c >> 6) & 63), 0x80 | (c & 63));
    }
    return b;
  }

  function codewords(bytes, v, e) {  // mode + count + data + terminator + padding, split into blocks, EC, interleaved
    const cap = dataCodewords(v, e), bits = [];
    const put = (val, n) => { for (let i = n - 1; i >= 0; i--) bits.push((val >>> i) & 1); };
    put(0b0100, 4);
    put(bytes.length, countBits(v));
    for (const b of bytes) put(b, 8);
    put(0, Math.min(4, cap * 8 - bits.length));
    put(0, (8 - (bits.length % 8)) % 8);
    const data = [];
    for (let i = 0; i < bits.length; i += 8) data.push(bits.slice(i, i + 8).reduce((a, b) => a * 2 + b, 0));
    for (let pad = 0xec; data.length < cap; pad ^= 0xec ^ 0x11) data.push(pad);

    const nb = BLOCKS[e][v], ec = ECC_PER_BLOCK[e][v], total = Math.floor(rawModules(v) / 8);
    const shortData = Math.floor(total / nb) - ec, numShort = nb - (total % nb);  // the last blocks get one more
    const blocks = [];
    for (let i = 0, k = 0; i < nb; i++) {
      const len = shortData + (i < numShort ? 0 : 1), d = data.slice(k, k + len);
      blocks.push({ d, r: rsRemainder(d, ec) });
      k += len;
    }
    const out = [];
    for (let i = 0; i <= shortData; i++) for (const b of blocks) if (i < b.d.length) out.push(b.d[i]);
    for (let i = 0; i < ec; i++) for (const b of blocks) out.push(b.r[i]);
    return out;
  }

  // ---------------------------------------------------------------- function patterns
  function alignmentPositions(v) {
    if (v === 1) return [];
    const n = Math.floor(v / 7) + 2, step = Math.floor((v * 8 + n * 3 + 5) / (n * 4 - 4)) * 2, out = [6];
    for (let p = 4 * v + 17 - 7; out.length < n; p -= step) out.splice(1, 0, p);
    return out;
  }
  function formatBits(eclCode, mask) {  // 5 data bits + BCH(15,5) remainder, XOR the fixed mask 0x5412
    const d = (eclCode << 3) | mask;
    let r = d;
    for (let i = 0; i < 10; i++) r = (r << 1) ^ ((r >>> 9) * 0x537);
    return ((d << 10) | r) ^ 0x5412;
  }
  function drawFormat(set, size, bits) {  // bit 0 = least significant; two copies
    const b = (i) => ((bits >>> i) & 1) === 1;
    for (let i = 0; i <= 5; i++) set(8, i, b(i));
    set(8, 7, b(6)); set(8, 8, b(7)); set(7, 8, b(8));
    for (let i = 9; i < 15; i++) set(14 - i, 8, b(i));
    for (let i = 0; i < 8; i++) set(size - 1 - i, 8, b(i));
    for (let i = 8; i < 15; i++) set(8, size - 15 + i, b(i));
    set(8, size - 8, true);  // the dark module
  }

  // ---------------------------------------------------------------- masks and penalty (ISO/IEC 18004 7.8.3)
  const MASKS = [
    (x, y) => (x + y) % 2 === 0,
    (x, y) => y % 2 === 0,
    (x, y) => x % 3 === 0,
    (x, y) => (x + y) % 3 === 0,
    (x, y) => (Math.floor(x / 3) + Math.floor(y / 2)) % 2 === 0,
    (x, y) => ((x * y) % 2) + ((x * y) % 3) === 0,
    (x, y) => (((x * y) % 2) + ((x * y) % 3)) % 2 === 0,
    (x, y) => (((x + y) % 2) + ((x * y) % 3)) % 2 === 0,
  ];
  function penalty(g) {
    const n = g.length, light = (line, a, b) => { for (let k = a; k < b; k++) if (k >= 0 && k < n && line[k]) return false; return true; };
    let p = 0, dark = 0;
    for (let i = 0; i < n; i++) {
      for (const line of [g[i], g.map((row) => row[i])]) {
        for (let k = 1, run = 1; k <= n; k++) {  // N1: 3 + (run - 5) for each run of >= 5 same-colour modules
          if (k < n && line[k] === line[k - 1]) run++;
          else { if (run >= 5) p += run - 2; run = 1; }
        }
        for (let k = 0; k + 7 <= n; k++) {  // N3: dark-light-dark×3-light-dark with 4 light modules on one side
          if (line[k] && !line[k + 1] && line[k + 2] && line[k + 3] && line[k + 4] && !line[k + 5] && line[k + 6] &&
              (light(line, k - 4, k) || light(line, k + 7, k + 11))) p += 40;
        }
      }
    }
    for (let y = 0; y < n; y++) {
      for (let x = 0; x < n; x++) {
        if (g[y][x]) dark++;
        if (x + 1 < n && y + 1 < n && g[y][x] === g[y][x + 1] && g[y][x] === g[y + 1][x] && g[y][x] === g[y + 1][x + 1]) p += 3;  // N2
      }
    }
    return p + Math.floor(Math.abs(dark * 20 - n * n * 10) / (n * n)) * 10;  // N4: 10 per 5 % away from half dark
  }

  // ---------------------------------------------------------------- the symbol
  function matrix(text, ecl = "M") {
    const E = String(ecl).toUpperCase(), lv = ECL[E];
    if (!lv) throw new Error(`PCQR: unknown error-correction level "${ecl}" (use L, M, Q or H)`);
    const e = lv[0], bytes = utf8(text == null ? "" : String(text));
    let v = 1;
    while (v <= 40 && 4 + countBits(v) + 8 * bytes.length > dataCodewords(v, e) * 8) v++;
    if (v > 40) {
      const max = Math.floor((dataCodewords(40, e) * 8 - 20) / 8);
      throw new Error(`PCQR: text too long for a QR code (${bytes.length} UTF-8 bytes; level ${E} holds at most ${max})`);
    }
    const size = 4 * v + 17, mod = [], fn = [];
    for (let y = 0; y < size; y++) { mod.push(new Array(size).fill(false)); fn.push(new Array(size).fill(false)); }
    const set = (x, y, dark) => { mod[y][x] = dark; fn[y][x] = true; };

    for (let i = 0; i < size; i++) { set(6, i, i % 2 === 0); set(i, 6, i % 2 === 0); }  // timing patterns
    for (const [cx, cy] of [[3, 3], [size - 4, 3], [3, size - 4]]) {  // finders with their light separators
      for (let dy = -4; dy <= 4; dy++) {
        for (let dx = -4; dx <= 4; dx++) {
          const x = cx + dx, y = cy + dy, d = Math.max(Math.abs(dx), Math.abs(dy));
          if (x >= 0 && x < size && y >= 0 && y < size) set(x, y, d !== 2 && d !== 4);
        }
      }
    }
    const ap = alignmentPositions(v), last = ap.length - 1;
    ap.forEach((ay, i) => ap.forEach((ax, j) => {
      if ((i === 0 && j === 0) || (i === 0 && j === last) || (i === last && j === 0)) return;  // under a finder
      for (let dy = -2; dy <= 2; dy++) for (let dx = -2; dx <= 2; dx++) set(ax + dx, ay + dy, Math.max(Math.abs(dx), Math.abs(dy)) !== 1);
    }));
    drawFormat(set, size, 0);  // reserve the format areas; the real bits go in per mask below
    if (v >= 7) {  // version information: 6 data bits + BCH(18,6) remainder, two 6×3 blocks
      let r = v;
      for (let i = 0; i < 12; i++) r = (r << 1) ^ ((r >>> 11) * 0x1f25);
      const bits = (v << 12) | r;
      for (let i = 0; i < 18; i++) {
        const a = size - 11 + (i % 3), b = Math.floor(i / 3), dark = ((bits >>> i) & 1) === 1;
        set(a, b, dark); set(b, a, dark);
      }
    }

    const data = codewords(bytes, v, e);  // zig-zag up/down two-column strips from the bottom-right, skipping x = 6
    for (let right = size - 1, i = 0; right >= 1; right -= 2) {
      if (right === 6) right = 5;
      const up = ((right + 1) & 2) === 0;
      for (let vert = 0; vert < size; vert++) {
        for (let j = 0; j < 2; j++) {
          const x = right - j, y = up ? size - 1 - vert : vert;
          if (!fn[y][x] && i < data.length * 8) { mod[y][x] = ((data[i >>> 3] >>> (7 - (i & 7))) & 1) === 1; i++; }
        }
      }
    }  // any remainder modules stay light (0) before masking

    let best = null;
    for (let m = 0; m < 8; m++) {
      const g = mod.map((row, y) => row.map((d, x) => (fn[y][x] ? d : d !== MASKS[m](x, y))));
      drawFormat((x, y, d) => { g[y][x] = d; }, size, formatBits(lv[1], m));
      const p = penalty(g);
      if (!best || p < best.p) best = { m, g, p };
    }
    const modules = best.g;
    return { version: v, ecl: E, mask: best.m, size, modules,
      get: (x, y) => x >= 0 && y >= 0 && x < size && y < size && modules[y][x] };
  }

  function svg(text, opts = {}) {
    const { ecl = "M", border = 4, dark = "currentColor", light = "transparent", title = "" } = opts || {};
    const q = matrix(text, ecl), b = Math.max(0, Math.floor(Number(border) || 0)), n = q.size + 2 * b;
    const esc = (s) => String(s).replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" })[c]);
    let d = "";
    q.modules.forEach((row, y) => {  // one rectangle per horizontal run of dark modules
      for (let x = 0; x < q.size; x++) {
        if (!row[x]) continue;
        let w = 1;
        while (x + w < q.size && row[x + w]) w++;
        d += `M${x + b} ${y + b}h${w}v1h-${w}z`;
        x += w - 1;
      }
    });
    const bg = light && light !== "transparent" && light !== "none" ? `<rect width="${n}" height="${n}" fill="${esc(light)}"/>` : "";
    return `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 ${n} ${n}" shape-rendering="crispEdges" role="img" ` +
      `aria-label="${esc(title || "QR code")}">${bg}<path fill="${esc(dark)}" d="${d}"/></svg>`;
  }

  const PCQR = { matrix, svg };
  if (typeof window !== "undefined") window.PCQR = PCQR;
  if (typeof module !== "undefined" && module.exports) module.exports = PCQR;
})();
