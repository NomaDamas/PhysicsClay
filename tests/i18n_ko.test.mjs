// Unit tests for web/ui/i18n-ko.js (the Korean UI chrome). The dictionary is read the way the browser would build it:
// every value is Korean, nothing maps to itself, a Korean value is never a key (so the page observer cannot loop), no
// key is listed twice and keys are trimmed and single-spaced (lookups normalise the page's text the same way). The
// browser path is then run in a minimal fake window/document: an English page is left alone and window.PCI18N is
// exposed. Run: node --test tests/i18n_ko.test.mjs (or tests/*.mjs)
import test from "node:test";
import assert from "node:assert/strict";
import { createRequire } from "node:module";
import { readFileSync } from "node:fs";
import vm from "node:vm";

const require = createRequire(import.meta.url);
const { KO, ENTRIES } = require("../web/ui/i18n-ko.js");
const SRC = readFileSync(new URL("../web/ui/i18n-ko.js", import.meta.url), "utf8");
const HANGUL = /[가-힣]/;

test("the dictionary loads outside a browser", () => {
  assert.ok(KO instanceof Map);
  assert.ok(Array.isArray(ENTRIES));
  assert.ok(KO.size >= 150, `only ${KO.size} entries`);
  for (const e of ENTRIES) {
    assert.equal(e.length, 2);
    assert.equal(typeof e[0], "string");
    assert.equal(typeof e[1], "string");
  }
});

test("every value contains Hangul", () => {
  const bad = ENTRIES.filter(([, ko]) => !HANGUL.test(ko)).map(([en]) => en);
  assert.deepEqual(bad, []);
});

test("no key maps to itself", () => {
  assert.deepEqual(ENTRIES.filter(([en, ko]) => en === ko).map(([en]) => en), []);
});

test("no Korean value is also a key", () => {
  assert.deepEqual(ENTRIES.filter(([, ko]) => KO.has(ko) || KO.has(ko.trim().replace(/\s+/g, " "))).map(([, ko]) => ko), []);
});

test("no duplicate keys", () => {
  const seen = new Set(), dups = [];
  for (const [en] of ENTRIES) { if (seen.has(en)) dups.push(en); seen.add(en); }
  assert.deepEqual(dups, []);
  assert.equal(KO.size, ENTRIES.length);
});

test("keys are trimmed and single-spaced", () => {
  const bad = ENTRIES.map(([en]) => en).filter((en) => !en || en !== en.trim() || /\s{2,}|[\n\t]/.test(en));
  assert.deepEqual(bad, []);
});

test("values are trimmed", () => {
  assert.deepEqual(ENTRIES.filter(([, ko]) => ko !== ko.trim()).map(([en]) => en), []);
});

test("in a browser on an English page: nothing touched, window.PCI18N exposed", () => {
  const listeners = {};
  const window = { PC: { lang: "en" }, addEventListener(type, fn) { (listeners[type] = listeners[type] || []).push(fn); } };
  const document = {
    readyState: "complete", documentElement: { lang: "en" }, body: {},
    addEventListener() { throw new Error("an English page should not wait for the DOM"); },
  };
  vm.runInNewContext(SRC, { window, document, console });
  assert.ok(window.PCI18N, "window.PCI18N");
  assert.equal(window.PCI18N.ko.size, KO.size);
  assert.equal(typeof window.PCI18N.translate, "function");
  assert.equal(typeof window.PCI18N.restore, "function");
  assert.equal(window.PCI18N.lang, "en");
  assert.equal(document.documentElement.lang, "en");
  assert.ok(listeners["pc-lang"] && listeners["pc-lang"].length === 1, "listens for pc-lang");
});
