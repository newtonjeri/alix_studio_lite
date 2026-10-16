// The spoken timer of Drag & Teach (the owner, 16 Oct: "the voiceover in the drag and teach should be very
// natural. And should count every number, every second ... natural and consistent. And tick like a second,
// not too fast or too slow"). The page spoke from its 0.1 s screen refresh, cancelled the number being said
// for the next one, and at 1.28 times the voice's pace; with the browser's remote voices (Google's are the
// only English ones Chrome has on Linux) the numbers came late, in bursts, or were cut. Here:
// - each number has its own second, on a clock of its own (the take's start, or the countdown's), and is
//   asked for once;
// - it is asked a moment early, by the voice's own delay to start speaking (learnt from the numbers said), so
//   it is heard on its second whatever the voice;
// - at the voice's natural pace; a number too long to be said within its second (past 99: "one hundred
//   and twenty-one") a little faster, so it is over before the next one;
// - never cut by the next number; one still waiting to start when the next is due (the voice stalled) gives
//   way to it, so the count never runs on in a burst;
// - in a natural English voice: Microsoft's natural voices, then Google's, US English first.

const synth = () => ('speechSynthesis' in window ? window.speechSynthesis : null);
const enabled = () => !!document.getElementById('voiceTimerEnabled')?.checked;

let picked = null;
function englishVoice() {
  const s = synth();
  const all = s ? s.getVoices() : [];
  if (picked && all.includes(picked)) return picked;
  const rank = (v) => (/natural|neural/i.test(v.name) ? 0 : /google/i.test(v.name) ? 2 : 4) + (/^en[-_]us/i.test(v.lang) ? 0 : 1);
  picked = all.filter(v => /^en([-_]|$)/i.test(v.lang)).sort((a, b) => rank(a) - rank(b))[0] || null;
  return picked;
}

// A number's syllables in English, for the pace that fits it into its second.
const ONES = [0, 1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 3, 1, 2, 2, 2, 2, 3, 2, 2];       // one … nineteen
const TENS = [0, 0, 2, 2, 2, 2, 2, 3, 2, 2];                                    // twenty … ninety
function syllables(n) {
  if (n < 20) return ONES[n] || 1;
  if (n < 100) return TENS[Math.floor(n / 10)] + (n % 10 ? ONES[n % 10] : 0);
  if (n < 1000) return ONES[Math.floor(n / 100)] + 2 + (n % 100 ? syllables(n % 100) : 0);
  return syllables(Math.floor(n / 1000)) + 2 + (n % 1000 ? syllables(n % 1000) : 0);
}
// a voice says about five syllables a second: every number up to 99 at its natural pace, a longer one faster
export const paceFor = (n) => Math.min(1.8, Math.max(1, syllables(n) / 5));

let lead = 0;                  // how long the voice takes to start speaking (ms), learnt from the numbers said

// Says `text` at `rate`. `fresh`: what is being said stops first (START's "Start", the voice test).
function say(text, rate = 1, fresh = false) {
  const s = synth();
  if (!s || !enabled()) return;
  const cut = fresh || s.pending;                      // a number still waiting to start is late: it gives way
  if (cut) s.cancel();
  const u = new SpeechSynthesisUtterance(String(text));
  u.lang = 'en-US';
  u.rate = rate;
  u.pitch = 1;
  u.volume = 1;
  const v = englishVoice();
  if (v) { u.voice = v; u.lang = v.lang; }
  const go = () => {
    const asked = performance.now();
    u.onstart = () => { lead = Math.round(0.6 * lead + 0.4 * Math.min(700, performance.now() - asked)); };
    if (s.paused) s.resume();
    s.speak(u);
  };
  if (cut) setTimeout(go, 60);                        // Chrome can drop a word asked at once after a cancel
  else go();
}

let clock = null;
// Says tick(k).text at tick(k).at (a performance.now() time) for k = 0, 1, 2 … until tick(k) is null or
// `still()` is false. A page held back (another window over the browser) comes back on the second it is at.
function run(tick, still = () => true) {
  stopCount();
  const me = { k: 0, timer: 0 };
  clock = me;
  const next = () => {
    if (clock !== me) return;
    let t = tick(me.k);
    while (t && performance.now() - t.at > 900) { me.k += 1; t = tick(me.k); }
    if (!t) { clock = null; return; }
    me.timer = setTimeout(() => {
      if (clock !== me) return;
      if (!still()) { clock = null; return; }
      say(t.text, t.rate);
      me.k += 1;
      next();
    }, Math.max(0, t.at - lead - performance.now()));
  };
  next();
}

export function stopCount() {
  if (clock) clearTimeout(clock.timer);
  clock = null;
}

// The take's seconds: "1" one second after `t0`, "2" at two … while `still()` (the take's timer runs).
export function countUp(t0, still) {
  run((k) => ({ at: t0 + (k + 1) * 1000, text: String(k + 1), rate: paceFor(k + 1) }), still);
}

// The countdown before drag teaching starts: "n" at `t0`, "n - 1" a second later … "1".
export function countDown(n, t0) {
  run((k) => (k < n ? { at: t0 + k * 1000, text: String(n - k), rate: paceFor(n - k) } : null));
}

export function installVoice() {
  // the page's own words (START's "Start", the voice test) in the same voice, at its natural pace
  window.speakEnglish = function speakEnglish(text, interrupt = false) { say(text, 1, interrupt); };
  const pageStop = window.stopVoice;
  window.stopVoice = function stopVoice() { stopCount(); pageStop(); };
  synth()?.addEventListener?.('voiceschanged', () => { picked = null; });
}
