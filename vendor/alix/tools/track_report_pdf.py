"""PDF bundle of upload-channel runs — graphs first, a few words second.

Called by ``tools/track_report.py report``; file-only like its caller (inert:
no SDK, no controller, no UDP). One document per period, every run in it,
grouped and colour-coded by the PROGRAM SETTING it ran under (acc-scale,
velocity margin, run speed) so runs are only compared with their peers.

Pages: overview table + file-time bars | the player clock | one page per task
(runs overlaid) | one page per run (commanded vs executed, all joints) | notes.
"""
from __future__ import annotations

import datetime as _dt
import hashlib
import json
import pathlib
import subprocess

import numpy as np

import track_report as tr

CLOCK_S = tr.CLOCK_S
ENV = tr.ENVELOPE_MS
RATED = np.array([180, 180, 225, 225, 225, 225, 225], float)
GREEN, RED, GREY, BLUE = "#15803D", "#B91C1C", "#6B7280", "#2563EB"
PALETTE = ["#2563EB", "#B45309", "#7C3AED", "#0891B2", "#DB2777", "#65A30D", "#EA580C", "#4B5563"]


# ------------------------------------------------------------------ inputs
def track_for_record(rec: dict, cache: pathlib.Path):
    """The commanded track AS IT RAN: the working file if its point count
    matches the record, else the git version with that count (tracks are
    re-emitted in place; a record pins `points` and, since F111, sha256)."""
    rel = rec.get("track")
    if not rel:
        return None
    path = tr.REPO / rel
    want = rec.get("points")
    want_sha = rec.get("track_sha256")

    def ok(p):
        if want_sha:
            return hashlib.sha256(p.read_bytes()).hexdigest() == want_sha
        return sum(1 for l in p.open() if l.strip()) == want
    if path.is_file() and ok(path):
        return tr.load_track(path)
    cache.mkdir(parents=True, exist_ok=True)
    try:
        shas = subprocess.run(["git", "-C", str(tr.REPO), "rev-list", "--all", "--", rel],
                              capture_output=True, text=True, timeout=60).stdout.split()
    except Exception:                                                  # noqa: BLE001
        shas = []
    for sha in shas:
        out = cache / f"{sha[:12]}_{path.name}"
        if not out.is_file():
            r = subprocess.run(["git", "-C", str(tr.REPO), "show", f"{sha}:{rel}"],
                               capture_output=True, text=True)
            if r.returncode != 0:
                continue
            out.write_text(r.stdout)
        if ok(out):
            return tr.load_track(out)
    return None


def setting_of(rec: dict) -> str:
    acc = rec.get("acc_scale")
    vel = rec.get("vel_scale")
    s = f"acc {acc:g}" if acc else "acc ? (pre-policy)"
    if vel:
        s += f" · vel {vel:g}"
    return s + f" · speed {rec.get('speed')}"


def analyse(rec_path: pathlib.Path, cache: pathlib.Path) -> dict:
    rec = json.loads(rec_path.read_text())
    out = dict(rec=rec, path=rec_path, name=(rec.get("name") or "")[10:], slot=rec.get("slot"),
               when=rec.get("when", "?"), setting=setting_of(rec), frames=None)
    pts = rec.get("points") or 0
    nominal = (pts - 1) * CLOCK_S if pts else None
    out["nominal_s"] = nominal
    rc = rec.get("replay_check") or {}
    # `phase_last` is the ordered lag-budget matcher; `phase_end` is the
    # commanded index nearest where the arm actually came to rest. They agree
    # on a sound measurement. When they do not, the window was wrong and the
    # matcher saturated: run_slot78_212332 replayed 0.33 s of a 4.2 s track
    # and reported phase_last 73 % against phase_end 4 %, because its window
    # was 7.7 s wide (a 6.8 s settle wait, cleaning_tracks.py:978). Report the
    # LOWER of the two and flag the disagreement rather than the flattering
    # number.
    # Neither stored value can be trusted unconditionally. `phase_last` (the
    # ordered matcher) SATURATES when the window was too wide —
    # run_slot78_212332 replayed 0.33 s of a 4.2 s track and stored 73 %,
    # because a 6.8 s settle made its window 7.7 s (cleaning_tracks.py:978).
    # `phase_end` is corrupted the other way by the park move (F125):
    # run_slot74_212134 covered 100 % and stored 85 %. So `cover` is REPLACED
    # below by this tool's own end_phase, measured over the window this file
    # clips itself; the stored pair is kept only to flag the disagreement.
    out["cover"] = rc.get("phase_last")
    out["cover_disputed"] = False
    out["cover_phase_last"], out["cover_phase_end"] = rc.get("phase_last"), rc.get("phase_end")
    out["complete"] = rec.get("complete")
    # A refused run never traced the track, so complete=False is not a cut.
    refused = str(rec.get("verdict", "")).startswith(("refused", "did_not_start"))
    out["refused"] = refused
    out["cut"] = (rec.get("complete") is False and not refused)
    out["outcome"] = ("refused" if refused else
                      "complete" if rec.get("complete") else
                      "cut" if rec.get("complete") is False else "unresolved")
    rep = rec.get("replay_s")
    out["replay_s"] = rep
    out["file_pct"] = (rep / nominal * 100) if (rep and nominal) else None
    out["clock_ms"] = (rep / (pts - 1) * 1e3) if (rep and pts and rec.get("complete")) else None
    out["clock_if_full_ms"] = (rep / (pts - 1) * 1e3) if (rep and pts) else None
    out["arm_rate_ms"] = (rep / (out["cover"] * pts) * 1e3) if (rep and pts and out["cover"]) else None
    fp = rec_path.with_name(rec_path.stem + "_frames.ndjson")
    if not fp.is_file():
        return out
    ts, q, raw = tr.load_frames(fp)
    if len(ts) < 10:
        return out
    T = track_for_record(rec, cache)
    out["track"] = T
    # replay window: the record's split when it has one, else the status field
    if rec.get("movej_s") is not None and rep:
        t0 = ts[0] + rec["movej_s"] + (rec.get("dwell_s") or 0.0)
        t1 = t0 + rep
    else:
        # No split. Take the controller's OWN label — first and LAST
        # move_through_joint sample. The old fallback took ts[-1], the end of
        # the capture, which since 2026-09-04 is the far side of the park move;
        # on a record whose only label is `stop` it also fell back to index 0
        # and plotted 174 stationary samples as an executed path.
        idx = [i for i, f in enumerate(raw)
               if f.get("arm_current_status") == "move_through_joint"]
        if not idx:
            return out
        t0, t1 = ts[idx[0]], ts[idx[-1]]
    out.update(ts=ts, q=q, raw=raw, t0=t0, t1=t1)
    # The 0.7 s tail catches the arm's braking past the last commanded point —
    # but since 2026-09-04 a PARK movej follows every replay under the same
    # capture, and on 5 of 116 runs the tail reached into it: run_slot72_213341
    # gained 139 samples, 15.85 deg of joint travel and a 72 mm TCP excursion
    # (30 % of that raster's extent) that were plotted as part of the wipe, and
    # its deviation p99 read 15 % high. Keep the tail, but only for samples the
    # controller does not label as another move.
    m = (ts >= t0) & (ts <= t1 + 0.7)
    lab = np.asarray([(f.get("arm_current_status") or "") for f in raw])
    if len(lab) == len(m):
        m &= (ts <= t1) | (lab != "move_J")
    tw, qw = ts[m] - t0, q[m]
    out["tw"], out["qw"] = tw, qw
    sp = np.asarray([f["joint_status"].get("joint_speed") or [0] * 7 for f in raw], float) * tr.JOINT_SPEED_SCALE
    out["speed_all"] = sp
    out["speed_w"] = sp[m]
    if len(sp[m]):
        out["jmax"] = np.abs(sp[m]).max(0)          # deg/s, per joint, replay window
        out["javg"] = np.abs(sp[m]).mean(0)         # mean |speed| per joint
    lift = np.asarray([(f.get("lift_state") or {}).get("height", np.nan) for f in raw], float)
    out["lift"] = lift
    out["t_cap"] = ts - ts[0]
    errs = [(f.get("err") or [0]) for f in raw]
    first_err = next(((ts[i] - t0, e) for i, e in enumerate(errs) if any(e)), None)
    out["first_err"] = first_err
    out["status"] = [f.get("arm_current_status") for f in raw]
    # progress along the commanded path: nearest commanded point inside a LAG
    # BUDGET around the nominal-clock index (400 points = 2.2 s behind, 150
    # ahead). Measured on every run with a capture: the arm's position tracks
    # the nominal index within 1-3 % from start to end — the player streams at
    # its clock and the arm keeps up; a "cut" is the program ending EARLY at
    # that pace, not a faster player (F111). A purely local matcher stalls at
    # raster corners; an unbounded one jumps stripes. This one does neither.
    if T is not None and len(qw):
        n = len(T)
        step = 2
        phase, dev, tt = [], [], []
        for i in range(0, len(qw), step):
            nom = min(n - 1, int(tw[i] / CLOCK_S))
            lo, hi = max(0, nom - 400), min(n, nom + 150)
            d = np.abs(T[lo:hi] - qw[i]).max(1)
            k = lo + int(d.argmin())
            phase.append(k / (n - 1) * 100)
            dev.append(float(d.min()))
            tt.append(tw[i])
        out["prog_t"], out["prog"], out["dev"] = np.array(tt), np.array(phase), np.array(dev)
        out["nom_prog"] = np.clip(np.array(tt) / CLOCK_S / (n - 1) * 100, 0, 100)
        # the arm's pace from its own progress: ms per commanded point actually
        # traversed — the period, readable on cut runs too
        end_phase = float(np.percentile(out["prog"][-max(3, len(out["prog"]) // 20):], 90)) / 100
        out["end_phase"] = end_phase
        # measured here, over the clipped window — see the note on `cover`
        stored = out["cover_phase_last"]
        out["cover"] = end_phase
        out["cover_disputed"] = bool(isinstance(stored, float)
                                     and abs(stored - end_phase) > 0.05)
        if rep and end_phase > 0.05:
            out["pace_ms"] = rep / (end_phase * (n - 1)) * 1e3
    em = tr.executed_motion([r for r, keep in zip(raw, m) if keep], ts[m], np)
    out["em"] = em
    return out


# ------------------------------------------------------------------ pages
def _fig(pdf, title, sub=None):
    import matplotlib.pyplot as plt
    fig = plt.figure(figsize=(11.69, 8.27))              # A4 landscape
    fig.text(0.04, 0.965, title, fontsize=15, fontweight="bold", color="#1E293B", va="top")
    if sub:
        fig.text(0.04, 0.935, sub, fontsize=9, color="#5B6B82", va="top")
    return fig


def _colour(run):
    return GREEN if run["outcome"] == "complete" else RED if run["outcome"] == "cut" else GREY


def page_overview(pdf, runs, period, groups):
    import matplotlib.pyplot as plt
    n_ok = sum(r["outcome"] == "complete" for r in runs)
    n_cut = sum(r["outcome"] == "cut" for r in runs)
    fig = _fig(pdf, f"Upload-channel runs — {period}",
               f"{len(runs)} recorded runs · {n_ok} complete · {n_cut} cut by the controller · "
               f"{len(runs) - n_ok - n_cut} unresolved (no departure latch) · settings: " +
               " | ".join(f"{g} ({c})" for g, c in groups))
    cols = ["time", "slot", "task", "setting", "pole mm", "devices", "replay s", "file %",
            "period ms/pt", "path %", "dev p99°", "outcome"]
    cells, colours = [], []
    for r in runs:
        lift = r.get("lift")
        pole = "-"
        if lift is not None and np.isfinite(lift).any():
            a, b = lift[np.isfinite(lift)][[0, -1]]
            pole = f"{a:.0f}" if a == b else f"{a:.0f}→{b:.0f}"
        cells.append([r["when"][11:19], r["slot"], r["name"], r["setting"], pole,
                      "flow" if r["rec"].get("pre_run") else "as found",
                      f"{r['replay_s']:.2f}" if r["replay_s"] else "-",
                      f"{r['file_pct']:.0f}" if r["file_pct"] else "-",
                      f"{r['pace_ms']:.2f}" if r.get("pace_ms") else "-",
                      (f"{r['cover'] * 100:.0f}" + ("?" if r.get("cover_disputed") else ""))
                      if r["cover"] is not None else "-",
                      f"{(r['rec'].get('replay_check') or {}).get('dev_p99_deg', float('nan')):.1f}"
                      if r["rec"].get("replay_check") else "-",
                      r["outcome"]])
        colours.append(_colour(r))
    # PAGINATE. The table was drawn on one sheet at loc="upper center" with no
    # row budget: at 68 runs it rendered 41 rows and silently dropped 27 — 40 %
    # of the day, the 21:23:45 cut among them — while overprinting the bar
    # chart's tick labels and the How-to-read text below it. One sheet per
    # ROWS_PER_PAGE rows, and the chart moves to its own.
    ROWS = 30
    pages = max(1, -(-len(cells) // ROWS))
    for pg in range(pages):
        lo, hi = pg * ROWS, min(len(cells), (pg + 1) * ROWS)
        f = fig if pg == 0 else _fig(
            pdf, f"Upload-channel runs — {period} (continued)",
            f"rows {lo + 1}-{hi} of {len(cells)}")
        axt = f.add_axes([0.03, 0.06, 0.94, 0.84])
        axt.axis("off")
        tab = axt.table(cellText=cells[lo:hi], colLabels=cols, loc="upper center",
                        cellLoc="center",
                        colWidths=[0.06, 0.04, 0.16, 0.16, 0.07, 0.07, 0.07, 0.06,
                                   0.08, 0.06, 0.07, 0.08])
        tab.auto_set_font_size(False)
        tab.set_fontsize(6.8)
        tab.scale(1, 1.05)
        for (i, j), cell in tab.get_celld().items():
            cell.set_edgecolor("#CBD5E1")
            if i == 0:
                cell.set_facecolor("#EEF1F5")
                cell.set_text_props(fontweight="bold", color="#1E293B")
            elif j == len(cols) - 1:
                cell.set_text_props(color=colours[lo + i - 1], fontweight="bold")
        if pg:
            pdf.savefig(f)
            plt.close(f)
    pdf.savefig(fig)
    plt.close(fig)
    # ---- the bars and the reading notes get their own sheet -----------------
    fig = _fig(pdf, f"Upload-channel runs — {period}",
               "replay time against the file's nominal time, every run")
    ax2 = fig.add_axes([0.16, 0.10, 0.46, 0.78])
    ys = np.arange(len(runs))
    vals = [r["file_pct"] or 0 for r in runs]
    ax2.barh(ys, vals, color=[_colour(r) for r in runs], alpha=0.85)
    ax2.axvline(100, color="#334155", ls="--", lw=1)
    ax2.set_yticks(ys)
    ax2.set_yticklabels([f"{r['when'][11:16]}  {r['slot']} {r['name'][:18]}" for r in runs], fontsize=5.5)
    ax2.invert_yaxis()
    ax2.set_xlabel("replay time as % of the file's nominal time ((points − 1) × 5.5545 ms)",
                   fontsize=8)
    ax2.tick_params(axis="x", labelsize=7)
    ax2.grid(axis="x", color="#E2E8F0")
    for s in ("top", "right"):
        ax2.spines[s].set_visible(False)
    fig.text(0.66, 0.86, "How to read", fontsize=9, fontweight="bold", color="#1E293B")
    fig.text(0.66, 0.83,
             "A COMPLETE run replays its whole file at the player's clock: the bar\n"
             "reaches ~100 % and the arm reaches 100 % of the path.\n\n"
             "A CUT run's bar stops short: the controller reported the program\n"
             "finished before the file's nominal time, with the arm mid-path.\n"
             "'period' is the arm's pace along the path (ms per commanded point\n"
             "traversed) — it reads the same 5.5–6.5 ms on cut runs: the player\n"
             "was at its normal clock and the program simply ended early.\n\n"
             "'pole' is the lift height the controller pushed, first→last frame.\n"
             "'devices': a flow set pole and hand from the plan; a single run\n"
             "left them as found.",
             fontsize=7.5, color="#334155", va="top", linespacing=1.4)
    pdf.savefig(fig)
    plt.close(fig)


def page_clock(pdf, runs):
    import matplotlib.pyplot as plt
    fig = _fig(pdf, "The period — ms per commanded point, every run",
               f"hardware envelope {ENV[0]}–{ENV[1]} ms/pt (F108b) shaded; nominal 5.5545 dashed.\n"
               "The period is the arm's pace along the commanded path (replay time ÷ points traversed), so it reads on cut runs too.\n"
               "Green = complete, red = cut, grey × = pace not measurable (the run stopped before 5 % of its file).\n"
               "Below: how much of the file had been traversed when the controller ended the program.")
    ax = fig.add_axes([0.07, 0.44, 0.9, 0.38])
    ax.axhspan(ENV[0], ENV[1], color="#DCFCE7", alpha=0.7, lw=0)
    ax.axhline(CLOCK_S * 1e3, color="#334155", ls="--", lw=1)
    xs = np.arange(len(runs))
    for i, r in enumerate(runs):
        if r.get("pace_ms"):
            ax.plot(i, r["pace_ms"], "o", color=_colour(r), ms=7)
            ax.annotate(f"{r['pace_ms']:.2f}", (i, r["pace_ms"]), textcoords="offset points",
                        xytext=(0, 7), ha="center", fontsize=6.5, color=_colour(r))
        else:
            ax.plot(i, 4.5, "x", color=GREY, ms=6)
    ax.set_xticks(xs)
    ax.set_xticklabels([f"{r['when'][11:16]}\n{r['slot']} {r['name'].replace('_right', '')[:12]}"
                        for r in runs], fontsize=5.6)
    for i, r in enumerate(runs):
        ax.text(i, 4.15, r["setting"].split(" · ")[0].replace(" (pre-policy)", "*"), ha="center",
                fontsize=5.2, color="#5B6B82")
    ax.set_ylabel("ms per point", fontsize=8)
    ax.tick_params(axis="y", labelsize=7)
    ax.set_ylim(4, 7.5)
    fig.text(0.97, 0.395, "* pre-policy = the 2026-09-01 acc-1.0 tracks", ha="right", fontsize=5.5, color="#5B6B82")
    ax.grid(axis="y", color="#E2E8F0")
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    # where the cut runs ended, at that pace
    ax2 = fig.add_axes([0.07, 0.10, 0.9, 0.25])
    for i, r in enumerate(runs):
        if r.get("end_phase") is not None and r["replay_s"]:
            ax2.bar(i, r["end_phase"] * 100, color=_colour(r), alpha=0.85, width=0.6)
    ax2.axhline(100, color="#334155", ls="--", lw=0.8)
    ax2.set_xticks(xs)
    ax2.set_xticklabels([""] * len(runs))
    ax2.set_ylabel("% of the file traversed\nwhen the program ended", fontsize=7)
    ax2.tick_params(axis="y", labelsize=7)
    ax2.grid(axis="y", color="#E2E8F0")
    for s in ("top", "right"):
        ax2.spines[s].set_visible(False)
    paces = [r["pace_ms"] for r in runs if r.get("pace_ms")]
    if paces:
        fig.text(0.07, 0.055,
                 f"Period on the {len(paces)} of {len(runs)} runs whose pace is measurable "
                 f"(past 5 % of the file): median {np.median(paces):.2f} ms/pt, range "
                 f"{min(paces):.2f}–{max(paces):.2f}; {sum(ENV[0] <= x <= ENV[1] for x in paces)} of {len(paces)} "
                 f"inside the envelope.",
                 fontsize=8, fontweight="bold", color="#1E293B")
        fig.text(0.07, 0.025,
                 # only cuts whose pace was actually measured belong in a
                 # sentence about pace; one 2026-09-04 cut stopped at 2 % with
                 # pace_ms None and a controller fault, and it was being quoted
                 # here as evidence that the pace was normal.
                 "Cut runs kept the same pace and ended at "
                 + (", ".join(f"{r['end_phase'] * 100:.0f}%" for r in runs
                              if r['cut'] and r.get('end_phase') and r.get('pace_ms'))
                    or "n/a")
                 + " of their files — the program ends early at normal pace."
                 + (f" {sum(1 for r in runs if r['cut'] and not r.get('pace_ms'))} further cut(s) "
                    f"stopped too early to measure a pace."
                    if any(r['cut'] and not r.get('pace_ms') for r in runs) else ""),
                 fontsize=8, color="#1E293B")
    pdf.savefig(fig)
    plt.close(fig)


def page_task(pdf, slot, runs):
    import matplotlib.pyplot as plt
    name = runs[0]["name"]
    fig = _fig(pdf, f"Task {name} — slot {slot} — {len(runs)} run(s)",
               " | ".join(f"{r['when'][11:16]} {r['setting']} → {r['outcome']}"
                          f"{f' at {r[chr(99)+chr(111)+chr(118)+chr(101)+chr(114)]*100:.0f}%' if r['cut'] and r['cover'] else ''}"
                          for r in runs))
    axs = [fig.add_axes(a) for a in ([0.06, 0.55, 0.42, 0.33], [0.55, 0.675, 0.42, 0.215],
                                     [0.06, 0.09, 0.42, 0.33], [0.55, 0.09, 0.42, 0.33])]
    for i, r in enumerate(runs):
        c = PALETTE[i % len(PALETTE)]
        lab = f"{r['when'][11:16]} {r['setting'].split(' · ')[0]} ({r['outcome']})"
        if r.get("prog") is not None:
            axs[0].plot(r["prog_t"], r["prog"], color=c, lw=1.2, label=lab)
            if r["cut"]:
                axs[0].plot(r["prog_t"][-1], r["prog"][-1], "x", color=RED, ms=8, mew=2)
            axs[2].plot(r["prog_t"], r["dev"], color=c, lw=0.9)
        if r.get("speed_w") is not None:
            axs[1].plot(r["tw"], np.abs(r["speed_w"]).max(1), color=c, lw=0.8)
        if r.get("lift") is not None:
            axs[3].plot(r["t_cap"], r["lift"], color=c, lw=1.2)
            if r["first_err"]:
                t_e = r["first_err"][0] + (r["t0"] - r["ts"][0])
                axs[3].axvline(t_e, color=c, ls=":", lw=0.9)
    r0 = runs[0]
    if r0.get("nom_prog") is not None:
        axs[0].plot(r0["prog_t"], r0["nom_prog"], color="#334155", ls="--", lw=1, label="nominal clock")
    axs[0].set_title("progress along the commanded path  (× = program ended here)", fontsize=8.5)
    axs[0].set_xlabel("s since replay start", fontsize=7)
    axs[0].set_ylabel("% of path", fontsize=7)
    axs[0].legend(fontsize=6, loc="lower right", frameon=False)
    axs[1].axhline(225, color=RED, ls="--", lw=0.8)
    axs[1].axhline(180, color="#B45309", ls="--", lw=0.8)
    axs[1].set_title("executed joint speed, max over joints (controller field); rated 180 / 225 dashed", fontsize=8.5)
    # per-joint max / mean |speed| table, one row per run, under the plot
    rows_ok = [r for r in runs if r.get("jmax") is not None]
    if rows_ok:
        tab_ax = fig.add_axes([0.55, 0.485, 0.42, 0.017 * (len(rows_ok) + 1)])
        tab_ax.axis("off")
        cells = [[r["when"][11:16]] + [f"{mx:.0f} / {av:.0f}" for mx, av in zip(r["jmax"], r["javg"])]
                 for r in rows_ok]
        tab = tab_ax.table(cellText=cells, colLabels=["run"] + [f"J{j + 1}" for j in range(7)],
                           loc="upper center", cellLoc="center",
                           colWidths=[0.10] + [0.128] * 7)
        tab.auto_set_font_size(False)
        tab.set_fontsize(5.6)
        tab.scale(1, 0.9)
        for (i, j), cell in tab.get_celld().items():
            cell.set_edgecolor("#CBD5E1")
            if i == 0:
                cell.set_facecolor("#EEF1F5")
                cell.set_text_props(fontweight="bold")
            elif j > 0 and rows_ok[i - 1]["jmax"][j - 1] > RATED[j - 1]:
                cell.set_text_props(color=RED, fontweight="bold")
        tab_ax.set_title("per joint: max / mean |speed| in deg/s over the replay (red = above the rating)",
                         fontsize=6.5, color="#334155", pad=2)
    axs[1].set_xlabel("s since replay start", fontsize=7)
    axs[1].set_ylabel("deg/s", fontsize=7)
    axs[2].set_title("distance to the commanded path (max joint, deg)", fontsize=8.5)
    axs[2].set_xlabel("s since replay start", fontsize=7)
    axs[2].set_ylabel("deg", fontsize=7)
    axs[3].set_title("pole height over the capture (dotted: first controller error code)", fontsize=8.5)
    axs[3].set_xlabel("s since capture start", fontsize=7)
    axs[3].set_ylabel("mm", fontsize=7)
    for a in axs:
        a.tick_params(labelsize=6.5)
        a.grid(color="#E2E8F0")
        for s in ("top", "right"):
            a.spines[s].set_visible(False)
    pdf.savefig(fig)
    plt.close(fig)


def page_run(pdf, r):
    import matplotlib.pyplot as plt
    rec = r["rec"]
    rep_txt = (f"replay {r['replay_s']:.2f} s ({r['file_pct'] or 0:.0f} % of file) · path {(r['cover'] or 0) * 100:.0f} %"
               if r["replay_s"] else "replay span unresolved (no departure latch) — whole capture shown")
    sub = (f"{r['when']} · {r['setting']} · {rec.get('points')} pts · {rep_txt} · "
           f"dev p50 {(rec.get('replay_check') or {}).get('dev_p50_deg', float('nan')):.2f}° "
           f"p99 {(rec.get('replay_check') or {}).get('dev_p99_deg', float('nan')):.2f}° · {r['outcome'].upper()}")
    fig = _fig(pdf, f"Run {r['when'][11:19]} — slot {r['slot']} {r['name']}", sub)
    T = r.get("track")
    if T is None or r.get("qw") is None:
        fig.text(0.1, 0.5, "no capture / the commanded track version could not be recovered", fontsize=10)
        pdf.savefig(fig)
        plt.close(fig)
        return
    tw, qw = r["tw"], r["qw"]
    n = len(T)
    idx = np.minimum((tw / CLOCK_S).astype(int), n - 1)
    for j in range(7):
        ax = fig.add_axes([0.05 + (j % 4) * 0.24, 0.55 if j < 4 else 0.28, 0.20, 0.20])
        ax.plot(tw, T[idx, j], color="#94A3B8", lw=1.0, label="commanded @ nominal clock")
        ax.plot(tw, qw[:, j], color=BLUE, lw=0.9, label="executed")
        ax.set_title(f"J{j + 1}", fontsize=8)
        ax.tick_params(labelsize=6)
        ax.grid(color="#E2E8F0")
        for s in ("top", "right"):
            ax.spines[s].set_visible(False)
        if j == 0:
            ax.legend(fontsize=5.5, frameon=False, loc="best")
    em = r.get("em") or {}
    ax = fig.add_axes([0.77, 0.28, 0.20, 0.20])
    if "tcp_linear_mps" in em:
        lin = em["tcp_linear_mps"]
        ax.plot(np.linspace(tw[0], tw[-1], len(lin)), lin, color="#7C3AED", lw=0.8, label="TCP m/s")
        ax.plot(np.linspace(tw[0], tw[-1], len(lin)), em["tcp_angular_rad_s"], color="#DB2777", lw=0.8,
                label="TCP rad/s")
        ax.legend(fontsize=5.5, frameon=False)
    ax.set_title("executed tool speed (controller waypoint)", fontsize=7.5)
    ax.tick_params(labelsize=6)
    ax.grid(color="#E2E8F0")
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    lines = []
    for key, label in (("controller_before", "before"), ("controller_after", "after"), ):
        c = rec.get(key)
        if isinstance(c, dict):
            a = c.get("arm_state") if isinstance(c.get("arm_state"), dict) else {}
            cs = c.get("collision_stage")
            cs = cs[1] if isinstance(cs, (list, tuple)) and len(cs) == 2 else cs
            lines.append(f"controller {label}: err {a.get('err_codes')} · collision stage {cs}")
    pre = rec.get("pre_run")
    if isinstance(pre, dict):
        p = pre.get("pole") or {}
        h = pre.get("hand") or {}
        lines.append(f"flow devices: pole {p.get('start_m')}→{p.get('final_m')} m in {p.get('t_s', 0):.2f} s; "
                     f"hand ANGLE_ACT {h.get('before')}→{h.get('after')}")
    else:
        lines.append("single run: devices left as found")
    if r.get("first_err"):
        lines.append(f"first controller error code {r['first_err'][1]} at {r['first_err'][0]:.2f} s into the replay "
                     "(the push latches the first code only)")
    if r.get("jmax") is not None:
        lines.append("executed joint speed over the replay, max / mean |deg/s|:  " +
                     "  ".join(f"J{j + 1} {mx:.0f}/{av:.0f}" for j, (mx, av) in enumerate(zip(r["jmax"], r["javg"])))
                     + "   (rated 180 J1-J2, 225 J3-J7)")
    fig.text(0.05, 0.20, "\n".join(lines), fontsize=7.5, color="#334155", va="top", linespacing=1.5)
    pdf.savefig(fig)
    plt.close(fig)


def _tcp_paths(r):
    """(commanded, executed) TCP positions in metres, or (None, None).

    Both through the URDF FK at the task's OWN ik_frame — the frame the stroke
    was authored against, so the overlay compares like with like.
    """
    import sys
    try:
        sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
        import cut_dossier as cd
    except Exception as exc:                                           # noqa: BLE001
        return None, None, f"cut_dossier unavailable ({exc!r})"
    # BY NAME, never by slot — see cut_dossier.manifest_row. Keyed by slot this
    # drew 21 of 116 records at another region's frame, a rigid 41-62 mm offset.
    row = cd.manifest_row(r.get("rec") or {})
    T = r.get("track")
    if row is None:
        return None, None, f"no manifest row for {(r.get('rec') or {}).get('name')!r}"
    if T is None:
        return None, None, "the commanded track could not be resolved"
    if r.get("qw") is None or not len(r["qw"]):
        return None, None, "no executed samples in the replay window"
    frame = row.get("ik_frame")
    try:
        Pc, _ = cd.fk_tool(T, r["rec"].get("side", "right"), frame)
        Pe, _ = cd.fk_tool(r["qw"][::4], r["rec"].get("side", "right"), frame)
    except Exception as exc:                                           # noqa: BLE001
        return None, None, f"FK at {frame} failed ({exc!r})"
    return Pc, Pe, frame


def page_cartesian(pdf, slot, runs):
    """Commanded vs executed TCP path for one task.

    Newton, 2026-09-04: the dominant-axes PLANE view takes priority — it is the
    one you can read a wipe off — with the isometric 3D beside it for the
    out-of-plane excursion. The plane is chosen from the commanded path's own
    extent, so each task gets the projection that actually shows its raster.
    """
    import matplotlib.pyplot as plt
    import numpy as np
    name = (runs[0]["name"] or "").replace("commode_c_", "")
    # Every run on this page is the SAME region (build() groups by name), so
    # one commanded curve is right — but take it from a run that resolved,
    # together with the frame it resolved AT, and say so on the page.
    Pc = Pe = frame = None
    why = "no run on this page produced a TCP path"
    for r in runs:
        Pc, Pe, tag = _tcp_paths(r)
        if Pc is not None:
            frame = tag
            break
        why = tag
    if Pc is None:
        _fig(pdf, f"Cartesian path — {name} — no plot",
             f"{len(runs)} run(s): {why}")
        pdf.savefig(); plt.close()
        return 1
    # Every executed path ONCE — this used to FK the whole set three times over
    # (main plane, isometric, out-of-plane section).
    execs = []
    for i, r in enumerate(runs):
        _, P, _tag = _tcp_paths(r)
        if P is not None:
            execs.append((r, P, PALETTE[i % len(PALETTE)]))
    ext = Pc.max(0) - Pc.min(0)
    order = list(np.argsort(-ext))
    a, b, c = order[0], order[1], order[2]           # dominant plane, then the thin axis
    axn = "XYZ"
    oop = (max(float(np.ptp(P[:, c])) for _, P, _ in execs) * 1000) if execs else 0.0
    slots = sorted({r["slot"] for r in runs if r.get("slot") is not None})
    fig = _fig(pdf, f"Cartesian path — {name} — slot{'s' if len(slots) > 1 else ''} "
                    f"{', '.join(str(x) for x in slots) or '?'}",
               f"commanded vs executed at {frame} · axes are URDF "
               f"{'R' if runs[0]['rec'].get('side', 'right') == 'right' else 'L'}_base_link "
               f"(the controller's base is this rotated 90 deg about Y) · dominant plane "
               f"{axn[a]}{axn[b]} · COMMANDED extent {ext[a]*1000:.0f} x {ext[b]*1000:.0f} mm, "
               f"out-of-plane {ext[c]*1000:.0f} mm commanded / {oop:.0f} mm executed")
    # -- the dominant plane, large, on the left --------------------------------
    ax = fig.add_axes([0.06, 0.10, 0.55, 0.76])
    ax.plot(Pc[:, a] * 1000, Pc[:, b] * 1000, color="#2563EB", lw=1.0,
            label="commanded", zorder=3)
    for r, P, col in execs:
        ax.plot(P[:, a] * 1000, P[:, b] * 1000, lw=0.7, alpha=0.75, color=col,
                label=f"executed {r['when'][11:19]} ({r['outcome']})", zorder=4)
    ax.set_xlabel(f"{axn[a]} mm")
    ax.set_ylabel(f"{axn[b]} mm")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=6, loc="best")
    ax.set_title("dominant plane", fontsize=8)
    # -- isometric, smaller, on the right --------------------------------------
    ax3 = fig.add_axes([0.64, 0.42, 0.33, 0.44], projection="3d")
    ax3.plot(Pc[:, 0] * 1000, Pc[:, 1] * 1000, Pc[:, 2] * 1000, color="#2563EB", lw=0.7)
    for _r, P, col in execs:
        ax3.plot(P[:, 0] * 1000, P[:, 1] * 1000, P[:, 2] * 1000,
                 lw=0.5, alpha=0.75, color=col)
    ax3.view_init(elev=22, azim=-58)                 # isometric-ish
    ax3.set_xlabel("X mm", fontsize=6)
    ax3.set_ylabel("Y mm", fontsize=6)
    ax3.set_zlabel("Z mm", fontsize=6)
    ax3.tick_params(labelsize=5)
    ax3.set_title("isometric", fontsize=8)
    # -- the out-of-plane section, bottom right --------------------------------
    ax2 = fig.add_axes([0.68, 0.10, 0.28, 0.24])
    ax2.plot(Pc[:, a] * 1000, Pc[:, c] * 1000, color="#2563EB", lw=0.8)
    for _r, P, col in execs:
        ax2.plot(P[:, a] * 1000, P[:, c] * 1000, lw=0.6, alpha=0.75, color=col)
    ax2.set_xlabel(f"{axn[a]} mm", fontsize=7)
    ax2.set_ylabel(f"{axn[c]} mm (out of plane)", fontsize=7)
    ax2.tick_params(labelsize=6)
    ax2.set_aspect("equal", adjustable="datalim")
    ax2.grid(alpha=0.3)
    pdf.savefig(fig)
    plt.close(fig)
    return 1


def page_notes(pdf):
    import matplotlib.pyplot as plt
    fig = _fig(pdf, "Notes — what the numbers are")
    fig.text(0.05, 0.88,
             "file %  — replay time ÷ (points − 1) × 5.5545 ms: how much of the file's nominal time the program ran.\n\n"
             "period  — the arm's pace along the commanded path, replay time ÷ points traversed (ms per point). The player's\n"
             "            nominal clock is 5.5545 ms/pt (F108); 5.4–6.2 ms/pt is the hardware envelope (F108b). Measured on every run\n"
             "            with a departure latch — cut runs included: they keep the pace and end early.\n\n"
             "path %  — how far along the commanded path the arm got (nearest commanded point, forward-windowed to survive raster\n"
             "            self-crossings). 100 % with 'complete' means the arm reached the last row.\n\n"
             "cut     — the controller reported the program finished (run_state 0) while the arm was still mid-path and braking;\n"
             "            no error code is read back afterwards (F111). The cut position is reproducible per track.\n\n"
             "dev p99 — 99th percentile of the distance from each executed sample to the nearest commanded point (deg, max joint).\n"
             "            Mostly servo lag at speed, not path error.\n\n"
             "executed joint speed — the controller's own joint_speed field (0.01 deg/s units), not a derivative of positions.\n"
             "            Rated: 180 deg/s on J1–J2, 225 on J3–J7; the emitted tracks stay 3 % under it (vel 0.97).\n\n"
             "pole    — lift_state.height pushed by the controller (mm). A single `run` never commands it; a `flow` sets it from\n"
             "            the plan (pre_stroke) before each program, and the hand over Modbus (ANGLE_ACT is its only feedback —\n"
             "            the push's hand section is always zero in modbus mode).\n\n"
             "setting — acc-scale × vel-scale of the emit and the run speed (global ratio). Runs are only comparable within a setting.\n"
             "            'pre-policy' records predate the per-row acc_scale (2026-09-01 tracks, acc 1.0).\n\n"
             "Sources: Resource/runs/tracks/<date>/run_slot*.json + _frames.ndjson (verbatim UDP push at 5 ms),\n"
             "Resource/tracks/commode_c/MANIFEST.json, plan/FINDINGS.md F108–F111.\n\n"
             "REPRODUCE THIS DOCUMENT, offline, no robot:\n"
             "    python3 tools/track_report.py report                    # latest day with records\n"
             "    python3 tools/track_report.py report --day 2026-09-02   # a day; --from A --to B for a period\n"
             "    python3 tools/track_report.py report --setting 'acc 2' --slots 72 73   # a subset\n"
             "Needs python3 with numpy + matplotlib (pip install numpy matplotlib); git is used only to recover a\n"
             "track version that has since been re-emitted. Output: Resource/reports/track_runs/<period>.pdf.",
             fontsize=8, color="#334155", va="top", linespacing=1.5)
    pdf.savefig(fig)
    plt.close(fig)


# ------------------------------------------------------------------ build
def build(rec_paths, out_path: pathlib.Path, period: str, cache: pathlib.Path) -> dict:
    import matplotlib
    matplotlib.use("Agg")
    from matplotlib.backends.backend_pdf import PdfPages
    runs = [analyse(p, cache) for p in rec_paths]
    runs.sort(key=lambda r: r["when"])
    groups = {}
    for r in runs:
        groups[r["setting"]] = groups.get(r["setting"], 0) + 1
    groups = sorted(groups.items(), key=lambda kv: -kv[1])
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with PdfPages(out_path) as pdf:
        page_overview(pdf, runs, period, groups)
        page_clock(pdf, runs)
        # BY (slot, REGION) — a slot alone is not a task. Slots are renumbered
        # whenever the schedule changes, so bucketing by slot put two different
        # regions on one page: "bowl_inside_ring — slot 76" drew that region's
        # commanded path under five seat_ring_bottom traces sitting a median
        # 174-193 mm away, while a correctly-grouped run sits at 1.8-3.1 mm.
        by_task = {}
        for r in runs:
            by_task.setdefault((r["slot"], r.get("name")), []).append(r)
        n_cart = 0
        for key in sorted((k for k in by_task if k[0] is not None),
                          key=lambda k: (k[0], k[1] or "")):
            page_task(pdf, key[0], by_task[key])
            n_cart += page_cartesian(pdf, key[0], by_task[key])
        for r in runs:
            page_run(pdf, r)
        page_notes(pdf)
        d = pdf.infodict()
        d["Title"] = f"Upload-channel runs {period}"
        d["Author"] = "tools/track_report.py report"
        d["CreationDate"] = _dt.datetime.now()
    return dict(runs=len(runs), pages=3 + len(by_task) + n_cart + len(runs), groups=groups,
                complete=sum(r["outcome"] == "complete" for r in runs),
                cut=sum(r["outcome"] == "cut" for r in runs))
