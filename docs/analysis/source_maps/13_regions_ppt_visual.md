# 13 · Regions deck (Toilet_regions_V1.pptx) — visual region / task catalogue for ALIX Studio Lite

Scope: R1 (region-wise drag-and-teach with cleaning time, auto-advance, Regions tab with the PPT, dashboard of taught / not-taught raw `.txt`) and R7 (per-region pole height + hand position before teaching).
All sources were read-only. Everything marked **(inferred)** is a reasoned guess, and the reason is given next to it.

---

## 0. Sources, conventions, artefacts

**Sources**

| What | Where | Notes |
|---|---|---|
| Regions deck | `/home/newtonjeri/alix_studio_lite_ws/Toilet_regions_V1.pptx` | 25 slides, 16:9 (13.333 × 7.5 in). Created 2026-10-07, last modified 2026-10-08 (revision 30), author "Admin". No speaker notes, no hidden slides. `slide1.xml … slide25.xml` follow presentation order. |
| V6 merged trajectory package | `/home/newtonjeri/alix_studio_lite_ws/Trajectories/DUAL_ARM_V6_PROGRAM_IDS_70_SERIES/` | Dated 2026-10-05, so it is **older than the deck**. It contains README, 15 program `.txt` files (LEFT/ and RIGHT/), 11 bridge files (BRIDGES_TEST_ONLY/), JSON/ and `*_FINAL.json`. |
| Lite requirements | `/home/newtonjeri/alix_studio_lite_ws/alix_studio_lite/docs/reference/ALIX_Studio_Lite_requirement_inputs.txt` | Item 1 is R1 and item 7 is R7. |
| Used only for cross-reference | full-Studio `regions.yaml` (`alix_ws/src/alix/ros/orchestration/alix_tasks/config/commode_cleaning/commode_c/models/corrected/`), `commode_c/scene.yaml`, URDF `alix_ws/src/alix/core/python/alix_workspace/src/alix_workspace/urdf/alix.urdf` | Used for region-name cross-reference and for the rough FK check in §6.2. |

**Artefacts written (scratchpad)**
- `pptx_render/hi-01.png … hi-25.png`: every slide at 110 dpi (1467 × 825 px). The existing `slide-NN.png` files are 60 dpi.
- `pptx_render/crop-sNN-T<task>-{L|R|LR}.png`: 37 per-arm crops at 150 dpi, listed in Appendix B.
- `scripts/pptx_detail.py` (shapes, positions, colours, text), `scripts/overlay_norm.py` (overlay boxes normalised to the source media), `scripts/arm_crops.py` (crop boxes and renders), `scripts/v6_fk_segments.py` and `scripts/v6_fk_segments_pct.py` (rough FK of the V6 programs).

**Deck conventions (checked in the XML)**
- **LEFT/RIGHT means the robot's left/right when it faces the toilet**, with the bowl front toward the robot.
  - In the top views, "LEFT HALF" overlays sit on the image-left half.
  - The unflipped side photo has the wall on the right and shows the toilet's **right** side.
  - Every LEFT-side view is the same photo **mirrored**. The "RALTO" logo reads "OTJAR", so the bracket plate and cable in the left views are mirrored copies, not the real left side.
- **LEFT ARM is blue.** Fill `#4472C4` at 18 % opacity, outline `#7A73FF` at 29 %. **RIGHT ARM is orange.** Fill `#ED7D31` at 20 %, solid `#ED7D31` outline. Label text is black.
- Motion arrows (tasks 3, 4, 7) and the PAGE 04 leader arrows use theme accent1 `#5B9BD5`.
- Exceptions to the box style:
  - The FL-R09 (10A/10B) and R-T05 (12A/12B) labels are outline-only boxes outside the picture, with separate filled marks on the picture.
  - The L-R10 (11A/11B) label boxes sit outside the picture.
  - Task 15 is plain text with no boxes.
  - Task 14 has no "LEFT ARM"/"RIGHT ARM" captions.
- Page labels: slides 1–4 say "PAGE 02 … PAGE 05". There is no PAGE 01 in the file. Task slides (5–25) carry only "TASK: n".

**Deck media that Lite can reuse directly** (`ppt/media`; renders are 1448 × 1086 px unless noted)

| Media | Content |
|---|---|
| image1 | Lid-closed top view |
| image2 | Lid-open top view |
| image3 | Seat-open top view |
| image4, image22 | Front view, lid closed |
| image5, image25 | Right-side view, lid closed (mirrored for the left) |
| image6.jpeg, image16.jpeg | Photos of the hand-hatched region maps. Stored rotated 90°. |
| image20, image21, image23 | Photos of the real mock-up for tasks 3, 4 and 7 (a human hand demonstrates) |
| image24 | Drain brush in its holder |
| image26 | Blue nitrile glove |
| image7–15, image17–19 | Pasted label snippets of the region maps |

---

## 1. State slides and region maps

### Slide 1 · PAGE 02 · fixture states (top views, product renders)

| Caption on slide | What is visible | Lite state code |
|---|---|---|
| STATE: LID COVER CLOSE TOP VIEW | Lid and seat down. Behind the lid is the rear deck ("hinge plate") with two steel hinge straps and round hinge bosses. | `CLOSED` |
| STATE: LID COVER OPEN TOP VIEW | Lid raised (underside with 2 round bumpers). Seat ring down. Bowl opening and trap visible. | `LID_OPEN` (seat down) |
| STATE: SEAT COVER OPEN TOP VIEW | Lid and seat raised. The seat underside shows 4 oval bumpers and a small green dot at lower-left. Rim, inner rim, bowl and drain are visible. | `SEAT_OPEN` (lid and seat up) |

### Slide 2 · PAGE 03 · exterior views
- **STATE: BOWL OUTER SIDE VIEW.** The right side of a back-to-wall bowl ("RALTO"), lid closed. Also visible: a wood-look wall panel with a black frame, two vertical steel straps above the rear deck, a white L-shaped steel plate at the lower rear, and a white cable down the wall.
- **STATE: BOWL OUTER FRONT VIEW.** The tapered pedestal from the front, lid and seat closed.

### Slide 3 · PAGE 04 · named region map (top views)
This slide is a photographed print. The regions are hatched in pencil and the labels are pasted on. There are three top views (closed / lid open / seat open).

| Region (label) | View / state | Hatched area | Later split in the task slides |
|---|---|---|---|
| **Hinge** | closed | Rear-deck strip behind the lid, both hinge straps included | R-R02 RIGHT HINGE / LEFT HINGE, each with a thin unlabelled strip along that side's lid edge |
| **Lid Top** | closed | The whole closed lid | R-R01 PART 01 (rear half) / PART 02 (front half), both on the right arm |
| **Lid Underside** | lid open | The whole raised lid underside | LEFT HALF (L) / RIGHT HALF (R) |
| **Seat Top** | lid open | The seat ring | PART 1 (left half) / PART 2 (right half), both on the left arm |
| **Seat Underside** | seat open | The raised seat ring underside | LEFT HALF (L) / RIGHT HALF (R) |
| **Rim Top** | seat open | The ceramic rim face | PART 1 (left) / PART 2 (right), both on the left arm |
| **Inner Rim** | seat open | Thin inner lip between the rim top and the bowl wall (leader arrow) | PART 1 (left half) / PART 2 (right half), left arm, code "FL-R09" |
| **Bowl Inside** | seat open | Bowl wall (leader arrow) | PART 1 (left of centre) / PART 2 (right of centre), left arm |
| **Drain Hole** | seat open | Trap opening at the bottom, a small solid patch (leader arrow) | R-T05 PART 1 / PART 2, same area both times, right arm with brush |

### Slide 4 · PAGE 05 · exterior region map (side + front)
This is a photographed print on reused paper; a faint table shows through, but it is not content.

| Region | Area | Task-slide split |
|---|---|---|
| **Side Exterior** | Upper band of the bowl side, from under the rim to mid-height | L-R02 SIDE EXTERIOR LEFT / SIDE EXTERIOR RIGHT |
| **Base** | Lower band, labelled on both the side view (around the bracket plate) and the front view | L-R01 BASE LEFT / BASE RIGHT |
| **Pedestal Upper** | Upper band of the front | L-R03 PEDESTAL UPPER LEFT / PEDESTAL UPPER RIGHT |

**Not on either map:** FLOOR SPILLAGE (tasks 8B, 9, 12A, 12B) and the lid-side-edge strips attached to the hinge tasks.

**In total:** 12 named map regions, plus floor spillage (13 region families), plus mechanism, tool and glove actions.

---

## 2. Slide-by-slide observations (task slides 5–25)

Overlay positions are given relative to the picture they sit on.

| Slide | Task | Fixture shown → required | LEFT ARM (label exactly as printed; overlay) | RIGHT ARM (label exactly as printed; overlay) | Type | Other visible details |
|---|---|---|---|---|---|---|
| 5 | 1A | Mirrored side view + lid-closed top view → `CLOSED` | **L-R01 BASE LEFT**: lower band of the bowl side including the steel bracket plate | **R-R01 LID TOP PART 01**: rear half of the closed lid (hinge edge to middle) | surface ‖ surface | The right top-view picture is uncropped (wide white margins). |
| 6 | 2A | same → `CLOSED` | **L-R02 SIDE EXTERIOR LEFT**: upper side band, front edge to wall, above the plate | **R-R01 LID TOP PART 02**: front half of the lid down to the front tip | surface ‖ surface | — |
| 7 | 1C | Front view (rotated −4°) + lid-closed top view → `CLOSED` | **L-R03 PEDESTAL UPPER LEFT**: upper-left quadrant of the front | **R-R02 RIGHT HINGE**: right half of the rear deck with the right strap, **plus an unlabelled thin vertical strip** along the right edge of the lid/seat stack | surface ‖ surface | The strip is probably the lid/seat side edge **(inferred)**. The full Studio has a separate `lid_side` region. |
| 8 | 2 | Top view + right side view → `CLOSED` | **LEFT HINGE** (no code): left half of the rear deck **plus an unlabelled thin strip** along the left lid edge | **SIDE EXTERIOR RIGHT** (no code): upper side band | surface ‖ surface | Neither arm has a code. |
| 9 | 3 | Photo of the real mock-up (orange/wood floor, wall panel, straps). A hand lifts lid **and** seat together at the front-left edge. Blue arrow ↑. `CLOSED` → both up | **OPEN LID + SEAT (MASTER)** | **OPEN LID + SEAT (SUPPORT)** | mechanism (master/support) | No codes, no overlays on the photo. The grip point (front-left edge) is visible. |
| 10 | 4 | Photo: lid fully up, a hand lowers the seat from the left. Arrow ↓. Both up → `LID_OPEN` | **CLOSE SEAT ONLY (MASTER)** | **CLOSE SEAT ONLY SUPPORT** (no parentheses) | mechanism | No codes. |
| 11 | 5A | One lid-open top view shared by both arms → `LID_OPEN` | **SEAT TOP PART 1**: left half of the seat ring | **LID UNDERSIDE RIGHT HALF**: right half of the raised lid | surface ‖ surface | Arm captions sit beside the overlays, not under separate pictures. |
| 12 | 5B | Lid-open top view + front view → `LID_OPEN` | **SEAT TOP PART 2**: right half of the seat ring (**right half done by the LEFT arm**) | **PEDESTAL UPPER RIGHT**: upper-right quadrant of the front | surface ‖ surface | The front picture (image22) shows the lid closed. Exterior regions do not depend on fixture state. |
| 13 | 6 | Lid-open top view + right side view → `LID_OPEN` | **LID UNDERSIDE LEFT HALF** | **BASE RIGHT**: lower band including the plate (rotated −3°) | surface ‖ surface | — |
| 14 | 7 | Photo: lid up, a hand lifts the seat from the front underside. Arrow ↑. `LID_OPEN` → `SEAT_OPEN` | **L-T03 OPEN SEAT (MASTER)** | **R-T03 OPEN SEAT (SUPPORT)** | mechanism | The only mechanism step with codes. |
| 15 | 8A | One seat-open top view shared by both arms → `SEAT_OPEN` | **RIM TOP PART 1**: left half of the rim, full length | **SEAT UNDERSIDE RIGHT HALF** | surface ‖ surface | — |
| 16 | 8B | Seat-open top view + right side view → `SEAT_OPEN` | **RIM TOP PART 2**: right half of the rim (**LEFT arm**) | **FLOOR SPILLAGE PART 1**: floor strip along the rear-right base near the plate (rotated −7°) | surface ‖ surface (floor) | The right floor label does not say "RIGHT". |
| 17 | 9 | same → `SEAT_OPEN` | **SEAT UNDERSIDE LEFT HALF** | **FLOOR SPILLAGE PART 2**: floor patch in front of the bowl, front-right (rotated −34°) | surface ‖ floor | — |
| 18 | 10A | Seat-open top view + brush photo → `SEAT_OPEN` | **FL-R09 INNER RIM PART 1**: 5 thin blue bars trace the left half of the inner rim (rear-centre, rear-left corner, left side, front-left corner, front-centre). The label is an outline box outside the picture. | **R-T04 PICK DRAIN BRUSH** | surface ‖ tool | Code prefix "FL-" is used only here. |
| 19 | 10B | same → `SEAT_OPEN` | **FL-R09 INNER RIM PART 2**: 4 bars on the right half (rear-right corner, right side, front-right corner, front-centre) | **R-T04 PICK DRAIN BRUSH** (repeated) | surface ‖ tool | The front-centre bar appears in both parts; the rear-centre bar only in part 1. The R-T04 repeat could be a copy-paste or a deliberate long pick. |
| 20 | 11A | same → `SEAT_OPEN` | **L-R10 BOWL INSIDE PART 1**: box over the bowl interior left of centre (rim edge to trap). The label sits outside, 12 pt. | **HOLD BRUSH OUTSIDE** (no code) | surface ‖ idle | The right arm waits outside the bowl while the left cleans it. |
| 21 | 11B | same → `SEAT_OPEN` | **L-R10 BOWL INSIDE PART 2**: right of centre | **HOLD BRUSH OUTSIDE** | surface ‖ idle | — |
| 22 | 12A | Mirrored side view + seat-open top view → `SEAT_OPEN` | **L-R11 FLOOR SPILLAGE LEFT PART 1**: floor strip along the rear-left base near the plate (rotated +7°) | **R-T05 CLEAN DRAIN HOLE PART1**: small filled orange box on the trap opening plus an outline-only label | floor ‖ tool-cleaning | "PART1" has no space. |
| 23 | 12B | same → `SEAT_OPEN` | **L-R11 FLOOR SPILLAGE LEFT PART 2**: front-left floor (rotated 46°) | **R-T05 CLEAN DRAIN HOLE PART 2**: **identical box** to part 1 | floor ‖ tool-cleaning | The drain-hole "parts" look like two passes, not two areas. |
| 24 | 13 (left half of slide) | Glove photo + brush photo | **DROP GLOVE** (no code) | **R-T06 RETURN DRAIN BRUSH AND DROP GLOVE** | glove ‖ tool + glove | A black vertical bar divides the slide. |
| 24 | 14 (right half of slide) | One glove photo | **L-T04 LEFT GLOVE CHANGE** | **R-T07 RIGHT GLOVE CHANGE** | glove ‖ glove | No LEFT/RIGHT ARM captions; the arm is implied by the code and colour. |
| 25 | 15 | Seat-open top view → `SEAT_OPEN` → `CLOSED` | **L-T05 CLOSE SEAT + LID (MASTER)** | **R-T09 CLOSE SEAT + LID (SUPPORT)** | mechanism | Plain text, no boxes. **R-T08 never appears in the deck.** |

**Choreography the deck implies (inferred from the pairings):**
- One arm works high or at the back while the other works low or at the front.
- Inside the bowl, only one arm works at a time. The right arm fetches and holds the brush while the left arm cleans the rim and bowl.
- Then the left arm goes to the floor while the right arm brushes the drain.

---

## 3. Catalogue (normalised; deck order = 22 dual-arm steps)

- **Type** is one of: `surface` (wipe a region), `mechanism` (lid/seat; MASTER/SUPPORT), `tool` (brush), `glove`, `idle` (hold).
- **IDs in bold** are proposed Lite IDs (§5).
- "code: inferred" means the deck prints no code, and the number fills a gap in the deck's own sequence.

| Step | Deck task (slide) | LEFT arm: ID · deck label · type | RIGHT arm: ID · deck label · type | Required fixture state | Notes |
|---|---|---|---|---|---|
| 1 | 1A (s5) | **L-R01** · L-R01 BASE LEFT · surface | **R-R01-P1** · R-R01 LID TOP PART 01 (rear half) · surface | `CLOSED` | Base is state-independent. The deck teaches it in the closed phase. |
| 2 | 2A (s6) | **L-R02** · L-R02 SIDE EXTERIOR LEFT · surface | **R-R01-P2** · R-R01 LID TOP PART 02 (front half) · surface | `CLOSED` | — |
| 3 | 1C (s7) | **L-R03** · L-R03 PEDESTAL UPPER LEFT · surface | **R-R02** · R-R02 RIGHT HINGE (+ right lid-edge strip) · surface | `CLOSED` | — |
| 4 | 2 (s8) | **L-R04** · LEFT HINGE (+ left lid-edge strip) · surface · code: inferred | **R-R03** · SIDE EXTERIOR RIGHT · surface · code: inferred | `CLOSED` | — |
| 5 | 3 (s9) | **L-T01** · OPEN LID + SEAT (MASTER) · mechanism · code: inferred | **R-T01** · OPEN LID + SEAT (SUPPORT) · mechanism · code: inferred | `CLOSED` → lid + seat up | Master grips the front-left edge (photo). |
| 6 | 4 (s10) | **L-T02** · CLOSE SEAT ONLY (MASTER) · mechanism · code: inferred | **R-T02** · CLOSE SEAT ONLY SUPPORT · mechanism · code: inferred | up → `LID_OPEN` | — |
| 7 | 5A (s11) | **L-R05-P1** · SEAT TOP PART 1 (left half) · surface · code: inferred | **R-R04** · LID UNDERSIDE RIGHT HALF · surface · code: inferred | `LID_OPEN` | Shared picture for both arms. |
| 8 | 5B (s12) | **L-R05-P2** · SEAT TOP PART 2 (right half) · surface | **R-R05** · PEDESTAL UPPER RIGHT · surface · code: inferred | `LID_OPEN` | — |
| 9 | 6 (s13) | **L-R06** · LID UNDERSIDE LEFT HALF · surface · code: inferred | **R-R06** · BASE RIGHT · surface · code: inferred | `LID_OPEN` | — |
| 10 | 7 (s14) | **L-T03** · L-T03 OPEN SEAT (MASTER) · mechanism | **R-T03** · R-T03 OPEN SEAT (SUPPORT) · mechanism | `LID_OPEN` → `SEAT_OPEN` | V6 runs this as 2 alternating sub-steps (§6). |
| 11 | 8A (s15) | **L-R07-P1** · RIM TOP PART 1 (left) · surface · code: inferred | **R-R07** · SEAT UNDERSIDE RIGHT HALF · surface · code: inferred | `SEAT_OPEN` | Shared picture for both arms. |
| 12 | 8B (s16) | **L-R07-P2** · RIM TOP PART 2 (right) · surface | **R-R08-P1** · FLOOR SPILLAGE PART 1 (right, rear) · surface · code: inferred | `SEAT_OPEN` | — |
| 13 | 9 (s17) | **L-R08** · SEAT UNDERSIDE LEFT HALF · surface · code: inferred | **R-R08-P2** · FLOOR SPILLAGE PART 2 (right, front) · surface | `SEAT_OPEN` | — |
| 14 | 10A (s18) | **L-R09-P1** · FL-R09 INNER RIM PART 1 · surface | **R-T04** · R-T04 PICK DRAIN BRUSH · tool | `SEAT_OPEN` | — |
| 15 | 10B (s19) | **L-R09-P2** · FL-R09 INNER RIM PART 2 · surface | **R-T04** (repeated label) · tool | `SEAT_OPEN` | Confirm whether 10B should read HOLD BRUSH OUTSIDE. |
| 16 | 11A (s20) | **L-R10-P1** · L-R10 BOWL INSIDE PART 1 · surface | **R-T10** · HOLD BRUSH OUTSIDE · idle · code: new | `SEAT_OPEN` | The idle time equals the left arm's duration. |
| 17 | 11B (s21) | **L-R10-P2** · L-R10 BOWL INSIDE PART 2 · surface | **R-T10** (continued) · idle | `SEAT_OPEN` | — |
| 18 | 12A (s22) | **L-R11-P1** · L-R11 FLOOR SPILLAGE LEFT PART 1 (rear) · surface | **R-T05-P1** · R-T05 CLEAN DRAIN HOLE PART1 · tool-cleaning | `SEAT_OPEN` | — |
| 19 | 12B (s23) | **L-R11-P2** · L-R11 FLOOR SPILLAGE LEFT PART 2 (front) · surface | **R-T05-P2** · R-T05 CLEAN DRAIN HOLE PART 2 · tool-cleaning | `SEAT_OPEN` | Same drain area as P1, so probably a second pass. |
| 20 | 13 (s24) | **L-T06** · DROP GLOVE · glove · code: new | **R-T06** · R-T06 RETURN DRAIN BRUSH AND DROP GLOVE · tool + glove | any (deck phase: `SEAT_OPEN`) | Needs a glove bin location. |
| 21 | 14 (s24) | **L-T04** · L-T04 LEFT GLOVE CHANGE · glove | **R-T07** · R-T07 RIGHT GLOVE CHANGE · glove | any | "Change" probably means putting on a fresh glove, since the old one was dropped in step 20. |
| 22 | 15 (s25) | **L-T05** · L-T05 CLOSE SEAT + LID (MASTER) · mechanism | **R-T09** · R-T09 CLOSE SEAT + LID (SUPPORT) · mechanism | `SEAT_OPEN` → `CLOSED` | Final state is all closed. |

**Totals:**
- 22 steps per arm; 44 arm-steps.
- Unique teachable units: **42** (22 left; 20 right, because R-T04 and R-T10 each span two steps).
- 13 region families and 4 action families (mechanism, tool, glove, idle).

**Cross-reference to the full ALIX Studio `regions.yaml`** (for interoperability only; the region IDs are full-Studio `region_id`):

| Deck family | Full-Studio region |
|---|---|
| LID TOP | `toplid` (0) |
| SIDE EXTERIOR | `side` (1) |
| SEAT TOP | `seat_ring` (3) |
| SEAT UNDERSIDE | `seat_ring_bottom` (4) |
| RIM TOP | `top` (6) |
| BOWL INSIDE | `bowl_inside_ring` (9) / `_front` (7) / `_back` (8) |
| LID UNDERSIDE | `bottomlid` (10) |
| PEDESTAL UPPER | `front` (11) |
| HINGE | `hinge_area` (13), plus the edge strip ≈ `lid_side` (21) |
| INNER RIM | `bowl_inside_rim` (15) |
| Mechanisms | `lid_open` (2) and `seat_open` (5); lid_close is a session phase |
| Gloves | glove_wear / glove_remove session phases |
| BASE, DRAIN HOLE, FLOOR SPILLAGE, brush handling | no full-Studio counterpart |

The fixture-state groups match the full Studio: `CLOSED` = EXT, `LID_OPEN` = INT-A, `SEAT_OPEN` = INT-B.

---

## 4. Per-arm ordered lists (deck order)

The trailing tag is the V6 status, explained in §6. Its values are COVERED, MERGED, PARTIAL, OTHER-ARM and NONE.

**LEFT arm (22)**
1. L-R01 BASE LEFT · `CLOSED` · COVERED (V6 L P3)
2. L-R02 SIDE EXTERIOR LEFT · `CLOSED` · COVERED (L P5)
3. L-R03 PEDESTAL UPPER LEFT · `CLOSED` · COVERED (L P4)
4. L-R04 LEFT HINGE (+ edge strip) · `CLOSED` · COVERED (L P2)
5. L-T01 OPEN LID + SEAT (MASTER) · NONE (V6 opens the lid by hand)
6. L-T02 CLOSE SEAT ONLY (MASTER) · NONE
7. L-R05-P1 SEAT TOP left half · `LID_OPEN` · COVERED (L P7)
8. L-R05-P2 SEAT TOP right half · `LID_OPEN` · OTHER-ARM (V6 right P6)
9. L-R06 LID UNDERSIDE LEFT HALF · `LID_OPEN` · COVERED (L P6)
10. L-T03 OPEN SEAT (MASTER) · COVERED (L P8 + P9: M3_OPNST01/02)
11. L-R07-P1 RIM TOP left · `SEAT_OPEN` · COVERED (L P11)
12. L-R07-P2 RIM TOP right · OTHER-ARM (V6 right P12)
13. L-R08 SEAT UNDERSIDE LEFT HALF · COVERED (L P12)
14. L-R09-P1 INNER RIM left · COVERED (L P10)
15. L-R09-P2 INNER RIM right · OTHER-ARM (V6 right P11)
16. L-R10-P1 BOWL INSIDE part 1 · MERGED (L P13 R15_BUPFUL covers P1 and P2 in one file)
17. L-R10-P2 BOWL INSIDE part 2 · MERGED (same file)
18. L-R11-P1 FLOOR SPILLAGE LEFT part 1 · NONE
19. L-R11-P2 FLOOR SPILLAGE LEFT part 2 · NONE
20. L-T06 DROP GLOVE · NONE
21. L-T04 LEFT GLOVE CHANGE · NONE
22. L-T05 CLOSE SEAT + LID (MASTER) · NONE

**RIGHT arm (22 steps, 20 units)**
1. R-R01-P1 LID TOP rear half · `CLOSED` · PARTIAL (V6 R P4 R2_LIDTOP is the *right half* of the lid; the left half is V6 L P1)
2. R-R01-P2 LID TOP front half · `CLOSED` · PARTIAL (same file)
3. R-R02 RIGHT HINGE (+ edge strip) · `CLOSED` · COVERED (R P5 R10_HNGR)
4. R-R03 SIDE EXTERIOR RIGHT · `CLOSED` · COVERED (R P3)
5. R-T01 OPEN LID + SEAT (SUPPORT) · NONE
6. R-T02 CLOSE SEAT ONLY (SUPPORT) · NONE
7. R-R04 LID UNDERSIDE RIGHT HALF · `LID_OPEN` · COVERED (R P7)
8. R-R05 PEDESTAL UPPER RIGHT · COVERED (R P2; V6 does it in the closed phase)
9. R-R06 BASE RIGHT · COVERED (R P1; V6 does it in the closed phase)
10. R-T03 OPEN SEAT (SUPPORT) · COVERED (R P8 + P9: M3_ASOPN01/02)
11. R-R07 SEAT UNDERSIDE RIGHT HALF · `SEAT_OPEN` · COVERED (R P10)
12. R-R08-P1 FLOOR SPILLAGE right/rear · MERGED (R P13 R_SPILLAGE covers P1 and P2)
13. R-R08-P2 FLOOR SPILLAGE right/front · MERGED (same file)
14. R-T04 PICK DRAIN BRUSH (steps 14–15) · NONE
15. R-T10 HOLD BRUSH OUTSIDE (steps 16–17) · NONE (an idle pose, but it still needs a taught hold pose)
16. R-T05-P1 CLEAN DRAIN HOLE pass 1 · NONE
17. R-T05-P2 CLEAN DRAIN HOLE pass 2 · NONE
18. R-T06 RETURN DRAIN BRUSH AND DROP GLOVE · NONE
19. R-T07 RIGHT GLOVE CHANGE · NONE
20. R-T09 CLOSE SEAT + LID (SUPPORT) · NONE

---

## 5. Naming inconsistencies and a normalised ID scheme

### 5.1 Inconsistencies found in the deck
1. **Code prefix "FL-R09"** (INNER RIM, 10A/10B). Every other left region uses `L-Rnn`, and L-R10 and L-R11 follow it, so FL-R09 is almost certainly **L-R09**. Keep "FL-R09" as an alias.
2. **21 of 44 arm-step labels carry no code** (10 left, 11 right).
   - Region labels without a code: LEFT HINGE, SIDE EXTERIOR RIGHT, SEAT TOP PART 1/2, LID UNDERSIDE RIGHT/LEFT HALF, PEDESTAL UPPER RIGHT, BASE RIGHT, RIM TOP PART 1/2, SEAT UNDERSIDE RIGHT/LEFT HALF, FLOOR SPILLAGE PART 1/2.
   - Action labels without a code: OPEN LID + SEAT (MASTER/SUPPORT), CLOSE SEAT ONLY (MASTER)/SUPPORT, HOLD BRUSH OUTSIDE (×2), DROP GLOVE.
3. **The gaps in the deck's own numbering match the uncoded items, which supports gap-filling:**
   - L-R01..L-R03 are followed by L-R09..L-R11. The 5 missing numbers (L-R04..L-R08) match exactly the 5 uncoded left region families in task order: hinge, seat top, lid underside, rim top, seat underside.
   - L-T03 and R-T03 are task 7, which implies T01 and T02 for tasks 3 and 4.
   - On the right, R-R01 and R-R02 are coded, and the 6 uncoded right families take R-R03..R-R08.
4. **R-T08 is missing** (R-T07 is followed by R-T09). DROP GLOVE (left, task 13) has no number and does not fit L-T03 → L-T04 by order.
5. **Part notation varies:** "PART 01/02" (lid top), "PART 1/2", "PART1" (12A) vs "PART 2" (12B). Left/right halves are also called "HALF" (lid/seat underside) in some places and "PART" (seat top, rim top) in others.
6. **Word order varies:** "LEFT HINGE / RIGHT HINGE" (side first) vs "BASE LEFT" (side last). "FLOOR SPILLAGE LEFT PART n" vs the right-arm "FLOOR SPILLAGE PART n" with no side. "OPEN LID + SEAT" vs "CLOSE SEAT + LID". "(MASTER)" vs "SUPPORT" without parentheses (task 4).
7. **Task numbering is irregular:** 1A, 2A, 1C, 2, 3, 4, 5A, 5B, 6, 7, 8A, 8B, 9, 10A, 10B, 11A, 11B, 12A, 12B, 13, 14, 15. There is no 1B or 2B, 1C follows 2A, and "2" exists alongside "2A". Tasks 13 and 14 share one slide.
8. **Labels repeat:** R-T04 PICK DRAIN BRUSH on both 10A and 10B; HOLD BRUSH OUTSIDE on both 11A and 11B.
9. **One code covers several actions:** "R-T06 RETURN DRAIN BRUSH AND DROP GLOVE" is a tool return plus a glove drop. "GLOVE CHANGE" vs "DROP GLOVE" leaves it unclear whether a change includes the drop.
10. **Type mixing:** R-T05 CLEAN DRAIN HOLE is a cleaning region (it is on the PAGE 04 map) but is coded as a T task because it uses the brush.
11. **Map vs tasks:** FLOOR SPILLAGE and the lid-edge strips are not on the region maps. Some overlays have no label (the hinge-task edge strips and the inner-rim bars).
12. **Part boundaries are ambiguous:**
    - Inner rim: the front-centre bar is in both parts and the rear-centre bar only in part 1.
    - Drain-hole parts are identical, so they are passes.
    - Lid-top parts split rear/front, while every other surface splits left/right.
13. **Pictures do not always show the required state.** The side and front pictures show the lid closed during lid-open and seat-open steps. Left-side pictures are mirrored.

### 5.2 Proposed Lite ID: `<ARM>-<KIND><NN>[-P<n>|-S<n>]`

| Field | Values | Rule |
|---|---|---|
| ARM | `L`, `R` | Physical arm. A mechanism is two IDs, one per arm, linked by `paired_with`. |
| KIND | `R` = region (anything wiped for a time), `T` = task (mechanism, tool, glove, idle) | Same two kinds as the deck, so codes stay traceable. A separate `type` field (`surface` / `mechanism` / `tool` / `glove` / `idle`) carries the finer meaning. |
| NN | 2 digits | (a) **Deck number where printed** (FL-R09 becomes 09). (b) **Gap-fill in deck order** where the deck leaves a gap of exactly the right size (L-R04..L-R08, R-R03..R-R08, L-T01/02, R-T01/02), flagged `code_source: inferred`. (c) **Otherwise, the next free number, append-only**: L-T06 DROP GLOVE and R-T10 HOLD BRUSH OUTSIDE. **R-T08 stays reserved** until the deck owner says what it was. |
| `-P<n>` | spatial or pass part | From "PART 1/01/PART1". Halves always number left→right, or rear→front for the lid top. Write the part's meaning in `part.where`. Single-part units have no suffix. |
| `-S<n>` | sequential hand-off sub-step | For MASTER/SUPPORT mechanisms taught as alternating sub-steps (V6: M3_OPNST01/02 ↔ M3_ASOPN01/02). Example: L-T03-S1, R-T03-S1, L-T03-S2, R-T03-S2. |

**Step ID.** Use `ST01…ST22` in deck order, with `deck_task` ("1A") and `deck_slide` (5) kept as attributes. The deck's task numbers are irregular, so they should not be keys.

**Always store the aliases:** `deck_code` as printed ("FL-R09", or null), `deck_label` verbatim, `deck_task`, `deck_slide`, `map_region` (PAGE 04/05 name, or "FLOOR SPILLAGE"), and `legacy` (V6 file / program id / segment).

**File naming.**
- Raw files: `<ID>__RAW__<yyyymmdd-hhmmss>.txt`, for example `L-R05-P1__RAW__20261013-101500.txt`.
- Optimised files: `<ID>__OPT_v<k>.txt`.
- Controller program: `DT_<ID>`.
- The existing upload helper `drag_program_name()` keeps `[A-Za-z0-9_-]`, adds the `DT_` prefix and truncates at 56 characters, so hyphens survive.
- V6 used compact names of 10 characters or fewer and program IDs 71–83. Reserve an ID range per arm and per fixture state.

**Full alias table**

| Lite ID | deck code / label | Lite ID | deck code / label |
|---|---|---|---|
| L-R01 | L-R01 BASE LEFT | R-R01-P1 | R-R01 LID TOP PART 01 |
| L-R02 | L-R02 SIDE EXTERIOR LEFT | R-R01-P2 | R-R01 LID TOP PART 02 |
| L-R03 | L-R03 PEDESTAL UPPER LEFT | R-R02 | R-R02 RIGHT HINGE |
| L-R04 | — / LEFT HINGE | R-R03 | — / SIDE EXTERIOR RIGHT |
| L-T01 | — / OPEN LID + SEAT (MASTER) | R-T01 | — / OPEN LID + SEAT (SUPPORT) |
| L-T02 | — / CLOSE SEAT ONLY (MASTER) | R-T02 | — / CLOSE SEAT ONLY SUPPORT |
| L-R05-P1 | — / SEAT TOP PART 1 | R-R04 | — / LID UNDERSIDE RIGHT HALF |
| L-R05-P2 | — / SEAT TOP PART 2 | R-R05 | — / PEDESTAL UPPER RIGHT |
| L-R06 | — / LID UNDERSIDE LEFT HALF | R-R06 | — / BASE RIGHT |
| L-T03 | L-T03 OPEN SEAT (MASTER) | R-T03 | R-T03 OPEN SEAT (SUPPORT) |
| L-R07-P1 | — / RIM TOP PART 1 | R-R07 | — / SEAT UNDERSIDE RIGHT HALF |
| L-R07-P2 | — / RIM TOP PART 2 | R-R08-P1 | — / FLOOR SPILLAGE PART 1 |
| L-R08 | — / SEAT UNDERSIDE LEFT HALF | R-R08-P2 | — / FLOOR SPILLAGE PART 2 |
| L-R09-P1 | FL-R09 INNER RIM PART 1 | R-T04 | R-T04 PICK DRAIN BRUSH (10A + 10B) |
| L-R09-P2 | FL-R09 INNER RIM PART 2 | R-T10 | — / HOLD BRUSH OUTSIDE (11A + 11B) |
| L-R10-P1 | L-R10 BOWL INSIDE PART 1 | R-T05-P1 | R-T05 CLEAN DRAIN HOLE PART1 |
| L-R10-P2 | L-R10 BOWL INSIDE PART 2 | R-T05-P2 | R-T05 CLEAN DRAIN HOLE PART 2 |
| L-R11-P1 | L-R11 FLOOR SPILLAGE LEFT PART 1 | R-T06 | R-T06 RETURN DRAIN BRUSH AND DROP GLOVE |
| L-R11-P2 | L-R11 FLOOR SPILLAGE LEFT PART 2 | R-T07 | R-T07 RIGHT GLOVE CHANGE |
| L-T06 | — / DROP GLOVE | (R-T08) | reserved: missing in the deck |
| L-T04 | L-T04 LEFT GLOVE CHANGE | R-T09 | R-T09 CLOSE SEAT + LID (SUPPORT) |
| L-T05 | L-T05 CLOSE SEAT + LID (MASTER) | | |

---

## 6. Mapping the deck to the V6 package (all mappings inferred)

### 6.1 What V6 contains
Each arm has 13 programs, P1–P13, shipped as 7–8 files. Program IDs are 70-series. Flows are paired by P number.

| Phase | LEFT (pole) | RIGHT (pole) | Rule |
|---|---|---|---|
| start | pole 200 → hand GRIP `[0,0,0,0,150,1000]` | same | — |
| A (lid closed) | `L_P01P02.txt` id 71 (150 mm): P1 + 1.25 s bridge + P2. Then `L_P03P05.txt` id 73 (50 mm): P3 + 1.38 s bridge + P4 + 1.38 s bridge + P5. | `R_P01P03.txt` id 71 (50 mm): P1 + P2 + P3. Then `R2_LIDTOP.txt` id 74 (150 mm) = P4. Then `R10_HNGR.txt` id 75 (50 mm) = P5. | TOGETHER (time-matched) |
| manual | pole 150 + **WAIT 2 s, "MANUAL LID COVER OPEN"** | same | Operator checkpoint. Per the README the runner does not pause by itself. |
| B (lid open) | `L_P06P07.txt` id 76 (150 mm): P6 + 1.6 s bridge + P7 | `R_P06P07.txt` id 76 (150 mm) | TOGETHER |
| mechanism | `M3_OPNST01.txt` id 78 = P8, `M3_OPNST02.txt` id 79 = P9 | `M3_ASOPN01.txt` id 78, `M3_ASOPN02.txt` id 79 | LEFT **RUN FIRST**, RIGHT **WAIT OTHER**, alternating L8 → R8 → L9 → R9 |
| C (seat open) | `L_P10P12.txt` id 80 (150 mm): P10 + P11 + P12 | `R_P10P12.txt` id 80 (150 mm) | TOGETHER |
| C, low | `R15_BUPFUL.txt` id 83 (50 mm) = P13, 48.7 s | `R_SPILLAGE.txt` id 83 (50 mm) = P13, 28.3 s, speed 50 % in FINAL | Start together, no finish match |

All programs run at speed 100 and transit 30, except R_SPILLAGE (50 in `*_FINAL.json`, 100 in `JSON/`). The hand preset is applied once, at the start.

**Durations are time-matched.** Solving the files gives P1+P2 = 33.775 s, P3 = 15.735 s, P4 = 18.755 s, P5 = 17.665 s, P6+P7 = 32.12 s and P10+P11+P12 = 50.62 s. These match on both arms.

### 6.2 Evidence used
- (a) **File names.** LIDTOP, HNGR (hinge right), SPILLAGE, M3 OPNST (open seat) / ASOPN (assist open). "M3" ↔ the deck's **T03**.
- (b) **Order and fixture phase.** P1–P5 come before the manual lid opening, P6–P7 are lid-open, P8–P9 open the seat, P10–P13 are seat-open.
- (c) **Pole height.** 50 mm means low work (base, floor, bowl); 150 mm means top-level work.
- (d) **Rough FK.** I ran each segment through the full-Studio URDF.
  - Joint units are 0.001°. Pole mm → sliding plate joint in m. The reported point is `<arm>_glove_frame_2` in `base_link`: x toward the fixture, +y = robot left, z up.
  - `commode_c/scene.yaml` places the bowl front tip at about x 0.48, the rear at about x 1.0, and the rim at about z 0.42, with the floor at z 0.
  - The real base placement during teaching is unknown, so positions are only relative. They agree with the fixture geometry, and every left wiping program except P13 has a mirror-image right program (same x and z, y negated). That consistency is the main support for the mapping. The mirror pairs are L1↔R4, L2↔R5, L3↔R1, L4↔R2, L5↔R3, L6↔R7, L7↔R6, L10↔R11, L11↔R12, L12↔R10.
- (e) **Segment boundaries.** Each `BRIDGES_TEST_ONLY` file appears verbatim inside its merged file, which gives exact sample boundaries (±1 point; bridge files include their two endpoints).

| Arm · P | File (prog id) · samples [start, end) | s | Pole | FK median glove xyz (m) · z p10–p90 | Inferred deck unit | Confidence · reason |
|---|---|---|---|---|---|---|
| L P1 | L_P01P02 (71) · 0–3275 | 16.4 | 150 | (0.82, +0.08, 0.49) · 0.48–0.89 | **Lid top, left half.** The deck has no left-arm lid unit: it gives the whole lid to R-R01. Status: EXTRA | med-high · FK: top-surface height, left half, whole lid length; mirrors R2_LIDTOP |
| L P2 | L_P01P02 (71) · 3527–7005 | 17.4 | 150 | (0.89, +0.23, 0.43) · 0.38–0.51 | **L-R04 LEFT HINGE + left lid edge** | med-high · rear-left at lid-edge height along the full length; mirrors R10_HNGR; matches the deck's hinge box + edge strip |
| L P3 | L_P03P05 (73) · 0–3146 | 15.7 | 50 | (0.58, +0.15, 0.13) · 0.06–0.22 | **L-R01 BASE LEFT** | med-high · low pole, low z, front-left; mirrors R P1 |
| L P4 | L_P03P05 (73) · 3424–7173 | 18.8 | 50 | (0.53, +0.14, 0.21) · 0.11–0.35 | **L-R03 PEDESTAL UPPER LEFT** | medium · front face, mid-to-upper height; mirrors R P2 |
| L P5 | L_P03P05 (73) · 7451–10983 | 17.7 | 50 | (0.73, +0.23, 0.32) · 0.14–0.39 | **L-R02 SIDE EXTERIOR LEFT** | med-high · constant y about +0.23 (the side wall), along the full length, upper half; mirrors R P3 |
| L P6 | L_P06P07 (76) · 0–3543 | 17.7 | 150 | (0.93, +0.09, 0.65) · 0.50–0.78 | **L-R06 LID UNDERSIDE LEFT HALF** | high · rear, high (lid standing up), left; lid-open phase |
| L P7 | L_P06P07 (76) · 3865–6744 | 14.4 | 150 | (0.77, +0.12, 0.49) · 0.46–0.54 | **L-R05-P1 SEAT TOP left half** | med-high · seat height, left; lid-open phase |
| L P8 / P9 | M3_OPNST01 (78) / 02 (79) | 2.3 / 1.3 | 150 | (0.63, +0.02, 0.54) / (0.64, +0.21, 0.57) | **L-T03 OPEN SEAT (MASTER)**, sub-steps S1/S2 | high · name, RUN FIRST, seat front edge, comes after the lid-open phase |
| L P10 | L_P10P12 (80) · 0–3225 | 16.1 | 150 | (0.80, +0.06, 0.39) · 0.36–1.08 | **L-R09-P1 INNER RIM left** | medium · about 3 cm below the rim level, left; mirrors R P11 |
| L P11 | L_P10P12 (80) · 3277–6695 | 17.1 | 150 | (0.80, +0.14, 0.47) · 0.45–0.64 | **L-R07-P1 RIM TOP left** | medium · rim level, left; mirrors R P12 |
| L P12 | L_P10P12 (80) · 6737–10214 | 17.4 | 150 | (0.90, +0.15, 0.64) · 0.47–0.93 | **L-R08 SEAT UNDERSIDE LEFT HALF** | high · rear, high (raised seat), left; mirrors R P10 |
| L P13 | R15_BUPFUL (83) · all 9741 | 48.7 | 50 | (0.76, +0.01, 0.28) · 0.26–0.38 | **L-R10-P1 + P2 BOWL INSIDE** (both in one file) | med-high · low pole, inside the bowl below the rim, centred. "BUPFUL" plausibly means "Bowl UPper FULl"; the meaning of "R15" is unknown. |
| R P1 | R_P01P03 (71) · 0–3275 | 16.4 | 50 | (0.58, −0.15, 0.15) · 0.05–0.29 | **R-R06 BASE RIGHT** (the deck does it in step 9, lid open) | med-high · low, front-right |
| R P2 | R_P01P03 (71) · 3527–7004 | 17.4 | 50 | (0.54, −0.10, 0.27) · 0.14–0.37 | **R-R05 PEDESTAL UPPER RIGHT** (deck step 8) | medium · front face, upper |
| R P3 | R_P01P03 (71) · 7282–10428 | 15.7 | 50 | (0.77, −0.21, 0.34) · 0.17–0.43 | **R-R03 SIDE EXTERIOR RIGHT** | med-high · side wall, y about −0.21 |
| R P4 | R2_LIDTOP (74) · all 3751 | 18.8 | 150 | (0.81, −0.12, 0.50) · 0.48–0.77 | **R-R01 LID TOP, right half only.** PARTIAL: the deck wants the whole lid split rear/front. | high for "lid top" (name + FK) · low for the part split |
| R P5 | R10_HNGR (75) · all 3533 | 17.7 | 50 | (0.85, −0.17, 0.43) · 0.39–0.46 | **R-R02 RIGHT HINGE + right lid edge** | high · name, plus right lid-edge height along the full length to the rear |
| R P6 | R_P06P07 (76) · 0–3543 | 17.7 | 150 | (0.83, −0.11, 0.51) · 0.46–0.94 | **Seat top, right half.** The deck gives this to the LEFT arm (L-R05-P2). Status: EXTRA / OTHER-ARM | medium · seat height, right; lid-open phase |
| R P7 | R_P06P07 (76) · 3865–6744 | 14.4 | 150 | (0.96, −0.11, 0.67) · 0.49–0.77 | **R-R04 LID UNDERSIDE RIGHT HALF** | high · rear, high, right |
| R P8 / P9 | M3_ASOPN01 (78) / 02 (79) | 1.0 / 2.6 | 150 | (0.58, −0.21, 0.49) / (0.74, −0.14, 0.59) | **R-T03 OPEN SEAT (SUPPORT)**, sub-steps S1/S2 | high · name, WAIT OTHER, pushes the seat up and back |
| R P10 | R_P10P12 (80) · 0–3225 | 16.1 | 150 | (0.90, −0.12, 0.58) · 0.48–0.81 | **R-R07 SEAT UNDERSIDE RIGHT HALF** | high |
| R P11 | R_P10P12 (80) · 3277–6695 | 17.1 | 150 | (0.75, −0.07, 0.37) · 0.36–0.84 | **Inner rim, right half.** The deck gives this to the LEFT arm (L-R09-P2). Status: EXTRA / OTHER-ARM | medium |
| R P12 | R_P10P12 (80) · 6737–10214 | 17.4 | 150 | (0.78, −0.12, 0.49) · 0.45–0.63 | **Rim top, right half.** The deck gives this to the LEFT arm (L-R07-P2). Status: EXTRA / OTHER-ARM | medium |
| R P13 | R_SPILLAGE (83) · all 5657 | 28.3 | 50 | (0.51, −0.19, 0.05) · 0.03–0.51 | **R-R08-P1 + P2 FLOOR SPILLAGE** (both in one file) | high · name, floor height, front-right |

**How V6 paired the arms (for comparison with the deck's pairs):**

| P | LEFT | RIGHT |
|---|---|---|
| P1 | lid-top left | base right |
| P2 | hinge left | pedestal right |
| P3 | base left | side right |
| P4 | pedestal left | lid-top right |
| P5 | side left | hinge right |
| P6 | lid-underside left | seat-top right |
| P7 | seat-top left | lid-underside right |
| P10 | inner-rim left | seat-underside right |
| P11 | rim-top left | inner-rim right |
| P12 | seat-underside left | rim-top right |
| P13 | bowl inside | floor spillage |

V6 splits every interior surface by arm. The deck instead gives seat top, rim top, inner rim and bowl inside fully to the left arm, so the right arm is free for the brush and drain.

### 6.3 Taught / not-taught status per deck unit (inferred)

| Status | LEFT units | RIGHT units | Count |
|---|---|---|---|
| **COVERED**: V6 has a same-arm segment (merged and time-stretched, not raw) | L-R01, L-R02, L-R03, L-R04, L-R05-P1, L-R06, L-T03 (S1+S2), L-R07-P1, L-R08, L-R09-P1 | R-R02, R-R03, R-R04, R-R05, R-R06, R-T03 (S1+S2), R-R07 | 17 |
| **MERGED**: two deck parts sit in one V6 file | L-R10-P1 + L-R10-P2 (R15_BUPFUL) | R-R08-P1 + R-R08-P2 (R_SPILLAGE) | 4 |
| **PARTIAL**: wrong split | — | R-R01-P1, R-R01-P2. R2_LIDTOP is the right half; the deck wants rear/front halves of the whole lid. | 2 |
| **OTHER-ARM**: taught on the opposite arm in V6 | L-R05-P2 (V6 R P6), L-R07-P2 (R P12), L-R09-P2 (R P11) | — | 3 |
| **NOT TAUGHT**: no V6 trajectory on any arm | L-T01, L-T02, L-R11-P1, L-R11-P2, L-T06, L-T04, L-T05 | R-T01, R-T02, R-T04, R-T10, R-T05-P1, R-T05-P2, R-T06, R-T07, R-T09 | **16** |
| **EXTRA**: in V6 but not in the deck allocation | lid-top-left (L P1) | seat-top-right (R P6), inner-rim-right (R P11), rim-top-right (R P12) | 4 segments |

- By arm-step (44): **21 covered** (12 left, 9 right), 2 partial, 3 other-arm, **18 not taught** (7 left, 11 right).
- Everything after the bowl is untaught: brush, drain, left floor, gloves, and closing. So are both robot lid/seat transitions that V6 does by hand or skips (tasks 3 and 4).
- **No raw `.txt` per region exists in the package.** V6 holds only merged, time-stretched programs. For R1's "taught raw txt" dashboard, every unit is *raw-not-present*. The V6 segments can be imported as **legacy reference** by splitting at the sample indices above. They should not count as raw.

### 6.4 Package caveats relevant to Lite
- **README and checksums do not match the package contents:**
  - The README says JSON/ holds `LEFT_V6_FINAL.json` / `RIGHT_V6_FINAL.json`, plus a `manual_checkpoint` field and `DUAL_FLOW_TIMELINE_V6.json`.
  - The actual files are `JSON/LEFT_V6_70SERIES.json`, `JSON/RIGHT_V6_70SERIES.json` and top-level `*_FINAL.json`. There is no `manual_checkpoint` field, only a wait note.
  - `SHA256SUMS.txt` has 34 entries. 28 pass: all 15 program `.txt`, all 11 bridges and both CSVs. 5 listed JSON files are missing, and README.txt fails its checksum. The JSON files actually shipped (`*_FINAL.json`, `JSON/*_70SERIES.json`) are not in the checksum list.
- **The bridges are joint-space only and not collision-checked.** The README asks for reduced-speed testing, especially L_P01→P02 (16.7° gap) and R_P06→P07 (39.1° gap).
- **Test status is unknown.** Treat V6 as "merged, untested" in Lite.

---

## 7. How Lite should present this to non-technical operators

### 7.1 Regions tab, three panes
1. **Overview.** Show PAGE 04 and PAGE 05 (renders `hi-03.png`, `hi-04.png`).
   - Make the 13 pasted labels clickable (pixel boxes in Appendix A.2) and add a **FLOOR SPILLAGE** hotspot, which is missing from the map.
   - Colour each label by the worst status of its units. Clicking a label filters the step list to that region family.
   - Keep a "View original slide" button that shows `hi-NN.png` for traceability.
2. **Step list.** Show the 22 deck steps grouped by fixture-state phase: Lid closed → *Open lid+seat* → *Close seat* → Lid open → *Open seat* → Seat open → Tools/gloves → *Close all*.
   - Each row is one step with a LEFT cell and a RIGHT cell (unit ID in small type and a status chip).
   - Transition steps are shown as dividers with the PAGE 02 state photo.
3. **"What to clean" card** for the selected unit (§7.2).

### 7.2 Card content (one card per teachable unit)
- **Title in plain words; codes in small type.** Example: "LEFT arm — wipe the **seat top, left half** (part 1 of 2)", with `L-R05-P1 · deck task 5A · slide 11` underneath.
  - Avoid "R/T/P" jargon in the title.
  - Use the deck colours for the arm badge only: **blue = LEFT, orange = RIGHT**. Never use those colours for statuses.
- **Picture.**
  - For v1, use the per-arm crops (Appendix B). They work for slides where each arm has its own picture.
  - The better option is to draw the overlay on the clean media image from the normalised boxes (Appendix A.1). Highlight only the active unit, dim the other arm, and mirror image5/25 for left-side units.
  - Overlays are required for shared-picture slides (5A, 8A, 13/14, 15) and for photos (3, 4, 7). Cropping cannot separate the arms there.
  - For mechanisms, show the deck photo with its arrow, plus a "grip here" note: front-left edge for task 3, left edge for task 4, front underside for task 7.
- **Fixture-state check.** Show the PAGE 02 thumbnail and ask, for example, "Is the lid UP and the seat DOWN?" with Yes/No. Teaching is blocked until the operator answers Yes. Exterior and floor units show "any state", but the deck phase is still the default order.
- **Setup before teaching (R7).**
  - Pole height in mm with Apply plus a live readback. Defaults come from V6 (§7.4).
  - A named hand preset (GRIP `[0,0,0,0,150,1000]`, ALL OPEN `[1000×6]`) with Apply plus readback.
  - A tool check: glove on? brush in its holder?
- **Teach.**
  1. A large START DRAG.
  2. A countdown of the specified cleaning time, with a beep at the end; the operator can extend.
  3. STOP & SAVE writes `<ID>__RAW__<ts>.txt`.
  4. Quick replay and REDO / SKIP (with a reason).
  5. Then **auto-advance**.
- **Auto-advance rules.**
  - Go to the next unit in plan order whose fixture state is already set.
  - If the state must change, show an interstitial ("Open the seat now"; teach L-T03 / R-T03).
  - Mechanisms run a paired flow: MASTER sub-step, then SUPPORT sub-step, alternating S1/S2 as in V6.
  - "Idle" units (R-T10) teach a hold pose only. Their duration is the partner's.
  - A single operator can drag only one arm at a time. The default is therefore all left units of a fixture phase, then all right units, then the state change. This minimises lid and seat changes.

### 7.3 Metadata each unit needs (one record per unit)

```yaml
- id: L-R05-P1                 # Lite ID (sec. 5.2)
  deck: {code: null, label: "SEAT TOP PART 1", task: "5A", slide: 11, code_source: inferred-gap-fill}
  arm: left                     # left | right
  type: surface                 # surface | mechanism | tool | glove | idle
  map_region: "Seat Top"        # PAGE 04/05 name or "FLOOR SPILLAGE"
  part: {n: 1, of: 2, where: "left half"}
  step: 7                       # ST07, deck order 1..22; per-arm order also stored
  paired_with: R-R04            # other arm in the same step
  sync: TOGETHER                # TOGETHER | MASTER_FIRST (RUN FIRST) | SUPPORT_WAIT (WAIT OTHER)
  role: null                    # MASTER | SUPPORT for mechanisms
  fixture_state: LID_OPEN       # CLOSED | LID_OPEN | SEAT_OPEN | ANY; mechanisms: {before, after}
  cleaning_time_s: 15           # set by supervisor; V6 reference 14.4 s
  pole_mm: 150                  # R7 default (V6), editable per region
  hand_preset: GRIP             # name + 6 values
  tool: glove                   # glove | brush | none
  image: {media: image2.png, box_uv: [0.334, 0.420, 0.516, 0.954], mirror: false}
  files: {raw: [], optimised: []}
  status: {raw: false, optimised: false, sim_ok: null, real_ok: null, merged: false, needs_reteach: false}
  legacy: {package: V6, file: LEFT/L_P06P07.txt, program_id: 76, segment: 2, samples: [3865, 6744], confidence: medium-high, reason: "FK seat height, left half, lid-open phase"}
```

### 7.4 R7 defaults that can be read from V6 (editable; "proposed" means unverified)

| Units | Pole | Source |
|---|---|---|
| Lid top L/R, left hinge, lid underside L/R, seat top, rim top, inner rim, seat underside, open-seat mechanism | **150 mm** | V6 |
| Right hinge | **50 mm** | V6 R10_HNGR |
| Base, pedestal, side (both arms), bowl inside, floor spillage right | **50 mm** | V6 |
| Floor spillage left, drain hole | 50 mm | proposed |
| Open lid + seat, close seat, close seat + lid | 150 mm | proposed, like M3 |
| Brush pick / hold / return, glove drop / change | **TBD** | depends on where the brush holder and glove station are |

- The start pose is pole 200 mm with the GRIP hand preset.
- Hand: V6 uses GRIP everywhere. Brush grasp and glove change probably need their own presets (ALL OPEN, then a grasp); these are TBD.
- **Cleaning time:** the deck gives none. V6 reference durations, which are post-stretch, are:
  - about 14–19 s per region or half-region;
  - bowl inside 48.7 s;
  - floor spillage (right, both parts) 28.3 s;
  - open-seat sub-steps 1.0–2.6 s.
- The process owner should set the target time per unit.

### 7.5 Dashboard (R1: regions with and without a taught raw .txt)
- **Matrix:** rows are the 22 steps (or the 13 region families, as a toggle); columns are LEFT and RIGHT. Each cell shows the unit ID, a thumbnail and a status chip with **icon + text**, so it is not colour-only.
- **Statuses, derived from files and logs wherever possible:**

| Status | Meaning |
|---|---|
| `NOT TAUGHT` | No raw file |
| `RAW TAUGHT` | `*__RAW__*.txt` exists |
| `OPTIMISED` | An edited `__OPT_v*` exists (R2) |
| `SIM OK` | A sim playback passed (R3) |
| `REAL TESTED` | The operator confirmed a pass on the real arms (R4) |
| `IN PROGRAM` | Merged and/or uploaded (R8/R13) |

- **Flags:**
  - `LEGACY V6` (with the mapping confidence and the file:segment);
  - `NEEDS RE-TEACH` (allocation or split changed: L-R05-P2, L-R07-P2, L-R09-P2, R-R01-P1/P2);
  - `FAILED` (sim or real test failed, with a link to the collision point, R5);
  - `BLOCKED` (tool or fixture not available).
- **Header counters per arm and per phase**, for example "LEFT 0/22 raw · 12 legacy". Add a filter for "only missing raw" and a one-click "Teach next missing".
- The overview map (§7.1) uses the same statuses.

---

## Appendix A.1: overlay boxes normalised to the source media image
- u and v run 0–1 across the **source** image after removing the slide crop. For mirrored pictures (left side) the box is given in the unmirrored source, so mirror it on display.
- Rotation is the overlay rotation as drawn on the slide.
- v > 1 means the box extends below the photo edge, onto the floor.
- Front-view pictures sit at −4° on the slide; this rotation is ignored here.

| Unit | Media | u0–u1 | v0–v1 | rot | Note |
|---|---|---|---|---|---|
| R-R01-P1 | image1 | 0.259–0.739 | 0.250–0.622 | 0 | rear half of the lid |
| R-R01-P2 | image1 | 0.259–0.738 | 0.537–0.987 | 0 | front half |
| R-R02 | image1 | 0.498–0.712 | 0.097–0.248 | 0 | plus edge strip u 0.718–0.749, v 0.248–0.824 |
| L-R04 | image1 | 0.292–0.503 | 0.093–0.245 | 0 | plus edge strip u 0.255–0.277, v 0.277–0.773 |
| L-R01 / R-R06 | image5 | 0.235–0.736 / 0.237–0.739 | 0.608–0.906 / 0.636–0.907 | 0 / −3 | L mirrored |
| L-R02 / R-R03 | image5 | 0.114–0.737 / 0.121–0.737 | 0.301–0.631 / 0.309–0.619 | 0 | L mirrored |
| L-R03 | image4 | 0.158–0.512 | 0.152–0.553 | 0 | front, left half |
| R-R05 | image22 | 0.471–0.803 | 0.144–0.598 | 0 | front, right half |
| L-R05-P1 / L-R05-P2 | image2 | 0.334–0.516 / 0.482–0.664 | 0.420–0.954 | 0 | seat ring halves |
| L-R06 / R-R04 | image2 | 0.304–0.498 / 0.516–0.692 | 0.017–0.419 / 0.020–0.420 | 0 | lid underside halves |
| L-R07-P1 / L-R07-P2 | image3 | 0.331–0.498 / 0.501–0.667 | 0.471–0.984 | 0 | rim halves |
| L-R08 / R-R07 | image3 | 0.307–0.488 / 0.498–0.684 | 0.034–0.476 / 0.043–0.471 | 0 | seat underside halves |
| L-R09-P1 bars | image3 | left side 0.382–0.403 (v 0.570–0.802); rear-centre 0.434–0.567 (v 0.500–0.534); front-centre 0.445–0.543 (v 0.871–0.905); front-left 0.373–0.470 (v 0.821–0.855, rot 56); rear-left 0.371–0.468 (v 0.532–0.566, rot 141) | | | inner rim, left |
| L-R09-P2 bars | image3 | right side 0.590–0.611 (v 0.570–0.802); front-centre 0.445–0.543 (v 0.871–0.905); front-right 0.523–0.621 (v 0.831–0.865, rot 323); rear-right 0.531–0.628 (v 0.535–0.569, rot 228) | | | inner rim, right |
| L-R10-P1 / L-R10-P2 | image3 | 0.410–0.507 / 0.501–0.610 | 0.543–0.868 / 0.537–0.862 | 0 | bowl, left / right of centre |
| R-T05-P1 = P2 | image3 | 0.471–0.543 | 0.615–0.756 | 0 | drain/trap opening |
| R-R08-P1 / R-R08-P2 | image5 | 0.387–0.737 / 0.158–0.312 | 0.894–1.019 / 0.781–1.038 | −7 / −34 | floor rear-right / front-right |
| L-R11-P1 / L-R11-P2 | image25 | 0.374–0.752 / 0.162–0.305 | 0.896–1.052 / 0.757–1.100 | +7 / +46 | floor rear-left / front-left, mirrored |

Units with no overlay, only a label:
- mechanisms: tasks 3, 4, 7 (photos) and 15 (text);
- R-T04 / R-T10 / R-T06: image24 (brush);
- glove units: image26.

## Appendix A.2: region-map label hotspots (px in the 110-dpi renders, x, y, w, h)
**hi-03.png (PAGE 04)**

| Label | x, y, w, h |
|---|---|
| Hinge | 297, 207, 75, 32 |
| Lid Top | 286, 402, 97, 42 |
| Lid Underside | 656, 274, 121, 30 |
| Seat Top | 676, 608, 87, 28 |
| Seat Underside | 1074, 182, 126, 25 |
| Rim Top | 1118, 622, 81, 23 |
| Drain Hole | 902, 493, 106, 29 (arrow tip 1157, 507) |
| Bowl Inside | 1309, 440, 114, 26 (arrow tip 1187, 452) |
| Inner Rim | 1321, 508, 94, 26 (arrow tip 1224, 521) |

**hi-04.png (PAGE 05)**

| Label | x, y, w, h |
|---|---|
| Side Exterior | 395, 398, 124, 30 |
| Base (side view) | 457, 519, 59, 32 |
| Pedestal Upper | 942, 318, 169, 30 |
| Base (front view) | 1081, 561, 59, 32 |

Accuracy is about ±2 px. The values come from the slide XML.

## Appendix B: per-arm crop renders (150 dpi, `pptx_render/`)
- **Own picture per arm (L and R files):** s05 T1A, s06 T2A, s07 T1C, s08 T2, s12 T5B, s13 T6, s16 T8B, s17 T9, s18 T10A, s19 T10B, s20 T11A, s21 T11B, s22 T12A, s23 T12B, plus `crop-s24-T13-L.png` and `crop-s24-T13-R.png`.
- **Shared picture (LR file):** s09 T3, s10 T4, s11 T5A, s14 T7, s15 T8A, `crop-s24-T14-LR.png`, s25 T15.
- File pattern: `crop-sNN-T<task>-<L|R|LR>.png`, 37 files.
- Each crop's box is the union of the arm's picture, overlays and captions, plus about 0.1 in margin and 0.4 in below for the two-line arm captions.
- `scripts/arm_crops.py` reproduces the crops. The s24 crops were cut by hand, because one slide holds tasks 13 and 14.
