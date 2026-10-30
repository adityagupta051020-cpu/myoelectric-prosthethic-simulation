# EMG-Controlled Prosthetic Hand — Final Simulink/Simscape Model (5-servo)

**Phase 1 deliverable, finalized.** Complete, validated Simulink implementation
of the EMG signal-conditioning chain driving **five servo actuators** that
open and close the fingers.

> Simulation results only. They validate the design and the model — **not** the
> physical hardware. Bench-test items are listed at the end. Do not begin the
> Falstad phase until this model is accepted (it already passes all 10 tests).

---

## What's new in this final version

- **Five-servo actuation with real movement.** The control logic outputs five
  coordinated servo commands; each passes through a slew-rate limiter
  (400 °/s ≈ 0.15 s per 60°) so the fingers physically travel between open (5°)
  and closed (65°) rather than jumping. A sixth scope shows all five angles.
- **Per-finger closed angles** (`P.CTL.angle_close_vec`, default all 65°) so a
  natural grip can be dialed in without changing the logic.
- **Documented plain comparator** (`P.CMP.hyst = 0`) — matches the report; the
  4 s firmware debounce provides stability.
- Everything re-validated end to end: **10/10 tests pass.**

---

## Files & how to run

| File | Purpose |
|---|---|
| `emg_params.m` | All circuit, control, and servo parameters (+ synthetic EMG). |
| `emg_synth.m` | Adjustable synthetic surface-EMG source. |
| `build_EMG_model.m` | Builds `EMG_Prosthetic.slx` (analog chain + logic + 5 slew-limited servos + scopes). |
| `validate_EMG_model.m` | Runs the 10 tests, writes `comparison_table.csv`, saves plots. Self-contained (no toolboxes). |
| `plots/` | Reference stage and servo plots. |

```matlab
emg_params          % load parameters + emg_ts
build_EMG_model     % create & open EMG_Prosthetic.slx  (Simulink)
validate_EMG_model  % run all tests + plots            (no toolboxes needed)
```

Scopes: `Scope1_IA`, `Scope2_BandPass`, `Scope3_Rectifier`, `Scope4_Envelope`,
`Scope5_Comparator`, and `Scope6_Servos` (all five finger angles).

---

## Signal path & component choices

`EMG → AD623 IA (×77.9) → MFB band-pass (100–500 Hz) → precision full-wave
rectifier → Sallen-Key envelope (9.95 Hz) → ×13 → comparator (3 V) → Arduino
toggle/debounce → 5 slew-limited servos.`

- **IA gain 77.9** (Rg = 1.3 kΩ): kept modest so the single 0–5 V rail does not
  clip; remaining gain added later. 2.5 V reference centres the signal.
- **Band-pass** C = 33 nF, R1 = 12 k, R3 = 96 k (80 k+16 k), R5 = 24 k → f₀ = 300 Hz,
  Q = 0.75. Single-op-amp MFB chosen to free an op-amp for the ×13 stage.
- **Precision full-wave rectifier** (2 op-amp, 2 diode): preserves signal energy,
  hides diode drop; baseline 2.5 V.
- **Envelope** 2nd-order Sallen-Key, R = 16 k, C = 1 µF → 9.95 Hz, Q = 0.5: smooth,
  no overshoot.
- **×13** (Rf = 12 k, Rg = 1 k, referenced to 2.5 V) → total gain ≈ 1013.
- **Comparator** 3 V threshold via divider.
- **Firmware** toggle + 4 s debounce; open 5° / closed 65°.
- **Servos** modelled as 400 °/s slew-limited actuators on a separate 5–6 V rail.

---

## Validation — 10/10 pass (reference run)

| # | Test | Result |
|---|---|---|
| 1 | IA gain | 77.92 (calc 77.9) — PASS |
| 2 | Band-pass passband | f₀ = 302 Hz, −3 dB 162–562 Hz — PASS |
| 3 | Full-wave rectification | output ≥ 0, mean = 2A/π — PASS |
| 4 | Envelope tracking | 63 % rise ≈ 98 ms (< 250 ms) — PASS |
| 5 | Comparator/debounce | 2 toggles for 2 contractions — PASS |
| 6 | Weak/Moderate/Strong | weak no-trigger; mod & strong trigger — PASS |
| 7 | Drift/60 Hz/noise | rest max env 2.59 V, no false trigger — PASS |
| 8 | Five servo commands | all 5 reach open & closed, coordinated — PASS |
| 9 | End-to-end response | comparator 116 ms, servo 95 % travel 259 ms — PASS |
| 10 | No clipping | IA a.c. ±0.28 V, env 2.5–3.2 V within 0–5 V — PASS |

### Calculated vs. simulated vs. reported

| Quantity | Calculated | Simulated | Reported |
|---|---|---|---|
| IA gain | 77.9 | 77.92 | 77.9 |
| Total gain | 1012.7 | 1013 | 1012.7 |
| Band-pass f₀ | 300 Hz | 302 Hz | ~300 Hz |
| Band-pass Q | 0.75 | 0.75 | 0.75 |
| Band-pass −3 dB | (100–500 target) | 162–562 Hz | — |
| Envelope fc | 9.95 Hz | 9.95 Hz | 9.95 Hz |
| Threshold | 3.0 V | 3.0 V | 3.0 V |
| Response (comparator) | — | ~116 ms | — |
| Response (servo 95 %) | — | ~259 ms | — |

---

## Assumptions

- **Five coordinated servos** instead of the prototype's single servo (per spec),
  driven by one toggle so all fingers move together; per-finger closed angles
  are configurable.
- **Servo dynamics** modelled as a 400 °/s slew limit (typical hobby servo);
  refine with a measured servo step response.
- **×13 stage referenced to 2.5 V** so it amplifies only the excursion above
  baseline (avoids single-supply saturation).
- **Envelope C = 1 µF** (report's "1 mF" is a typo).
- **Comparator plain (no hysteresis) by default**; 4 s debounce gives stability.
- **Separate servo power rail** modelled as documentation only (does not load the
  signal chain in simulation).

## Still requires physical bench testing

- Electrode–skin impedance, motion artefact, real AD623 CMRR.
- Per-user threshold calibration.
- LM358 non-idealities (output not to +rail, input CM to V+−1.5 V), slew, offset.
- Real diode drops in the precision rectifier at low current.
- Actual servo current, stall, and rail isolation under simultaneous 5-finger
  actuation; confirm the separate servo supply holds up.
- Firmware timing on the real Arduino and the empirical 4 s debounce.
- Component tolerances (±1–5 %) shifting f₀, fc, and gains.
- End-to-end latency including mechanical tendon travel and spring return.

Once accepted, translate the same verified circuit into Falstad as the next
phase; `emg_params.m` is the hand-off specification.
