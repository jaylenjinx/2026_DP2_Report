# 41277 Control Design – DP2: State-Feedback Control

Demonstration Practical 2 for the QUBE-Servo 2 project (UTS, Spring 2026). Group report: model the setup, then design and simulate a discrete-time state-feedback controller and a Luenberger observer.

**Deadline:** Sunday 18 October 2026 (+3 days automatically approved on Canvas). Submit one report per team on Canvas and the simulation files on MS Teams.

## Tasks (see [`docs/2026_DP2.pdf`](docs/2026_DP2.pdf))

| Q | Topic | Marks |
|---|---|---|
| Q1 | State-space model: first-principles derivation (Jacobian linearisation if nonlinear), parameters / system ID, discretisation | 40 |
| Q2 | State-feedback control: controllability, `x*`/`u*` references, closed-loop eigenvalues, gain `F`, Simulink implementation (`run_observer = 0`) | 40 |
| Q3 | Luenberger observer: observability, observer eigenvalues, gain `L`, observer-based SFC in Simulink (`run_observer = 1`, `x̂(0) ≠ x(0)`) | 20 |

Project control requirements: overshoot < 15 %, settling time < 5 s, steady-state accuracy < 50.

## Marking checklist

- Signed statement of originality at the start (Turnitin-checked)
- All text and equations typed; only your own figures
- Engineering comments on every question (−2 per question if missing)
- Show every step / MATLAB function used for numerical answers
- Every figure numbered, captioned, referenced and commented in the text (−2)
- Axis labels with units, e.g. `Time [s]` (−2)
- Several variables in one plot: different colours and a legend (−2)
- Zoomed plots for transients and steady-state error
- MATLAB `plot()` figures (white background), no Simulink scope screenshots
- Contribution level per student in the report (scales individual marks)

## Layout

```
Report/                   LaTeX report (main.tex), styled after the Word answer template
  assets/images/          exported MATLAB figures referenced by \optfig
  assets/logos/
MATLAB/                   DP2 simulation files (main_SFC.m, linear/nonlinear Simulink models, plot scripts)
  qube_lab_template/      QUBE-Servo 2 hardware lab template (Ts = 1 ms, data save/plot scripts)
docs/                     DP2 brief, Word answer template, QUBE-Servo 2 manual, data sheet, courseware
```

## Building the report

The VS Code LaTeX Workshop settings in `.vscode/` build with tectonic on save into `build/`. From the command line:

```bash
cd Report && tectonic -X compile main.tex --outdir ../build
```

Missing figures render as placeholder boxes, so the report always compiles. Red `[TODO: …]` markers flag what still needs filling in.

## Running the simulation

Open `MATLAB/` in MATLAB and run `main_SFC.m`. Set `linear_model` (1 = linear, 0 = nonlinear) and `run_observer` (0 = SFC only, 1 = SFC + observer), and resize `x_hat_o` to the number of states.
