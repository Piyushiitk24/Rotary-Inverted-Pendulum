# Thesis Audit and Simplification Review

Scope: audit-only pass over the thesis chapters included by `thesis/main.tex`, cross-checked against `src/main.cpp`, `tools/modelling_complete.md`, the pinned 7-trial thesis tables, and the pinned sessions H1-H3, N1, T1-T3. No `.tex`, code, or figures were edited in this pass.

Conventions used below:
- `Weak claim` means the sentence is not fully false, but it is written more strongly than the available evidence supports.
- File references are written as `path:line-line`.
- For Chapter 3, rewrite recommendations target `tools/modelling_complete.md`, not the generated `thesis/chapters/03_modelling_derivation_generated.tex`.

## Chapter 0. Problem Framing, Contributions, and Reading Guide

### Problems
- The contribution list is stronger than the actual evidence base. The thesis later states that each condition is represented by only one pinned representative trial, so the work is a qualitative comparison, not a broad experimental comparison (`thesis/chapters/00_problem_framing.tex:7-19`, `thesis/chapters/10_results_discussion.tex:6-9`).
- The contribution bullet on reference tracking sounds project-wide, but the pinned dataset evaluates nudge mode only for the linear controller (`thesis/chapters/00_problem_framing.tex:17`, `thesis/chapters/10_results_discussion.tex:36-46`).
- The opening page uses advanced terms like `acceleration-input formulation` and `servo-like base positioning` before the report has explained the hardware or notation (`thesis/chapters/00_problem_framing.tex:9,17`).

### Easy to understand or not
- Partly easy. The chapter is short and well structured.
- Not easy for a first-year reader because the contribution bullets assume the reader already knows what `acceleration-input`, `sliding mode`, and `nudge mode` mean.

### False or weak claims
- Weak claim: `implemented and experimentally compared` is too broad for a thesis whose results chapter uses one pinned trial per condition (`thesis/chapters/00_problem_framing.tex:11`, `thesis/chapters/10_results_discussion.tex:7`).
- Weak claim: `Reference tracking while balancing` reads as a report-wide controller capability, but the results chapter evaluates only linear nudge mode in the pinned dataset (`thesis/chapters/00_problem_framing.tex:17`, `thesis/chapters/10_results_discussion.tex:37`).
- Weak claim: `Acceleration-input control matched to stepper actuation` should be softened to `matched to the firmware command structure`, because the physical actuator still receives velocity commands and can stall or miss steps (`thesis/chapters/00_problem_framing.tex:9`, `src/main.cpp:194-195`, `src/main.cpp:1402-1437`).

### Additions/alterations required
- Add one plain-language sentence after the problem statement: the project tries to keep the pendulum upright on real hardware where the motor, sensors, and timing are imperfect.
- Add one sentence in the contribution list stating that the comparison is qualitative on a fixed 7-run dataset.
- Reword the nudge contribution so it explicitly says `demonstrated with the linear controller in the pinned dataset`.
- Add a one-line warning that the base angle is measured, but the stepper is still not torque-controlled and can miss motion.

### Language simplification guidance
- Replace noun-heavy bullets with short action-result sentences.
- Good pattern: `This project keeps the pendulum upright on a real stepper-driven rig. It compares one linear controller and two sliding-mode controllers on a fixed dataset.`

## Chapter 1. System Hardware and Constraints

### Problems
- The chapter repeatedly calls the base actuator `closed-loop`, but the firmware and results chapters clearly treat the stepper drive as open-loop actuation with measured state feedback. This is the biggest wording conflict in the report (`thesis/chapters/01_hardware_setup.tex:19`, `thesis/chapters/01_hardware_setup.tex:42-43`, `thesis/chapters/01_hardware_setup.tex:65`, `src/main.cpp:32-36`, `src/main.cpp:1402-1437`, `thesis/chapters/10_results_discussion.tex:67`).
- The sentence `To eliminate this ambiguity` is too strong. The encoder removes ambiguity in base-angle measurement, but it does not eliminate missed-step or stall behavior (`thesis/chapters/01_hardware_setup.tex:43`, `src/main.cpp:1402-1437`).
- The bill of materials still contains many `TBD` entries, so the chapter cannot currently support a strong reproducibility claim (`thesis/chapters/01_hardware_setup.tex:173-214`).
- The actuation-model paragraph says the theoretical input `coincides` with the practical signal sent to the actuator. In reality the motor is sent a speed command after integration, and the realized motion can still differ from the commanded motion (`thesis/chapters/01_hardware_setup.tex:108-113`, `src/main.cpp:1704-1727`).

### Easy to understand or not
- The build-up order is clear and useful.
- The chapter is harder than it needs to be because many sentences are long and mechanically dense.
- The sensor-housing figure set is useful, but the reader still does not get a simple system-level picture of `Arduino -> driver -> stepper -> sensors -> mux -> logger`.

### False or weak claims
- Weak claim: `closed-loop single unit` should be changed to `measured-axis stepper unit` or `sensorized base axis` (`thesis/chapters/01_hardware_setup.tex:43`).
- Weak claim: `closed-loop base actuator` has the same problem (`thesis/chapters/01_hardware_setup.tex:65`).
- Weak claim: `To make the platform reproducible for students` is too strong while the BOM still has unresolved motor, supply, magnet, capacitor, and connector details (`thesis/chapters/01_hardware_setup.tex:15`, `thesis/chapters/01_hardware_setup.tex:187-214`).

### Additions/alterations required
- Replace `closed-loop` with `measured-axis` or `encoder-instrumented` wherever you are describing the base axis.
- Add one explicit sentence: `The encoder closes the measurement loop around theta, but the stepper drive still uses open-loop STEP/DIR actuation and can miss motion under load.`
- Add a simple block diagram of the whole rig. This is not a data plot, but it would immediately improve readability.
- Fill all `TBD` BOM entries or remove the reproducibility wording.
- Add the exact final sign settings used in the pinned dataset somewhere in the hardware/notation material: `motorSign=1`, `ALPHA_SIGN=-1`, `THETA_SIGN=1`, `CTRL_SIGN=1` (`thesis/tables/raw/metrics_trials.csv:2-8`).

### Language simplification guidance
- Use shorter hardware sentences with one mechanical idea per sentence.
- Example direction: `An AS5600 sensor measures the base angle directly. This improves state measurement, but it does not turn the stepper into a true servo motor.`

## Chapter 2. Notation, Units, and Recorded Signals

### Problems
- The chapter mixes model variables, firmware variables, and logged CSV variables without separating them clearly (`thesis/chapters/02_notation_units.tex:33-56`).
- The raw log description does not match the actual CSV schema exactly. The raw CSV contains `position_clamped`, but not a dedicated acceleration-saturation flag column (`thesis/chapters/02_notation_units.tex:52-54`, `logs/balance/session_20260209_114605/balance_log.csv:1`).
- The same symbol `u` is used in the report for both `ddot(theta)` in the model and `acc_cmd_steps_s2` in the logs. The report needs a stronger unit bridge (`thesis/chapters/02_notation_units.tex:33-56`, `thesis/chapters/06_controller_linear.tex:135-141`).
- The chapter says there is an `experimental sign-validation procedure` but does not say what the procedure is (`thesis/chapters/02_notation_units.tex:15-21`).

### Easy to understand or not
- The coordinate definitions are clear.
- The wrap-safe difference explanation is good and matches the firmware implementation (`thesis/chapters/02_notation_units.tex:23-31`, `src/main.cpp:380-384`).
- The recorded-signals section is not easy enough because it does not separate `what the firmware stores` from `what the analysis later computes`.

### False or weak claims
- False/weak claim: the list of `principal recorded signals` implies a boolean acceleration-saturation indicator is present in the raw trial log, but the sample header shows no such raw column (`thesis/chapters/02_notation_units.tex:52-54`, `logs/balance/session_20260209_114605/balance_log.csv:1`).
- Weak claim: writing `commanded base acceleration u` in `step/s^2` is mathematically okay only if the chapter clearly says the logged signal is the implementation-space version of `u`, not the SI-model-space variable (`thesis/chapters/02_notation_units.tex:52`).

### Additions/alterations required
- Split the section into three lists:
- `Model variables`: `theta`, `alpha`, `dot(theta)`, `dot(alpha)`, `u = ddot(theta)` in SI or degree units depending on chapter.
- `Firmware variables`: `accCmdPhysical`, `thetaDotCmdMotor`, `thetaErr`, `thetaDotErr`.
- `Raw CSV columns`: `alpha_deg`, `alpha_dot_deg_s`, `theta_deg`, `theta_dot_deg_s`, `acc_cmd_steps_s2`, `vel_cmd_steps_s`, `pos_meas_steps`, `position_clamped` (`logs/balance/session_20260209_114605/balance_log.csv:1`).
- Add one explicit sentence saying `thetaErr = current - reference`, matching `getAngleDiffDeg(currentDeg, targetDeg)` in the code (`src/main.cpp:380-384`).
- Add one short note saying the sign settings were validated experimentally and list the values used in the pinned dataset (`thesis/tables/raw/metrics_trials.csv:2-8`).

### Language simplification guidance
- Replace phrases like `wrap-safe shortest-path difference operator` with `shortest signed angle difference`.
- Replace `principal recorded signals` with `main columns saved in the CSV log`.

## Chapter 3. Mathematical Modelling from First Principles

### Problems
- The chapter is mathematically complete, but not beginner-friendly. It moves from geometry to Euler-Lagrange equations with very little guiding explanation (`thesis/chapters/03_modelling_derivation.tex:4-9`, `tools/modelling_complete.md:385-456`).
- The report needs a clearer line between the ideal rigid-body model and the real hardware. The model uses commanded `ddot(theta)` as an abstract input, while the real rig applies integrated speed commands and can still stall or miss motion (`tools/modelling_complete.md:385-391`, `tools/modelling_complete.md:423-429`, `src/main.cpp:194-195`, `src/main.cpp:1402-1437`).
- There is no short summary box at the end of the chapter telling the reader exactly which constants feed the firmware.

### Easy to understand or not
- Not easy for a first-year student in its current form.
- The derivation itself is fine, but the chapter needs more bridge sentences such as `what problem this equation solves`, `why this approximation is made`, and `which final constants are used later`.

### False or weak claims
- Weak claim: the reduced relation `ddot(alpha) = 100.8 alpha - 1.952 ddot(theta)` is a model relation, not proof that the real hardware realizes the commanded acceleration exactly (`tools/modelling_complete.md:427-429`, `src/main.cpp:1704-1727`, `src/main.cpp:1402-1437`).
- Weak claim: any wording that sounds like the model directly captures actuator dropouts or sensor glitches should be avoided. Those effects are handled later by firmware safeguards, not by the Chapter 3 equations (`src/main.cpp:24-37`, `src/main.cpp:1162-1179`, `src/main.cpp:1402-1437`).

### Additions/alterations required
- Add a plain-language opening paragraph after `objective` saying what the chapter produces: a nonlinear model, a linearized upright model, and the constants `A` and `B` used later in control design.
- Add an explicit assumptions list near the start: rigid links, frictionless joints, commanded base acceleration as abstract input, no missed steps, no sensor glitches, and no actuator saturation.
- Add a closing summary box with `A = 100.8`, `B = 1.952`, `k_s = 4.444 step/deg`, and a one-line note that these values match the firmware constants (`tools/modelling_complete.md:427-429`, `src/main.cpp:12`, `src/main.cpp:100-103`).
- Make all rewrite changes in `tools/modelling_complete.md`, not in the generated LaTeX file.

### Language simplification guidance
- Keep the equations, but add a short plain sentence before and after each major derivation block.
- Example direction: `This equation tells us how fast the pendulum angle changes when the base is accelerated.`

## Chapter 4. Firmware Pipeline: Measurement and Estimation

### Problems
- The chapter says each tick uses measured `dt` for all discrete-time updates. That is too strong. The derivative filter coefficients are computed from the nominal 5 ms period and reused; they are not recalculated from the measured per-tick `dt` (`thesis/chapters/04_pipeline_measurement.tex:5-10`, `thesis/chapters/04_pipeline_measurement.tex:24-34`, `src/main.cpp:354-358`, `src/main.cpp:1215-1235`, `src/main.cpp:1341-1363`).
- The derivative section does not say that both derivatives are clamped to `±300 deg/s`, which is an important practical implementation detail (`src/main.cpp:20-21`, `src/main.cpp:1223-1235`, `src/main.cpp:1350-1363`).
- The glitch section is good, but it could more clearly separate `raw sample rejection` from `derivative-difference zeroing`.

### Easy to understand or not
- Medium difficulty. The control-timing section is readable.
- The Tustin-derivation paragraph becomes formal too quickly for a first-year reader.

### False or weak claims
- Weak claim: `uses the measured dt for all discrete-time updates` should be changed to `uses measured dt for the main integrators and reference updates; the derivative filter uses nominal-period coefficients` (`thesis/chapters/04_pipeline_measurement.tex:8`, `src/main.cpp:354-358`).

### Additions/alterations required
- Add the actual filter clamp values: `ALPHADOT_CLAMP = 300 deg/s`, `THETADOT_CLAMP = 300 deg/s`, `ALPHA_JUMP_REJECT_DEG = 30 deg`, `THETA_JUMP_REJECT_DEG = 30 deg` (`src/main.cpp:20-27`).
- Add one line stating that the nominal coefficients for `omega_c = 450 rad/s` are `b0 = 211.7647` and `a1 = 0.0588`, matching the events log (`src/main.cpp:356-358`, `logs/balance/session_20260209_114605/events.txt:5`).
- Reword the control-loop paragraph so it does not suggest the derivative filter is fully variable-`dt`.
- If you keep the Tustin derivation, add one line in plain English: `This filter is a smoothed numerical derivative.`

### Language simplification guidance
- Replace derivation-heavy transitions with direct statements.
- Example direction: `The derivative is estimated with a filtered difference, not with a raw sample-to-sample subtraction, because the sensor is noisy.`

## Chapter 5. Firmware Pipeline: Actuation and Safety

### Problems
- The chapter omits the motor speed deadband `speedStopHz = 50`, even though that setting affects low-speed behavior and helps explain calm linear hold behavior near zero velocity (`src/main.cpp:208-211`, `src/main.cpp:445-457`, `logs/balance/session_20260209_114605/events.txt:3,13`).
- The chapter does not restate the important distinction between measured base feedback and open-loop STEP/DIR actuation.

### Easy to understand or not
- This is one of the clearer technical chapters.
- The stall-detection paragraph is accurate but too long. It will be easier to read if the threshold, persistence, and diagnostic purpose are separated.

### False or weak claims
- No major false statement was found here.
- Weak omission: the current text gives a complete-looking actuation picture without mentioning the velocity deadband that is active in firmware.

### Additions/alterations required
- Add `speedStopHz = 50 step/s` to the actuation-pipeline description and to the experiment-parameter chapter (`src/main.cpp:211`, `logs/balance/session_20260209_114605/events.txt:3,13`).
- Add one sentence saying `The base encoder measures theta for control and safety, but the motor command is still an open-loop speed command.`
- Keep the stop-before-reverse paragraph, but mention that it is implemented by forcing a stop on the current tick and allowing reversal only on a later tick (`src/main.cpp:462-468`).
- Split the stall paragraph into threshold, persistence, and diagnostic output.

### Language simplification guidance
- Use direct `if-then` phrasing in the safety sections.
- Example direction: `If the motor is commanded to move but theta does not change for 30 control ticks, the firmware prints a stall warning.`

## Chapter 6. Linear Full-State Feedback Controller

### Problems
- The chapter moves from an analytic pole-placement example to the deployed experimental gains without clearly saying these are different things (`thesis/chapters/06_controller_linear.tex:99-125`, `thesis/chapters/06_controller_linear.tex:160-179`, `src/main.cpp:57-61`).
- The nudge pause/resume description says the reference is frozen without chasing the measured angle, but the firmware does a one-shot handoff `thetaRefDeg = thetaDeg` at the moment pause is entered (`thesis/chapters/06_controller_linear.tex:146-148`, `src/main.cpp:1481-1487`).
- The implementation-details section omits the active motor speed deadband `speedStopHz = 50` (`src/main.cpp:208-211`, `src/main.cpp:445-457`).

### Easy to understand or not
- The mathematics is coherent, but it is still heavy for a first-year student.
- The chapter would be much easier if it first said what each gain does in words before showing coefficient matching.

### False or weak claims
- Weak claim: the numerical example can be misread as the actual implemented parameter set. The example produces `[-9.92, -858.4, -20.9, -69.9]` in stepper units, while the pinned experiments used positive magnitudes `[31.5, 725.6, 28.3, 73.5]` with separate sign settings and additional tuning (`thesis/chapters/06_controller_linear.tex:118-124`, `thesis/chapters/06_controller_linear.tex:172-175`, `src/main.cpp:55-61`, `logs/balance/session_20260209_114605/events.txt:4,15`).
- Weak claim: `reference is frozen` needs a small correction because the code first hands the reference to the current measured theta once and then holds it (`thesis/chapters/06_controller_linear.tex:146-148`, `src/main.cpp:1481-1487`).

### Additions/alterations required
- Add one sentence immediately after the pole-placement example: `This is a design example, not the final tuned parameter set used in the experiments.`
- Add one short explanation of why the deployed gains differ from the analytic example: sign convention handling, stronger base centering, and practical tuning on hardware (`src/main.cpp:57-61`).
- Add `speedStopHz = 50 step/s` to the implementation details.
- Clarify the pause logic: `when instability is confirmed, the reference is handed once to the current theta and then held until the pendulum settles again`.
- If possible, rewrite the rate-gain units in the clearer Chapter 9 style: `step/s^2 per (deg/s)` rather than the shorter but less obvious `step/s/deg` (`thesis/chapters/06_controller_linear.tex:174-175`, `thesis/chapters/09_experiments_processing.tex:67-68`).

### Language simplification guidance
- Start the chapter with: `This controller is the baseline. It uses all four states and tries to keep the pendulum upright while keeping the arm near the reference angle.`
- For the pole-placement subsection, explain the meaning of `fast pendulum mode` and `slow base-centering mode` in plain language before the equations.

## Chapter 7. Hybrid Sliding Mode Controller

### Problems
- The ideal SMC derivation is not clearly separated from the implemented controller, which includes actuator limits, gating, sign handling, alpha-dot abort logic, and a base-centering assist (`thesis/chapters/07_controller_hybrid_smc.tex:54-92`, `thesis/chapters/07_controller_hybrid_smc.tex:94-120`, `src/main.cpp:124-136`, `src/main.cpp:1528-1575`, `src/main.cpp:1680-1717`).
- The base-centering assist equation uses `theta_ref - theta`, but the linear chapter and the firmware use `thetaErr = theta - thetaRef` with positive gains. The sign convention is inconsistent on paper (`thesis/chapters/07_controller_hybrid_smc.tex:104-113`, `thesis/chapters/06_controller_linear.tex:127-141`, `src/main.cpp:1506-1508`, `src/main.cpp:1561-1564`).
- The experiments table omits `smcSign`, even though the firmware includes it as a real implementation parameter (`src/main.cpp:108`, `thesis/tables/raw/metrics_trials.csv:2-8`).

### Easy to understand or not
- Medium difficulty. The chapter is shorter than the modelling chapter, which helps.
- Still too abstract for a first-year student because `equivalent control`, `reaching law`, and `Lyapunov` are introduced with almost no plain-language preparation.

### False or weak claims
- Weak claim: `reaching is finite-time` and `alpha decays approximately exponentially` are properties of the ideal control law inside the validity assumptions. They are not unconditional guarantees for the implemented saturated, sampled, stepper-driven controller (`thesis/chapters/07_controller_hybrid_smc.tex:76-92`, `src/main.cpp:1539-1557`, `src/main.cpp:1675-1687`, `src/main.cpp:1402-1437`).
- Weak claim: `exact (nonlinear) upright relation` should be read as exact within the reduced rigid-body model, not exact for the whole real plant (`thesis/chapters/07_controller_hybrid_smc.tex:4-20`, `src/main.cpp:194-195`, `src/main.cpp:1402-1437`).
- Sign inconsistency: the printed base-assist equation is not aligned with the code and with the linear chapter (`thesis/chapters/07_controller_hybrid_smc.tex:104-113`, `src/main.cpp:1561-1564`).

### Additions/alterations required
- Add a `Theory versus firmware` note after the Lyapunov section. It should say the proof applies to the analytic law before saturation, deadband, stalls, and abort guards.
- Rewrite the base-assist equation so it uses the same error definition as Chapter 6, or explicitly define a new error sign and keep it consistent.
- Add `smcSign` to the implementation-parameter discussion, even if it stayed at `+1` in the pinned dataset (`thesis/tables/raw/metrics_trials.csv:2-8`).
- Add one plain-language sentence saying the base-centering assist is a practical fix for drift, not part of the classical pendulum-only surface.

### Language simplification guidance
- Define terms in one line before using them.
- Example direction: `The sliding surface is a combined error signal. If this signal goes to zero, the pendulum angle also goes back toward zero.`

## Chapter 8. Full-Surface Sliding Mode Controller (Four-State Surface)

### Problems
- The gap between analytic controller and implemented controller is even larger here than in Chapter 7. The firmware does not apply the clean symbolic law directly; it uses `kEff`, ramping, denominator margin enforcement, clamped `thetaErr`, clamped `thetaDotErr`, clamped `thetaTerm`, clamped internal `alphaDot`, and tighter actuator caps (`thesis/chapters/08_controller_smc4_full_surface.tex:57-117`, `src/main.cpp:1581-1657`, `src/main.cpp:1680-1717`).
- The parameter summary omits the added base-centering assist scale, even though the chapter says that assist is implemented (`thesis/chapters/08_controller_smc4_full_surface.tex:116-117`).
- The chapter is mathematically dense for the intended simplified writing style.

### Easy to understand or not
- Not easy enough for a first-year student.
- The denominator discussion is important, but the chapter needs a much more direct physical explanation: `if k gets too close to B cos(alpha), the controller loses authority and asks for very large accelerations`.

### False or weak claims
- Weak claim: the reaching and Lyapunov statements again need an explicit caveat that they describe the analytic law, not the full saturated firmware implementation (`thesis/chapters/08_controller_smc4_full_surface.tex:87-101`, `src/main.cpp:1603-1636`, `src/main.cpp:1680-1687`).
- Weak claim: the printed control law uses `k`, but the firmware actually uses `kEff` after ramping and denominator-margin correction (`thesis/chapters/08_controller_smc4_full_surface.tex:68-85`, `src/main.cpp:1603-1623`).

### Additions/alterations required
- Add an explicit sentence under the control law: `Equation (...) is the analytic form; the firmware uses an effective coupling kEff after ramping and denominator protection.`
- Add the assist scale parameter to the experiment summary if it affected the runs, or state explicitly that it stayed at the default value.
- Add one plain-language interpretation of `thetaTerm = thetaDotErr + lambda_theta thetaErr` so the reader knows it is a combined base-tracking error.
- Keep the denominator-protection subsection, but simplify it heavily and connect it to the firmware fields `den`, `kEff`, and `kRamp` printed in status lines (`src/main.cpp:1758-1763`).

### Language simplification guidance
- Introduce each symbol in words before the boxed equation.
- Example direction: `SMC4 tries to control pendulum angle and base-position error together using one combined surface.`

## Chapter 9. Experiments and Data Processing

### Problems
- The chapter is well organized, but the long parameter table is dense and hard to scan.
- The table omits `speedStopHz = 50`, which is a real shared parameter in the pinned runs (`src/main.cpp:211`, `logs/balance/session_20260209_114605/events.txt:3,13`).
- The chapter should say more explicitly that the tap runs do not contain explicit disturbance markers, so true impulse-response timing metrics are not available in this dataset (`experiments/manifest_thesis_20260209.json:24-37` via the notes text already reflected in `thesis/tables/raw/metrics_trials.csv:6-8` and the future-work note `thesis/chapters/11_conclusion_future.tex:19-20`).

### Easy to understand or not
- The structure is good: dataset, parameters, logging, alignment, metrics.
- The big longtable is not easy for a first-year reader. It would be easier as a short main table plus an appendix table.

### False or weak claims
- No major false statement was found here.
- Weak claim risk: `reproducible` is acceptable for the fixed dataset selection, but it should not be read as `statistically representative` (`thesis/chapters/09_experiments_processing.tex:4-5`).

### Additions/alterations required
- Add `speedStopHz = 50 step/s` to the shared or actuation-related parameter list.
- Add the final sign settings used in the pinned dataset somewhere in this chapter or in Chapter 2, because they affect reproducibility (`thesis/tables/raw/metrics_trials.csv:2-8`).
- Add one line under metrics saying that nudge rows can show `--` when the available post-step window is too short or the settle criterion is not met (`thesis/tables/raw/metrics_nudge_steps.csv:2-4`, `thesis/chapters/10_results_discussion.tex:43`).
- Add one line under tap experiments saying that the reported tap metrics are whole-trial metrics, not disturbance-onset metrics, because the taps are unmarked.

### Language simplification guidance
- Shorten the prose around the longtable and tell the reader what to look for.
- Example direction: `Table X lists the exact settings used in the pinned dataset. These are the settings that produced the results in Chapter 10.`

## Chapter 10. Results and Discussion

### Problems
- The nudge interpretation is too positive relative to the actual step table. All three steps have `settle = --`, step 0 has no reported rise time, and step 1 overshoot reaches `9.61 deg` (`thesis/chapters/10_results_discussion.tex:36-46`, `thesis/tables/tab_nudge_steps.tex:13-15`, `thesis/tables/raw/metrics_nudge_steps.csv:2-4`).
- The tap-effort statement is not supported as written. Hybrid SMC does not have larger RMS effort than linear in the pinned tap trials; only its peak effort is higher. Full-surface SMC is higher in both RMS and capped peak (`thesis/chapters/10_results_discussion.tex:52-57`, `thesis/tables/tab_summary_tap.tex:13-15`).
- The chapter uses only the time-series plots even though the export pipeline already generated phase portraits, effort histograms, alpha spectra, and SMC4 sparse diagnostics for the pinned dataset (`tools/export_thesis_assets.py:60-88`, `tools/analyze_experiments.py:597-654`).
- The current figure captions are too short. They label the figure but do not tell the reader what to notice (`thesis/chapters/10_results_discussion.tex:29-31`, `thesis/chapters/10_results_discussion.tex:46`, `thesis/chapters/10_results_discussion.tex:63-65`).
- The plotting code itself leaves important cues out:
- The alpha subplot has no hard-limit or SMC-abort guide lines (`tools/balance_analysis/plots.py:50-56`).
- The SMC4 effort plots use the generic `±20000 step/s^2` scale, even though SMC4 is actually capped at `12000 step/s^2`, so the limit context is weaker than it should be (`tools/balance_analysis/plots.py:68-76`, `tools/analyze_experiments.py:597-613`, `src/main.cpp:151-152`, `src/main.cpp:1680-1685`, `thesis/tables/tab_summary_hold.tex:15`, `thesis/tables/tab_summary_tap.tex:15`).
- The nudge plot does not overlay the `±1 deg` settle band used in the table note (`tools/balance_analysis/plots.py:129-193`, `thesis/chapters/10_results_discussion.tex:43`).

### Easy to understand or not
- The chapter is readable, but it makes the reader infer too much.
- It is not easy enough for a first-year student because the captions and plot selection do not explain the comparison logic clearly.

### False or weak claims
- Weak claim: `tracks the commanded sequence with finite rise times and limited overshoot` is too strong for the pinned N1 evidence (`thesis/chapters/10_results_discussion.tex:39`, `thesis/tables/tab_nudge_steps.tex:13-15`).
- False/weak claim: `Both sliding-mode variants command larger acceleration effort during recovery` is not supported if `effort` means RMS effort. Hybrid SMC tap RMS effort is `1493.067 step/s^2`, which is lower than linear tap RMS effort `1695.658 step/s^2`. Hybrid SMC only exceeds linear in peak effort `15384` vs `12717` (`thesis/tables/tab_summary_tap.tex:13-15`, `thesis/tables/raw/metrics_trials.csv:6-8`).
- Weak claim: `Linear provides the calmest steady balancing` and `Hybrid SMC provides the best peak suppression` are okay only if you say `among the three pinned hold trials, and for the specific metrics alpha RMS and max |alpha|` (`thesis/chapters/10_results_discussion.tex:19-24`, `thesis/tables/tab_summary_hold.tex:13-15`).

### Additions/alterations required
- `already available`: add the phase portraits for hold and tap. These were already exported to `thesis/figures/*_phase_alpha.pdf` and generated from the pinned trials (`tools/export_thesis_assets.py:64-72`, `tools/analyze_experiments.py:606-608`).
- `already available`: add the effort histograms for hold, nudge, and tap. These already exist in `thesis/figures/*_effort_hist.pdf` (`tools/export_thesis_assets.py:64-72`, `tools/analyze_experiments.py:611-613`).
- `already available`: add the low-frequency alpha spectra, but state clearly that the logger is about `50 Hz`, so the spectrum is meaningful only up to about `25 Hz` and cannot support high-frequency chattering claims (`tools/balance_analysis/plots.py:196-223`, `tools/analyze_experiments.py:649-654`).
- `already available`: add the SMC4 sparse diagnostic plots `s`, `den`, `kEff`, and `kRamp` for H3 and T3 (`tools/export_thesis_assets.py:66,72`, `tools/analyze_experiments.py:616-647`).
- `needs new plot generation from existing data`: add controller-comparison bar charts for hold metrics and tap metrics using the pinned metric table (`thesis/tables/raw/metrics_trials.csv:2-8`).
- `needs new plot generation from existing data`: add nudge step-response panels with the `±1 deg` settle band overlaid, because the current table already shows that the steps did not meet the settle criterion within the available windows (`thesis/tables/tab_nudge_steps.tex:13-15`, `thesis/tables/raw/metrics_nudge_steps.csv:2-4`).
- `needs new plot generation from existing data`: use common y-axis limits across the three hold plots and across the three tap plots so the comparison is visually fair.
- `needs new plot generation from existing data`: use mode-specific effort-limit lines, especially for SMC4, where the meaningful cap is `12000 step/s^2`, not `20000 step/s^2` (`src/main.cpp:151-152`, `src/main.cpp:1680-1685`).
- `needs new plot generation from existing data`: add alpha-limit guide lines (`±30 deg` hard limit and `±25 deg` SMC validity/abort line where relevant) to the alpha subplot.
- Rewrite the captions so each caption answers `what is this figure showing`, `what should the reader notice`, and `did the run end successfully or by falling`.

### Language simplification guidance
- Use metric-first writing, not adjective-first writing.
- Example direction: `In the pinned tap trials, the linear controller stayed upright for 31.293 s. The hybrid SMC and SMC4 trials both ended by falling.`

## Chapter 11. Conclusion and Future Work

### Problems
- The conclusion is mostly fair, but it should repeat the `one pinned trial per condition` limitation more explicitly.
- The reference-tracking conclusion is stronger than the nudge evidence.

### Easy to understand or not
- Fairly easy to read.
- Still more formal than needed for the target simplified style.

### False or weak claims
- Weak claim: `Reference tracking while balancing ... converting the system from a regulator to a servo without sacrificing upright stability` is too smooth for the actual pinned nudge evidence. The trial stayed upright, but none of the three steps met the stated settle-time criterion within the available windows, and step 1 overshoot reached `9.61 deg` (`thesis/chapters/11_conclusion_future.tex:9`, `thesis/tables/tab_nudge_steps.tex:13-15`).
- Weak claim: `linear and hybrid SMC controllers achieved stable hold-at-center behavior` is true for the pinned hold runs, but it should stay clearly tied to that limited dataset (`thesis/chapters/11_conclusion_future.tex:12`, `thesis/tables/tab_summary_hold.tex:13-14`).

### Additions/alterations required
- Add one sentence in the conclusion that all controller comparisons in this thesis are qualitative because each experiment type uses one pinned representative trial per controller.
- Rephrase the nudge conclusion as `The linear controller showed that balance-with-reference-tracking is feasible on this rig in one pinned trial.`
- Add one future-work item about cleaning up the thesis itself before submission: resolve `closed-loop` wording, remove `TBD` hardware items, and simplify language.

### Language simplification guidance
- End the report with 3-4 short plain statements: what was built, what worked best on this rig, what failed under tap disturbances, and what limits remain.

## Appendix A. Cross-check Matrix

| Claim or parameter | Status | Evidence used | Audit note |
| --- | --- | --- | --- |
| Base conversion `k_s = 4.444 step/deg` | Confirmed | `thesis/chapters/01_hardware_setup.tex:36-40`; `src/main.cpp:12`; `thesis/chapters/09_experiments_processing.tex:42` | Consistent across thesis and firmware. |
| Hard limits `|theta| = 80 deg`, `|alpha| = 30 deg` | Confirmed | `thesis/chapters/01_hardware_setup.tex:227-235`; `src/main.cpp:13-14`, `src/main.cpp:1294-1315`; `thesis/chapters/09_experiments_processing.tex:47-49` | Correct and consistent. |
| Nonlinear-controller abort `|alpha| = 25 deg`, `|alphaDot| = 250 deg/s` | Confirmed | `thesis/chapters/01_hardware_setup.tex:232`; `thesis/chapters/07_controller_hybrid_smc.tex:97-100`; `thesis/chapters/08_controller_smc4_full_surface.tex:107-117`; `src/main.cpp:124-128`, `src/main.cpp:1322-1388` | Correct, but Chapters 7-8 should state these guards limit the ideal-theory claims. |
| Derivative filter default `omega_c = 450 rad/s`, `b0 = 211.7647`, `a1 = 0.0588` | Confirmed | `thesis/chapters/04_pipeline_measurement.tex:29-34`; `src/main.cpp:66`, `src/main.cpp:354-358`; `logs/balance/session_20260209_114605/events.txt:5` | Good numeric agreement. |
| `Measured dt is used for all discrete-time updates` | Needs correction | `thesis/chapters/04_pipeline_measurement.tex:8`; `src/main.cpp:354-358`, `src/main.cpp:1215-1235`, `src/main.cpp:1341-1363` | True for some updates, not for derivative coefficients. |
| Linear experimental gains `31.5, 725.6, 28.3, 73.5` | Confirmed | `thesis/chapters/06_controller_linear.tex:172-175`; `src/main.cpp:58-61`; `logs/balance/session_20260209_114605/events.txt:4,15` | Correct for the pinned runs. |
| Analytic linear example equals deployed gains | Needs correction | `thesis/chapters/06_controller_linear.tex:99-124`; `thesis/chapters/06_controller_linear.tex:160-179`; `src/main.cpp:57-61` | The example and deployed tuned gains are different and should be labeled clearly. |
| Model constants `A = 100.8`, `B = 1.952` | Confirmed | `tools/modelling_complete.md:427-429`; `src/main.cpp:100-103`; `thesis/chapters/07_controller_hybrid_smc.tex:16-19`; `thesis/chapters/08_controller_smc4_full_surface.tex:55` | Consistent across model and firmware. |
| Raw trial log contains saturation and clamp booleans | Needs correction | `thesis/chapters/02_notation_units.tex:52-54`; `logs/balance/session_20260209_114605/balance_log.csv:1` | Raw CSV clearly shows `position_clamped`, but not a raw saturation column. |
| Hold result: linear has lowest `alpha RMS`, hybrid SMC has lowest `max |alpha|` | Confirmed for pinned hold trials | `thesis/tables/tab_summary_hold.tex:13-15`; `thesis/chapters/10_results_discussion.tex:19-24` | Phrase it as a pinned-trial result, not a general controller ranking. |
| Nudge result: `limited overshoot` and good settling | Needs correction | `thesis/chapters/10_results_discussion.tex:39-43`; `thesis/tables/tab_nudge_steps.tex:13-15`; `thesis/tables/raw/metrics_nudge_steps.csv:2-4` | Overshoot and unmet settling are more mixed than the text suggests. |
| Tap result: both sliding modes have larger effort than linear | Needs correction | `thesis/chapters/10_results_discussion.tex:52-57`; `thesis/tables/tab_summary_tap.tex:13-15`; `thesis/tables/raw/metrics_trials.csv:6-8` | True for SMC4 RMS and for both sliding modes in peak effort, but not true for hybrid SMC RMS effort. |
| Results are qualitative, not statistical | Confirmed | `thesis/chapters/10_results_discussion.tex:7`, `thesis/chapters/10_results_discussion.tex:77`; `experiments/manifest_thesis_20260209.json:1-37` | This caveat is correct and should be repeated in the conclusion. |
| Spectrum plots can support high-frequency chattering claims | False if implied | `tools/balance_analysis/plots.py:196-223` | The plot code itself says the spectra are low-frequency only because the logger is about 50 Hz. |

## Appendix B. Rewrite Guide

### Style rules for the full rewrite
- Keep equations, symbols, and citations.
- Use one idea per sentence.
- Prefer `what the system does` before `how the math is written`.
- Define each technical term once in plain words before the formal equation.
- Tie every result claim to a specific metric or table.
- Avoid words like `therefore`, `hence`, `equivalent control`, and `robustness` unless the sentence also explains them plainly.

### Sample rewrites

| Current idea | Simpler version |
| --- | --- |
| `This project studies upright-only stabilization of a rotary (Furuta) inverted pendulum under real hardware constraints...` | `This project tries to keep a rotary inverted pendulum upright on real hardware, where the motor, sensors, and timing are not perfect.` |
| `Acceleration-input control matched to stepper actuation` | `The controller is written in terms of base acceleration, and the firmware then converts that into a motor speed command.` |
| `To eliminate this ambiguity, the motor was retrofitted as a closed-loop single unit` | `A base encoder was added so the report uses measured base angle instead of only commanded step count.` |
| `For control and reference tracking we use a wrap-safe shortest-path difference operator` | `When two angles cross 0/360 deg, we always take the shortest signed angle difference.` |
| `Direct finite differences are too noisy at 200 Hz` | `A raw sample-to-sample derivative was too noisy, so the firmware uses a filtered derivative instead.` |
| `This form enables hand-calculable coefficient matching` | `This form lets the gains be calculated by matching the desired polynomial coefficients.` |
| `The sliding surface encodes the desired transient behavior` | `The sliding surface is one combined error signal. If it goes to zero, the pendulum motion also settles down.` |
| `This structure preserves nonlinear robustness during recovery while providing a practical centering mechanism when conditions are safe` | `This part helps keep the arm near the reference, but it is only turned on when the pendulum is already near upright.` |
| `The pinned run tracks the commanded sequence with finite rise times and limited overshoot` | `In the pinned nudge run, the arm follows the commanded targets, but the settling is incomplete in the available windows and one step overshoots by 9.61 deg.` |
| `The observed tap behavior reflects practical actuator constraints and saturation dynamics rather than a contradiction of sliding-mode robustness theory` | `The tap results are limited mainly by the real stepper hardware. The controller can ask for strong recovery motion, but the motor may not produce that motion exactly.` |

## Priority Fix List

1. Resolve the `closed-loop` versus `open-loop measured-axis stepper` wording conflict across Chapters 0, 1, 5, 10, and 11.
2. Add explicit `ideal theory versus implemented firmware` caveats to Chapters 7 and 8.
3. Separate the analytic linear-gain example from the tuned experimental gains in Chapter 6.
4. Rewrite the nudge results and conclusion to match the actual step metrics, especially the missing settle times and the `9.61 deg` overshoot.
5. Correct the tap-effort statement in Chapter 10 so it distinguishes RMS effort from peak effort.
6. Fix the Chapter 2 logged-signal description so it matches the raw CSV schema.
7. Add the missing implementation parameters that materially affect behavior, especially `speedStopHz = 50`, and list the final sign settings used in the pinned dataset.
8. Improve Chapter 10 figure usage and captions, and use the already-generated plots that are currently ignored.
9. Fill or remove the remaining `TBD` BOM items before submission if reproducibility is a stated goal.
