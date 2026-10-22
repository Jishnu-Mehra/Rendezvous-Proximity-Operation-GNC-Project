# RPO GNC Project — Development Log

MATLAB/Simulink/Stateflow spacecraft rendezvous and proximity operations (RPO) simulation.

**Purpose:** Record design decisions, implementations, tests, numerical results, failures, and corrective actions. This is a development record, not a claim that the complete system has been validated. Values marked *provisional* are assumptions to be revisited before the final report.

**Units and frame:** SI units. Relative LVLH state `X = [x; y; z; vx; vy; vz]`, with `x` radial, `y` along-track and `z` cross-track. `X_true` denotes truth and `X_hat` the navigation estimate.

## 1. Mission definition and architecture

- **Objective:** Simulate a chaser rendezvous with a target in low Earth orbit, beginning approximately 10 km behind the target, approaching a 1 km hold point, and ultimately extending to close approach and docking.
- **Reference orbit:** Circular orbit at 450 km altitude; Earth gravitational parameter `mu = 3.986004418e14 m^3/s^2`, Earth radius `Re = 6,378,137 m`, orbital radius `rt = Re + 450 km`. Mean motion `n = sqrt(mu/rt^3)` and period `T = 2*pi/n` (approximately 5,615 s).
- **Intended architecture:** Stateflow guidance and CW targeting → translation controller → thruster execution/allocation → nonlinear relative truth dynamics; sensor and navigation feedback; later coupled attitude GNC and full 6-DoF dynamics.
- **Design principle:** Impulsive CW manoeuvres are the analytical guidance benchmark; physical thrusters generate finite force over time. Do not conflate guidance-level `Delta-V` commands with actuator-level acceleration or firing commands.

## 2. Relative dynamics and CW model validation

**Implemented**

- Nonlinear relative dynamics in a Simulink MATLAB Function block, integrated as a six-state continuous system.
- Parallel linear Clohessy–Wiltshire (CW) state-space model with matrices `A` (6×6) and `B = [zeros(3); eye(3)]` (6×3).
- Zero-input comparisons of linear and nonlinear propagation.

**Tests and results**

- For `x0 = [100; 0; 0; 0; -2*n*100; 0]`, propagated for `3*T`: observed maximum radial difference approximately **0.041 m**, along-track difference approximately **0.0034 m**, negligible cross-track difference. This supported correct implementation in the small-separation test.
- For the approximately 10 km Phase 0 handoff state, linear/nonlinear differences were much larger over multiple orbits (up to approximately **830 m** in a reported component). This was not reconciled fully and should not be interpreted as a universal CW accuracy bound.
- Recorded handoff state, when `handoff.mat` is available:

  ```matlab
  x0 = [-7.32270070294746;
        -9999.99493384722;
        -1.36321816057716e-08;
         2.86729218146320e-09;
         7.00456480543649e-08;
         8.30269186735677e-12];
  ```

**Issues and fixes**

- Initial Simulink dimension errors were resolved by explicitly setting six-element state and three-element acceleration signal sizes, rather than forcing dimensions with `reshape`.
- **Report note:** Explain CW assumptions and quantify the validity region, particularly for 10 km separations and long propagation intervals.

## 3. Translation LQR controller

**Implemented**

- Continuous-time state-feedback law `u_LQR = -K*(X_hat - X_ref)` with `K = lqr(A,B,Qw,Rw)`.
- Initial weights: `Qw = diag([ones(1,3)/100^2, ones(1,3)/0.5^2])`, `Rw = eye(3)/0.01^2`.
- Controller selection via Simulink Switch: `use_lqr = true` selects LQR; otherwise selects the guidance acceleration command. Switch criterion `u2 ~= 0`, LQR on top data port and guidance on bottom data port.
- Provisional actuator acceleration saturation of **±0.01 m/s² per axis** in the Thrusters subsystem.

**Tests and results**

- Initial closed-loop test from `[30; -100; 20; 0; 0; 0]` to `[0; -20; 0; 0; 0; 0]` converged in simulation.
- Later guidance simulations showed LQR reducing errors after the far-range transfer, but this must not be used to claim the guidance manoeuvres themselves achieved accurate arrival.

**Report note:** Record eigenvalues of `A-B*K`, settling times, control effort, saturation behaviour and sensitivity to tuning in future tests.

## 4. Sensors and navigation

**Implemented**

- Provisional position-only relative measurement: `z = H*X_true + noise`, `H = [eye(3), zeros(3)]`.
- Independent Gaussian measurement noise with standard deviation **1 m per axis** (*provisional*).
- Discrete six-state linear Kalman filter at **1 Hz**, using exact zero-order-hold CW matrices `Phi`, `Gam` and process covariance `Qd` (Van Loan method).
- Initial covariance standard deviations: **5 m** position and **0.05 m/s** velocity.
- Process acceleration-noise PSD parameter `q = 1e-9` in the setup script; provisional truth thrust-gain error `thr_err = 0.02`.
- Applied acceleration fed to the filter through a one-sample delay. Joseph-form covariance update and covariance symmetrization implemented.

**Status / limitations**

- KF implemented and connected into the simulation; a comprehensive filter-consistency assessment (e.g., innovation statistics, NEES/NIS, 3-sigma coverage) has **not** been documented yet.
- This is a **linear KF**, not an EKF. Nonlinear range/bearing measurements and EKF are planned for later phases.

## 5. Analytical far-range rendezvous targeting

**Implemented**

- CW two-impulse targeting with transfer time `t_tr = 0.75*T`, avoiding known problematic transfer times near multiples of half an orbit.
- Desired 1 km V-bar hold state `X_hold = [0; -1000; 0; 0; 0; 0]`.
- Used CW state-transition partition:

  ```text
  Phi(tf) = [Prr Prv; Pvr Pvv]
  v_depart = Prv \ (rf - Prr*r0)
  dv1 = v_depart - v0
  dv2 = vf - (Pvr*r0 + Pvv*v_depart)
  ```

**Standalone test results**

- `dv1 ≈ [-0.9113; -0.4393; 0] m/s`.
- `dv2 ≈ [-0.9031; +0.4557; 0] m/s`.
- Total impulsive budget approximately **2.03 m/s**, excluding midcourse corrections.
- Applying these as ideal velocity jumps in the CW model produced terminal-state residuals effectively zero: only a reported cross-track position residual of **2.09e-25 m** (numerical roundoff).
- **Validated scope:** The *ideal impulsive CW algebra*, not finite-duration thrust execution or nonlinear closed-loop arrival.

**Issues and fixes**

- MATLAB `expm` initially failed inside a test function despite working in the workspace. Diagnosed input-dimension/function-interface concerns; the submitted function also incorrectly used `return [dv_1,dv_2]`. Corrected by assigning outputs `dv1` and `dv2` directly and ending the function with `end`.
- Workspace `Phi` is the **1-second KF transition**, not the transfer STM; use a separate name such as `Phit` for transfer targeting.

## 6. Stateflow far-range guidance integration

**Implemented**

- Seven sequential states: `Coast_start → Burn1 → Transfer → Midcourse → Transfer2 → Burn2 → Hold`.
- Chart inputs: `x_hat` and simulation clock `t`.
- Chart outputs: guidance acceleration `u_guid`, reference state `X_ref`, Boolean `use_lqr`, diagnostic `mode` (1–7).
- Local burn-command vectors latched on entry: `dv1_cmd`, `dv_mid_cmd`, `dv2_cmd`.
- Stateflow-local CW targeting function using analytical CW position transition blocks, avoiding a continuously recalculated external targeting feedback loop.
- First and midcourse burns compute CW corrections from the estimated state. Arrival burn initially uses `dv2_cmd = -x_hat(4:6)`.
- `Hold` enables LQR; coast states command zero guidance acceleration.

**Timing currently used**

- First burn starts at `t_burn0 = 60 s`.
- Provisional burn duration `t_burn = 100 s` for each of the three burns.
- Burn midpoints scheduled so first-to-arrival midpoint interval equals `t_tr = 0.75*T`.
- Setup script defines `t_depart`, `t_mid`, `t_arrival`, and start/end times for all burns.
- Current state transitions are **time-based**; later versions should check commanded/delivered manoeuvre completion and navigation/attitude readiness.

**Discarded implementation approaches**

- Separate `Delta V Calc` and `Remaining Time` Simulink blocks driven by Stateflow `mode`, with a mode Unit Delay. This approach was abandoned because continuously evaluated commands and delayed mode can produce stale or zero values at state entry.
- Earlier 30-second fixed burns were too short for the provisional ±0.01 m/s² per-axis acceleration limit: a ~0.91 m/s component requires at least ~91 s at that limit.

## 7. Integrated far-range simulation and diagnostics

**Results (current baseline)**

- Stateflow progressed through all seven modes and entered `Hold` at **t = 4373.0 s**.
- Truth state at hold entry, rounded from logged values: position approximately **[68.8; -1061.4; 0.1] m**.
- Relative to `[0; -1000; 0] m`: **position error 92.201 m** and **velocity error 0.131770 m/s**.
- LQR subsequently reduced the error, but guidance-only arrival was **not yet satisfactory**.

**Actuation measurements**

- Integrating applied acceleration over the *entire run* gave net components **[-2.2924; -0.0145; 0.0184] m/s**. This includes HOLD/LQR corrections and is **not** the manoeuvre ΔV budget.
- Maximum applied absolute acceleration over the run: **[0.0100; 0.0086; 0.0001] m/s²**.
- Isolating the three planned burn windows gave:

  | Burn | Delivered ΔV components (m/s) | Magnitude (m/s) | Max. abs. acceleration (m/s²) |
  |---|---|---:|---|
  | Burn 1 | `[-0.8996; -0.4468; 0.0053]` | 1.0045 | `[0.00904; 0.00449; 0.00005]` |
  | Midcourse | `[-0.1451; 0.0419; 0.0076]` | 0.1513 | `[0.00147; 0.00043; 0.00008]` |
  | Burn 2 | `[-0.9850; 0.4133; 0.0056]` | 1.0682 | `[0.01000; 0.00420; 0.00006]` |

- Burn 2 commanded mean acceleration approximately **[-0.0107; +0.0042; +0.0001] m/s²**, with corresponding commanded ΔV approximately **[-1.0741; +0.4154; +0.0056] m/s**.
- **Confirmed:** Burn 2 x-axis acceleration exceeds the provisional −0.01 m/s² limit and saturates. The missing x-component is about **0.074 m/s** relative to the commanded burn, based on the 100 s command estimate. Saturation alone has **not** been shown to explain the full 92 m / 0.132 m/s hold-entry error.

**Data-logging issues resolved**

- Scope visualization did not automatically provide workspace variables. Added To Workspace logging with Timeseries format.
- With Single simulation output enabled, logged data are accessed as `out.get('X_true_log')`, `out.get('mode_log')`, etc.; the first run had `X_true_log` but no `mode_log` until mode logging was added.
- MATLAB logged signal arrays can have time in different dimensions; check `size(Data)` and orient the array before indexing.

## 8. Open technical issue: impulsive guidance vs finite burns

**Observed problem:** The standalone two-impulse CW solution reaches the reference to numerical precision, but the closed-loop implementation distributes each nominal impulse over 100 s. The arrival burn also uses velocity cancellation evaluated at burn entry, not a predicted post-burn state. Navigation error, nonlinear truth dynamics and saturation add further discrepancies.

**Decision:** Keep impulsive CW targeting as a benchmark, but resolve the measured hold-entry error *before* moving to close-proximity operations. The immediate proposed improvement is **finite-duration CW targeting with feasibility checks**, within the existing seven-state chart. This is not yet implemented or validated.

**Planned corrective work**

1. Derive and implement finite-duration CW state propagation for constant acceleration: `X(tf) = Phi(tf)*X0 + Gamma(tf)*u` for each burn/coast segment.
2. Calculate departure and midcourse manoeuvres with burn duration and remaining transfer time included; compute arrival burn from predicted dynamics rather than `-x_hat(4:6)` alone.
3. Check feasibility against per-axis acceleration limits before accepting manoeuvre commands. If infeasible, adjust burn duration/timing or adopt constrained targeting; do not silently assume saturation delivers the requested ΔV.
4. Re-run identical initial conditions and compare HOLD-entry position/velocity error, commanded versus delivered ΔV, and total control effort against the baseline above.
5. Retain the ideal impulsive CW result as a regression test; subsequently compare with nonlinear truth and noisy navigation.

**Architectural clarification:** A Dirac impulse is an analytical orbital-guidance idealization. Real thrusters apply finite force over nonzero time. Later thruster allocation should convert manoeuvre requests and feedback-control demands into actual on/off firing times, incorporating force, mass, minimum impulse bit, misalignment and attitude coupling. The current 100 s duration and ±0.01 m/s² limit are *provisional*, not established spacecraft specifications.

## 9. Planned next milestones

- [ ] Finish finite-burn far-range guidance and quantify improvement at HOLD entry.
- [ ] Run deterministic truth-state tests (navigation noise off) separately from noisy-navigation tests to isolate error sources.
- [ ] Quantify KF estimation errors and consistency; later introduce range/bearing sensors and EKF.
- [ ] Develop close-range approach, hold points, safety corridors, retreat/abort logic and suitable control modes.
- [ ] Define spacecraft mass, thruster configuration and firing constraints; implement thruster allocation and delivered-ΔV accounting.
- [ ] Integrate attitude guidance, reaction-wheel ADCS, body/LVLH transformations and force/torque coupling.
- [ ] Extend to coupled nonlinear 6-DoF truth simulation, actuator imperfections, disturbances, delays and Monte Carlo analysis.
- [ ] Document assumptions, numerical methods, controller tuning, verification cases, plots and limitations for the final Overleaf report.

## 10. Reporting and reproducibility checklist

For every future experiment, append a short entry with **date**, **model/script revision or Git commit**, **test objective**, **initial conditions**, **solver/sample time**, **parameter changes**, **plots/data filenames**, **measured results**, **pass/fail criteria**, **interpretation**, and **next action**. Keep baseline and revised results side by side. Avoid overwriting prior findings when a later model improves.

### Experiment entry template

```markdown
### YYYY-MM-DD — Test name
- **Objective:**
- **Model/commit:**
- **Setup:** Initial state, parameters, solver, simulation duration.
- **Changes:**
- **Results:** Numerical metrics and figure/data references.
- **Assessment:** Pass/fail against a stated criterion; limitations.
- **Next action:**
```

---

*Log reconstructed from the project discussions and reported simulation outputs through 22 October 2026. Some early milestones lack exact dates, saved plots or commit hashes; add those from Git history where available. Numerical values are rounded where the original outputs were rounded.*
