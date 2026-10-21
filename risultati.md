# Assignment 01 — Results

All numbers and figures are produced by `python3 experiments.py qN` (no viewer, default parameters of `romeo_conf.py` plus the overrides listed for each case). Figures are in `figures/`.

Robot weight: $mg = 40.5\,\text{kg}\cdot 9.81 = 397.6$ N. Time $t = 0$ is the start of the reference walk (after `T_pre`).

---

# Question 1 — Walking with double support

## Results

| Case | Walk completed | CoM error RMS / max [mm] | Max normal force rate [N/s] | Max joint torque rate [Nm/s] | Max \|tau\|/tau_max |
| --- | --- | --- | --- | --- | --- |
| With double support (DS), `transition_time = 0` | yes | 2.9 / 6.8 | 17 785 | 6 325 | 0.99 (LKneePitch) |
| Without DS, `transition_time = 0` | yes | 2.2 / 5.3 | 195 150 | 28 779 | 1.00 (LKneePitch) |
| With DS, `transition_time = 0.1`, template code | **no** (falls at t = 2.81 s) | – | – | – | – |
| With DS, `transition_time = 0.1`, with fix | yes | 2.9 / 6.8 | 14 997 | 4 604 | 0.99 (LKneePitch) |
| Without DS, `transition_time = 0.1`, with fix | **no** (falls at t = 3.10 s) | – | – | – | – |

Largest change of a normal force in one control step (2 ms): 35.6 N with DS (t = 2.698 s, lift-off), **390.3 N** without DS (t = 2.498 s, foot switch), 30.0 N with DS and the ramp (t = 7.298 s, touchdown).

Figures: `q1_cop_forces_with_ds.png`, `q1_cop_forces_without_ds.png`, `q1_switch_zoom_ds_vs_no_ds.png`, `q1_ramp_fall_template.png`, `q1_ramp_fixed.png`, `q1_ramp_zoom_liftoff.png`.

## Weight transfer and reference CoP

- **Single support phases (both walks)**: the stance foot carries the whole weight (normal force between ~384 and ~414 N, mean ≈ $mg$), the swing foot 0 N. The ±4% oscillation is the vertical inertia of the body: the total normal force is $m(g+\ddot c_z)$.
- **Reference CoP** (`q1_cop_forces_*.png`, top): in single support it is fixed under the stance foot ($y = \pm 0.096$ m, feet 19.2 cm apart). With DS it moves **continuously** (S-shaped curve) from one foot to the other during the 0.2 s double support phase; without DS it **jumps** by 19.2 cm in one control step (`q1_switch_zoom_ds_vs_no_ds.png`, top).
- **Relation forces–CoP**: with both feet on the ground the CoP is the average of the feet positions weighted by the normal forces, $p_{CoP} = (f_L p_L + f_R p_R)/(f_L + f_R)$, so with $f_L + f_R \approx mg$:
  $$f_L = mg\,\frac{p_{CoP}-p_R}{p_L-p_R},\qquad f_R = mg\,\frac{p_L-p_{CoP}}{p_L-p_R}.$$
  Each normal force is a linear function of the CoP position (half of the weight on each foot when the CoP is midway). Hence the forces have the same shape as the CoP: a **gradual** transfer in 0.2 s with DS (e.g. RF 373 → 35 N, LF 29 → 358 N in the first DS), a **step** of ≈ 390 N in 2 ms without DS (`q1_switch_zoom_ds_vs_no_ds.png`, bottom). With a single foot on the ground the CoP must stay inside that sole (normal forces ≥ 0), so without a phase with two contacts the CoP, and the weight, can only jump.
- **Difference between the two cases**: the max normal force rate is **11 times** larger without DS (195 kN/s vs 17.8 kN/s) and the joint torques follow (max torque rate 28.8 vs 6.3 kNm/s, 4.5 times). With DS the residual 17.8 kN/s comes from two small jumps left because `transition_time = 0`: ~29 N when the new contact is added at touchdown and ~35 N when the old one is removed at lift-off.

## Why no DS could be a problem on a real robot

It works here because the simulation is ideal: there is no physics simulator (the state is obtained by integrating the QP accelerations, so the robot does exactly what TSID computes), contacts are rigid and instantaneous and the touchdown time is known exactly. On a real robot:
- the actuators cannot change torques instantaneously (limited bandwidth, transmissions), so a 390 N transfer in 2 ms cannot be realized and the CoP would leave the stance foot;
- the swing foot hits the ground with an impact and the real contact is compliant: it would have to take the whole weight during the impact;
- model errors, tracking errors and uneven ground make the touchdown instant uncertain: if the foot touches the ground late, the weight is transferred to a foot still in the air;
- the foot force sensors are noisy and delayed, so the controller cannot reliably check in 2 ms that the contact is established.

## `transition_time = 0.1`: why the robot falls and fix

`remove_contact_*(transition_time)` calls `formulation.removeRigidContact(name, transition_time)`: TSID keeps the contact for `transition_time` seconds while decreasing **linearly** its maximum normal force $f_{max}$ (from the current normal force to 0), and only then removes the contact. The ramp changes $f_{max}$ inside the `Contact6d` object and **leaves it at 0**; `add_contact_*()` re-adds the same object with `addRigidContact`, which does not restore it.

`q1_ramp_fall_template.png`: the left foot is removed with the ramp at the start of the walk ($f_{max}^{LF} \to 0$ at t ≈ 1.4 s). At t = 2.5 s it touches the ground again, but with $f_{max} = 0$ the friction-cone constraint $0 \le f_N \le f_{max}$ forces its normal force to 0: it is on the ground but **cannot carry any load** (LF force stays at 0 during the double support phase, while it should rise to ~360 N). At t = 2.6–2.7 s the right foot is unloaded by its own ramp and lifted: the robot is left on a foot that cannot push and falls (t = 2.81 s). With `transition_time = 0` no ramp is started and $f_{max}$ keeps its nominal value `conf.fMax` = $10^6$ N.

**Fix** (`tsid_biped.py`, `add_contact_RF/LF`): restore the nominal value before re-adding the contact,
```python
self.contactRF.setMaxNormalForce(self.conf.fMax)   # same with contactLF in add_contact_LF
```
With the fix the walk is completed (`q1_ramp_fixed.png`): $f_{max}$ of each foot goes to 0 with the ramp before every lift-off and is back to the nominal value at every touchdown.

Note: without DS the ramp makes the robot fall even with the fix (t = 3.10 s): the ramp starts 0.1 s before lift-off, when the other foot is still in the air, so the only stance foot is asked to unload.

## Does the ramp make a large difference with this gait?

No (table above): the walk, the CoM tracking (2.9 / 6.8 mm) and the torque saturation are unchanged; the max normal force rate decreases only from 17.8 to 15.0 kN/s and the max torque rate from 6.3 to 4.6 kNm/s. The reason is that the weight transfer is **already done by the double support phase** through the reference CoP. `q1_ramp_zoom_liftoff.png`: the ramp of the right foot starts at t = 2.6 s from ~220 N, but the actual normal force is already below the ramp (the CoP is moving to the left foot), so the constraint becomes active only in the last ~20 ms, where it removes the residual jump of ~35 N at lift-off. The largest remaining jump (~30 N) is now at **touchdown**, when a new contact is added with a non-zero force, which the ramp does not affect. The ramp is essential only when the foot to be lifted still carries a large load, which in this gait never happens.

---

# Question 2 — Moving the tray along a sinusoid while standing

## Implementation

- **Hand task** (`tsid_biped.py`): `TaskSE3Equality` on `RWristPitch`, $K_p$ = `kp_hand` = 10 and $K_d = 2\sqrt{K_p} = 6.32$ on all 6 components, `setMask(conf.hand_mask)`, `useLocalFrame(conf.hand_local_frame)`, added with `addMotionTask(handTask, w_hand, hand_priority, 0.0)` only if `w_hand > 0` or `hand_priority == 0`.
- **Reference** (`tray_sinusoid.py`, `sinusoid_reference`), $\omega = 2\pi f$, $A$ = 5 cm along z:
  $p = c + A\sin(\omega t)$, $\dot p = A\omega\cos(\omega t)$, $\ddot p = -A\omega^2\sin(\omega t)$.
- `tray_sinusoid.py`: the simulation loop was moved into `simulate_tray_sinusoid()` (unchanged behaviour), so that `experiments.py` can run several frequencies. Settings as in the script: `tray = True`, `w_hand = 1`, full mask, local frame, 6 s, indicators computed after 2 s.

## Results

| | 0.5 Hz, with FF | 0.5 Hz, without FF | 2 Hz, with FF | 2 Hz, without FF |
| --- | --- | --- | --- | --- |
| Amplitude ratio (hand / ref) | 0.87 | 0.44 | 0.43 | 0.05 |
| Phase lag [deg] | 8.1 | 97.2 | 7.7 | 156.5 |
| Hand tracking error RMS [mm] | 6.7 | 40.4 | 46.7 | 36.8 |
| Max CoM displacement [mm] | 7.3 | 3.6 | 54.7 | 0.9 |
| Max tray tilt [deg] | 0.02 | 0.01 | 0.08 | 0.00 |

Frequency sweep (validation, `q2_frequency_response.png`):

| f [Hz] | 0.25 | 0.5 | 1.0 | 1.5 | 2.0 | 2.5 |
| --- | --- | --- | --- | --- | --- | --- |
| Ratio / lag with FF | 0.94 / 4.8° | 0.87 / 8.1° | 0.75 / 8.8° | 0.52 / 9.8° | 0.43 / 7.7° | 0.21 / 3.1° |
| Ratio / lag without FF | 0.75 / 57.8° | 0.44 / 97.2° | 0.16 / 133.5° | 0.08 / 148.5° | 0.05 / 156.5° | 0.03 / 161.4° |
| Max CoM displacement with FF [mm] | 5.3 | 7.3 | 13.0 | 26.9 | 54.7 | 101.0 |

Torques with FF (diagnostic run): at 0.5 Hz all joints stay below 32% of their limit; at 2 Hz `RAnklePitch` is at its torque limit for 0.41 s and the knees reach 97% (right) and 87% (left).

Figures: `q2_hand_height_tracking.png`, `q2_frequency_response.png`, `q2_com_displacement_ff.png`. The tray stays horizontal in all cases (tilt < 0.1°).

## Without feedforward: lag, reduced motion, worse at 2 Hz

The task desired acceleration is $\ddot p^d = \ddot p^{ref} + K_d(\dot p^{ref}-\dot p) + K_p(p^{ref}-p)$. Without feedforward $\dot p^{ref} = \ddot p^{ref} = 0$ and only
$$\ddot p^d = K_p(p^{ref}-p) - K_d\,\dot p$$
is left: the hand is driven **only by the position error**, and the derivative term damps the absolute hand velocity instead of the velocity error. If $\ddot p = \ddot p^d$, the hand is a second-order low-pass filter of the reference:
$$\frac{P(s)}{P^{ref}(s)} = \frac{K_p}{s^2 + K_d s + K_p},\qquad \omega_n = \sqrt{K_p} = 3.16\ \text{rad/s} \approx 0.5\ \text{Hz},\quad \zeta = 1.$$
- The hand moves only if there is an error, so it is always **behind** the reference and moves **less** than it.
- At 0.5 Hz the reference is exactly at $\omega_n$: theory gives ratio 0.50 and lag 89.6° (simulation 0.44 / 97°).
- At 2 Hz ($\approx 4\omega_n$) the reference is far above the bandwidth: the attenuation goes as $K_p/\omega^2$, theory gives 0.06 / 151.8° (simulation 0.05 / 156.5°).
- The whole sweep follows the theoretical curve (`q2_frequency_response.png`); the small extra attenuation/lag comes from the other tasks, since the hand task is a weighted cost and $\ddot p \neq \ddot p^d$ exactly.

## With feedforward: much better, but not perfect, especially at 2 Hz

With feedforward the error obeys $\ddot e + K_d\dot e + K_p e = 0$, so ideally the hand would track exactly (ratio 1, lag 0). The lag is indeed small at every frequency (≤ 10°), but the amplitude decreases with the frequency (0.87 at 0.5 Hz, 0.43 at 2 Hz).

The reason is the **CoM**. Romeo has a floating base: to accelerate the arm and the tray, the rest of the body receives the opposite reaction (momentum conservation), so the hand motion moves the CoM, while the CoM task wants to keep it still. The two tasks have the **same weight** ($w_{com} = w_{hand} = 1$), so the QP finds a compromise (eq. 2.36) and sacrifices part of the hand motion. The required hand acceleration is $A\omega^2$: 0.49 m/s² at 0.5 Hz and 7.9 m/s² at 2 Hz (16 times larger). Accordingly the max CoM displacement grows from 7 mm at 0.5 Hz to **5.5 cm** at 2 Hz and 10 cm at 2.5 Hz (`q2_com_displacement_ff.png`): at 2 Hz the CoM drifts by several cm in x and y, the hand oscillates around a lower mean height (`q2_hand_height_tracking.png`), and the reaction is so large that the right ankle reaches its torque limit, so the legs can no longer compensate it. Without feedforward the conflict is negligible (CoM displacement ≤ 3.6 mm) because the hand barely moves.
