# 25 — Wave Direction & Irregular Sea Impact Quantification

This document quantifies how **wave propagation angle** (from following seas at `0°` through beam seas at `90°` to head seas at `180°`) and **irregular directional JONSWAP wave spectra** impact foilborne motion, Kalman heave estimation, actuator load, independent split front flaps, and top-down stability. All quantitative tables below come directly from [tools/wave_study.py](../tools/wave_study.py) running the full `MANNED_PARAMS` closed-loop simulator (`controller="auto"`, `sensors="noisy"`, `supervisor=True`, `allocator="jacobian"`, `cavitation_model=True`) at `8.0 m/s`.

---

## 1. Encounter-Frequency Kinematics (`encounter_frequency`)

In deep water ($\omega = \sqrt{g k}$, wave phase speed $c_p = g/\omega$), a craft translating at speed $U$ with relative wave travel angle $\mu = \psi_{\text{wave}} - \psi_{\text{boat}}$ (using `waves/airy.py` convention: `0°` = wave traveling $+x$ with the boat = following sea, `90°` = beam sea, `180°` = head sea) encounters wave crests at angular frequency:

$$\omega_e = \omega - \frac{\omega^2 U}{g}\cos\mu, \qquad T_e = \frac{2\pi}{|\omega_e|}$$

For `MANNED_PARAMS` (`U = 8.0 m/s`, peak wavelength $\lambda_p = 12.0\text{ m} \Rightarrow \omega_p = 2.27\text{ rad/s}$, phase speed $c_p = 4.33\text{ m/s < }U$):

| Sea Heading $\mu$ | Physical Regime at $8\text{ m/s}$ | $|\omega_e|$ [rad/s] | $T_e$ [s] | Primary Coupling Mechanism |
|---|---|---|---|---|
| **Following (`0°`)** | Boat overtakes waves ($\omega_e < 0$) | `1.92` | `3.27` | Moderate encounter rate; orbital surge + heave velocity phase modulation |
| **Quarter-Follow (`45°`)** | **Zero-crossing resonance zone** ($\omega_e \approx -0.70$) | **`0.70`** | **`9.03`** | Ultra-slow encounter dwells inside `HeaveFilter` (`kf_wave` w0=2.0) & pitch/heave integrator crossover |
| **Beam (`90°`)** | Pure side-on progression ($\cos\mu = 0$) | `2.27` | `2.77` | Spanwise phase difference across port (`-0.60 m`) & starboard (`+0.60 m`) panels creates rolling moments & asymmetric orbital heave |
| **Quarter-Head (`135°`)** | Oblique fast encounter | `5.23` | `1.20` | Combined longitudinal pitch pitching + spanwise L/R wave roll asymmetry at high encounter rate |
| **Head (`180°`)** | Maximum crest encounter rate | **`6.46`** | **`0.97`** | High heave/pitch orbital velocity frequency; pushes servo rate/current demand toward motor/hinge caps |

---

## 2. Irregular Directional JONSWAP Wave Field (`IrregularWave`)

Real lake and coastal seas are never monochromatic sinusoids. `IrregularWave` in [hydrofoil/waves/airy.py](../hydrofoil/waves/airy.py) models a multi-component directional sea using linear superposition over $N_\omega = 24$ stratified non-commensurate frequency bins ($[0.55\,\omega_p,\, 2.60\,\omega_p]$) and $N_\theta = 5$ directional bins ($\cos^{2s}(\theta - \theta_0)$ spreading with $s = 4$):

$$S_0(\omega) = \omega^{-5}\exp\!\left(-\frac{5}{4}\left(\frac{\omega_p}{\omega}\right)^4\right)\gamma^{\exp\left(-\frac{(\omega/\omega_p - 1)^2}{2\sigma_w^2}\right)}, \qquad \gamma = 3.3,\quad \sigma_w = \begin{cases}0.07 & \omega \le \omega_p \\ 0.09 & \omega > \omega_p\end{cases}$$

Energy is normalized so total surface variance matches $\sum_{i,j} \frac{1}{2}a_{ij}^2 = m_0 = (H_s/4)^2$, while subsurface orbital velocities clamp depth above SWL ($z_{\text{eff}} = \min(z, 0)$) to remain physically bounded at wave crests.

**Three structural consequences of irregular directional spectra:**
1. **Resonance Spreading vs. Group Surges:** Narrowband monochromatic quarter-following waves lock onto a single low-frequency resonance for tens of seconds, whereas a JONSWAP spectrum spreads spectral energy across frequency & direction—reducing monochromatic resonance heave RMS while producing stochastic wave-group envelope extremes ($H_{\max} \approx 1.8\,H_s$).
2. **Off-Axis Roll Excitation in Nominal "Head" & "Following" Seas:** Because directional spreading distributes energy $\sim \pm 22^\circ$ around $\theta_0$ when $s = 4$, even `0°` following and `180°` head seas acquire oblique wave components that asymmetrically load left and right front flaps (`2.4×–2.8×` larger roll and `2.2×–2.6×` larger split-flap peaks than unidirectional Airy waves).
3. **High-Frequency Torque & Current Peaks:** Summed upper-tail spectral frequencies ($\omega > 1.5\,\omega_p$) in quarter-head/beam seas drive higher peak flap angular accelerations and motor phase currents despite similar RMS elevation variance.

---

## 3. Quantitative Sweep Results (`tools/wave_study.py`)

Conditions: `MANNED_PARAMS`, `8.0 m/s`, `controller="auto"`, `sensors="noisy"` (matched sensor noise RNG seed `100` across all runs), `supervisor=True`, `allocator="jacobian"`, `cavitation_model=True`, `2.0 s` calm warmup followed by `30.0 s` wave exposure ($A_{\text{reg}} = 0.12\text{ m} \leftrightarrow H_s = 0.24\text{ m}$, $\lambda_p = 12.0\text{ m}$, $s = 4$).

### Regular Airy Wave (`A = 0.12 m`, `lambda = 12.0 m`)

| Direction | $\|\omega_e\|$ [rad/s] | $T_e$ [s] | Heave RMS / Pk [cm] | Roll RMS / Pk [deg] | Pitch RMS / Pk [deg] | Flap Rate RMS [deg/s] | Split Pk [deg] | L/W Min | Torque / I Pk | Max Vent | Faults |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|
| Following (`0°`) | `1.92` | `3.27` | `5.0 / 14.1` | `0.05 / 0.18` | `0.68 / 1.42` | `18.7` | `1.43` | `0.18` | `4.0 Nm / 34.7 A` | `0.00` | `none` |
| Quarter-follow (`45°`) | `0.70` | `9.03` | **`11.4 / 18.4`** | `0.14 / 0.35` | `0.58 / 1.07` | `17.7` | `3.52` | `0.00` | `3.5 Nm / 22.1 A` | `0.00` | `none` |
| Beam (`90°`) | `2.27` | `2.77` | `3.2 / 8.9` | `0.37 / 0.65` | `0.43 / 0.76` | `17.9` | `4.63` | `0.00` | `3.1 Nm / 24.6 A` | `0.00` | `none` |
| Quarter-head (`135°`) | `5.23` | `1.20` | `2.3 / 6.0` | `0.44 / 0.79` | `0.35 / 0.91` | `20.0` | **`5.05`** | `0.00` | `4.0 Nm / 36.1 A` | `0.00` | `none` |
| Head (`180°`) | `6.46` | `0.97` | `2.2 / 5.8` | `0.05 / 0.18` | `0.34 / 1.06` | **`21.1`** | `1.55` | `0.04` | **`6.0 Nm / 35.2 A`** | `0.00` | `none` |

### Irregular Directional JONSWAP (`Hs = 0.24 m`, `lambda_p = 12.0 m`, `s = 4`)

| Direction | $\|\omega_e\|$ [rad/s] | $T_e$ [s] | Heave RMS / Pk [cm] | Roll RMS / Pk [deg] | Pitch RMS / Pk [deg] | Flap Rate RMS [deg/s] | Split Pk [deg] | L/W Min | Torque / I Pk | Max Vent | Faults |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|
| Following (`0°`) | `1.92` | `3.27` | `3.3 / 7.8` | `0.19 / 0.52` | `0.46 / 1.41` | `20.3` | `3.67` | `0.07` | `3.5 Nm / 25.7 A` | `0.00` | `none` |
| Quarter-follow (`45°`) | `0.70` | `9.03` | **`5.5 / 13.0`** | `0.22 / 0.51` | `0.42 / 1.27` | `18.4` | `4.70` | `0.00` | `4.0 Nm / 22.1 A` | `0.00` | `none` |
| Beam (`90°`) | `2.27` | `2.77` | `4.2 / 7.5` | **`0.34 / 0.86`** | `0.30 / 0.90` | `19.3` | **`6.38`** | `0.11` | `3.6 Nm / 34.9 A` | `0.00` | `none` |
| Quarter-head (`135°`) | `5.23` | `1.20` | `2.0 / 5.4` | `0.27 / 0.80` | `0.26 / 0.90` | `22.2` | `5.85` | `0.18` | **`4.8 Nm / 27.7 A`** | `0.00` | `none` |
| Head (`180°`) | `6.46` | `0.97` | `2.1 / 5.7` | `0.15 / 0.43` | `0.20 / 0.66` | **`23.6`** | `3.44` | `0.29` | **`5.0 Nm / 30.7 A`** | `0.00` | `none` |

### Irregular-to-Regular Peak Magnification Summary

| Direction | Heave Peak Ratio (`Irr / Reg`) | Roll Peak Ratio (`Irr / Reg`) | Split Peak Ratio (`Irr / Reg`) | Torque Peak Ratio |
|---|---:|---:|---:|---:|
| Following (`0°`) | `0.55×` | **`2.80×`** | **`2.56×`** | `0.86×` |
| Quarter-follow (`45°`) | `0.71×` | `1.44×` | `1.33×` | `1.16×` |
| Beam (`90°`) | `0.84×` | `1.32×` | `1.38×` | `1.15×` |
| Quarter-head (`135°`) | `0.91×` | `1.01×` | `1.16×` | **`1.18×`** |
| Head (`180°`) | `0.98×` | **`2.40×`** | **`2.22×`** | `0.84×` |

---

## 4. Key Insights & Control Recommendations

1. **Quarter-Following Seas (`45°`) Dominate Height Excursion (`11.4 cm` RMS regular, `5.5 cm` RMS JONSWAP):**
   When $U > c_p$, quartering-following angles drive $|\omega_e| \to 0.7\text{ rad/s}$, dwelling near the bottom edge of the `HeaveFilter` `WaveObserver` (`w0 = 2.0 rad/s` on manned). To improve ride height tracking in quartering seas without increasing calm-water wander, adapt `kf_wave.w0` online with $|\omega_e|$ or increase LQR heave-integral gain slightly (`iz`) during slow-encounter tracking.
2. **Beam (`90°`) & Quarter-Head (`135°`) Dominate Independent Front-Flap Differential Demand (`6.38°` Split Pk):**
   Beam and quarter-head seas require the largest independent `left`/`right` flap split (`5°–6.4°`) and raise roll peak to `0.80°–0.86°`. Keep `--allocator jacobian` active in beam/quartering seas so asymmetric free-surface and local orbital-AoA bounds protect the shallower windward foil.
3. **Directional Spreading Doubles Off-Axis Split Demand in Head & Following Seas:**
   Testing control/estimation solely on `direction=0°` or `180°` regular waves misses **`2.2×–2.8×`** roll/split-flap excitation from real short-crested directional seas. Include `--wave-type irregular` in tuning and endurance verification.
4. **Momentary Wave-Trough Unloading (`L/W Min ≈ 0.00–0.18`) Without Touchdown:**
   Brief trough vertical velocity downwashes momentarily drop allocator `lift_margin` cleanly while controller integral recovery holds hull clearance (`0` touchdowns, `0` ventilation events, and `0` faults across all 10 cases at $H_s = 0.24\text{ m}$). For steeper waves ($H_s \ge 0.4\text{ m}$), switch `--safety-margin` to `conservative` or increase `flying_height` margin.
