# 03 — Math and Physics

All equations are as implemented. References: Fossen (2011) *Handbook of Marine Craft Hydrodynamics and
Motion Control*, ch. 3–4; Newman (1977) *Marine Hydrodynamics*.

## 1. State and frames

```
η = [x, y, z, φ, θ, ψ]ᵀ   NED position (z DOWN) + ZYX Euler angles   [m, rad]
ν = [u, v, w, p, q, r]ᵀ   body-frame linear (x fwd, y stbd, z down) + angular velocity   [m/s, rad/s]
state = [η; ν]  (12-vector)
```

## 2. Kinematics

η̇ = J(η) ν, J = blockdiag(R(φ,θ,ψ), T(φ,θ))

R = Rz(ψ) Ry(θ) Rx(φ)   (body → NED)

T = [[1, sφ tθ, cφ tθ], [0, cφ, −sφ], [0, sφ/cθ, cφ/cθ]]

Singular at θ = ±90°. Guarded by an exception in `T_euler` and by the ±70° clamp in `vehicle.py`.

## 3. Rigid-body dynamics

M ν̇ + C_RB(ν) ν = τ  (Fossen eq. 3.46 style)

- M = [[m·I₃, −m·S(r_g)], [m·S(r_g), I_b]] with r_g = 0 and I_b = diag(Ixx, Iyy, Izz).
- Added mass folded into M: `M[2,2] += m_a,heave`, `M[4,4] += I_a,pitch` (no other added mass; no off-diagonals).
- C_RB: upper-right and lower-left blocks = −S(m ν₁ + m S(r_g)ᵀ ν₂); lower-right = −S(I_b ν₂).
  (With r_g = 0 the C matrix is the standard gyroscopic/Coriolis term, which is what couples roll↔yaw↔pitch.)
- Solved each call with `np.linalg.solve(M, τ − C ν)`.
- **Note:** C is built only from `m` and `I_b`; the added mass is *not* included in C (no added-mass Coriolis), a standard simplification but worth knowing.

### Added mass (heave, pitch) — `strip_added_mass_heave_pitch`
Per surface: m_a = ρ π c² / 4 · span (2D flat-plate/ellipse result, per unit span integrated over span).
Heave: Σ m_a,i. Pitch: Σ m_a,i x_i² (lumped parallel-axis).

For defaults (ρ = 1025): main half = 1025·π·0.15²/4·0.6 ≈ 10.9 kg ×2 halves; elevator = 1025·π·0.10²/4·0.3 ≈ 2.4 kg.
Heave added mass ≈ 24.1 kg (about equal to the 25 kg vehicle mass, so effective heave mass ≈ 49 kg). Pitch added inertia ≈ 2.4·1.2² ≈ 3.5 kg·m² (main halves sit at x = 0 so contribute nothing), versus Iyy = 5.0.
*(Hand calculation from the formulas, not executed.)*

## 4. Generalised forces τ (per step)

τ = τ_leftfoil + τ_rightfoil + τ_elev + τ_hull_buoy + τ_gravity + τ_hull_drag + [thrust,0,0,0,0,0] + [0,0,−c_heave·w,0,0,0]

The three foils see the **relative** velocity ν_rel = ν with its linear part reduced by the wave orbital velocity
(in the body frame). Hull drag and heave damping use the raw ν.

### 4.1 Foil force (one strip per surface) — `Foil.generalized_force`

1. Velocity at attachment: V_f = ν₁ + ω × r_attach; speed = |V_f|. If speed < 0.01 m/s → zero force.
2. Flow direction V̂ = V_f/speed.
3. **Angle of attack** α = atan2(V_f,z, V_f,x) [deg] + aoa_offset + k_flap · δ,
   where k_flap = flap effectiveness (0.21 main, 1.0 elevator) and δ is the actuator-filtered command in degrees.
   *Body pitch θ does not enter α* — only the velocity direction in the body frame does (foils have zero mounting incidence unless `aoa_offset` is set; it is 0 everywhere).
4. Dynamic pressure q = ½ρ|V_f|², area A = chord × span.
5. 2D polar lookups CL₂D(α), CD₂D(α) — linear interpolation, α clamped to table range (−10°…17.8°).
6. **Finite span:** CL = CL₂D · AR/(AR+2), AR = span/chord (each half is its own panel).
7. **Free surface:** CL ← CL · (1 − exp(−(h/c)/1.58)), h/c = foil depth/chord. Skipped if `submergence_over_chord is None`.
8. **Induced drag:** CD = CD₂D + CL²/(π·AR·e), e = 0.80 for main halves; elevator uses the default e = 0.80 too.
9. L = q·CL·A, D = q·CD·A.
10. Lift direction = normalise(V̂ × (lift_axis × V̂)), lift_axis = (0,0,−1) (up). Force = L·lift_dir − D·V̂.
11. Moment about body origin = r_attach × force. τ = [force; moment].

Flap-to-roll mechanism: left foil δ = δ_main − δ_roll, right foil δ = δ_main + δ_roll. Differential lift × lateral offset (0.35 m) is the only roll moment source.

### 4.2 Hull buoyancy — `hull_buoyancy_force`
- hull_bottom_z = η_z − hull_offset (NED; offset positive = hull above CoG).
- draft = clip(hull_bottom_z − water_surface_z, 0, max_draft); water_surface_z = −wave_elevation.
- V = A_wp · draft; F_b = ρ g V, acting straight up in NED, rotated into the body frame by Rᵀ.
- Linear/wall-sided: valid only near the design waterline.
- Foilborne ⇔ draft = 0 (`draft ≤ 1e-6` in the code).

### 4.3 Hull drag — `hull_drag_force`
F_x = −C_hull · (draft/max_draft) · u|u|. Surge only. No hump, no wave-making, no sway/yaw drag.

### 4.4 Gravity — `gravity_force`
f_ned = [0,0,m g] → body via Rᵀ. g = 9.81.

### 4.5 Thrust and heave damping
Thrust acts as a pure +x force at the body origin (no moment). Heave damping: τ_z −= 50·w (placeholder, body-frame w).

## 5. Wave model — `AiryWave`

η_w(x,y,t) = A cos(k·s − ω t + φ₀), s = x cosβ + y sinβ, k = 2π/λ, ω = 2π/T.
Orbital velocity at NED depth z_wave = −η_z (negative below surface) with decay e^{k z_wave}:

- horizontal Vh = A ω e^{kz} cos(k s − ωt + φ₀) along direction β
- vertical Vv = A ω e^{kz} sin(…); NED z-component = −Vv

Deep-water assumption. λ and T are independent inputs; deep-water dispersion would require ω² = g k
(T = √(2πλ/g); for λ = 5 m that gives T ≈ 1.79 s versus the default 2.0 s). Wave state is
evaluated once at the start of a step and held through the four RK4 stages.

Waves influence the vehicle through (a) relative flow at the foils, (b) local surface z in draft, (c) local
surface z in the free-surface factor. There is no direct wave force on the hull.

## 6. Integration — RK4 with held controls

At each step: controller and surge PID are evaluated **once** with the current state, then RK4 integrates
`physics_rhs` four times with those commands, thrust and wave state frozen. dt default 0.01 s (range 0.001–0.05).
Controller discrete time = integration time. After integration: NaN → roll back; else clamp attitude/velocity (doc 05).

## 7. Hand-calculated sanity numbers (estimates, not run output)

Using defaults (m 25 kg, W = 245 N; main half 0.15×0.6 m, AR 4; ρ 1025):

- Main-foil effective lift slope ≈ 0.108/deg × AR/(AR+2)=0.667 × free-surface ≈0.90 ≈ 0.065 per deg of AoA.
  Free-surface factor at nominal flying height 0.55 m: foil depth = (hull_offset − flying_height) + main_pos_z = 0.05 + 0.5 = 0.55 m → h/c = 3.7 → 0.90.
- At 4 m/s, q = 8200 Pa; both halves area 0.18 m² → lift ≈ 1476·CL N. Weight alone needs CL ≈ 0.17 → AoA_eff ≈ 2.6° → main flap ≈ 12° (÷0.21), before accounting for the elevator's load share.
- At the ±20° flap limit: AoA_eff = 4.2° → ≈ 400 N at 4 m/s; scaling with V² this meets 245 N at ≈ 3.1 m/s. So minimum foilborne speed on flap authority alone is roughly 3 m/s with current constants. Stall (CL_max 1.42 at 14.4°) is far above anything the flap range reaches.
- Rest draft = 25/(1025·0.6) ≈ 0.041 m of the 0.30 m maximum.
- Max hull buoyancy = 1025·9.81·0.6·0.30 ≈ 1810 N (7× vehicle weight).
