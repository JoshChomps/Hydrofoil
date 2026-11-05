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

M ν̇ + C(ν) ν = τ,  C = C_RB + C_A

- M = [[m·I₃, −m·S(r_g)], [m·S(r_g), I_b]] with r_g = 0 and I_b = diag(Ixx, Iyy, Izz); M[2,2] += m_a,heave and M[4,4] += I_a,pitch (diagonal added mass, heave and pitch only).
- C_RB (Fossen 3.46): blocks −S(m ν₁ + m S(r_g)ᵀ ν₂) off-diagonal and −S(I_b ν₂) lower-right.
- **C_A (added-mass Coriolis, fixed 2026-11-04, doc 14 E7):** same skew structure with M_A: a₁ = [0,0,m_a w], a₂ = [0, I_a q, 0]; blocks −S(a₁) off-diagonal and −S(a₂) lower-right. It contains the **Munk moment** (C·ν)_q = −m_a·u·w, which is destabilising in pitch (m_a ≈ 24 kg, u = 4 m/s ⇒ 97 Nm per m/s of w). νᵀCν = 0 (checked numerically and in a test).
- Solved each call with `np.linalg.solve(M, τ − C ν)`.

### Added mass (heave, pitch) — `strip_added_mass_heave_pitch`
Per surface: m_a = ρ π c² / 4 · span (2-D flat-plate/ellipse result, integrated over span). Heave: Σ m_a,i. Pitch: Σ m_a,i x_i² (lumped parallel-axis).
Defaults (ρ = 1025): main half ≈ 10.9 kg ×2, elevator ≈ 2.4 kg ⇒ heave 24.15 kg (≈ the vehicle's own mass), pitch 3.48 kg·m² (main halves are at x = 0), versus Iyy = 5. (Confirmed numerically.)
Caveat (open, E-O3): the 2-D value over-estimates a finite AR-4 panel by roughly 20 %.

## 4. Generalised forces τ (per step)

τ = τ_left + τ_right + τ_elev + τ_mainstrut + τ_elevstrut + τ_hull_buoy + τ_gravity + τ_hull_drag + [thrust,0,0,0,0,0] + [0,0,−c_heave·w,0,0,0]

Each foil/strut sees the body velocity minus the wave orbital velocity **at its own world position** (`WaveSample`, doc 14 E13). Hull drag and heave damping use the raw ν.

### 4.1 Foil force (one strip per surface) — `Foil.generalized_force`

1. V_f = ν₁ + ω × r (r = attachment point; `_cross` helper).
2. Each surface has a body-frame **normal axis** n (horizontal foils: +z; vertical struts: +y). Only the chordwise (x) and normal components load it: V_n = V_f·n, V_plane = (V_x, 0, 0) + V_n·n; speed = |V_plane| (spanwise flow is dropped, E9). If speed < 0.01 m/s → 0.
3. **AoA** α = atan2(V_n, V_x) [→ deg] + aoa_offset + k_flap·δ (k_flap = 0.21 main, 1.0 elevator, 0 struts). Body pitch enters only through the velocity direction.
4. q = ½ρ speed²; area A = chord × span (span overridden per call for struts).
5. 2-D polar CL₂D(α), CD₂D(α), linear interpolation, α clamped to −10°…+17.8°.
6. **Finite span (E8):** CL = CL₂D · 1/(1 + a₂D/(π e AR)), a₂D = polar slope per rad (≈ 6.19), same e as induced drag. Main half (AR 4, e 0.8): 0.619; elevator (AR 3): 0.549.
7. Free surface: CL ← CL·(1 − exp(−(h/c)/1.58)) for foils (skipped for struts).
8. Induced drag: CD = CD₂D + CL²/(π·AR·e), e = 0.80.
9. L = q·CL·A, D = q·CD·A; lift direction = (a − (a·V̂)V̂)/‖·‖ with a = −n (up for foils, −y for struts); force = L·lift_dir − D·V̂; moment = r × force.

Roll mechanism: left foil δ = δ_main − δ_roll, right foil δ = δ_main + δ_roll; differential lift × 0.35 m lateral offset.

**Struts (E10, new, placeholder chords 0.12 / 0.08 m):** same code with n = +y, so AoA = side-slip angle, force along ±y. Wetted span = foil depth below the local surface (clipped to hull_offset + foil z), force at mid-wetted length.

### 4.2 Hull buoyancy — `HullHydrostatics` (E20; derivation and checks in doc 15)
The hull footprint (rectangle, 11 × 7 cells) is integrated: per cell z_i = η_z + R[2,:]·p_i, d_i = clip(z_i − z_surf,i, 0, max_draft), F_i = ρ g dA d_i, f_body,i = −F_i R[2,:], at r_i = (x_i, y_i, −(hull_offset + d_i/2)); τ = Σ(f_i, r_i × f_i). This reproduces ρ g A_wp, ρ g ∇ GM_T and ρ g ∇ GM_L to within 1 %. The paragraphs below describe the earlier lumped model for reference.

#### (earlier lumped model — `hull_buoyancy_force`)
- Hull-bottom reference point (0,0,−hull_offset) rotated to NED: z_bottom = η_z − hull_offset·cosφ·cosθ (E11). draft = clip(z_bottom − z_surface, 0, max_draft); z_surface = −wave elevation at the hull.
- F_b = ρ g A_wp draft, up in NED, rotated into the body by Rᵀ, **applied at r_cb = (0,0,−(hull_offset + draft/2))** so it produces a moment r_cb × F (E12).
- Linear/wall-sided: valid only near the design waterline. Foilborne ⇔ draft = 0 (`≤ 1e-6`).

### 4.3 Hull drag — `hull_drag_force`
F_x = −C_hull · (draft/max_draft) · u|u|. Surge only. Placeholder.

### 4.4 Gravity — `gravity_force`
f_ned = [0,0,m g] → body via Rᵀ.

### 4.5 Thrust and heave damping
Thrust is a +x force at the body origin. Heave damping τ_z −= 50·w (placeholder; the foils' own heave damping is ≈ 1400 N·s/m at 4 m/s, so this term is ~4 %).

## 5. Wave model — `AiryWave`

η_w(x,y,t) = A cos(k·s − ω t + φ₀), s = x cosβ + y sinβ, k = 2π/λ, β = travel direction.
**ω = √(g k)** (deep-water dispersion; period is derived unless explicitly passed — E15). Orbital velocity at NED depth z_w = −(z_NED) ≤ 0, decay e^{k z_w}:
horizontal Vh = A ω e^{kz} cos(k s − ωt + φ₀) along β; vertical Vv = A ω e^{kz} sin(…), NED z-component −Vv. (Sign convention checked against dη/dt.)

Sampling (E13/E14): `VehiclePhysics.wave_sample` evaluates elevation, local surface z and orbital velocity at the hull and at **each surface's world position** (foil AC = η + R·r), so the foil's own depth sets the decay and beam/oblique seas load the left and right halves differently. Frozen across the four RK4 stages of a step. Waves act through relative flow at the surfaces, local surface z in draft, and the free-surface factor; there is no direct wave force on the hull.

## 6. Integration — RK4 with held controls

At each step: controller and surge PID are evaluated **once** with the current state, then RK4 integrates
`physics_rhs` four times with those commands, thrust and wave state frozen. dt default 0.01 s (range 0.001–0.05).
Controller discrete time = integration time. After integration: NaN → roll back; else clamp attitude/velocity (doc 05).

## 7. Sanity numbers (all confirmed by running the code; values for the corrected model are in doc 14 §H)

Using defaults (m 25 kg, W = 245 N; main half 0.15×0.6 m, AR 4; ρ 1025):
- Main flap lift slope ≈ 10.2 N/deg per half at 4 m/s *before* E8 (finite-span change lowers it ≈ 7 %).
- Flap limit ±20° ⇒ ≈ 400 N total before E8 (≈ 370 N after), so the minimum foilborne speed on flap authority alone is ≈ 3.1–3.3 m/s.
- Rest draft 0.041 m; max hull buoyancy 1810 N; heave added mass 24.15 kg; pitch added inertia 3.48 kg·m².
- Foil free-surface factor at nominal flying height (foil 0.55 m deep, h/c = 3.7): 0.904.
- Open-loop eigenvalues and controllability: see doc 13 §1.

