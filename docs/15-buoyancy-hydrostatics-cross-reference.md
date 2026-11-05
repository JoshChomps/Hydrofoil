# 15 — Buoyancy and Hydrostatics: Cross-Reference Against Theory

Cross-check of the hull model against classical ship hydrostatics (Fossen 2011, ch. 4 "restoring forces"; Newman 1977 §2) and the kinematics of a
hull element moving with the vehicle. Numbers were produced by running the code (2026-11-04). The result changed the model: **E20** in
[doc 14](14-error-register-and-fixes.md).

## 1. Theory used as the reference

For a floating prism with waterplane area A_wp, displaced volume ∇ = A_wp·d, linearised about equilibrium (Fossen eq. 4.x):

| DOF | Restoring stiffness | Notes |
|---|---|---|
| Heave | K_z = ρ g A_wp | N/m |
| Roll | K_φ = ρ g ∇ GM_T | GM_T = KB + BM_T − KG, BM_T = I_T/∇, I_T = L B³/12 |
| Pitch | K_θ = ρ g ∇ GM_L | GM_L = KB + BM_L − KG, BM_L = I_L/∇, I_L = B L³/12 |

Here the reference point is the CoG and the hull sits **above** it: the hull bottom is `hull_offset` = 0.6 m above the CoG and the submerged slab extends a further draft d up,
so the centre of buoyancy is `hull_offset + d/2` above the CoG ("KB − KG" = 0.6 + d/2, positive, hence stabilising). The waterplane-inertia term BM is what a lumped model misses.

Equilibrium: ρ g ∇ = m g ⇒ d = m/(ρ A_wp) = 25/(1025 × 0.6) = 0.0407 m (this is what `rest_state` uses).

## 2. What the previous model did, and the numbers

The earlier model (single draft at the origin, force applied at one lever arm above the CoG) was evaluated by finite differences at the rest draft
(box hull 2.0 × 0.30 m, A_wp = 0.6 m²):

| Stiffness | Theory | Old lumped model | New distributed model |
|---|---|---|---|
| Heave K_z [N/m] | 6033.1 | 6033.1 (exact) | 6033.1 |
| Roll K_φ [Nm/rad] | 197.4 (GM_T 0.805 m) | 152.1 (**23 % low**) | 196.5 (−0.5 %) |
| Pitch K_θ [Nm/rad] | 2163.2 (GM_L 8.82 m) | 152.1 (**14× too low**) | 2146.6 (−0.8 %) |

The old model's roll and pitch stiffness were identical because both came only from the single lever arm `(hull_offset + d/2)·F`; it contained KB − KG but no BM at all.
For a long thin hull BM_L dominates (≈ 8.2 of the 8.8 m), so the boat's displacement-mode pitch stiffness was essentially absent.

## 3. The model now (`forces/hull.py::HullHydrostatics`)

**Kinematics.** The hull is a rigid prism. Its footprint (rectangle `hull_length × hull_beam`, centred `hull_x` ahead of the CoG) is cut into an 11 × 7 grid of cells;
cell *i* has body-frame bottom point **p_i** = (x_i, y_i, −hull_offset). With the ZYX rotation R(φ,θ,ψ):

```
z_i       = η_z + R[2,:] · p_i                      # NED depth of the cell's bottom (follows full attitude)
d_i       = clip(z_i − z_surf,i , 0, max_draft)     # z_surf,i = −(wave elevation under that cell, at its own world x,y)
F_i       = ρ g dA d_i                              # buoyancy of the column, vertical in NED
f_body,i  = Rᵀ (0,0,−F_i) = −F_i R[2,:]
r_i       = (x_i, y_i, −(hull_offset + d_i/2))      # centroid of the submerged column
τ         = [ Σ f_body,i ;  Σ r_i × f_body,i ]      # about the CoG, body frame
```
Heave, roll and pitch stiffness, partial emergence during takeoff (one side or the bow lifting out), and wave slope across the hull all follow from this single integral;
no separate GM parameters are needed. The reported `draft` is the footprint-averaged draft, and "foilborne" means every cell is dry. Wave elevation is sampled **at each cell's own world position**.

Hull drag (placeholder quadratic surge drag) scales with mean draft / max_draft as before. Because the stiffness is now correct, a floating hull would rock almost undamped (≈ 2.4 Hz;
pitch ±2° at t = 0 was still 0.6° after 20 s), so a **placeholder** roll/pitch damping `hull_rot_damping = 400 N·m·s/rad` (scaled by the wetted fraction, zero when foilborne) stands in for radiation damping
(≈ 0.2 of critical in pitch at rest draft). After it, the rocking is gone within ~3 s.

## 4. Checks performed (all pass)

| Check | Result |
|---|---|
| Net vertical force at rest draft | exactly cancels weight (0.0 N); pitch moment 1e-14 |
| F vs ρ g A_wp d at d = 0.01, 0.05, 0.15, 0.30 m | equal to 4+ digits (linear in draft) |
| Heave stiffness | exact |
| Roll / pitch stiffness vs theory | −0.5 % / −0.8 % (cell-grid discretisation of I_T, I_L) |
| 15° roll with 2 cm mean draft | draft varies linearly across the beam, 0.007…0.074 m (one side emerging is clipped at 0) |
| Rolled hull moment sign | restoring (starboard-down ⇒ negative K), vertical force still upward |
| Floating at speed 0 | settles to the equilibrium draft 0.0406 vs 0.0407 m; rocking damped |
| Beam waves A 0.05 m at rest | roll ±1°, pitch ±2.7°, mean draft 0.0405 m (rides the surface) |
| Foilborne closed-loop metrics after the change | unchanged (hull is dry): takeoff 2.9 s, height 2 cm in 17.3 s, roll kick 0.69 s, 8° bank OK |

Tests: `test_hull_hydrostatic_stiffness_matches_metacentric_theory`, `test_hull_buoyancy_force_is_linear_in_draft_when_level`,
`test_hull_rolled_partially_emerged_buoyancy_is_restoring_and_vertical_force_positive`, `test_floating_hull_rocking_is_damped`. 46 tests pass.

## 5. Assumptions and what is still not modelled

- **Footprint** (2.0 × 0.30 m, centred on the CoG) is a placeholder; area 0.6 m² is the old `waterplane_area`. Real hull lines change BM, KB and the CB location. Set `hull_x` so the CB sits under the CoG for a level float.
- **Wall-sided prism**: draft is clipped to `max_draft` (0.3 m), so a deeper hull stops gaining buoyancy (correct once the whole prism is submerged, but there is no flare, chine or bow shape).
- **Hydrostatic only**: no dynamic sinkage/trim with speed (Froude effects), no planing lift, no wave-making resistance or the pre-takeoff "hump", no spray; hull drag stays a placeholder u|u| term.
- **Hull hydrodynamic added mass and damping** in roll/pitch/heave: only the placeholder damping above; added mass covers just the foils.
- **Foils, struts and mounts displace water** (small buoyancy) — assumed absorbed in the mass; fully submerged so they add no restoring stiffness, but the **struts' free-surface piercing** forces are modelled only through the side-force strip (doc 14 E10).
- **Wave slope** is included through per-cell sampling, but the wave field is frozen across the four RK4 stages and hull cell positions are taken at the start of the step.
- **Buoyancy of the CoG offset**: weight acts at the CoG; if the real CoG is not under the CB (hull_x ≠ 0) the boat floats at a trim angle — as it should, but it changes the rest state (the sim still starts at 2° pitch).

## 6. What to measure to replace the placeholders
Hull length/beam/centre and draft at rest (gives A_wp and CB), a free-decay roll/pitch test (gives K and damping), and the CoG location (gives GM directly: `GM_T`, `GM_L` from an inclining test).
