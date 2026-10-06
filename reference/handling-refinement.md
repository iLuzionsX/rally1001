# Handling refinement — October 6, 2026

Settings → Handling selects Refined (default) or Original. The preference persists
across reloads and vehicle changes. Switching while paused starts a fresh run;
best times and splits are kept separately. Original retains the existing record
keys and reproduces the pre-change handling report exactly.

Refined adds lateral contact-patch relaxation before the existing combined-slip
gravel force calculation. Relaxation length is 0.32 m for the Subaru and 0.40 m
for the truck. The first-order state uses exponential integration, caps response
time at 50 ms, and fades in between 2 and 6 m/s. Contact loss and resets clear the
state. Longitudinal slip and ABS telemetry remain instantaneous. This is a small
transient extension to the existing donor, not a replacement tire model.

Brake demand uses pedal^1.15, a 35/s pressure buildup and 14/s release. Full pedal
retains full torque, and raw brake input still cuts propulsion immediately.
Original retains the prior immediate application and 20/s release.

Conceptual reference: Project Chrono's tire-model documentation describes
contact-patch state equations for transient handling:
https://api.projectchrono.org/9.0.0/wheeled_tire.html
The response values here are game tuning, not measured Subaru/Ford tire data.

Validation:

- `node scripts/check-handling-comparison.mjs`: both vehicles, dry/wet stopping,
  trail braking, slalom, contact lifecycle, and 30/60/120 Hz comparisons.
- `node scripts/check-handling.mjs`: acceleration, braking, countersteering,
  handbrake, landing, camera, and complete production-circuit laps.
- `RALLY_HANDLING_MODE=baseline node scripts/check-handling.mjs`: original path.
- Existing rally-dynamics, grip checks, and TypeScript checks also pass.

Reports are in `handling-refinement-validation.json` and
`handling-refinement-regression.json`. These are scripted physics checks;
browser/GPU verification and hands-on DiRT Rally comparison were not available.
