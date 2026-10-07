# Corner balance — October 6, 2026

Settings → Handling → Rally adds a conservative differential setup for corner
entry and power exit. Refined and Original remain unchanged. Existing saved
preferences are respected; new players default to Rally. Best times and splits
use a separate `-rally-v1` record namespace. Changing modes while paused restarts
the run, as before.

The existing Stunt Rally/VDrift differential adapter, gravel tires, combined
slip, physical weight transfer, spring/damping, ABS and engine remain the
foundation. This pass changes differential configuration only, plus mode wiring.
There are no steering-dependent grip multipliers, forced yaw, or velocity snaps.

The Subaru center changes from fixed viscous coupling to torque-sensitive
coupling, so it releases more on coast while retaining its 900 N·m transfer cap
under power. The final coast factor is 0.65: the initially tested 0.18 increased
dry stopping distance more than warranted by its small rotation benefit.
Front and rear coast coupling are reduced conservatively on both vehicles; the
truck center gains power sensitivity. These are game setup values, not measured
Subaru/Ford factory differential specifications.

## Results

`node scripts/check-corner-balance.mjs` tests 24 matched entry/power/coast traces
across two vehicles, two modes and dry/wet surfaces. Dry entry is 70 km/h; wet
entry is 55 km/h. The driver begins active countersteering at 2.5 seconds.
All traces recover within 2.23 seconds, stay upright, and retain forward motion.
Matched 30/60/120 Hz reversal traces produce identical outcomes within 1 cm.
Torque conservation, dissipative coupling, reverse shaft rotation, reset/mode
switching and parked stability checks pass.

The dry coast trace turns the Subaru 36.09° by 1.5 s versus Refined's 35.69°;
the next second adds 30.42° versus 29.22°. The truck improvement is smaller:
22.70° versus 22.65° at entry. These are modest changes, not a large physics leap.

80–0 km/h straight braking in matched fixtures:

| Vehicle / surface | Refined | Rally |
| --- | ---: | ---: |
| Subaru / dry | 26.05 m | 26.56 m |
| Subaru / wet | 33.83 m | 34.10 m |
| Truck / dry | 27.73 m | 27.70 m |
| Truck / wet | 38.97 m | 38.90 m |

`RALLY_HANDLING_MODE=rally node scripts/check-handling.mjs` passes launch,
80/120 km/h braking, pedal overlap, reverse, countersteer, handbrake, landing,
weight-transfer and full production-circuit laps for both vehicles (12 gates).
Reports: `corner-balance-validation.json` and `corner-balance-regression.json`.
The existing Refined/Original comparison report exactly matches the pre-edit run.

## Limits

These are production-solver tests, not hands-on driving or DiRT Rally benchmarks.
Browser/GPU verification is unavailable in this managed environment. Exploratory
long-duration power-on wet turns overwhelmed grip in both Refined and Rally;
this setup does not promise recovery without corrective steering/throttle use.
Further suspension or surface-model work remains a separate pass.
