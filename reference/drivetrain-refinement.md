# Drivetrain and hairpin refinement (Rally v4)

Rally now closes the throttle faster on lift-off and slews the retained donor's
gear-dependent engine drag rather than introducing it as a single torque step.
Engine drag is constrained to oppose shaft rotation. This retains the existing
engine curve, gearing and physical weight transfer.

Gear changes follow a short torque-disconnect, ratio-swap and reconnect cycle,
adapted from the retained Stunt Rally UpdateTransmission / ShiftAutoClutch
reference. Subaru shifts take 0.14 s; truck shifts take 0.18 s. The ratio changes
only on a step with zero clutch torque. A 75 ms sustained RPM decision and
post-shift cooldown suppress threshold chatter. Road speed must confirm an
upshift, preventing wheelspin from causing false upshifts. New shifts are
inhibited while airborne or using the handbrake; downshifts check the predicted
RPM in the next gear. These are game settings, not manufacturer specifications.

The handbrake releases its brake pressure sooner and reconnects drive over
0.16 s (Subaru) or 0.20 s (truck), with progressive center-differential locking.
It retains the donor engine/differential model. The clutch is a bounded torque
envelope, not a new engine-inertia, rev-matching or clutch-temperature simulation.

## Comparison and records

Settings → Handling → Rally selects v4. Rally Terrain preserves the v3 surface
pass, Rally Classic preserves v2, and Refined/Original retain their behavior.
Rally v4 records use a separate namespace; existing records are retained.
The new drivetrain is independent of the terminal UI: the latest UI and iPhone
viewport changes from PR #5 are carried forward unchanged apart from the new
handling option and description.

## Evidence

`node scripts/check-drivetrain.mjs` validates transmission disconnect timing,
moderate wheelspin rejection, airborne/handbrake shift inhibition, RPM-threshold
chatter, lower-gear drag, flat/downhill lift-off, handbrake recovery, acceleration
and 30/60/120 Hz outcomes. Its 32 geometric hairpin runs cover two vehicles,
old/new handling, gravel/mud, level/8% downhill entry and handbrake on/off.
The fixture has a 25 m approach, 14 m radius 180-degree bend and 30 m exit.
A deterministic preview driver follows it; this is not a new selectable track
or an in-game driving assist. All runs complete, with maximum path offset 0.872 m.

The largest per-step drivetrain torque change during the handbrake-release
window falls 77–98% across the four matched vehicle/surface cases. Initial drive
returns after approximately 0.192 s versus 0.325 s previously, then reaches full
engagement at 0.350 s (Subaru) / 0.388 s (truck). Exit speed is not universally
higher: Subaru gravel exit changes from 64.60 to 61.28 km/h; mud exit from 51.24
to 53.05 km/h. This is smoother torque delivery, not a blanket speed improvement.

Flat gravel 0–100 km/h changes from 4.53 to 4.55 s for the Subaru and 5.73 to
5.90 s for the truck, with no launch downshifts. Existing corner, surface,
rough-road and complete handling suites pass, including all 12 checkpoints for
both vehicles (128.1 s Subaru, 135.7 s truck). Original/Refined comparison output
is unchanged. Rally Terrain's corner and surface reports match the v3 baseline
exactly after normalizing its mode label. TypeScript and production build pass.

Reports: hairpin-drivetrain-validation.json, drivetrain-handling.json,
corner-balance-validation.json, surface-validation.json, rough-road-validation.json.
Browser/GPU and hands-on driving were unavailable. These are production-solver
regression tests, not measured DiRT Rally equivalence.
