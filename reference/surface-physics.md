# Surface response and slide-control pass

Rally v3 extends the retained Stunt Rally/VDrift tire and Ecctrl/Rapier wheel
solver. It does not replace the donor model or apply a yaw/velocity alignment
assist. Vehicle mass, powertrain, springs, differential settings and steering
input curves are retained from the suspension pass.

## What changes

- The road has a packed central lane, loose gravel outside that lane, the two
  existing wet-mud bands, and a forest-floor shoulder. Each uses its own peak
  grip, longitudinal optimum slip, lateral response width, contact relaxation,
  rolling resistance and contact drag. These are game-tuned values, not measured
  tire compounds or a reconstruction of DiRT Rally's proprietary physics.
- Material position is projected onto the adjacent road segments instead of
  jumping between approximately two-metre road samples. Mud ramps extend two
  metres inside each band; the shoulder feathers over 1.1 metres. Every wheel
  samples its actual contact point. A 0.22 m patch response smooths changes;
  entering lower grip applies the new traction ceiling immediately. Airborne,
  reset and handling-mode changes clear the contact memory.
- Rally blends the donor's conservative combined-force diamond 16–22% toward
  an ellipse, staying within the pure-axis ceilings. Tire deformation releases
  faster as slip reduces or changes sign. Throttle, lift, brakes and countersteer
  act through these tire forces, the retained differential and physical wheel
  loading. ABS follows each material's optimum longitudinal slip.

Settings → Handling → Rally uses this version. Rally Classic reproduces the
previous Rally v2, including the recent suspension work. Original and Refined
are retained. Existing handling preferences remain respected. Rally v3 records
use their own namespace; Classic uses the previous v2 records without deleting
any saved times.

## Validation

`node scripts/check-surfaces.mjs` runs 8 uniform-surface stops, 24 coast / trail
brake / power corner-and-recovery cases, 12 transverse and split-surface cases,
plus surface-map continuity, force-envelope, airborne/reset and 30/60/120 Hz
cadence checks. The split-grip tests mirror the slick side and check that the
physical yaw response reverses. Transverse boundaries reach the front tires
before the rear tires. Lower-grip crossings cannot retain the dry friction
ceiling. All acceptance checks pass.

80–0 km/h on a level synthetic fixture, metres:

| Vehicle | Packed | Gravel | Mud | Shoulder |
|---|---:|---:|---:|---:|
| Subaru | 25.75 | 26.93 | 33.69 | 29.71 |
| Truck | 27.08 | 28.26 | 38.76 | 32.33 |

The existing 24-case corner-balance suite passes unchanged, including its
corner-entry and braking regression bounds. In its matched Subaru power trace,
dry peak sideslip changes from 16.39° to 14.87° and countersteer recovery from
0.88 s to 0.83 s. Improvements are modest and not universal: the truck's wet
coast recovery changes from 0.43 s to 0.48 s. All remain within the existing
acceptance bounds. The older suite supplies legacy dry/mud friction fixtures;
the new surface suite exercises the complete material profiles.

The 24 rough-road cases pass. Full handling checks pass for acceleration,
braking, reverse, overlapping pedals, handbrake, weight transfer, slide recovery,
landing and production-mesh laps. Both vehicles clear all 12 checkpoints; lap
fixture times are 127.1 s (Subaru) and 135.4 s (truck). These are scripted driver
results, not player performance targets.

Rally Classic's corner/braking/cadence report exactly matches the prior v2
report after normalizing the mode name. Original/Refined's complete comparison
report is byte-for-byte unchanged. TypeScript and the production build pass.

Reports: `surface-validation.json`, `surface-handling.json`,
`corner-balance-validation.json`, and `rough-road-validation.json`.
Browser/GPU and hands-on driving were unavailable; feel still requires a driving
comparison. The primary design reference for separating material friction,
slip response and combined-force limits is NVIDIA's vehicle documentation:
https://nvidia-omniverse.github.io/PhysX/physx/5.6.1/docs/Vehicles.html . The shipped
implementation remains the repository's retained donor, with the changes above.
