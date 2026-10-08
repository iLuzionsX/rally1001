# Rally Physics 2.0 integration

Built on main `8ccda49`, including the approved terminal UI, iPhone viewport behavior and boosted recorded audio from PR #8. Applied physics commits from PR #3, #4 and #6 in order. No map, graphics, camera, audio or UI redesign.

## Verified on the integrated tree

- TypeScript: `node node_modules/typescript/bin/tsc --noEmit` passed.
- Production build: `node scripts/run-framework.mjs build` passed.
- Suspension: `node scripts/check-rough-road.mjs` passed 24 vehicle/speed/terrain cases.
- Surfaces: `node scripts/check-surfaces.mjs` passed 8 braking, 24 recovery and 12 surface-transition runs, material mapping and 30/60/120 Hz checks.
- Drivetrain: `node scripts/check-drivetrain.mjs` passed 32 hairpins, shift guards/timing, engine-drag, handbrake recovery, launches and cadence checks.
- Corner balance, Rally dynamics, full Rally v4 handling, Refined handling and handling comparison suites passed. Both vehicles completed all 12 production-mesh checkpoints (Subaru 128.1 s, truck 135.7 s in the Rally v4 handling run).
- Regenerated tracked fixture reports exactly match the imported reports. Main CSS, layout, audio module and credits remain byte-for-byte unchanged. The race component changes only the handling choices/description.

These are deterministic solver checks. Browser/GPU rendering, actual touch input and human driving feel are unverified; the available environment does not provide the supported browser QA surface. Do not merge on automated evidence alone.

## Driving review gate

Run each row for both Subaru and F-150, desktop keyboard and iPhone Safari. Compare Settings > Handling > Rally (v4) against Rally Terrain (v3) and Rally Classic (v2). Original and Refined remain available. New Rally records use v4 so older bests remain intact.

| Check | Pass criterion |
| --- | --- |
| Bumps, crest, landing | Suspension settles without repeated uncontrolled bouncing, chassis roll or loss of steering on landing. |
| Packed lane, outer gravel, wet mud, shoulder | Grip/braking differences are understandable; one-wheel transitions cause no abrupt artificial yaw kick. |
| Hairpin with and without handbrake | Release returns drive progressively; countersteer and throttle permit recovery without sudden snap. |
| Full-throttle launch and lift-off | No shift chatter; brief torque interruption is readable; lower gears produce stronger engine braking. |
| Mobile multitouch | Steering + gas, steering + brake, and steering + slide work; release/cancel clears inputs; no stuck throttle after pause/app switch. |
| Desktop | WASD/arrows, Space, pause/resume, recovery and vehicle switching remain usable. |
| Presentation | Existing terminal layout, viewport fill, engine loudness and separate vehicle recordings remain intact; check portrait and landscape. |
| Complete run | Both vehicles finish, splits/results/bests save under the correct physics mode. |

## Next phase after driving acceptance

Build one point-to-point stage with elevation, cambers, ruts and surface changes. Keep the established graphics, terminal UI and recorded audio. Add route-aware co-driver calls for corners/crests/hazards, timing/splits/results, roadside scenery and surface-driven dust/tire marks. Tune and validate separate Subaru/truck driving lines against this accepted physics baseline. This integration does not add that stage.
