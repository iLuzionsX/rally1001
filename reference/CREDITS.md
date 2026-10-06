# Downloaded foundations

- Map foliage, rocks, SUV, pickup and wheels: Kenney Nature Kit and Car Kit, downloaded as embedded GLB files from https://github.com/Hidencod/tge-assets (CC0). Exact URLs and SHA-256 hashes are in public/assets/sources.json.
- Original tire/suspension foundation (replaced by the rally model below): https://github.com/dimforge/rapier and https://github.com/dimforge/rapier.js (Apache-2.0). The original implementation used DynamicRayCastVehicleController, derived from Bullet. Rapier 0.21.0 now resolves chassis and world collisions; the downloaded rally model below supplies wheel support and tire forces.
- Wheel setup and update-order integration reference: https://github.com/mrdoob/three.js/blob/dev/examples/physics_rapier_vehicle_controller.html (MIT).

Upstream reference source and licenses are kept in this directory. The current handling uses downloaded tire/contact algorithms, adapted to the existing game world and vehicle geometry.

## Detailed vehicles and ground materials

Subaru Impreza WRX STi 2004 Custom by MAC ULT ARTS (CC BY 4.0), https://sketchfab.com/3d-models/subaru-impreza-wrx-sti-2004-custom-08296bc950364621b6174a3078bb19e0. Geometry obtained from JonnyChappp/2005-sti-configurator. Adapted orientation, paint materials and wheel pivots.

F-150 model obtained from TyByers21/f150-customizer, public/models/F150.glb. Adapted and reduced for this owner-private game.

Brown Mud 02 and Brown Mud Leaves 01 by Rob Tuytel / Poly Haven (CC0), https://polyhaven.com/a/brown_mud_02 and https://polyhaven.com/a/brown_mud_leaves_01. Diffuse, normal and roughness maps optimized to 1K JPEG.

## Dawn circuit environment and audio upgrade

Island Tree 01, Island Tree 03, Fern 02, Grass Medium 02, Rock 09 and Kiara 1 Dawn from Poly Haven (CC0). Meshes reduced using Meshoptimizer, with embedded 1K WebP diffuse, normal, occlusion and roughness maps. Trees are instanced in spatial cells with matching wind deformation in shadow and color passes.

- https://polyhaven.com/a/island_tree_01
- https://polyhaven.com/a/island_tree_03
- https://polyhaven.com/a/fern_02
- https://polyhaven.com/a/grass_medium_02
- https://polyhaven.com/a/rock_09
- https://polyhaven.com/a/kiara_1_dawn

Recorded engine: MarlonHJ, Firebird_v2 (CC0), via esensar/bugged-racing. Tire squeal: Tom Haigh / audible-edge, submitted by qubodup (CC BY 3.0). Gravel loop adapted from Minetest Game recordings (CC BY-SA 3.0); the adapted gravel audio remains CC BY-SA 3.0. Detailed credits and source links are in public/assets/audio/CREDITS.txt. MP3 versions are provided for Safari compatibility.

The current upgrade's CPU scene, geometry, full-lap and split-persistence verification is in upgrade-validation.json. GPU rendering and physical mobile controls have not been playtested.

## Photographed daytime sky and exposure

Kloppenheim 03 (Pure Sky) by Greg Zaal (original) and Jarod Guest (sky edits), Poly Haven, CC0: https://polyhaven.com/a/kloppenheim_03_puresky. The downloaded tonemapped 8K JPEG is resized to a 4K WebP for the visible sky. The matching 2K HDR is reduced to 1K for reflections; luminance above 2 uses a smooth shoulder approaching 12, preserving chroma. Source URLs, original-source SHA-256 hashes, output hashes and adaptations are in `public/assets/sources.json`.

The sky samples the panorama directly on a camera-centered sphere, avoiding the six 2K cube faces that the 4K native equirectangular background would create. It has no fog, no depth writes and no unbounded HDR sun in the bloom input. Both the image and reflection environment use the same rotation. The directional light's azimuth and elevation match the solar centroid measured in the source HDR; its shadow camera follows the car with that same offset. Exposure, fill light, fog density, color grading and bloom are balanced for daylight.

`lighting-validation.json` records the initial sky integration's UV alignment, camera centering, solar alignment and HDR decoding. Its brightness figures are historical and superseded by `exposure-regression.json`.

## Shaded-track exposure regression

`scripts/check-exposure.mjs` exports the current lighting profile, production color-grade fragment shader, installed Three.js ACES/sRGB shader functions and the decoded HDR. `scripts/check-exposure.py` executes the shaders in an offscreen EGL OpenGL context with the downloaded dirt diffuse/normal maps and cosine-integrated HDR diffuse lighting. Sunlight, shadow, partial canopy/contact shade, 18% gray and white diffuse probes are measured in high, balanced and battery quality profiles. A nine-profile sweep selected exposure 1.1, environment intensity 1.15, hemisphere intensity 0.7 and AO intensity 0.38. Sky intensity 0.85 retains the separately bounded sky highlights.

`exposure-before.json` preserves the preceding version's profile. `exposure-regression.json` records the paired brightness and clipping results; `exposure-probes.png` shows the material comparisons. The shaded gray probe changes from median display brightness 0.2402 to 0.4712. These are diffuse-material probes, not complete-game screenshots: diffuse environment convolution approximates PMREM, and actual tree visibility, specular reflection and geometry-derived GTAO are not reconstructed. The execution renderer is Mesa llvmpipe. Browser/phone visual playtesting remains unverified.

Run `npm run check:exposure` with Python packages `moderngl`, `numpy`, and `Pillow`, plus an available EGL context. The test compiles its grading/tonemapping shaders from production source rather than maintaining copied calibration math.

## Engine and transmission

Ecctrl by Erdong Chen (MIT), downloaded from https://github.com/pmndrs/ecctrl at commit e2cab804f9f15661a642e76f52d09f0b2db63f35. The runtime uses its baked curve implementation unchanged and adapts its weighted wheel-RPM automatic transmission and wheel torque/steering equations to the wheel-contact model below. Combustion-engine curve and vehicle presets are game-specific. Original source, license, exact URLs and hashes are preserved under reference/ecctrl.

## Gravel tire and wheel-contact replacement

Stunt Rally 3 / VDrift by Crystal Hammer and contributors (GPL-3.0-or-later), downloaded from https://github.com/stuntrally/stuntrally3 at commit fe3ecd73fff671c3d75ecaeadccea5c5e6afe0d8. `CARTIRE::GetForce`, Pacejka longitudinal/lateral force equations, optimum-slip load lookup and `CARDYNAMICS::DoABS` are adapted to TypeScript. The hard-mode gravel tire coefficients are preserved unchanged. Original source, data, license and hashes are in reference/stunt-rally.

Ecctrl's `ShapeCastWheel` supplies contact detection in its supported rayCast mode, spring/damping support, contact-plane axes, low-speed static blending and contact-point impulses. The earlier effective wheel/load inertia coupling and full-speed desired-impulse bound have since been replaced by the fixed-inertia, finer-step rally drivetrain described below. Adaptations use ordinary classes instead of React hooks and the Stunt Rally gravel force law instead of the generic friction curve.

The old Rapier/Bullet wheel controller is no longer instantiated. Tire forces are applied at actual contact patches; there is no direct heading/velocity alignment or synthetic yaw force. Chassis forces create physical pitch, roll and wheel-load transfer. Vehicle dimensions, suspension rates, input response and surface friction are game presets; these are not certified real-car performance figures or Dirt Rally's proprietary physics.

`rally-validation.json` records production-code tests for retained lateral momentum, progressive dirt slides, countersteer recovery, rear-wheel locking, physical braking load transfer, acceleration/braking, all circuit checkpoints and existing camera settings. Browser/gamepad/phone driving feel has not been playtested.

## Dry-dirt grip and lift-off refinement

The same Stunt Rally gravel coefficients and combined-slip model remain in use. Front/rear cornering-response gains build lateral force at smaller actual slip angles while retaining the donor's force ceiling. Dry road grip changes from 0.55 to 0.59; mud grip remains 0.34. Ground rolling resistance is now surface-specific.

Closed-throttle engine friction is adapted from the pinned Stunt Rally/VDrift `CARENGINE::GetFrictionTorque` and its `SetTorqueCurve` normalization, using active gearing and engine RPM. It fades below idle, disengages with the handbrake, and is scaled modestly for keyboard/touch driving. It opposes wheel rotation without applying synthetic chassis yaw or heading/velocity alignment. Full-throttle torque remains unchanged.

`grip-tuning-comparison.json` records paired previous/revised runs with the same road speed, gear, inputs and production simulation. `grip-tuning-validation.json` records the complete regression checks after the refinement. The gains are game setup choices; browser driving feel still requires hands-on feedback.

## Differential and driveline integration

The same pinned Stunt Rally/VDrift download now also supplies `CARDIFFERENTIAL::ComputeWheelTorques`, the center/front/rear AWD differential tree from `CalculateDriveTorque`, configured fixed wheel inertia from `CARWHEEL`, `NextGear`'s average-slip shift inhibition, and `ApplyTireForce`'s separate tangential loose-ground drag. Original differential, wheel, and simulation sources and hashes are retained under `reference/stunt-rally`.

One common engine torque curve at driven shaft speed feeds the differentials. Coupling transfers equal and opposite torque between shafts, including while foot braking. Acceleration/coast locking and nominal torque splits are vehicle setup choices. The numerical coupling bound prevents a transfer from overshooting speed equalization in one integration step. The handbrake disengages the driveline and center coupling before locking the rear wheels. Propulsive throttle torque remains inhibited by the foot brake.

Wheel inertia is fixed at 1.5 kg·m² for the Subaru and 3.2 kg·m² for the pickup; contact load and airborne state do not change it. Chassis/contact physics runs at 240 Hz; when a supported wheel's forward contact speed is below 6 m/s, wheel/tire/driveline integration uses adaptive 960 Hz steps with refreshed contact velocities. This addresses the stiff low-speed slip response without adding load-dependent rotational inertia. The static rolling bound remains only below 2 m/s. Full road-speed tire forces retain the downloaded gravel coefficients and combined-slip law.

Dry road, forest floor and mud use separate loose-ground drag values of 4, 25 and 45 kg/s per wheel, adapted to this game's scale. Ground drag opposes the full tangential contact velocity, and does not align velocity to chassis heading. Surface grip and front/rear tire-response gains from the preceding refinement remain unchanged. The controller steering dead zone is reduced from 14% to 6%; chassis steering response and the existing camera settings remain in use.

`rally-driveline-validation.json` records production acceleration, braking, genuine slides, countersteer recovery, handbrake rotation, weight transfer, complete circuit laps and camera regression checks. `rally-dynamics-validation.json` records a paired open/limited-slip center launch, torque conservation, throttle-dependent cornering, coupling during foot braking, fixed airborne wheel inertia and identical results for the same controls at 30/60/120 Hz. `rally-grip-isolation.json` isolates the previous grip/engine-braking setup gains within the current drivetrain. These are internal acceptance tests, not DiRT Rally benchmarks or certified real-car measurements. GPU/device performance and side-by-side driving feel have not been playtested.
