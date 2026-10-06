# Wildtrail Rally

Private jungle time trial for Sites, with downloaded GitHub assets and the existing Rapier/Bullet raycast vehicle controller. The SUV and pickup have distinct mass, wheelbase, suspension, steering response, acceleration and braking settings.

## Driving

Touch: drag the wheel, hold Gas or Brake, and hold Slide for the rear handbrake. Holding Brake at rest reverses. Keyboard: WASD or arrows, Space to slide, R to recover, C for camera, Escape to pause. Gamepad: left stick, triggers, A handbrake.

Cross 12 checkpoints in order. Recovery adds 3 seconds. Times and preferences stay on the current device.

## Rendering

Three.js WebGL2 with vehicle environment reflections, a planar pond reflection, moving shadow maps, instanced foliage, tire-contact dirt clumps, dust, and persistent tracks. Balanced targets 60 rendered frames per second, Save battery targets 30; physics remains at 60 Hz. These are targets rather than guarantees on every device.

A WebGL2-capable browser is required. A clear fallback appears when graphics are unavailable. Imported models are local files, so gameplay has no third-party asset dependency.

## Foundations and verification

See reference/CREDITS.md and the retained upstream sources. Vehicle mass is recomputed before its first tire query to make vehicle switches safe. Braking cuts engine force exactly because the upstream solver prioritizes a nonzero engine force over braking. Tire tracks use partial buffer uploads.

Build with the supplied Sites execution profile and pnpm. Validation results are recorded in reference/validation.json when complete. The Site remains private.
