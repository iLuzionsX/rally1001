# Rough-road suspension pass

Rally handling now resolves suspension velocity against the contact normal,
following the retained Rapier/Bullet raycast vehicle reference's projected
relative-velocity calculation. The old chassis-up projection misses compression
from forward travel into a slope and can damp tangent travel incorrectly.

Fast compression uses a digressive damper curve: above 0.65 m/s for the Subaru
and 0.5 m/s for the truck, incremental damping is 25% and 20%, respectively.
Slow compression, rebound damping, springs, travel, tire model and differential
settings retain their existing values. These thresholds are game tuning.
Refined and Original retain their previous suspension calculation.

The new automated collision-mesh course contains flat ground, three 10 cm bumps,
the same bumps with steering input, one-sided 14 cm bumps, a 60 cm / 20 m crest,
and a 60 cm drop landing. Each is exercised with both vehicles at 40 and 70 km/h.
This is a solver test fixture, not a new selectable in-game map.

`node scripts/check-rough-road.mjs` checks 24 runs for finite forces, bounded
travel, upright motion, traversal of the entire section, no persistent bouncing,
and timely unloading/recontact over the fast crest. Separate slope checks ensure
tangent travel generates no damping and approaching a rising plane does.

`RALLY_ROUGH_BEFORE=1 node scripts/check-rough-road.mjs` loads the actual wheel
implementation from Site commit 5bf089441a4b010bf2df931be363fb2bf8b97409 for
comparison; it needs that historical Git object. It records diagnostic outcomes
without enforcing the new acceptance bounds. The checked-in before report can
also be read without that source history.

In the 70 km/h crest fixture the previous Subaru lost its line and left the test
lane. The corrected car stays aligned, spends 0.258 s airborne, and traverses the
section. The truck's airborne time falls from 0.438 to 0.238 s; peak wheel load
falls from 3.40 to 3.19 times quarter-car static load. Both remain upright and
settle after the crest. Sharp bumps create stronger transient wheel loads than
before because road-induced compression is now included. The drop test is
stable, but this pass does not claim reduced impact loads in every landing.

The complete handling and corner-balance suites pass, including braking,
countersteering, landing, and 12-checkpoint circuit laps. Refined/Original
comparison results are unchanged. Rally times use a new `-rally-v2` namespace;
old records remain stored but are not compared against the new suspension.

Reports: rough-road-before.json, rough-road-validation.json,
rough-road-handling.json, and the refreshed corner-balance-validation.json.
Browser/GPU and hands-on verification are unavailable in this environment.
