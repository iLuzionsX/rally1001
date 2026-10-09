/*! Stunt Rally/VDrift tire coefficients.
 * GPL-3.0-or-later; see reference/stunt-rally/LICENSE.
 * Source: fe3ecd73fff671c3d75ecaeadccea5c5e6afe0d8/data/carsim/hard/tires/gravel.tire
 * The donor's hard-mode gravel set is kept unchanged as `stuntRally`.
 * Modified: added per-surface sets in the donor's CARTIRE layout. They keep the
 * donor's peak force levels (the D terms) and refit the shape terms (C, the
 * BCD stiffness and its load peak, E) for tarmac, loose gravel and mud.
 */
/** Donor's hard-mode gravel tire, unchanged. Its side force peaks at ~45°. */
export const stuntRally={
  lateral:[1.18301, -0.0, 1719.79, 636.963, 13.0, 0.013, -0.596032, 0.00622891, 0.0308322, -0.0319974, 0.0496577, 0.0, 0.0, 0.242624, 0.0],
  longitudinal:[1.56981, 108.03, 1401.45, 71.7884, 229.0, 0.257879, 0.0, 0.0, -0.0, 0.0, 0.0],
  aligning:[2.07, -6.49, -21.9, -0.416, -21.3, 0.029, 0.0, -1.2, 5.23, -14.8, 0.0, 0.0, -0.0035, 0.038, 0.0, 0.0, 0.63, 1.69],
};
// Shapes at a 3.7 kN corner load (2.5 / 6 kN in brackets):
/** Tarmac: side force peaks at 8.9° (8.0 / 11.5) and falls to ~0.88 of peak by
 * 30°; drive/brake force peaks at slip ratio 0.10 and a locked wheel keeps ~0.77. */
export const tarmac={
  lateral:[1.45, 0.0, 1719.79, 2300.0, 8.0, 0.0, 0.0, -0.3],
  longitudinal:[1.5, 108.03, 1401.45, 120.0, 370.0, 0.12, 0.0, 0.0, 0.2, 0.0, 0.0],
};
/** Loose gravel and dirt: side force peaks at 13.5° (12.6 / 16.2) and holds
 * ~0.97 of peak out to 45°; drive/brake force peaks at 0.18 and holds ~0.9 locked. */
export const gravel={
  lateral:[1.22, 0.0, 1719.79, 2150.0, 10.0, 0.0, 0.0, -0.8],
  longitudinal:[1.35, 108.03, 1401.45, 62.0, 180.0, 0.12, 0.0, 0.0, -0.4, 0.0, 0.0],
};
/** Mud: low and very flat. Side force is within 5% of its plateau from 8.4°,
 * peaks at 19.7° (18.4 / 23.6) and holds there; drive/brake force peaks at
 * slip ratio 0.27 and a locked wheel keeps ~0.98. */
export const mud={
  lateral:[1.12, 0.0, 1719.79, 2000.0, 10.0, 0.0, 0.0, -1.0],
  longitudinal:[1.15, 108.03, 1401.45, 55.0, 160.0, 0.12, 0.0, 0.0, -1.0, 0.0, 0.0],
};
