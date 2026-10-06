/*!
 * Stunt Rally / VDrift CARDIFFERENTIAL adapter. GPL-3.0-or-later.
 * Pinned original: reference/stunt-rally/cardifferential.cpp and .h.
 * Modified: TypeScript, final gearing applied by the engine adapter, reverse
 * torque classified relative to shaft rotation, and a timestep stability bound.
 * Coupling transfers equal and opposite torque; it never adds engine power.
 */
export type DifferentialConfig={antiSlip:number;torqueSensitivity:number;coastFactor:number;split:number};
const clamp=(v:number,a:number,b:number)=>Math.max(a,Math.min(b,v));

export function differentialTorques(torque:number,speed1:number,speed2:number,
  inertia1:number,inertia2:number,dt:number,config:DifferentialConfig){
  const difference=speed1-speed2;
  const direction=Math.sign(speed1+speed2)||Math.sign(torque)||1;
  let coupling=config.torqueSensitivity>0?config.torqueSensitivity*torque*direction:config.antiSlip;
  if(coupling<0)coupling*=-config.coastFactor;
  let drag=clamp(Math.max(0,coupling)*difference,-config.antiSlip,config.antiSlip);
  // Equalization is the maximum dissipative transfer for this integration step.
  // This numerical bound prevents a stiff diff from reversing the speed gap.
  const maximum=Math.abs(difference)/(dt*(1/inertia1+1/inertia2));
  drag=Math.sign(drag)*Math.min(Math.abs(drag),maximum);
  return {side1:torque*(1-config.split)-drag,side2:torque*config.split+drag,transfer:drag};
}
