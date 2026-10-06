/*!
 * Stunt Rally/VDrift CARENGINE::GetFrictionTorque and SetTorqueCurve adapter.
 * Copyright 2010-2026 Crystal Hammer and contributors. GPL-3.0-or-later.
 * Pinned original source: reference/stunt-rally/carengine.cpp.
 * Modified: take the preset's maximum engine torque instead of a torque-curve
 * object; express wheel torque through the active gear; fade below idle and
 * scale the preset for keyboard/touch rally driving. Full throttle is unchanged.
 */
export function engineFrictionTorque(wheelAngularSpeed:number,driveRatio:number,maxEngineRPM:number,idleRPM:number,maxEngineTorque:number,throttle:number,scale:number){
  const redline=maxEngineRPM*Math.PI/30;
  const angularSpeed=Math.sign(wheelAngularSpeed)*Math.min(Math.abs(wheelAngularSpeed*driveRatio),redline);
  const friction=maxEngineTorque/(redline*redline),b=230*friction;
  const idle=idleRPM*Math.PI/30;
  const idleBlend=Math.max(0,Math.min(1,(Math.abs(angularSpeed)-idle*.65)/idle));
  return -angularSpeed*b*(1-Math.max(0,Math.min(1,throttle)))*driveRatio*scale*idleBlend;
}
