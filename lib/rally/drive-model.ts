/*!
 * Ecctrl drivetrain adapter, derived from EcctrlVehicle and ShapeCastWheel.
 * https://github.com/pmndrs/ecctrl/tree/e2cab804f9f15661a642e76f52d09f0b2db63f35
 * SPDX-FileCopyrightText: 2023-2026 Erdong Chen
 * SPDX-License-Identifier: MIT
 *
 * Adaptation: remove React hooks; pass Rapier wheel velocities directly;
 * expose wheel torque to the downloaded wheel-contact/gravel tire adapter.
 * Rapier retains chassis collision resolution.
 */
import {bakeCurveLUT,evaluateCurveLUT,type CurvePoint} from './vendor/ecctrl/CurveLUT';
import {engineFrictionTorque} from './vendor/stunt-rally/engine-friction';

export type PowertrainConfig={
  engineHorsepower:number;engineMaxRPM:number;idleRPM:number;
  gearRatios:number[];finalDriveRatio:number;shiftUpRPM:number;shiftDownRPM:number;
  shiftCooldown:number;reverseRatio:number;reverseTorqueScale:number;
  /** Crank + flywheel inertia (kg·m²), clutch capacity (N·m), the engine speed
   * the auto-clutch holds while launching, and the ignition-cut shift time (s). */
  engineInertia:number;clutchTorque:number;launchRPM:number;shiftTime:number;
};

// Combustion-engine adaptation of Ecctrl's editable normalized torque curve.
const torquePoints:CurvePoint[]=[
  {x:0,y:.82},{x:.2,y:1.06},{x:.5,y:1.22},{x:.75,y:1.14},
  {x:.94,y:1},{x:1.04,y:0},
];
// Ecctrl's default speed steering curve, baked without modifications.
const steerPoints:CurvePoint[]=[
  {x:0,y:1,r_out:0},{x:.2,y:1,r_in:0,r_out:0},{x:1,y:.4,r_in:0},
];
const torqueCurve=bakeCurveLUT(torquePoints,128),steerCurve=bakeCurveLUT(steerPoints,50);
const clamp=(v:number,a:number,b:number)=>Math.max(a,Math.min(b,v));
const damp=(v:number,t:number,k:number,dt:number)=>v+(t-v)*(1-Math.exp(-k*dt));

export class EcctrlDriveModel{
  gearIndex=0;engineRPM=900;gearboxRPM=0;shiftCooldownTimer=0;driveRatio=0;
  /** Engine state: crank speed (rad/s), clutch torque last step (N·m, engine side). */
  engineSpeed=0;demand=0;combustionTorque=0;clutchTorque=0;clutchSlip=0;shiftTimer=0;blip=0;limiterTimer=0;
  private previousShaft=0;private externalAccel=0;private feedforward=false;
  constructor(public config:PowertrainConfig,public radius:number,private engineBraking=.45){this.reset();}
  reset(){this.gearIndex=0;this.shiftCooldownTimer=0;this.syncRatio();this.engineSpeed=this.config.idleRPM*Math.PI/30;this.engineRPM=this.config.idleRPM;this.shiftTimer=this.blip=this.limiterTimer=this.combustionTorque=this.clutchTorque=this.clutchSlip=0;this.feedforward=false;}
  private syncRatio(){this.driveRatio=(this.config.gearRatios[this.gearIndex]??this.config.gearRatios[0])*this.config.finalDriveRatio;}
  private setGear(index:number){
    const next=clamp(Math.floor(index),0,this.config.gearRatios.length-1);
    if(this.gearIndex===next)return;
    // Sequential flat shift: clutch out, ignition cut on upshifts, a throttle
    // blip on downshifts so the crank is near the new gear's speed on re-engage.
    this.blip=next<this.gearIndex?.75:0;
    this.gearIndex=next;this.shiftCooldownTimer=this.config.shiftCooldown;this.shiftTimer=this.config.shiftTime;this.syncRatio();this.feedforward=false;
  }
  /** Signed engine-to-wheel ratio; negative in reverse. */
  ratio(reverse:boolean){return reverse?-this.config.reverseRatio*this.config.finalDriveRatio:this.driveRatio;}
  /** Match the crank to the current wheel speeds, e.g. after placing the car. */
  syncEngine(shaftSpeed:number,reverse:boolean){
    this.engineSpeed=clamp(shaftSpeed*this.ratio(reverse),this.config.idleRPM*Math.PI/30,this.config.engineMaxRPM*Math.PI/30);
    this.engineRPM=this.engineSpeed*30/Math.PI;this.feedforward=false;
  }
  /** Ecctrl weighted wheel-RPM transmission and hysteresis, now outside React.
   * Shift decisions use gearbox speed; the crank speed is the engine's own state. */
  updateTransmission(wheels:{angularSpeed:number;weight:number;longSlip?:number}[],dt:number,reverse:boolean){
    let totalWheelRPM=0,totalDriveTorqueWeight=0;
    for(const wheel of wheels){
      const weight=Math.max(0,wheel.weight);
      totalWheelRPM+=Math.abs(wheel.angularSpeed)*60/(Math.PI*2)*weight;
      totalDriveTorqueWeight+=weight;
    }
    const averageWheelRPM=totalDriveTorqueWeight>0?totalWheelRPM/totalDriveTorqueWeight:0;
    if(reverse){this.gearboxRPM=averageWheelRPM*this.config.reverseRatio*this.config.finalDriveRatio;return;}
    this.gearboxRPM=averageWheelRPM*Math.abs(this.driveRatio);
    if(this.shiftCooldownTimer>0){this.shiftCooldownTimer=Math.max(0,this.shiftCooldownTimer-dt);return;}
    // Stunt Rally CARDYNAMICS::NextGear suppresses autoshifts during large
    // average tire slip: wheelspin is not forward road acceleration.
    if(wheels.reduce((sum,w)=>sum+Math.abs(w.longSlip??0),0)/Math.max(1,wheels.length)>1)return;
    if(this.gearboxRPM>this.config.shiftUpRPM&&this.gearIndex<this.config.gearRatios.length-1)this.setGear(this.gearIndex+1);
    else if(this.gearboxRPM<this.config.shiftDownRPM&&this.gearIndex>0)this.setGear(this.gearIndex-1);
    this.gearboxRPM=averageWheelRPM*Math.abs(this.driveRatio);
  }
  /**
   * Advance the crank and return the torque the clutch delivers to the
   * driveline, at the wheels (N·m). The clutch torque is solved implicitly for
   * the two inertias so a locked clutch carries the engine exactly, including
   * when the wheels are free (airborne, spinning); it is clamped to the clutch
   * capacity, which is where launches, shifts and over-rev slip come from.
   */
  engineStep(dt:number,throttle:number,shaftSpeed:number,reverse:boolean,wheelInertia:number,disengaged:boolean){
    const c=this.config,ratio=this.ratio(reverse);this.demand=throttle;
    const idle=c.idleRPM*Math.PI/30,redline=c.engineMaxRPM*Math.PI/30;
    const maxTorque=c.engineHorsepower*7022/c.engineMaxRPM;
    const gearbox=shaftSpeed*ratio;
    this.shiftTimer=Math.max(0,this.shiftTimer-dt);
    if(this.shiftTimer===0)this.blip=0;
    if(this.engineSpeed>redline)this.limiterTimer=.05;
    this.limiterTimer=Math.max(0,this.limiterTimer-dt);
    // Idle governor: the engine never stalls.
    const idleThrottle=clamp((idle-this.engineSpeed)/(idle*.12),0,1)*.3;
    const upshiftCut=this.shiftTimer>0&&this.blip===0;
    let pedal=Math.max(throttle*(reverse?c.reverseTorqueScale:1),idleThrottle,this.blip);
    if(upshiftCut||this.limiterTimer>0)pedal=0;
    const combustion=maxTorque*evaluateCurveLUT(this.engineSpeed/redline,torqueCurve)*pedal;
    // Stunt Rally pumping friction (engine braking), plus internal losses so a
    // free-revving engine falls at a believable rate once the throttle closes.
    const friction=engineFrictionTorque(this.engineSpeed,1,c.engineMaxRPM,c.idleRPM,maxTorque,pedal,this.engineBraking)-(8+.035*this.engineSpeed)*(1-pedal);
    const engineTorque=combustion+friction;this.combustionTorque=combustion;
    // Auto-clutch: slips to hold launch revs from rest, fully closed once the
    // gearbox turns faster than idle, open while shifting or on the handbrake.
    const launch=clamp((this.engineSpeed-idle*1.05)/((c.launchRPM-c.idleRPM)*Math.PI/30),0,1);
    const rolling=clamp(Math.abs(gearbox)/(idle*1.3),0,1);
    const engagement=disengaged||this.shiftTimer>0?0:Math.max(launch,rolling);
    const capacity=c.clutchTorque*engagement;
    // Driveline inertia reflected to the crank; external (tire, brake) acceleration
    // of the gearbox is estimated from last step with the clutch's share removed.
    const reflected=wheelInertia/(ratio*ratio);
    const measured=this.feedforward?(shaftSpeed-this.previousShaft)/dt*ratio:0;
    const external=this.feedforward?clamp(measured-this.clutchTorque/reflected,-4000,4000):0;
    this.externalAccel=damp(this.externalAccel,external,120,dt);
    let torque=(this.engineSpeed-gearbox+(engineTorque/c.engineInertia-this.externalAccel)*dt)/(dt*(1/c.engineInertia+1/reflected));
    torque=clamp(torque,-capacity,capacity);
    this.engineSpeed=clamp(this.engineSpeed+(engineTorque-torque)/c.engineInertia*dt,0,redline*1.08);
    this.clutchTorque=torque;this.clutchSlip=this.engineSpeed-gearbox;
    this.previousShaft=shaftSpeed;this.feedforward=true;
    this.engineRPM=this.engineSpeed*30/Math.PI;
    return torque*ratio;
  }
  steeringLimit(speed:number,maxSpeed:number,maxSteer:number){
    return maxSteer*evaluateCurveLUT(clamp(Math.abs(speed)/maxSpeed,0,1),steerCurve);
  }
  get gear(){return this.gearIndex+1;}
}
