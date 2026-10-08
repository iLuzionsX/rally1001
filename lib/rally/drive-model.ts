/*!
 * Ecctrl drivetrain adapter, derived from EcctrlVehicle and ShapeCastWheel.
 * https://github.com/pmndrs/ecctrl/tree/e2cab804f9f15661a642e76f52d09f0b2db63f35
 * SPDX-FileCopyrightText: 2023-2026 Erdong Chen
 * Original Ecctrl portions: MIT (retained notice and reference/ecctrl/LICENSE).
 * Timed Rally transmission adaptation: Stunt Rally/VDrift contributors,
 * GPL-3.0-or-later; reference/stunt-rally/cardynamics_simulate.cpp and LICENSE.
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Adaptation: remove React hooks; pass Rapier wheel velocities directly;
 * expose wheel torque to the downloaded wheel-contact/gravel tire adapter.
 * Rapier retains chassis collision resolution.
 */
import {bakeCurveLUT,evaluateCurveLUT,type CurvePoint} from './vendor/ecctrl/CurveLUT';

export type PowertrainConfig={
  engineHorsepower:number;engineMaxRPM:number;idleRPM:number;
  gearRatios:number[];finalDriveRatio:number;shiftUpRPM:number;shiftDownRPM:number;
  shiftCooldown:number;reverseRatio:number;reverseTorqueScale:number;
  shiftDuration?:number;clutchRecovery?:number;
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

export type TransmissionContext={roadSpeed:number;grounded:number;handbrake:number};
export class EcctrlDriveModel{
  gearIndex=0;engineRPM=900;shiftCooldownTimer=0;driveRatio=0;
  shiftClutch=1;shiftRemaining=0;pendingGear=-1;shiftCount=0;
  private candidateGear=-1;private candidateTime=0;
  constructor(public config:PowertrainConfig,public radius:number){this.reset();}
  reset(){this.gearIndex=0;this.engineRPM=this.config.idleRPM;this.shiftCooldownTimer=0;this.clearShift();this.shiftCount=0;this.syncRatio();}
  clearShift(){this.shiftClutch=1;this.shiftRemaining=0;this.pendingGear=-1;this.candidateGear=-1;this.candidateTime=0;}
  private syncRatio(){this.driveRatio=(this.config.gearRatios[this.gearIndex]??this.config.gearRatios[0])*this.config.finalDriveRatio;}
  private setGear(index:number){
    const next=clamp(Math.floor(index),0,this.config.gearRatios.length-1);
    if(this.gearIndex===next)return;
    this.gearIndex=next;this.shiftCooldownTimer=this.config.shiftCooldown;this.syncRatio();
  }
  /** Ecctrl weighted wheel-RPM transmission and hysteresis, now outside React. */
  updateTransmission(wheels:{angularSpeed:number;weight:number;longSlip?:number}[],dt:number,reverse:boolean,context?:TransmissionContext){
    let totalWheelRPM=0,totalDriveTorqueWeight=0;
    for(const wheel of wheels){
      const weight=Math.max(0,wheel.weight);
      totalWheelRPM+=Math.abs(wheel.angularSpeed)*60/(Math.PI*2)*weight;
      totalDriveTorqueWeight+=weight;
    }
    const averageWheelRPM=totalDriveTorqueWeight>0?totalWheelRPM/totalDriveTorqueWeight:0;
    if(context){this.updateRallyTransmission(averageWheelRPM,dt,reverse,context,wheels);return;}
    if(reverse){this.engineRPM=Math.max(this.config.idleRPM,averageWheelRPM*this.config.reverseRatio*this.config.finalDriveRatio);return;}
    this.engineRPM=Math.max(this.config.idleRPM,averageWheelRPM*Math.abs(this.driveRatio));
    if(this.shiftCooldownTimer>0){this.shiftCooldownTimer=Math.max(0,this.shiftCooldownTimer-dt);return;}
    // Stunt Rally CARDYNAMICS::NextGear suppresses autoshifts during large
    // average tire slip: wheelspin is not forward road acceleration.
    if(wheels.reduce((sum,w)=>sum+Math.abs(w.longSlip??0),0)/Math.max(1,wheels.length)>1)return;
    if(this.engineRPM>this.config.shiftUpRPM&&this.gearIndex<this.config.gearRatios.length-1)this.setGear(this.gearIndex+1);
    else if(this.engineRPM<this.config.shiftDownRPM&&this.gearIndex>0)this.setGear(this.gearIndex-1);
    this.engineRPM=Math.max(this.config.idleRPM,averageWheelRPM*Math.abs(this.driveRatio));
  }
  /** Timed disconnect / ratio swap / reconnect adapted from Stunt Rally's
   * UpdateTransmission and ShiftAutoClutch. No engine inertia is simulated:
   * this is a bounded torque envelope around the retained engine curve. */
  private updateRallyTransmission(wheelRPM:number,dt:number,reverse:boolean,context:TransmissionContext,wheels:{longSlip?:number}[]){
    const c=this.config,duration=c.shiftDuration??.16;
    this.shiftCooldownTimer=Math.max(0,this.shiftCooldownTimer-dt);
    if(reverse){this.clearShift();this.engineRPM=Math.max(c.idleRPM,wheelRPM*c.reverseRatio*c.finalDriveRatio);return;}
    if(this.shiftRemaining>0){
      this.shiftRemaining=Math.max(0,this.shiftRemaining-dt);
      const phase=1-this.shiftRemaining/duration;
      const swapped=phase>=.35&&this.pendingGear>=0;
      if(swapped){this.gearIndex=this.pendingGear;this.pendingGear=-1;this.syncRatio();this.shiftCount++;}
      // Torque reaches zero before the ratio changes, then returns smoothly.
      const t=phase<.2?1-phase/.2:phase<.35?0:(phase-.35)/.65;
      const x=clamp(t,0,1);this.shiftClutch=swapped?0:x*x*(3-2*x);
      this.engineRPM=Math.max(c.idleRPM,wheelRPM*Math.abs(this.driveRatio));return;
    }
    this.shiftClutch=1;this.engineRPM=Math.max(c.idleRPM,wheelRPM*Math.abs(this.driveRatio));
    const averageSlip=wheels.reduce((sum,w)=>sum+Math.abs(w.longSlip??0),0)/Math.max(1,wheels.length);
    const roadRPM=Math.abs(context.roadSpeed)/this.radius*30/Math.PI;
    // Locked or airborne wheels are not evidence for selecting a lower gear.
    // Require road speed to confirm an upshift rather than chasing wheelspin.
    if(this.shiftCooldownTimer>0||context.handbrake>.02||context.grounded<2||averageSlip>.7){this.candidateGear=-1;this.candidateTime=0;return;}
    let next=this.gearIndex;
    if(this.engineRPM>c.shiftUpRPM&&roadRPM*this.driveRatio>c.shiftUpRPM*.9&&next<c.gearRatios.length-1)next++;
    else if(roadRPM*this.driveRatio<c.shiftDownRPM&&next>0){
      const predicted=roadRPM*c.gearRatios[next-1]*c.finalDriveRatio;
      if(predicted<c.shiftUpRPM*.82)next--;
    }
    if(next===this.gearIndex){this.candidateGear=-1;this.candidateTime=0;return;}
    if(next!==this.candidateGear){this.candidateGear=next;this.candidateTime=0;}
    this.candidateTime+=dt;
    if(this.candidateTime>=.075){this.pendingGear=next;this.shiftRemaining=duration;this.shiftCooldownTimer=c.shiftCooldown+duration;this.candidateGear=-1;this.candidateTime=0;}
  }
  /** ShapeCastWheel's torque law; N·m/r becomes Rapier tire force in N. */
  wheelForce(demand:number,angularSpeed:number,weight:number,reverse:boolean){
    const ratio=reverse?this.config.reverseRatio*this.config.finalDriveRatio:this.driveRatio;
    const maxWheelAngVel=this.config.engineMaxRPM/ratio*(2*Math.PI/60);
    const angularRatio=maxWheelAngVel>0?Math.abs(angularSpeed)/maxWheelAngVel:1;
    const engineMaxTorque=this.config.engineHorsepower*7022/this.config.engineMaxRPM;
    const driveTorque=demand*engineMaxTorque*weight*ratio*(reverse?this.config.reverseTorqueScale:1)*evaluateCurveLUT(angularRatio,torqueCurve);
    return driveTorque/this.radius;
  }
  steeringLimit(speed:number,maxSpeed:number,maxSteer:number){
    return maxSteer*evaluateCurveLUT(clamp(Math.abs(speed)/maxSpeed,0,1),steerCurve);
  }
  get gear(){return this.gearIndex+1;}
}
