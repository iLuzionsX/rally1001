import {clamp} from './course';

export type SteeringRackConfig={
  /** Rack, column and road-wheel inertia about the kingpins, referred to the
   * road-wheel angle (kg·m²), and the rack's own viscous damping (N·m·s/rad). */
  inertia:number;damping:number;
  /** The driver's grip on the wheel, referred to the road wheels: stiffness
   * (N·m/rad) and damping (N·m·s/rad) toward the commanded angle, the most
   * torque hands plus power assistance can apply (N·m), and the stiffness
   * left when the driver relaxes their hands (N·m/rad). */
  handStiffness:number;handDamping:number;handTorque:number;relaxedStiffness:number;
  /** Share of the front tires' peak side force the driver steers up to at speed;
   * the matching slip angle bounds turn-in for keyboard and gamepad input. */
  slipShare:number;
  /** Below `fullLockSpeed` the whole lock is available; the tire bound is fully in
   * by `slipLockSpeed` (m/s). */
  fullLockSpeed:number;slipLockSpeed:number;
};

/**
 * Steering rack driven by the front tires' aligning torque and the driver's
 * hands. The road-wheel angle is a state with inertia, not the input: the
 * tires pull the wheels toward their direction of travel, the hands pull them
 * toward the commanded angle, and the steered angle is where those balance.
 * With relaxed hands the aligning torque alone trails the wheels into
 * countersteer; near the tires' limit the torque falls away and the rack
 * goes light.
 */
export class SteeringRack{
  angle=0;rate=0;
  /** Aligning torque from the front tires (N·m, + left) and the torque the hands apply. */
  aligning=0;hands=0;
  constructor(public config:SteeringRackConfig,public lock:number){}
  reset(){this.angle=this.rate=this.aligning=this.hands=0;}
  /**
   * Advance by dt toward `target` (rad, + left), with the aligning torque from
   * the last tire solve. `grip` (0-1) blends from relaxed to firm hands. Spring
   * and damping terms are integrated implicitly so stiff hands stay stable.
   */
  step(dt:number,target:number,aligning:number,grip:number){
    const c=this.config,k=c.relaxedStiffness+(c.handStiffness-c.relaxedStiffness)*grip,d=c.handDamping*grip;
    this.aligning=aligning;
    const momentum=c.inertia*this.rate;
    let rate=(momentum+dt*(aligning+k*(target-this.angle)))/(c.inertia+dt*(c.damping+d)+dt*dt*k);
    let hands=k*(target-this.angle-rate*dt)-d*rate;
    if(Math.abs(hands)>c.handTorque){
      // Saturated hands push with constant torque; only the rack damps.
      hands=Math.sign(hands)*c.handTorque;
      rate=(momentum+dt*(aligning+hands))/(c.inertia+dt*c.damping);
    }
    this.hands=hands;this.rate=rate;
    this.angle+=rate*dt;
    if(Math.abs(this.angle)>this.lock){this.angle=Math.sign(this.angle)*this.lock;if(this.rate*this.angle>0)this.rate=0;}
    return this.angle;
  }
}

/** Steering bounds for keyboard/gamepad input: `slip` either way, extended on
 * the side the front axle is travelling toward (`travel`, rad, + left) so full
 * countersteer can point the wheels `slip` past the direction of travel. Turn-in
 * is never cut below `slip` when the rear steps out. Returns [right, left]
 * magnitudes, within the lock. */
export function steerBounds(travel:number,slip:number,lock:number):[number,number]{
  return [clamp(slip+Math.max(0,-travel),0,lock),clamp(slip+Math.max(0,travel),0,lock)];
}
