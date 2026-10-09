/*!
 * Ecctrl ShapeCastWheel adapter. Copyright 2023-2026 Erdong Chen, MIT.
 * Pinned source and license: reference/ecctrl/ShapeCastWheel.tsx / LICENSE.
 * Modified: React refs -> class state; production Rapier world and car geometry;
 * Stunt Rally/VDrift per-surface tire curves replace the generic slip-curve
 * impulse law, fed by a relaxed contact-patch slip model that is also the
 * low-speed tire (it replaces the static and near-rest rolling bounds).
 * Suspension casts, spring/damping, effective inertia, reaction/drive/brake
 * rotation and contact-point impulses follow the downloaded wheel controller.
 */
import RAPIER from '@dimforge/rapier3d-compat';
import {Vector3,Quaternion} from 'three';
import {TIRES,type TireCurve} from './vendor/stunt-rally/pacejka';
import {clamp,surfaceAt,type SurfaceType} from './course';
import {VEHICLES,type VehicleKind,type HandlingMode} from './vehicle-config';

const localUp=new Vector3(0,1,0);
// Fore/aft ray positions across the contact patch, as fractions of tire radius.
const PATCH=[0,-.4,.4,-.75,.75];
// Ploughing force per unit load per unit surface looseness, at full slide.
const PLOUGH_SIDE=.34,PLOUGH_LOCK=.2;
// Bump stop: linear and cubic stiffness (x spring rate) across the stop's depth,
// extra compression damping at full depth (x damper), and the per-wheel force
// bound in multiples of the car's weight.
const BUMP_LINEAR=3,BUMP_PROGRESSIVE=14,BUMP_DAMPING=3,BUMP_LIMIT=4.5;
// Low-speed contact patch. Below PATCH_HOLD the patch stores no more deflection
// than its peak force needs, since past that it slides rather than winding up;
// the bound is released by PATCH_RELEASE (m/s).
const PATCH_HOLD=3,PATCH_RELEASE=6;
// Carcass damping on the patch deflection rate, as fractions of critical. The
// wheel's spin on its tread spring is damped at every speed; the corner mass on
// the patch springs only near rest, fading out by PATCH_DAMP_SPEED (m/s) where
// slip relaxation damps the chassis.
const WHEEL_DAMPING=.7,CORNER_DAMPING=1,PATCH_DAMP_SPEED=3;
// Baseline mode keeps immediate slip at road speed: its relaxation lengths
// fade out between these speeds (m/s).
const IMMEDIATE_FROM=2,IMMEDIATE_TO=6;
/**
 * Advance a contact-patch slip state `z` (carcass deflection over relaxation
 * length) by dt: dz/dt = (slipVelocity - speed*z)/length, exact for constant
 * inputs. Rolling, z relaxes to slipVelocity/speed within a few lengths; at
 * rest it integrates the slip velocity, so the patch is a spring that holds a
 * parked car. A zero length gives immediate slip. `peak` bounds z (up to the
 * release speed, `hold` 0-1) at the curve's peak slip.
 */
function relaxSlip(z:number,slipVelocity:number,speed:number,length:number,dt:number,peak:number,hold:number){
  if(length<=1e-6)return slipVelocity/Math.max(speed,.01);
  const rate=speed/length,gain=rate>1e-9?-Math.expm1(-rate*dt)/rate:dt;
  z=z*Math.exp(-rate*dt)+slipVelocity/length*gain;
  if(hold<1){const bound=peak/(1-hold);z=clamp(z,-bound,bound);}
  return z;
}
export class RallyWheel {
  x:number;y:number;z:number;steer=0;rotation=0;angularSpeed=0;contact=false;
  contactPoint=new Vector3();normal=new Vector3(0,1,0);supportPoint=new Vector3();
  force=0;slip=0;mud=false;longSlip=0;slipAngle=0;driveTorque=0;brakeTorque=0;
  suspensionLength=0;longVelocity=0;sideVelocity=0;friction=0;loadMass=0;inertia=0;
  handlingMode:HandlingMode='refined';
  /** Contact-patch slip states: slip angle (rad) and slip ratio. */
  relaxedSlipAngle=0;relaxedSlip=0;loose=0;bump=0;plough=0;
  surface:SurfaceType='gravel';
  /** Torque about the kingpin from the tire's side force acting behind the
   * patch centre (N·m, + steers left), and the trail it acts at (m). */
  aligning=0;trail=0;
  private config;private frontWheel:boolean;
  rollingResistance=.025;rollingDrag=0;
  private origin=new Vector3();private direction=new Vector3();private longitudinal=new Vector3();private lateral=new Vector3();
  private up=new Vector3();private rayOrigin=new Vector3();private rayNormal=new Vector3();private q=new Quaternion();private steerQ=new Quaternion();private shapeQ=new Quaternion();
  private velocity=new Vector3();private impulse=new Vector3();
  private ground:RAPIER.RigidBody|null=null;
  constructor(public world:RAPIER.World,public body:RAPIER.RigidBody,kind:VehicleKind,x:number,z:number){
    const c=this.config=VEHICLES[kind];this.x=x;this.z=z;this.y=c.mount-c.suspension;this.frontWheel=z===c.front;
    this.inertia=c.wheelInertia;this.reset();
  }
  reset(){this.relaxedSlipAngle=this.relaxedSlip=this.bump=this.plough=this.aligning=this.trail=0;this.steer=this.rotation=this.angularSpeed=this.force=this.slip=this.longSlip=this.slipAngle=this.driveTorque=this.brakeTorque=0;this.contact=false;this.suspensionLength=this.config.suspension;this.y=this.config.mount-this.suspensionLength;}

  contactStep(dt:number){
    const c=this.config,r=c.radius;this.q.copy(this.body.rotation());
    this.origin.set(this.x,c.mount,this.z).applyQuaternion(this.q).add(this.body.translation());
    this.up.set(0,1,0).applyQuaternion(this.q);this.direction.copy(this.up).negate();
    this.steerQ.setFromAxisAngle(localUp,this.steer);this.shapeQ.copy(this.q).multiply(this.steerQ);
    this.longitudinal.set(0,0,-1).applyQuaternion(this.shapeQ);
    const surface=surfaceAt(this.origin.x,this.origin.z);this.surface=surface.type;this.mud=surface.mud;this.friction=surface.grip*c.tireGrip;this.rollingResistance=surface.rollingResistance;this.rollingDrag=surface.rollingDrag;this.loose=surface.loose??0;
    // The tire carcass envelopes corrugations shorter than its patch: low-pass them.
    this.bump+=((surface.bump??0)-this.bump)*(1-Math.exp(-dt/.012));
    // Rays across the patch see a bump before the axle reaches it. Each ray's
    // hit is converted to the axle height at which the tire circle touches it.
    // Ecctrl's ray mode stays stable across the road's triangle seams.
    let best=Infinity,bestHit:RAPIER.RayColliderIntersection|null=null,bestOffset=0;
    for(const fraction of PATCH){
      const offset=fraction*r,drop=Math.sqrt(r*r-offset*offset);
      this.rayOrigin.copy(this.origin).addScaledVector(this.longitudinal,offset);
      const hit=this.world.castRayAndGetNormal(new RAPIER.Ray(this.rayOrigin,this.direction),c.suspension+drop,false,RAPIER.QueryFilterFlags.EXCLUDE_SENSORS,undefined,undefined,this.body);
      // Only road-facing support can suspend the wheel; obstacle sides collide with the chassis.
      if(!hit||this.rayNormal.copy(hit.normal).dot(this.up)<.2)continue;
      const length=hit.timeOfImpact-this.bump-drop;
      if(length<best){best=length;bestHit=hit;bestOffset=offset;}
    }
    this.contact=!!bestHit;this.force=0;
    if(!bestHit){this.relaxedSlipAngle=this.relaxedSlip=this.aligning=this.trail=0;this.bump=0;this.suspensionLength=c.suspension;this.y=c.mount-c.suspension;this.slip=this.longSlip=this.slipAngle=0;return;}
    const hit=bestHit;
    this.suspensionLength=clamp(best,0,c.suspension);this.y=c.mount-this.suspensionLength;
    this.normal.copy(hit.normal).normalize();
    this.supportPoint.copy(this.origin).addScaledVector(this.direction,this.suspensionLength);
    this.contactPoint.copy(this.origin).addScaledVector(this.longitudinal,bestOffset).addScaledVector(this.direction,hit.timeOfImpact-this.bump);
    this.longitudinal.projectOnPlane(this.normal).normalize();
    this.lateral.crossVectors(this.longitudinal,this.normal).normalize();
    this.ground=hit.collider.parent();this.refreshVelocity();
    const vertical=this.velocity.dot(this.up),compression=c.suspension-this.suspensionLength;
    const damping=vertical<0?c.compressionDamping:c.reboundDamping;
    // Progressive rubber bump stop over the last of the travel: stiffens with
    // depth and damps the closing speed, so a landing is caught by the
    // suspension rather than the chassis collider. Bounded only for stability.
    const stopRange=c.suspension-c.suspensionTravel,depth=clamp(Math.max(0,compression-c.suspensionTravel)/stopRange,0,1.5);
    const bumpStop=c.springRate*stopRange*depth*(BUMP_LINEAR+BUMP_PROGRESSIVE*depth*depth)-(vertical<0?c.compressionDamping*BUMP_DAMPING*depth*vertical:0);
    this.force=clamp(c.springRate*compression-damping*vertical+bumpStop,0,c.mass*9.81*BUMP_LIMIT);
  }

  refreshVelocity(){
    if(!this.contact)return;
    this.velocity.copy(this.body.velocityAtPoint(this.contactPoint));
    if(this.ground?.isDynamic())this.velocity.sub(this.ground.velocityAtPoint(this.contactPoint));
    this.longVelocity=this.velocity.dot(this.longitudinal);this.sideVelocity=this.velocity.dot(this.lateral);
  }

  solve(dt:number,driveTorque:number,brakeTorque:number){
    const c=this.config,r=c.radius;this.driveTorque=driveTorque;this.brakeTorque=brakeTorque;
    this.loadMass=this.contact?this.force/9.81:20;
    // CARWHEEL's configured rotational inertia is independent of normal load.
    // The finer production simulation resolves tire/drive/brake reaction torques.
    this.inertia=c.wheelInertia;
    if(this.contact&&this.force>0){
      this.body.applyImpulseAtPoint(this.impulse.copy(this.normal).multiplyScalar(this.force*dt),this.supportPoint,true);
      const tire=TIRES[this.surface],mu=this.friction,speed=Math.abs(this.longVelocity);
      const slipVelocity=this.angularSpeed*r-this.longVelocity,denominator=Math.max(speed,.01);
      // Kinematic slip, for telemetry and ABS.
      this.longSlip=slipVelocity/denominator;this.slipAngle=-Math.atan2(this.sideVelocity,denominator);
      const deflectionX=this.relaxedSlip,deflectionY=Math.tan(this.relaxedSlipAngle);
      const length=this.relax(dt,tire,speed,slipVelocity);
      let {long,side}=tire.force(this.force,mu,this.relaxedSlip,this.relaxedSlipAngle);
      // Carcass damping acts on the rate the patch deflects (m/s), so it adds
      // nothing in steady rolling and cannot reshape the curves. Only the
      // adhered part of the patch is elastic, so it fades as grip is used up.
      // Implicit: the force cannot reverse the deflection within a step. The
      // wheel's inertia shares the longitudinal force with the corner mass.
      const peakX=Math.max(1,tire.longitudinalPeak(this.force,mu)),peakY=Math.max(1,tire.lateralPeak(this.force,mu));
      const adhered=clamp(1-Math.hypot(long/peakX,side/peakY),0,1);
      const fade=clamp(1-speed/PATCH_DAMP_SPEED,0,1)**2*adhered,k=tire.stiffness(this.force,mu);
      const wheelMass=this.inertia/(r*r),mass=this.loadMass,reduced=wheelMass*mass/(wheelMass+mass);
      if(length.longitudinal>0){
        const spring=k.longitudinal/length.longitudinal,rate=length.longitudinal*(this.relaxedSlip-deflectionX)/dt;
        const cx=2*Math.sqrt(spring)*Math.max(WHEEL_DAMPING*adhered*Math.sqrt(wheelMass),CORNER_DAMPING*fade*Math.sqrt(mass));
        long+=cx*rate/(1+cx*dt/reduced);
      }
      if(length.lateral>0&&fade>0){
        const spring=k.lateral/length.lateral,rate=length.lateral*(Math.tan(this.relaxedSlipAngle)-deflectionY)/dt;
        const cy=2*CORNER_DAMPING*fade*Math.sqrt(spring*mass);
        side+=cy*rate/(1+cy*dt/mass);
      }
      // Spring, damping and curve together stay inside the friction ellipse.
      const usage=Math.hypot(long/peakX,side/peakY);
      if(usage>1){long/=usage;side/=usage;}
      this.body.applyImpulseAtPoint(this.impulse.copy(this.longitudinal).multiplyScalar(long*dt).addScaledVector(this.lateral,side*dt),this.contactPoint,true);
      if(this.frontWheel){
        // Brush-model pneumatic trail: a third of the half patch with the patch
        // fully adhered, shrinking to nothing as the sliding zone spreads
        // forward. The adhered share follows from the force utilisation, so
        // braking and drive force lighten the steering as they use up grip.
        const used=clamp(Math.hypot(long,side)/Math.max(1,tire.lateralPeak(this.force,mu)),0,1);
        const sliding=1-Math.cbrt(1-used);
        const pneumatic=used<1e-4?c.tirePatch/3:c.tirePatch*sliding*(1-sliding)**3/used;
        this.trail=pneumatic+c.casterTrail;this.aligning=this.trail*side;
      }
      // Loose-surface ploughing: a sliding tire bulldozes gravel into a berm and
      // pushes against it, so side force holds up past the tire's peak instead
      // of fading, and a slide scrubs speed. A locked tire wedges gravel ahead.
      // Bounded so it can only stop, never reverse, the sliding velocity.
      this.plough=0;
      if(this.handlingMode!=='baseline'&&this.loose>0){
        const sideBuild=clamp((Math.abs(this.sideVelocity)-2)/6,0,1);
        if(sideBuild>0){
          this.plough=Math.min(this.loose*this.force*PLOUGH_SIDE*sideBuild,Math.abs(this.sideVelocity)*this.loadMass/dt);
          this.body.applyImpulseAtPoint(this.impulse.copy(this.lateral).multiplyScalar(-Math.sign(this.sideVelocity)*this.plough*dt),this.contactPoint,true);
        }
        const skid=this.longVelocity-this.angularSpeed*r;
        if(skid*this.longVelocity>0){
          const wedge=Math.min(this.loose*this.force*PLOUGH_LOCK*clamp((Math.abs(skid)-2)/6,0,1),Math.abs(this.longVelocity)*this.loadMass/dt);
          if(wedge>0)this.body.applyImpulseAtPoint(this.impulse.copy(this.longitudinal).multiplyScalar(-Math.sign(this.longVelocity)*wedge*dt),this.contactPoint,true);
        }
      }
      // CARDYNAMICS::ApplyTireForce adds separate loose-ground contact drag in
      // both tangent directions. This dissipates motion, without aligning it.
      const drag=Math.min(this.rollingDrag,this.loadMass/dt);
      this.body.applyImpulseAtPoint(this.impulse.copy(this.longitudinal).multiplyScalar(-this.longVelocity*drag*dt).addScaledVector(this.lateral,-this.sideVelocity*drag*dt),this.contactPoint,true);
      this.angularSpeed-=long*r/this.inertia*dt;
      this.slip=clamp(Math.max(Math.abs(this.longSlip)*.6,Math.abs(this.slipAngle)/.35),0,1);
    }
    if(!this.contact||this.force<=0)this.relaxedSlipAngle=this.relaxedSlip=this.aligning=this.trail=0;
    // CARDYNAMICS::ApplyWheelTorque combines tire reaction and shaft torque
    // before applying the brake's lock-up bound. Differential coupling remains
    // active while braking; it is not a propulsive engine-throttle command.
    this.angularSpeed+=driveTorque/this.inertia*dt;
    if(brakeTorque>0)this.angularSpeed-=Math.sign(this.angularSpeed)*Math.min(Math.abs(this.angularSpeed),brakeTorque/this.inertia*dt);
    {
      const rolling=this.contact?this.rollingResistance*this.force*r:Math.abs(this.angularSpeed)*.03;
      this.angularSpeed-=Math.sign(this.angularSpeed)*Math.min(Math.abs(this.angularSpeed),rolling/this.inertia*dt);
    }
    this.rotation+=this.angularSpeed*dt;
  }

  /** Advance both contact-patch slip states; returns the relaxation lengths used (m). */
  private relax(dt:number,tire:TireCurve,speed:number,slipVelocity:number){
    const lengths=this.config.tireRelaxationLength;
    const immediate=this.handlingMode==='baseline'?clamp((speed-IMMEDIATE_FROM)/(IMMEDIATE_TO-IMMEDIATE_FROM),0,1):0;
    const lateral=lengths.lateral*(1-immediate),longitudinal=lengths.longitudinal*(1-immediate);
    const {sigmaHat,alphaHat}=tire.optimumSlip(this.force),hold=clamp((speed-PATCH_HOLD)/(PATCH_RELEASE-PATCH_HOLD),0,1);
    this.relaxedSlipAngle=Math.atan(relaxSlip(Math.tan(this.relaxedSlipAngle),-this.sideVelocity,speed,lateral,dt,Math.tan(alphaHat*Math.PI/180),hold));
    this.relaxedSlip=relaxSlip(this.relaxedSlip,slipVelocity,speed,longitudinal,dt,sigmaHat,hold);
    return {lateral,longitudinal};
  }
}
