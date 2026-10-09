/*!
 * Ecctrl ShapeCastWheel adapter. Copyright 2023-2026 Erdong Chen, MIT.
 * Pinned source and license: reference/ecctrl/ShapeCastWheel.tsx / LICENSE.
 * Modified: React refs -> class state; production Rapier world and car geometry;
 * Stunt Rally/VDrift gravel forces replace the generic slip-curve impulse law.
 * Suspension casts, spring/damping, effective inertia, reaction/drive/brake
 * rotation and contact-point impulses follow the downloaded wheel controller.
 */
import RAPIER from '@dimforge/rapier3d-compat';
import {Vector3,Quaternion} from 'three';
import {gravelForce} from './vendor/stunt-rally/pacejka';
import {clamp,surfaceAt} from './course';
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
export class RallyWheel {
  x:number;y:number;z:number;steer=0;rotation=0;angularSpeed=0;contact=false;
  contactPoint=new Vector3();normal=new Vector3(0,1,0);supportPoint=new Vector3();
  force=0;slip=0;mud=false;longSlip=0;slipAngle=0;driveTorque=0;brakeTorque=0;
  suspensionLength=0;longVelocity=0;sideVelocity=0;friction=0;loadMass=0;inertia=0;
  handlingMode:HandlingMode='refined';
  relaxedSlipAngle=0;loose=0;bump=0;plough=0;
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
  reset(){this.relaxedSlipAngle=this.bump=this.plough=0;this.steer=this.rotation=this.angularSpeed=this.force=this.slip=this.longSlip=this.slipAngle=this.driveTorque=this.brakeTorque=0;this.contact=false;this.suspensionLength=this.config.suspension;this.y=this.config.mount-this.suspensionLength;}

  contactStep(dt:number){
    const c=this.config,r=c.radius;this.q.copy(this.body.rotation());
    this.origin.set(this.x,c.mount,this.z).applyQuaternion(this.q).add(this.body.translation());
    this.up.set(0,1,0).applyQuaternion(this.q);this.direction.copy(this.up).negate();
    this.steerQ.setFromAxisAngle(localUp,this.steer);this.shapeQ.copy(this.q).multiply(this.steerQ);
    this.longitudinal.set(0,0,-1).applyQuaternion(this.shapeQ);
    const surface=surfaceAt(this.origin.x,this.origin.z);this.mud=surface.mud;this.friction=surface.grip*c.tireGrip;this.rollingResistance=surface.rollingResistance;this.rollingDrag=surface.rollingDrag;this.loose=surface.loose??0;
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
    if(!bestHit){this.relaxedSlipAngle=0;this.bump=0;this.suspensionLength=c.suspension;this.y=c.mount-c.suspension;this.slip=this.longSlip=this.slipAngle=0;return;}
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
      const response=this.frontWheel?c.tireLateralResponse.front:c.tireLateralResponse.rear;
      // First-order contact-patch relaxation, integrated exactly over distance.
      // Keep longitudinal slip immediate for the stiff wheel/ABS solve; lateral
      // deformation builds over a fraction of a metre, not an input delay.
      const speed=Math.abs(this.longVelocity);
      const rawAngle=-Math.atan2(this.sideVelocity,Math.max(speed,.01));
      // Fade into the rolling model to avoid a force discontinuity near rest.
      const relaxationTime=Math.min(.05,c.tireRelaxationLength/Math.max(speed,.01))*clamp((speed-2)/4,0,1);
      const blend=this.handlingMode==='baseline'||relaxationTime===0?1:
        1-Math.exp(-dt/relaxationTime);
      this.relaxedSlipAngle+=(rawAngle-this.relaxedSlipAngle)*blend;
      const tire=gravelForce(this.force,this.friction,this.longVelocity,this.sideVelocity,this.angularSpeed*r,response,
        this.handlingMode==='baseline'?undefined:this.relaxedSlipAngle);
      let long=tire.long,side=tire.side;
      // Only use the Ecctrl static rolling bound near rest, where a slip-ratio
      // tire formula is singular. At road speed the downloaded tire supplies Fx.
      if(Math.abs(this.longVelocity)<2){
        const desired=(this.angularSpeed*r-this.longVelocity)*this.inertia/(r*r*dt);
        long=Math.sign(long)*Math.min(Math.abs(long),Math.abs(desired));
      }
      // Ecctrl's low-speed static blend keeps parked cars still and permits a clean launch.
      const staticWeight=clamp(1-Math.max(Math.abs(this.longVelocity),Math.abs(this.sideVelocity),Math.abs(this.angularSpeed*r-this.longVelocity))/.6,0,1);
      long=long*(1-staticWeight)+(this.angularSpeed*r-this.longVelocity)*this.inertia/(r*r*dt)*staticWeight;
      side=side*(1-staticWeight)-this.sideVelocity*this.loadMass/dt*staticWeight;
      const cap=this.force*this.friction*1.75,magnitude=Math.hypot(long,side);
      if(magnitude>cap){long*=cap/magnitude;side*=cap/magnitude;}
      this.body.applyImpulseAtPoint(this.impulse.copy(this.longitudinal).multiplyScalar(long*dt).addScaledVector(this.lateral,side*dt),this.contactPoint,true);
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
      this.longSlip=tire.slipRatio;this.slipAngle=tire.slipAngle;
      this.slip=clamp(Math.max(Math.abs(this.longSlip)*.6,Math.abs(this.slipAngle)/.35),0,1);
    }
    if(!this.contact||this.force<=0)this.relaxedSlipAngle=0;
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
}
