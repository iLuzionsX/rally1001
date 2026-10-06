import RAPIER from '@dimforge/rapier3d-compat';
import {EcctrlDriveModel} from './drive-model';
import {RallyWheel} from './wheel-model';
import {GravelABS} from './vendor/stunt-rally/abs';
import {engineFrictionTorque} from './vendor/stunt-rally/engine-friction';
import {differentialTorques} from './vendor/stunt-rally/differential';
import {DRIVELINE_STEP} from './simulation';
import {clamp,damp,courseAt,surfaceAt,type CoursePoint} from './course';
import {VEHICLES,type VehicleKind,type DriveInput,type V3,type HandlingMode} from './vehicle-config';
export {VEHICLES,type VehicleKind,type DriveInput,type V3,type HandlingMode} from './vehicle-config';
export type WheelPose=RallyWheel;
/** Downloaded Stunt Rally gravel tire forces + Ecctrl contact/rotation model.
 * Rapier resolves chassis/world collisions. There is no sideways velocity clamp. */
export class RallyVehicle{
 body:RAPIER.RigidBody;config:typeof VEHICLES[VehicleKind];
 steer=0;throttle=0;brake=0;speed=0;forwardSpeed=0;rpm=900;gear=1;surface='LOOSE DIRT';airborne=0;
 driveModel:EcctrlDriveModel;driveDirection:1|-1=1;directionTimer=0;handbrake=0;engineWheelTorque=0;diffTransfer={front:0,rear:0,center:0};
 wheels:WheelPose[]=[];position:V3={x:0,y:0,z:0};rotation={x:0,y:0,z:0,w:1};velocity:V3={x:0,y:0,z:0};forward:V3={x:0,y:0,z:-1};previousPosition:V3={x:0,y:0,z:0};previousRotation={x:0,y:0,z:0,w:1};
 private wheelABS=Array.from({length:4},()=>new GravelABS());
 constructor(public world:RAPIER.World,public kind:VehicleKind,public handlingMode:HandlingMode='refined'){
 const c=this.config=VEHICLES[kind];this.driveModel=new EcctrlDriveModel(c.powertrain,c.radius);this.body=world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setCcdEnabled(true).setCanSleep(false).setLinearDamping(0).setAngularDamping(.08).setAdditionalSolverIterations(4));
 world.createCollider(RAPIER.ColliderDesc.cuboid(c.width*.42,.26,c.length*.45).setTranslation(0,-.18,0).setDensity(0).setFriction(.6).setRestitution(.02),this.body);
 world.createCollider(RAPIER.ColliderDesc.cuboid(c.width*.36,.46,kind==='suv'?1.24:.84).setTranslation(0,.48,kind==='suv'?.18:-.58).setDensity(0).setFriction(.5),this.body);
 this.body.setAdditionalMassProperties(c.mass,{x:0,y:c.centerOfMass,z:kind==='suv'?-.08:-.22},{x:c.mass*(c.length*c.length+1.7)/12,y:c.mass*(c.length*c.length+c.width*c.width)/12,z:c.mass*(c.width*c.width+1.7)/12},{x:0,y:0,z:0,w:1},true);
 // Apply deferred mass changes before the first vehicle query, including swaps.
 this.body.recomputeMassPropertiesFromColliders();
 this.wheels=[{x:-c.track/2,z:c.front},{x:c.track/2,z:c.front},{x:-c.track/2,z:c.back},{x:c.track/2,z:c.back}].map(p=>new RallyWheel(world,this.body,kind,p.x,p.z));
 this.setHandlingMode(handlingMode);
 this.reset(courseAt(0));
 }
 setHandlingMode(mode:HandlingMode){
  this.handlingMode=mode;
  for(const wheel of this.wheels){wheel.handlingMode=mode;wheel.relaxedSlipAngle=0;}
 }
 reset(point:CoursePoint){const yaw=Math.atan2(-point.tx,-point.tz);this.body.setTranslation({x:point.x,y:point.y+this.config.radius+this.config.suspension-this.config.mount+.12,z:point.z},true);this.body.setRotation({x:0,y:Math.sin(yaw/2),z:0,w:Math.cos(yaw/2)},true);this.body.setLinvel({x:0,y:0,z:0},true);this.body.setAngvel({x:0,y:0,z:0},true);this.body.resetForces(true);this.body.resetTorques(true);this.steer=this.throttle=this.brake=this.speed=this.forwardSpeed=this.airborne=this.handbrake=this.directionTimer=0;this.driveDirection=1;this.engineWheelTorque=0;this.diffTransfer={front:0,rear:0,center:0};this.driveModel.reset();this.rpm=this.config.powertrain.idleRPM;this.gear=1;for(const wheel of this.wheels)wheel.reset();for(const abs of this.wheelABS)abs.active=false;this.readPose();this.previousPosition={...this.position};this.previousRotation={...this.rotation};}
 beforeStep(input:DriveInput,dt:number,enabled:boolean){
 const c=this.config,vel=this.body.linvel(),q=this.body.rotation();
 this.body.resetForces(false);this.body.resetTorques(false);
 this.forward={x:-2*(q.x*q.z+q.w*q.y),y:-2*(q.y*q.z-q.w*q.x),z:-(1-2*(q.x*q.x+q.y*q.y))};
 this.forwardSpeed=vel.x*this.forward.x+vel.y*this.forward.y+vel.z*this.forward.z;
 this.speed=Math.hypot(vel.x,vel.z);
 const gas=enabled?clamp(input.throttle,0,1):0,pedal=enabled?clamp(input.brake,0,1):1;
 // A held brake first stops the car, then dwells at rest before selecting reverse.
 const stopped=this.speed<.4&&Math.abs(this.forwardSpeed)<.3;
 if(enabled&&this.driveDirection===1&&pedal>.15&&gas<.1&&stopped){
  this.directionTimer+=dt;
  if(this.directionTimer>=.55){this.driveDirection=-1;this.throttle=0;this.directionTimer=0;}
 }else this.directionTimer=0;
 if(enabled&&this.driveDirection===-1&&gas>.1&&stopped){this.driveDirection=1;this.throttle=0;}
 const drivePedal=enabled?(this.driveDirection===1?gas:pedal):0;
 const stopPedal=enabled?(this.driveDirection===1?pedal:gas):1;
 this.throttle=damp(this.throttle,drivePedal,c.throttleRate,dt);
 // Preserve the original brake response in baseline mode. Brake overrides gas.
 if(this.handlingMode==='baseline')this.brake=stopPedal>this.brake?stopPedal:damp(this.brake,stopPedal,20,dt);
 else{
  // Progressive partial pedal pressure and a short hydraulic buildup/release.
  // Full pedal retains full torque. Raw pedal still cuts propulsion immediately.
  const pressure=stopPedal**1.15;
  this.brake=enabled?damp(this.brake,pressure,pressure>this.brake?35:14,dt):1;
 }
 this.handbrake=damp(this.handbrake,enabled?clamp(input.handbrake,0,1):0,12,dt);
 const wheelbase=c.back-c.front;
 this.driveModel.updateTransmission(this.wheels.map((w,i)=>({angularSpeed:w.angularSpeed,longSlip:w.longSlip,weight:i<2?c.frontDrive/2:(1-c.frontDrive)/2})),dt,this.driveDirection===-1);
 // Ecctrl speed-based steering curve retains low-speed lock and fades at speed.
 const maxSteer=this.driveModel.steeringLimit(this.speed,c.maxSpeed,c.steering);
 const command=enabled?clamp(input.steer,-1,1):0;
 const target=Math.sign(command)*Math.abs(command)**1.15*maxSteer;
 const returning=Math.abs(target)<Math.abs(this.steer)||target*this.steer<0;
 this.steer=damp(this.steer,target,returning?c.steerReturn:c.steerRate,dt);
 // Keep Ecctrl's engine/gearing force law instead of fading torque at 100 km/h.
 const reverse=this.driveDirection===-1;
 const governedSpeed=reverse?7:c.maxSpeed;
 const limiter=clamp((governedSpeed-Math.max(0,this.forwardSpeed*this.driveDirection))/(governedSpeed*.04),0,1);
 const driveDemand=stopPedal>.01||this.brake>.025?0:this.throttle*limiter;
 if(this.speed>.1){const drag=.43*this.speed*this.speed,inv=1/this.speed;this.body.addForce({x:-vel.x*inv*drag,y:0,z:-vel.z*inv*drag},true);}
 for(let i=0;i<4;i++){
  const w=this.wheels[i];let angle=0;
  if(i<2&&Math.abs(this.steer)>.0001){const radius=wheelbase/Math.tan(Math.abs(this.steer)),inside=this.steer>0?i===0:i===1;angle=Math.sign(this.steer)*Math.atan(wheelbase/(radius+(inside?-1:1)*c.track/2));}
  w.steer=angle;w.contactStep();
 }
 // Physical support transfer across each axle; tire forces use the resulting loads.
 for(const [left,right,rate] of [[0,1,c.frontAntiRoll],[2,3,c.rearAntiRoll]]){
  const a=this.wheels[left],b=this.wheels[right];if(!a.contact||!b.contact)continue;
  const transfer=clamp((a.suspensionLength-b.suspensionLength)*rate,-b.force*.8,a.force*.8);
  a.force-=transfer;b.force+=transfer;
 }
 // The force/slip slope is stiff near walking speed. Resolve that regime at
 // 960 Hz inside the 240 Hz chassis step, using fresh contact velocities.
 const repeats=this.wheels.some(w=>w.contact&&Math.abs(w.longVelocity)<6)?Math.max(1,Math.ceil(dt/DRIVELINE_STEP-1e-8)):1;
 for(let n=0;n<repeats;n++)this.solveDriveline(dt/repeats,driveDemand,reverse);
 this.previousPosition={...this.position};this.previousRotation={...this.rotation};
 }

 private solveDriveline(dt:number,driveDemand:number,reverse:boolean){
 const c=this.config;
 for(const wheel of this.wheels)wheel.refreshVelocity();
 const maxWheelSpeed=Math.max(...this.wheels.map(w=>Math.abs(w.angularSpeed)));
 // One engine curve at driven shaft speed, then the donor's center/front/rear
 // differential tree. Wheel-speed differences redistribute torque physically.
 const shaftSpeed=this.wheels.reduce((sum,w,i)=>sum+w.angularSpeed*(i<2?c.frontDrive/2:(1-c.frontDrive)/2),0);
 const ratio=reverse?c.powertrain.reverseRatio*c.powertrain.finalDriveRatio:this.driveModel.driveRatio;
 const driveTorque=this.driveModel.wheelForce(driveDemand,shaftSpeed,1,reverse)*c.radius*this.driveDirection;
 const friction=engineFrictionTorque(shaftSpeed,ratio,c.powertrain.engineMaxRPM,c.powertrain.idleRPM,c.powertrain.engineHorsepower*7022/c.powertrain.engineMaxRPM,this.throttle,c.engineBraking);
 const disengaged=this.handbrake>.02;
 this.engineWheelTorque=disengaged?0:driveTorque+friction;
 const center=differentialTorques(this.engineWheelTorque,(this.wheels[0].angularSpeed+this.wheels[1].angularSpeed)/2,(this.wheels[2].angularSpeed+this.wheels[3].angularSpeed)/2,c.wheelInertia*2,c.wheelInertia*2,dt,disengaged?{...c.differential.center,antiSlip:0}:c.differential.center);
 const front=differentialTorques(center.side1,this.wheels[0].angularSpeed,this.wheels[1].angularSpeed,c.wheelInertia,c.wheelInertia,dt,c.differential.front);
 const rear=differentialTorques(center.side2,this.wheels[2].angularSpeed,this.wheels[3].angularSpeed,c.wheelInertia,c.wheelInertia,dt,c.differential.rear);
 const torques=[front.side1,front.side2,rear.side1,rear.side2];
 this.diffTransfer={front:front.transfer,rear:rear.transfer,center:center.transfer};
 for(let i=0;i<4;i++){
  const w=this.wheels[i],hand=i>1?this.handbrake:0;
  const brakeShare=i<2?c.frontBrake/2:(1-c.frontBrake)/2;
  const absDemand=this.wheelABS[i].update(this.brake,w.longSlip*Math.sign(w.longVelocity),w.force,maxWheelSpeed);
  const brakeTorque=absDemand*c.brake*c.radius*brakeShare+hand*c.brake*c.radius*.38;
  const wheelTorque=torques[i];
  w.solve(dt,hand>.02||Math.abs(wheelTorque)<.05?0:wheelTorque,brakeTorque);
 }
 }
 afterStep(dt:number){
  this.readPose();const vel=this.velocity,q=this.rotation;
  this.forward={x:-2*(q.x*q.z+q.w*q.y),y:-2*(q.y*q.z-q.w*q.x),z:-(1-2*(q.x*q.x+q.y*q.y))};
  this.speed=Math.hypot(vel.x,vel.z);this.forwardSpeed=vel.x*this.forward.x+vel.y*this.forward.y+vel.z*this.forward.z;
  this.airborne=this.wheels.some(w=>w.contact)?0:this.airborne+dt;
  this.surface=surfaceAt(this.position.x,this.position.z).label;
  this.gear=this.driveDirection===-1?-1:this.driveModel.gear;this.rpm=damp(this.rpm,this.driveModel.engineRPM,12,dt);
 }

 private readPose(){this.body.translation(this.position);this.body.rotation(this.rotation);this.body.linvel(this.velocity);}
 dispose(){this.world.removeRigidBody(this.body);}
}
