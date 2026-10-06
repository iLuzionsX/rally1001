import * as THREE from 'three';
import RAPIER from '@dimforge/rapier3d-compat';
import {clamp, damp, terrainHeight} from './course';
import {VEHICLES, type V3, type VehicleKind} from './vehicle-config';

type CameraFrame = {
  position:THREE.Vector3; rotation:THREE.Quaternion; velocity:V3;
  speed:number; forwardSpeed:number; steer:number; kind:VehicleKind;
  mode:'chase'|'hood'; portrait:boolean; dt:number; force?:boolean;
  dynamic?:boolean; lowAngle?:boolean;
};

/** Shorten the chase arm before static geometry; offset rays protect the near plane. */
export function obstructedEye(world:RAPIER.World, anchor:THREE.Vector3, desired:THREE.Vector3){
  const delta=desired.clone().sub(anchor),distance=delta.length();
  if(distance<.001)return desired.clone();
  const direction=delta.divideScalar(distance),side=new THREE.Vector3(-direction.z,0,direction.x).normalize();
  let safeDistance=distance;
  for(const [x,y] of [[0,0],[.22,0],[-.22,0],[0,.22],[0,-.22]]){
    const origin=anchor.clone().addScaledVector(side,x);origin.y+=y;
    const hit=world.castRay(new RAPIER.Ray(origin,direction),distance,true,RAPIER.QueryFilterFlags.EXCLUDE_DYNAMIC|RAPIER.QueryFilterFlags.EXCLUDE_SENSORS);
    if(hit)safeDistance=Math.min(safeDistance,Math.max(0,hit.timeOfImpact-.35));
  }
  return anchor.clone().addScaledVector(direction,safeDistance);
}

/** Horizon-stable camera with separate horizontal, suspension and heading response. */
export class RallyDrivingCamera {
  initialized=false;
  heading=0; height=0; pitch=0;
  position=new THREE.Vector3();target=new THREE.Vector3();
  update(f:CameraFrame, world?:RAPIER.World, groundHeight=terrainHeight){
    const c=VEHICLES[f.kind],dt=clamp(f.dt,0,.1),snap=!!f.force||!this.initialized,dynamic=!!f.dynamic;
    const bodyForward=new THREE.Vector3(0,0,-1).applyQuaternion(f.rotation);
    const desiredPitch=clamp(bodyForward.y,-.16,.16);
    bodyForward.y=0;bodyForward.normalize();
    const follow=bodyForward.clone();
    // Only bias toward travel when moving forward; reversing cannot flip the camera.
    if(dynamic&&f.mode==='chase'&&f.forwardSpeed>2){
      const motion=new THREE.Vector3(f.velocity.x,0,f.velocity.z);
      if(motion.lengthSq()>4)follow.lerp(motion.normalize(),clamp(f.speed/25,0,1)*.12).normalize();
    }
    const desiredHeading=Math.atan2(-follow.x,-follow.z);
    if(snap){this.heading=desiredHeading;this.height=f.position.y;this.pitch=desiredPitch;}
    else{
      const angle=Math.atan2(Math.sin(desiredHeading-this.heading),Math.cos(desiredHeading-this.heading));
      // Standard chase stays aligned with the chassis instead of orbiting in turns.
      if(f.mode==='chase'&&!dynamic)this.heading+=angle;
      else this.heading+=angle*(1-Math.exp(-(f.mode==='hood'?18:c.camera.headingRate)*dt));
      this.height=damp(this.height,f.position.y,f.mode==='hood'?14:c.camera.heightRate,dt);
      this.pitch=damp(this.pitch,desiredPitch,5,dt);
    }
    const forward=new THREE.Vector3(-Math.sin(this.heading),0,-Math.cos(this.heading));
    const right=new THREE.Vector3(-bodyForward.z,0,bodyForward.x);
    let eye:THREE.Vector3,target:THREE.Vector3;
    if(f.mode==='hood'){
      // Keep the eye attached horizontally, with no body-roll-induced horizon tilt.
      eye=f.position.clone().addScaledVector(bodyForward,c.camera.hoodForward);
      eye.y=this.height+c.camera.hoodHeight;
      target=eye.clone().addScaledVector(forward,28);target.y+=this.pitch*12;
      this.position.copy(eye);this.target.copy(target);
    }else{
      const speedBlend=dynamic?clamp(f.speed/c.maxSpeed,0,1):0,distance=(c.camera.distance+speedBlend*1.15)*(f.portrait?1.1:1);
      eye=f.position.clone().addScaledVector(forward,-distance);
      const height=f.lowAngle?(f.kind==='truck'?1.35:1.05):c.camera.height-.3;
      eye.y=this.height+height+speedBlend*.3;
      eye.y=Math.max(eye.y,groundHeight(eye.x,eye.z)+(f.lowAngle?.55:1.2));
      target=f.position.clone().addScaledVector(bodyForward,dynamic?3.2+speedBlend*3.4:2);
      // A small look into the corner helps place the car without swinging the eye.
      if(dynamic)target.addScaledVector(right,-clamp(f.steer*f.speed*.3,-.85,.85));
      target.y=this.height+.45+(dynamic?this.pitch*2:0);
      if(snap||!dynamic){this.position.copy(eye);this.target.copy(target);}
      else{
        this.position.x=damp(this.position.x,eye.x,14,dt);
        this.position.z=damp(this.position.z,eye.z,14,dt);
        this.position.y=damp(this.position.y,eye.y,9,dt);
        this.target.lerp(target,1-Math.exp(-12*dt));
      }
      this.position.y=Math.max(this.position.y,groundHeight(this.position.x,this.position.z)+(f.lowAngle?.45:.75));
      if(world){
        const anchor=f.position.clone();anchor.y+=.55;
        this.position.copy(obstructedEye(world,anchor,this.position));
      }
    }
    this.initialized=true;
    // Modest speed widening avoids large perspective changes while braking.
    const fov=(f.portrait?64:57)+(f.mode==='hood'?2:0)+(dynamic?clamp(f.speed/c.maxSpeed,0,1)*5:0);
    return {position:this.position,target:this.target,fov};
  }
}
