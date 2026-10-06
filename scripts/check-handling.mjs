import assert from 'node:assert/strict';
import {readFileSync, writeFileSync, mkdirSync, mkdtempSync, rmSync} from 'node:fs';
import {createRequire} from 'node:module';
import {dirname, join} from 'node:path';
import {fileURLToPath} from 'node:url';
import ts from 'typescript';

// Exercise the shipped Rapier vehicle code without a renderer or browser.
const root = dirname(dirname(fileURLToPath(import.meta.url)));
const cache = join(root, 'node_modules/.cache');
mkdirSync(cache, {recursive:true});
const temp = mkdtempSync(join(cache, 'rally-handling-'));
const require = createRequire(import.meta.url);
const RAPIER = require('@dimforge/rapier3d-compat');
const zero = {steer:0, throttle:0, brake:0, handbrake:0};
const dt = 1/60;
try {
  for (const name of ['course', 'vehicle-config', 'vehicle', 'simulation', 'camera', 'drive-model', 'wheel-model', 'vendor/ecctrl/CurveLUT', 'vendor/stunt-rally/gravel', 'vendor/stunt-rally/pacejka', 'vendor/stunt-rally/abs', 'vendor/stunt-rally/engine-friction', 'vendor/stunt-rally/differential']) {
    const source = readFileSync(join(root, 'lib/rally', name+'.ts'), 'utf8');
    const {outputText} = ts.transpileModule(source, {compilerOptions:{module:ts.ModuleKind.CommonJS, target:ts.ScriptTarget.ES2022, esModuleInterop:true}});
    mkdirSync(dirname(join(temp,name+'.cjs')),{recursive:true});
    writeFileSync(join(temp, name+'.cjs'), outputText.replace(/require\("\.\/(.*?)"\)/g, 'require("./$1.cjs")'));
  }
  const course = require(join(temp, 'course.cjs'));
  const {RallyVehicle} = require(join(temp, 'vehicle.cjs'));
  const {advanceVehicle} = require(join(temp, 'simulation.cjs'));
  const {RallyDrivingCamera, obstructedEye} = require(join(temp, 'camera.cjs'));
  const THREE = require('three');
  const productionSurface = course.surfaceAt;
  await RAPIER.init();
  function rig(kind, grip=.59) {
    course.surfaceAt = ()=>({grip, rollingResistance:grip<.4?.045:.025, rollingDrag:grip<.4?45:4, mud:grip<.4, label:grip<.4?'WET MUD':'LOOSE DIRT'});
    const world = new RAPIER.World({x:0,y:-9.81,z:0});
    world.timestep = dt;
    world.integrationParameters.numSolverIterations = 8;
    world.createCollider(RAPIER.ColliderDesc.cuboid(2000, .5, 2000).setTranslation(0,-.5,0).setFriction(.9));
    const v = new RallyVehicle(world, kind, process.env.RALLY_HANDLING_MODE==='baseline'?'baseline':'refined');
    v.reset({x:0,y:0,z:0,tx:0,tz:-1,width:12,s:0,distance:0});
    const step = (input=zero, enabled=true)=>advanceVehicle(v,world,input,dt,enabled);
    for (let i=0;i<120;i++) step(zero,false);
    return {world,v,step,dispose:()=>world.free()};
  }
  const results = [];
  for (const kind of ['suv','truck']) {
    const r=rig(kind), {v,step}=r;
    let accelTime=null,zeroTo100=null,maxGear=1;
    for(let i=0;i<600;i++){step({...zero,throttle:1});if(accelTime===null&&v.speed*3.6>=60)accelTime=(i+1)*dt;if(zeroTo100===null&&v.speed*3.6>=100)zeroTo100=(i+1)*dt;maxGear=Math.max(maxGear,v.gear);}
    assert(accelTime!==null,kind+' must reach 60 km/h');
    const speed=v.speed*3.6;
    assert(zeroTo100!==null&&zeroTo100<8,kind+' must reach 100 km/h without a low-speed torque fade');
    assert(maxGear>=3,'Automatic gearbox must shift under acceleration');
    for(let i=0;i<900;i++){step({...zero,throttle:1});maxGear=Math.max(maxGear,v.gear);}
    const sustainedSpeed=v.speed*3.6;
    assert(sustainedSpeed>(kind==='suv'?200:165),kind+' must keep accelerating well beyond 100 km/h');
    // Higher road speeds must remain stoppable with the same production brakes.
    v.body.setLinvel({x:0,y:0,z:-120/3.6},true);v.wheels.forEach(w=>w.angularSpeed=120/3.6/v.config.radius);
    const fastBrakeStart={...v.position};let brake120Time=0;
    for(let i=0;i<360;i++){step({...zero,brake:1});if(v.speed<.15){brake120Time=(i+1)*dt;break;}}
    const brake120Distance=Math.hypot(v.position.x-fastBrakeStart.x,v.position.z-fastBrakeStart.z);
    assert(brake120Time>0&&brake120Time<5&&brake120Distance<80,'Higher speed must retain effective braking');
    assert(Math.abs(v.position.x-fastBrakeStart.x)<.5,'Straight braking must not introduce a spin');
    // Compare braking from the same 80 km/h with an already grounded chassis.
    v.body.setLinvel({x:0,y:0,z:-80/3.6},true);v.wheels.forEach(w=>w.angularSpeed=80/3.6/v.config.radius);
    v.throttle=0;
    const start={...v.position};let brakeTime=0;
    for(let i=0;i<600;i++){step({...zero,brake:1});if(v.speed<.15){brakeTime=(i+1)*dt;break;}}
    assert(brakeTime>0&&brakeTime<3.5,kind+' braking should stop promptly');
    assert.equal(v.driveDirection,1,'Braking must stop before selecting reverse');
    const distance=Math.hypot(v.position.x-start.x,v.position.z-start.z);
    assert(distance<40,kind+' should stop from 80 km/h within 40 m on dry dirt');
    for(let i=0;i<15;i++)step({...zero,brake:1});
    assert.equal(v.driveDirection,1,'A short brake hold must not reverse');
    for(let i=0;i<180;i++)step({...zero,brake:1});
    assert.equal(v.driveDirection,-1,'A sustained hold at rest should reverse');
    assert(v.forwardSpeed<-.5&&v.forwardSpeed>-7.2,'Reverse must move backward with limited speed');
    for(let i=0;i<240;i++)step({...zero,throttle:1});
    assert.equal(v.driveDirection,1,'Gas must brake reverse motion before returning to drive');
    assert(v.forwardSpeed>1);
    // Two-finger touch input: the brake must defeat a still-held accelerator.
    v.body.setLinvel({x:0,y:0,z:-80/3.6},true);v.wheels.forEach(w=>w.angularSpeed=80/3.6/v.config.radius);
    const overlapStart={...v.position};let overlapStopped=false;
    for(let i=0;i<180;i++){
      step({...zero,throttle:1,brake:1});
      assert(v.engineWheelTorque<=0,'Brake must cut propulsive engine torque immediately; diffs can still transfer it');
      if(v.speed<.15){overlapStopped=true;break;}
    }
    assert(overlapStopped,'Brake and gas held together must stop the car');
    assert(Math.hypot(v.position.x-overlapStart.x,v.position.z-overlapStart.z)<40,'Overlapping pedals must not extend stopping distance');
    // Sharp steering at speed, releasing the wheel, and a brief handbrake turn.
    v.body.setLinvel({x:0,y:0,z:-80/3.6},true);v.wheels.forEach(w=>w.angularSpeed=80/3.6/v.config.radius);
    let minimumUp=1,maxYawRate=0;
    for(let i=0;i<180;i++){
      step({...zero,steer:1,throttle:.3});
      const q=v.rotation;minimumUp=Math.min(minimumUp,1-2*(q.x*q.x+q.z*q.z));
      maxYawRate=Math.max(maxYawRate,Math.abs(v.body.angvel().y));
    }
    assert(minimumUp>.8,kind+' must stay upright under full steering at speed');
    const turningSteer=Math.abs(v.steer);
    for(let i=0;i<30;i++)step(zero);
    assert(Math.abs(v.steer)<turningSteer*.08,'Wheel should smoothly return toward center');
    for(let i=0;i<45;i++)step({...zero,steer:.6,handbrake:1});
    assert(Math.abs(v.wheels[2].angularSpeed)<1,'Handbrake should lock the rear wheel');
    for(let i=0;i<60;i++)step(zero);
    assert(v.wheels.every(w=>Number.isFinite(w.angularSpeed)),'Wheel rotation must stay finite after releasing handbrake');
    assert(Number.isFinite(v.position.y)&&Number.isFinite(v.rotation.w));
    results.push({kind,zeroTo60:+accelTime.toFixed(2),tenSecondSpeed:+speed.toFixed(1),zeroTo100:+zeroTo100.toFixed(2),sustainedSpeed:+sustainedSpeed.toFixed(1),maxGear,brake120Time:+brake120Time.toFixed(2),brake120Distance:+brake120Distance.toFixed(1),brake80Time:+brakeTime.toFixed(2),brake80Distance:+distance.toFixed(1),minimumUp:+minimumUp.toFixed(3),maxYawRate:+maxYawRate.toFixed(3)});
    r.dispose();
    const wet=rig(kind,.34);
    wet.v.body.setLinvel({x:0,y:0,z:-80/3.6},true);wet.v.wheels.forEach(w=>w.angularSpeed=80/3.6/wet.v.config.radius);v.wheels.forEach(w=>w.angularSpeed=80/3.6/v.config.radius);
    const wetStart={...wet.v.position};let wetBrakeTime=0;
    for(let i=0;i<300;i++){wet.step({...zero,brake:1});if(wet.v.speed<.15){wetBrakeTime=(i+1)*dt;break;}}
    assert(wetBrakeTime>0&&wetBrakeTime<5.5,'Wet braking must still stop the car');
    const wetDistance=Math.hypot(wet.v.position.x-wetStart.x,wet.v.position.z-wetStart.z);
    results.at(-1).wetBrake80Distance=+wetDistance.toFixed(1);
    assert(wetDistance>distance,'Wet dirt should still require more stopping distance');
    for(let i=0;i<300;i++)wet.step({...zero,throttle:1,steer:.25});
    assert(wet.v.wheels.every(w=>w.mud));
    assert(1-2*(wet.v.rotation.x**2+wet.v.rotation.z**2)>.8,'Mud turn must stay upright');
    wet.v.reset({x:0,y:8,z:0,tx:0,tz:-1,width:12,s:0,distance:0});
    for(let i=0;i<15;i++)wet.step({...zero,steer:1,handbrake:1});
    assert.equal(wet.v.wheels.filter(w=>w.contact).length,0,'Airborne tires must have no ground contact');
    for(let i=0;i<240;i++)wet.step(zero,false);
    assert(wet.v.wheels.filter(w=>w.contact).length>=3,'Suspension must settle after landing');
    wet.dispose();
  }
  assert(results[0].zeroTo100<results[1].zeroTo100,'Subaru must be quicker through 100 km/h; the initial launch can be traction limited');
  assert(results[0].brake80Distance<results[1].brake80Distance,'Pickup should take longer to stop from the same speed');

  // Regression for the user's rail-like handling: velocity must remain
  // independent of chassis heading, with physical slip and driver recovery.
  const rallyDynamics=[];
  const beta=v=>Math.atan2(-v.velocity.x*v.forward.z+v.velocity.z*v.forward.x,v.forwardSpeed);
  function atSpeed(r,kmh){
    const {v}=r;v.reset({x:0,y:0,z:0,tx:0,tz:-1,width:12,s:0,distance:0});
    for(let i=0;i<120;i++)r.step(zero,false);
    const staticFront=(v.wheels[0].force+v.wheels[1].force)/v.wheels.reduce((sum,w)=>sum+w.force,0);
    v.body.setLinvel({x:0,y:0,z:-kmh/3.6},true);v.wheels.forEach(w=>w.angularSpeed=kmh/3.6/v.config.radius);
    return staticFront;
  }
  for(const kind of ['suv','truck']){
    const r=rig(kind),{v}=r;atSpeed(r,80);
    v.body.setLinvel({x:2,y:0,z:-80/3.6},true);r.step(zero);
    assert(v.velocity.x>1.75,'Tires must build lateral force through slip instead of snapping velocity onto heading');
    atSpeed(r,80);
    for(let i=0;i<75;i++)r.step({...zero,steer:.4,throttle:.7});
    const entry=Math.abs(beta(v));
    assert(entry>6*Math.PI/180&&entry<35*Math.PI/180,kind+' should produce a progressive, controllable dirt slide');
    let recoveredAt=0;
    for(let i=0;i<240;i++){
      const slip=beta(v),yaw=v.body.angvel().y;
      r.step({...zero,steer:-course.clamp(slip*2+yaw*.35,-1,1),throttle:.15});
      if(Math.abs(beta(v))<5*Math.PI/180&&Math.abs(yaw)<.25&&recoveredAt===0)recoveredAt=(i+1)*dt;
    }
    assert(recoveredAt>0&&recoveredAt<3,'Countersteering and lifting must recover the slide');
    assert(Math.abs(beta(v))<3*Math.PI/180&&v.speed>12,'Slide recovery must retain forward momentum');
    const handbrakeTurns=[];
    for(const handbrake of [0,1]){
      atSpeed(r,70);let maxBeta=0,maxYaw=0;
      for(let i=0;i<75;i++){
        r.step({...zero,steer:.35,handbrake:i<36?handbrake:0});
        maxBeta=Math.max(maxBeta,Math.abs(beta(v)));maxYaw=Math.max(maxYaw,Math.abs(v.body.angvel().y));
        if(handbrake&&i===30)assert(Math.abs(v.wheels[2].angularSpeed)<1&&Math.abs(v.wheels[3].angularSpeed)<1,'Handbrake must lock both rear tires');
      }
      handbrakeTurns.push({sideslip:maxBeta*180/Math.PI,yaw:maxYaw});
    }
    assert(handbrakeTurns[1].sideslip>handbrakeTurns[0].sideslip*1.1,'Handbrake must increase rear slide by at least ten percent');
    assert(handbrakeTurns[1].yaw>handbrakeTurns[0].yaw*1.1,'Rear wheel locking must change physical yaw response');
    const staticFront=atSpeed(r,70);
    for(let i=0;i<30;i++)r.step({...zero,brake:.55,steer:.15});
    const front=(v.wheels[0].force+v.wheels[1].force)/v.wheels.reduce((sum,w)=>sum+w.force,0);
    assert(front>staticFront+.1,'Braking must transfer tire load to the front axle');
    rallyDynamics.push({kind,cornerEntrySideslipDegrees:+(entry*180/Math.PI).toFixed(1),countersteerRecoverySeconds:+recoveredAt.toFixed(2),coastTurnSideslipDegrees:+handbrakeTurns[0].sideslip.toFixed(1),handbrakeSideslipDegrees:+handbrakeTurns[1].sideslip.toFixed(1),brakingFrontLoadPercent:+(front*100).toFixed(1)});
    r.dispose();
  }

  // Complete a lap on the exact production terrain and road collision meshes.
  course.surfaceAt=productionSurface;
  const terrain=course.makeTerrainData(),road=course.makeRoadData();
  const laps=[];
  for(const kind of ['suv','truck']){
    const world=new RAPIER.World({x:0,y:-9.81,z:0});world.timestep=dt;world.integrationParameters.numSolverIterations=8;
    for(const mesh of [terrain,road])world.createCollider(RAPIER.ColliderDesc.trimesh(mesh.vertices,mesh.indices).setFriction(.9));
    const v=new RallyVehicle(world,kind,process.env.RALLY_HANDLING_MODE==='baseline'?'baseline':'refined');
    for(let i=0;i<120;i++)advanceVehicle(v,world,zero,dt,false);
    let lastS=0,progress=0,maxOffset=0,gate=1;
    for(let tick=0;tick<60*320&&gate<=course.GATE_COUNT;tick++){
      const n=course.nearestRoad(v.position.x,v.position.z),s=n.point.s;
      let change=s-lastS;if(change<-.5)change++;if(change>.5)change--;
      progress+=change;lastS=s;maxOffset=Math.max(maxOffset,n.distance);
      const c=v.config,look=4+v.speed*.65,target=course.courseAt(s+look/course.COURSE_LENGTH);
      const dx=target.x-v.position.x,dz=target.z-v.position.z;
      const left=v.forward.z*dx-v.forward.x*dz;
      const demand=Math.atan((c.back-c.front)*2*left/Math.max(1,dx*dx+dz*dz));
      const surf=course.surfaceAt(v.position.x,v.position.z);
      const limit=v.driveModel.steeringLimit(v.speed,c.maxSpeed,c.steering);
      const steering=Math.sign(demand)*Math.min(1,Math.abs(demand/limit))**(1/1.15);
      let desired=c.maxSpeed*.72;
      for(let distance=4;distance<32;distance+=4){
        const a=course.courseAt(s+distance/course.COURSE_LENGTH),b=course.courseAt(s+(distance+4)/course.COURSE_LENGTH);
        const curve=Math.abs(Math.atan2(a.tx*b.tz-a.tz*b.tx,a.tx*b.tx+a.tz*b.tz))/4;
        desired=Math.min(desired,Math.sqrt(c.corneringAcceleration*.48/Math.max(.001,curve)));
      }
      desired=Math.max(3.2,desired);
      const input={...zero,steer:steering,throttle:v.speed<desired?.65:0,brake:course.clamp((v.speed-desired)*.45,0,1)};
      advanceVehicle(v,world,input,dt,true);
      const next=course.courseAt(gate/course.GATE_COUNT);
      if(Math.hypot(next.x-v.position.x,next.z-v.position.z)<8.6&&v.velocity.x*next.tx+v.velocity.z*next.tz>0&&progress>(gate-.6)/course.GATE_COUNT)gate++;
      assert(Number.isFinite(v.position.y),'Lap simulation must remain finite');
      assert(1-2*(v.rotation.x**2+v.rotation.z**2)>.4,kind+' must remain upright on the circuit');
      if(gate>course.GATE_COUNT)laps.push({kind,seconds:+((tick+1)*dt).toFixed(1),checkpoints:gate-1,maxCenterOffset:+maxOffset.toFixed(2)});
    }
    assert.equal(gate,course.GATE_COUNT+1,kind+' must pass every checkpoint');
    assert(maxOffset<8,kind+' must stay within the road and immediate shoulder');
    world.free();
  }

  // Camera response at multiple render rates, reversing, obstacles and vehicle swaps.
  const frame={position:new THREE.Vector3(0,1,0),rotation:new THREE.Quaternion(),velocity:{x:0,y:0,z:-20},speed:20,forwardSpeed:20,steer:.1,kind:'suv',mode:'chase',portrait:false,dt};
  const standard=new RallyDrivingCamera();
  const slow=standard.update({...frame,speed:0,force:true},undefined,()=>0);
  const slowEye=slow.position.clone(),slowTarget=slow.target.clone();
  const fast=standard.update({...frame,speed:30,steer:-.5,velocity:{x:12,y:0,z:-24},force:true},undefined,()=>0);
  assert(slowEye.distanceTo(fast.position)<.001,'Standard camera must keep fixed framing despite speed and steering');
  assert(slowTarget.distanceTo(fast.target)<.001,'Standard camera must not swing its target into corners');
  assert.equal(slow.fov,fast.fov,'Standard camera must not zoom with speed');
  const dynamic=standard.update({...frame,speed:30,dynamic:true,force:true},undefined,()=>0);
  assert(dynamic.fov>fast.fov&&dynamic.position.z>slowEye.z,'Optional dynamic camera should retain speed effects');
  for(const kind of ['suv','truck'])for(const dynamicMode of [false,true]){
    const high=standard.update({...frame,kind,dynamic:dynamicMode,force:true},undefined,()=>0).position.clone();
    const low=standard.update({...frame,kind,dynamic:dynamicMode,lowAngle:true,force:true},undefined,()=>0).position.clone();
    assert(high.y-low.y>.7,'Low angle must lower both vehicles in either chase style');
  }
  const cameraResults=[];
  for(const hz of [30,60,120]){
    const camera=new RallyDrivingCamera();camera.update({...frame,dt:1/hz},undefined,()=>0);
    let pose;
    for(let i=0;i<hz;i++)pose=camera.update({...frame,position:new THREE.Vector3(0,1,-20*(i+1)/hz),dt:1/hz},undefined,()=>0);
    assert(pose.position.z-frame.position.z<0,'Chase camera should follow forward motion');
    cameraResults.push({hz,z:pose.position.z});
  }
  assert(Math.max(...cameraResults.map(p=>p.z))-Math.min(...cameraResults.map(p=>p.z))<.4,'Render rate must not materially change camera tracking');
  const camera=new RallyDrivingCamera();
  const chase=camera.update(frame,undefined,()=>0),chaseDistance=chase.position.length();
  const pickup=camera.update({...frame,kind:'truck',force:true},undefined,()=>0);
  assert(pickup.position.length()>chaseDistance+1,'Pickup should get wider framing');
  const reverse=camera.update({...frame,velocity:{x:0,y:0,z:4},speed:4,forwardSpeed:-4,force:true},undefined,()=>0);
  assert(reverse.position.z>0,'Reverse must keep camera behind the vehicle');
  const hood=camera.update({...frame,kind:'truck',mode:'hood',force:true},undefined,()=>0);
  assert.equal(hood.position.z,-1.8,'Truck hood view must use its own eye position');
  const rotated=new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0,1,0),Math.PI-.02);
  camera.update({...frame,rotation:rotated,dynamic:true,force:true},undefined,()=>0);
  const oldHeading=camera.heading;
  camera.update({...frame,dynamic:true,rotation:new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0,1,0),-Math.PI+.02)},undefined,()=>0);
  assert(Math.abs(camera.heading-oldHeading)<.02,'Heading wrap must take the short path');
  const bumpCamera=new RallyDrivingCamera();bumpCamera.update({...frame,speed:0,forwardSpeed:0},undefined,()=>0);
  let minHeight=Infinity,maxHeight=-Infinity;
  for(let i=0;i<180;i++){
    const pose=bumpCamera.update({...frame,position:new THREE.Vector3(0,1+Math.sin(i*Math.PI/3)*.2,0),speed:0,forwardSpeed:0},undefined,()=>0);
    minHeight=Math.min(minHeight,pose.position.y);maxHeight=Math.max(maxHeight,pose.position.y);
  }
  assert(maxHeight-minHeight<.12,'Suspension oscillations should not shake the chase camera');
  const hillside=camera.update({...frame,force:true},undefined,()=>5);
  assert(hillside.position.y>=6.2,'Chase eye must clear raised terrain');
  const obstacleWorld=new RAPIER.World({x:0,y:0,z:0});
  obstacleWorld.createCollider(RAPIER.ColliderDesc.cuboid(4,4,.2).setTranslation(0,1,4));obstacleWorld.step();
  const safe=obstructedEye(obstacleWorld,new THREE.Vector3(0,1,0),new THREE.Vector3(0,2,8));
  assert(safe.z<3.8&&safe.z>2,'Chase camera must stop before the wall');
  obstacleWorld.free();
  console.log(JSON.stringify({handling:results,rallyDynamics,circuitLaps:laps,camera:'standard and dynamic framing, low angle, frame rates, reverse, hood, yaw wrap and obstacles passed'},null,2));

} finally {
  rmSync(temp,{recursive:true,force:true});
}
