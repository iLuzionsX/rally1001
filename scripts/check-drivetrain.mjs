import assert from 'node:assert/strict';
import {readFileSync,writeFileSync,mkdirSync,mkdtempSync,rmSync} from 'node:fs';
import {createRequire} from 'node:module';
import {dirname,join} from 'node:path';
import {fileURLToPath} from 'node:url';
import ts from 'typescript';
const root=dirname(dirname(fileURLToPath(import.meta.url))),cache=join(root,'node_modules/.cache');
mkdirSync(cache,{recursive:true});const temp=mkdtempSync(join(cache,'drivetrain-'));
const require=createRequire(import.meta.url),RAPIER=require('@dimforge/rapier3d-compat');
const zero={steer:0,throttle:0,brake:0,handbrake:0},dt=1/240,round=n=>+n.toFixed(4);
try{
 for(const name of ['course','vehicle-config','vehicle','simulation','drive-model','wheel-model','vendor/ecctrl/CurveLUT','vendor/stunt-rally/gravel','vendor/stunt-rally/pacejka','vendor/stunt-rally/abs','vendor/stunt-rally/engine-friction','vendor/stunt-rally/differential']){
  const {outputText}=ts.transpileModule(readFileSync(join(root,'lib/rally',name+'.ts'),'utf8'),{compilerOptions:{module:ts.ModuleKind.CommonJS,target:ts.ScriptTarget.ES2022,esModuleInterop:true}});
  mkdirSync(dirname(join(temp,name+'.cjs')),{recursive:true});writeFileSync(join(temp,name+'.cjs'),outputText.replace(/require\("\.\/(.*?)"\)/g,'require("./$1.cjs")'));
 }
 const course=require(join(temp,'course.cjs')),{RallyVehicle,VEHICLES}=require(join(temp,'vehicle.cjs')),{advanceVehicle}=require(join(temp,'simulation.cjs'));
 const {EcctrlDriveModel}=require(join(temp,'drive-model.cjs'));
 await RAPIER.init();
 const beta=v=>Math.atan2(-v.velocity.x*v.forward.z+v.velocity.z*v.forward.x,v.forwardSpeed);
 const transmissions=[];
 for(const kind of ['suv','truck'])for(const advanced of [false,true]){
  const c=VEHICLES[kind],m=new EcctrlDriveModel(c.powertrain,c.radius);
  const angular=(c.powertrain.shiftUpRPM+150)/m.driveRatio*Math.PI/30;
  const wheels=Array.from({length:4},()=>({angularSpeed:angular,longSlip:0,weight:.25}));
  const ctx={roadSpeed:angular*c.radius,grounded:4,handbrake:0};
  let previousGear=m.gear,ratioChangeClutch=null,maxClutchStep=0,previousClutch=1;
  for(let i=0;i<120;i++){
   m.updateTransmission(wheels,dt,false,advanced?ctx:undefined);
   maxClutchStep=Math.max(maxClutchStep,Math.abs(m.shiftClutch-previousClutch));previousClutch=m.shiftClutch;
   if(m.gear!==previousGear){ratioChangeClutch=m.shiftClutch;previousGear=m.gear;}
  }
  assert(ratioChangeClutch!==null,'Sustained road RPM must produce an upshift');
  if(advanced){assert(ratioChangeClutch===0,'Ratio must change with clutch torque disconnected');assert(maxClutchStep<.23,'Shift torque envelope must ramp');assert(m.shiftRemaining===0&&m.shiftClutch===1,'Shift must finish and reconnect');}
  m.reset();const spinWheels=wheels.map(w=>({...w,longSlip:.5}));
  for(let i=0;i<240;i++)m.updateTransmission(spinWheels,dt,false,advanced?{...ctx,roadSpeed:ctx.roadSpeed*.65}:undefined);
  const spinGear=m.gear;if(advanced)assert(spinGear===1,'Moderate wheelspin must not trigger a road-speed-invalid upshift');
  m.reset();m.gearIndex=2;m.driveRatio=c.powertrain.gearRatios[2]*c.powertrain.finalDriveRatio;
  for(let i=0;i<240;i++)m.updateTransmission(wheels.map(w=>({...w,angularSpeed:0})),dt,false,advanced?{roadSpeed:20,grounded:0,handbrake:0}:undefined);
  if(advanced)assert(m.gear===3,'Airborne wheel lock must not trigger downshifts');
  m.reset();for(let i=0;i<240;i++)m.updateTransmission(wheels,dt,false,advanced?{...ctx,handbrake:1}:undefined);
  if(advanced)assert(m.gear===1,'Handbrake must inhibit automatic shift initiation');
  m.reset();let shifts=0,last=1;
  for(let i=0;i<240;i++){
   const rpm=c.powertrain.shiftUpRPM+(Math.floor(i/6)%2===0?120:-120),omega=rpm/m.driveRatio*Math.PI/30;
   m.updateTransmission(wheels.map(w=>({...w,angularSpeed:omega})),dt,false,advanced?{roadSpeed:omega*c.radius,grounded:4,handbrake:0}:undefined);
   if(m.gear!==last){shifts++;last=m.gear;}
  }
  if(advanced)assert(shifts===0,'Brief RPM threshold crossings must not cause hunting');
  transmissions.push({kind,mode:advanced?'rally':'rally-surface',ratioChangeClutch,maxClutchStep:round(maxClutchStep),wheelspinGear:spinGear,thresholdShifts:shifts});
 }
 function rig(kind,mode,surface='gravel',grade=0,kmh=55,startZ=0,gearIndex=1){
  const tire=course.SURFACE_TIRES[surface];course.surfaceAt=()=>({...tire,tire,mud:surface==='mud'});
  const world=new RAPIER.World({x:0,y:-9.81,z:0});world.integrationParameters.numSolverIterations=8;
  const vertices=new Float32Array([-1000,-1000*grade,-1000,1000,-1000*grade,-1000,-1000,1000*grade,1000,1000,1000*grade,1000]);
  world.createCollider(RAPIER.ColliderDesc.trimesh(vertices,new Uint32Array([0,2,1,1,2,3])));
  const v=new RallyVehicle(world,kind,mode);
  v.reset({x:0,y:grade*startZ,z:startZ,tx:0,tz:-1,width:12,s:0,distance:0});
  for(let i=0;i<480;i++)advanceVehicle(v,world,zero,dt,false);
  v.brake=0;v.throttle=0;v.body.setLinvel({x:0,y:-grade*kmh/3.6,z:-kmh/3.6},true);v.wheels.forEach(w=>w.angularSpeed=kmh/3.6/v.config.radius);
  v.driveModel.gearIndex=gearIndex;v.driveModel.driveRatio=v.config.powertrain.gearRatios[gearIndex]*v.config.powertrain.finalDriveRatio;
  function step(input=zero,frame=dt){
   advanceVehicle(v,world,input,frame,true);
   assert(Number.isFinite(v.speed)&&v.wheels.every(w=>Number.isFinite(w.angularSpeed)&&Number.isFinite(w.longForce)),'Finite drivetrain and tire state');
   assert(1-2*(v.rotation.x**2+v.rotation.z**2)>.8,'Vehicle must remain upright');
   if(v.handbrake<.02)assert(Math.abs(v.wheels.reduce((sum,w)=>sum+w.driveTorque,0)-v.engineWheelTorque)<.2,'Conserve engine torque through differentials');
   if(input.brake>.01)assert(v.engineWheelTorque<=0,'Foot brake must override propulsion');
  }
  return {world,v,step};
 }
 const liftOff=[],handbrake=[],launches=[],hairpins=[],cadence=[];
 for(const kind of ['suv','truck'])for(const mode of ['rally-surface','rally']){
  for(const grade of [0,.08])for(const gearIndex of [1,2]){
   const {world,v,step}=rig(kind,mode,'gravel',grade,55,0,gearIndex);v.driveModel.shiftCooldownTimer=10;v.throttle=.6;
   let maxDelta=0,previous=null,peakFront=0,minTorque=0;const begin={...v.position};
   for(let i=0;i<240;i++){
    step();if(previous!==null)maxDelta=Math.max(maxDelta,Math.abs(v.engineWheelTorque-previous));previous=v.engineWheelTorque;minTorque=Math.min(minTorque,v.engineWheelTorque);
    peakFront=Math.max(peakFront,(v.wheels[0].force+v.wheels[1].force)/v.wheels.reduce((sum,w)=>sum+w.force,0));
    const shaft=v.wheels.reduce((sum,w,i)=>sum+w.angularSpeed*(i<2?v.config.frontDrive/2:(1-v.config.frontDrive)/2),0);
    if(mode==='rally')assert(v.engineDragTorque*shaft<=1e-6,'Engine drag must dissipate energy');
   }
   liftOff.push({kind,mode,grade,gear:gearIndex+1,exitKmh:round(v.speed*3.6),peakFront:round(peakFront),minTorque:round(minTorque),maxTorqueStep:round(maxDelta),distance:round(Math.hypot(v.position.x-begin.x,v.position.z-begin.z))});world.free();
  }
  for(const surface of ['gravel','mud']){
   const {world,v,step}=rig(kind,mode,surface,0,45);let peakSlip=0,maxTorqueStep=0,previousTorque=0,reconnected=0,firstTorque=0,firstReleaseClutch=0,gearDuringHold=0,releaseTime=.8;
   for(let i=0;i<720;i++){
    const t=i*dt,hold=t>=.4&&t<releaseTime;
    step({...zero,steer:t<1?.35:-course.clamp(beta(v)*2+v.body.angvel().y*.35,-1,1),handbrake:hold?1:0,throttle:t>=releaseTime?.65:0});
    peakSlip=Math.max(peakSlip,Math.abs(beta(v))*180/Math.PI);
    if(t>=releaseTime){
     maxTorqueStep=Math.max(maxTorqueStep,Math.abs(v.engineWheelTorque-previousTorque));
     if(!firstTorque&&v.engineWheelTorque>5){firstTorque=t-releaseTime;firstReleaseClutch=v.handbrakeClutch;}
     if(!reconnected&&v.handbrakeClutch===1&&v.handbrake<.02)reconnected=t-releaseTime;
    }
    if(i===110)gearDuringHold=v.gear;
    if(hold&&i>110&&mode==='rally')assert(v.gear===gearDuringHold,'Handbrake must retain gear');
    previousTorque=v.engineWheelTorque;
   }
   assert(Math.abs(beta(v))<.15&&v.speed>8,'Handbrake release must recover and retain drive');
   if(mode==='rally'){assert(firstReleaseClutch>0&&firstReleaseClutch<.5,'First returning torque must use partial engagement');assert(reconnected>0&&reconnected<.6,'Clutch recovery must be prompt');}
   handbrake.push({kind,mode,surface,peakSlip:round(peakSlip),maxTorqueStep:round(maxTorqueStep),firstTorque:round(firstTorque),reconnected:round(reconnected),exitKmh:round(v.speed*3.6),finalSlip:round(beta(v)*180/Math.PI)});world.free();
  }
  {
   const {world,v,step}=rig(kind,mode,'gravel',0,0,0,0);let reached100=0,lastGear=1,downshifts=0;
   for(let i=0;i<240*9;i++){step({...zero,throttle:1});if(!reached100&&v.speed*3.6>=100)reached100=(i+1)*dt;if(v.gear<lastGear)downshifts++;lastGear=v.gear;}
   assert(reached100>0&&reached100<8&&downshifts===0,'Launch retains performance without gear hunting');
   launches.push({kind,mode,zeroTo100:round(reached100),downshifts,finalGear:v.gear});world.free();
  }
  // Geometric 180-degree hairpin: 25m approach, 14m radius, 30m exit.
  // A deterministic preview driver modulates pedals and countersteers; none of
  // this controller runs in the game. Surface and grade are actual tire/mesh inputs.
  for(const surface of ['gravel','mud'])for(const grade of [0,.08])for(const useHandbrake of [false,true]){
   const R=14,length=25+Math.PI*R+30;
   const point=s=>s<25?{x:0,z:25-s}:s<25+Math.PI*R?{x:-R+R*Math.cos((s-25)/R),z:-R*Math.sin((s-25)/R)}:{x:-2*R,z:s-25-Math.PI*R};
   const path=Array.from({length:Math.ceil(length*2)+1},(_,i)=>point(Math.min(length,i*.5)));
   const {world,v,step}=rig(kind,mode,surface,grade,50,25);let index=0,finished=0,maxOffset=0,peakSlip=0,handStart=-1,shifts=0,lastGear=v.gear;
   for(let tick=0;tick<240*35;tick++){
    let best=Infinity,next=index;
    for(let j=Math.max(0,index-3);j<Math.min(path.length,index+45);j++){
     const d=Math.hypot(v.position.x-path[j].x,v.position.z-path[j].z);if(d<best){best=d;next=j;}
    }
    index=Math.max(index,next);const progress=index*.5;maxOffset=Math.max(maxOffset,best);
    const look=3+v.speed*.4,target=point(Math.min(length,progress+look)),dx=target.x-v.position.x,dz=target.z-v.position.z;
    const left=v.forward.z*dx-v.forward.x*dz,demand=Math.atan((v.config.back-v.config.front)*2*left/Math.max(1,dx*dx+dz*dz));
    const limit=v.driveModel.steeringLimit(v.speed,v.config.maxSpeed,v.config.steering);
    const pursuit=Math.sign(demand)*Math.min(1,Math.abs(demand/limit))**(1/1.15);
    const steer=course.clamp(pursuit-beta(v)*.7,-1,1);
    const desired=progress<12?11:progress<25+Math.PI*R-5?(surface==='mud'?5.4:6.5):12;
    if(useHandbrake&&handStart<0&&progress>=24)handStart=tick*dt;
    const hand=handStart>=0&&tick*dt-handStart<.2?1:0;
    const brake=course.clamp((v.speed-desired)*.4,0,.8),throttle=brake>.01?0:v.speed<desired?.6:.15;
    step({steer,throttle,brake,handbrake:hand});
    if(v.gear!==lastGear){shifts++;lastGear=v.gear;}
    if(v.speed>2)peakSlip=Math.max(peakSlip,Math.abs(beta(v))*180/Math.PI);
    if(progress>length-2&&v.forward.z>.8){finished=(tick+1)*dt;break;}
   }
   const result={kind,mode,surface,grade,useHandbrake,seconds:round(finished),maxOffset:round(maxOffset),peakSlip:round(peakSlip),exitKmh:round(v.speed*3.6),shifts};hairpins.push(result);
   if(!finished||maxOffset>=7)console.log('hairpin failure',JSON.stringify(result));
   assert(finished>0&&maxOffset<7,'Complete the 180-degree hairpin within road and shoulder');
   assert(v.speed>5&&peakSlip<65,'Hairpin must preserve controlled forward drive');world.free();
  }
 }
 for(const kind of ['suv','truck']){
  for(const grade of [0,.08]){
   const rows=liftOff.filter(r=>r.kind===kind&&r.mode==='rally'&&r.grade===grade);
   assert(Math.abs(rows[0].minTorque)>Math.abs(rows[1].minTorque)*1.15,'Lower gear must produce stronger engine braking');
  }
  for(const surface of ['gravel','mud']){
   const before=handbrake.find(r=>r.kind===kind&&r.mode==='rally-surface'&&r.surface===surface),after=handbrake.find(r=>r.kind===kind&&r.mode==='rally'&&r.surface===surface);
   assert(after.maxTorqueStep<before.maxTorqueStep*.5,'Handbrake release must reduce the drivetrain torque step');
   assert(after.exitKmh>before.exitKmh*.9,'Smoother handbrake recovery must preserve exit drive');
  }
  const runs=[];
  for(const hz of [30,60,120]){
   const {world,v,step}=rig(kind,'rally','gravel',0,40,0,0);
   for(let i=0;i<hz*5;i++){const t=i/hz;step({...zero,throttle:.7,handbrake:t>=2&&t<2.5?1:0},1/hz);}
   runs.push({hz,x:v.position.x,z:v.position.z,speed:v.speed,gear:v.gear});
   v.setHandlingMode('rally-surface');v.setHandlingMode('rally');assert(v.handbrakeClutch===1&&v.engineDragTorque===0&&v.driveModel.shiftClutch===1&&v.driveModel.pendingGear===-1,'Mode change clears drivetrain transients');world.free();
  }
  for(const r of runs)assert(Math.abs(r.z-runs[0].z)<.01&&Math.abs(r.speed-runs[0].speed)<.01&&r.gear===runs[0].gear,'Shift and handbrake response must be cadence independent');cadence.push({kind,runs});
 }
 const report={transmissions,liftOff,handbrake,launches,hairpins,cadence,limitations:'Synthetic 180-degree hairpin and graded collision-plane tests with a deterministic preview driver. No browser/GPU or hands-on DiRT comparison. Clutch is a torque envelope, not an engine-inertia simulation.'};
 writeFileSync(join(root,'reference/hairpin-drivetrain-validation.json'),JSON.stringify(report,null,2)+'\n');
 console.log(JSON.stringify({result:'passed',transmissions,handbrake,launches,hairpins:hairpins.length,cadence:cadence.length},null,2));
}finally{rmSync(temp,{recursive:true,force:true});}
