import assert from 'node:assert/strict';
import {readFileSync,writeFileSync,mkdirSync,mkdtempSync,rmSync} from 'node:fs';
import {createRequire} from 'node:module';
import {dirname,join} from 'node:path';
import {fileURLToPath} from 'node:url';
import ts from 'typescript';

// Test the shipped physics, including numerical stability and observable AWD
// behavior. These are internal acceptance checks, not DiRT Rally benchmarks.
const root=dirname(dirname(fileURLToPath(import.meta.url))),cache=join(root,'node_modules/.cache');
mkdirSync(cache,{recursive:true});const temp=mkdtempSync(join(cache,'rally-dynamics-'));
const require=createRequire(import.meta.url),RAPIER=require('@dimforge/rapier3d-compat');
const zero={steer:0,throttle:0,brake:0,handbrake:0},dt=1/60;
try{
  for(const name of ['course','vehicle-config','vehicle','simulation','drive-model','wheel-model','vendor/ecctrl/CurveLUT','vendor/stunt-rally/gravel','vendor/stunt-rally/pacejka','vendor/stunt-rally/abs','vendor/stunt-rally/engine-friction','vendor/stunt-rally/differential']){
    const {outputText}=ts.transpileModule(readFileSync(join(root,'lib/rally',name+'.ts'),'utf8'),{compilerOptions:{module:ts.ModuleKind.CommonJS,target:ts.ScriptTarget.ES2022,esModuleInterop:true}});
    mkdirSync(dirname(join(temp,name+'.cjs')),{recursive:true});
    writeFileSync(join(temp,name+'.cjs'),outputText.replace(/require\("\.\/(.*?)"\)/g,'require("./$1.cjs")'));
  }
  const course=require(join(temp,'course.cjs')),{VEHICLES}=require(join(temp,'vehicle-config.cjs'));
  const {RallyVehicle}=require(join(temp,'vehicle.cjs')),{advanceVehicle}=require(join(temp,'simulation.cjs'));
  await RAPIER.init();
  const beta=v=>Math.atan2(-v.velocity.x*v.forward.z+v.velocity.z*v.forward.x,v.forwardSpeed);
  function rig(kind){
    course.surfaceAt=()=>({grip:.59,rollingResistance:.025,rollingDrag:4,mud:false,label:'LOOSE DIRT'});
    const world=new RAPIER.World({x:0,y:-9.81,z:0});world.integrationParameters.numSolverIterations=8;
    world.createCollider(RAPIER.ColliderDesc.cuboid(2000,.5,2000).setTranslation(0,-.5,0));
    const v=new RallyVehicle(world,kind,process.env.RALLY_HANDLING_MODE==='baseline'?'baseline':'refined'),step=(input=zero,frameStep=dt)=>advanceVehicle(v,world,input,frameStep,true);
    v.reset({x:0,y:0,z:0,tx:0,tz:-1,width:12,s:0,distance:0});
    for(let i=0;i<120;i++)advanceVehicle(v,world,zero,dt,false);
    const atSpeed=kmh=>{
      v.body.setLinvel({x:0,y:0,z:-kmh/3.6},true);v.wheels.forEach(w=>w.angularSpeed=kmh/3.6/v.config.radius);
      const c=v.config.powertrain,wheelRPM=kmh/3.6/v.config.radius*30/Math.PI;
      let gear=0;for(let i=0;i<c.gearRatios.length;i++)if(wheelRPM*c.gearRatios[i]*c.finalDriveRatio>c.shiftDownRPM+1200)gear=i;
      v.driveModel.gearIndex=gear;v.driveModel.driveRatio=c.gearRatios[gear]*c.finalDriveRatio;v.syncEngine();
    };
    return {world,v,step,atSpeed};
  }
  const launches=[];
  const c=VEHICLES.suv,savedCenter={...c.differential.center};
  for(const enabled of [false,true]){
    c.differential.center={...savedCenter,antiSlip:enabled?savedCenter.antiSlip:0};
    const {world,v,step}=rig('suv');let maxAxleDifference=0,previousGear=1,downshifts=0,zeroTo60=0;
    for(let i=0;i<240;i++){
      step({...zero,throttle:1});
      if(v.speed>2&&v.speed<6){const w=v.wheels;maxAxleDifference=Math.max(maxAxleDifference,Math.abs((w[0].angularSpeed+w[1].angularSpeed-w[2].angularSpeed-w[3].angularSpeed)/2));}
      if(v.gear<previousGear)downshifts++;previousGear=v.gear;
      if(!zeroTo60&&v.speed*3.6>=60)zeroTo60=(i+1)*dt;
      assert(Math.abs(v.wheels.reduce((sum,w)=>sum+w.driveTorque,0)-v.engineWheelTorque)<.2,'Differentials must conserve total input torque');
      assert(v.wheels.every(w=>w.inertia===v.config.wheelInertia),'Contact load must not change wheel inertia');
    }
    launches.push({centerDifferential:enabled?'limited-slip':'open',zeroTo60:+zeroTo60.toFixed(3),maxAxleDifference:+maxAxleDifference.toFixed(2),downshifts,speedAfterFourSeconds:+(v.speed*3.6).toFixed(1)});
    world.free();
  }
  c.differential.center=savedCenter;
  assert(launches[1].maxAxleDifference<launches[0].maxAxleDifference*.35,'The center differential must prevent one axle running away during launch');
  assert(launches[1].zeroTo60>0&&launches[1].zeroTo60<launches[0].zeroTo60,'Transferring torque to the loaded axle must improve the launch');
  assert.equal(launches[1].downshifts,0,'The tuned launch must not hunt between gears');

  const throttleControl=[];
  for(const kind of ['suv','truck']){
    const profiles=[];
    for(const throttle of [.15,.75]){
      const {world,v,step,atSpeed}=rig(kind);atSpeed(80);
      for(let i=0;i<75;i++)step({...zero,steer:.35,throttle});
      profiles.push({throttle,sideslipDegrees:+(beta(v)*180/Math.PI).toFixed(2),speed:+(v.speed*3.6).toFixed(1),x:v.position.x,z:v.position.z});
      world.free();
    }
    const [lift,power]=profiles;
    assert(Math.abs(power.sideslipDegrees-lift.sideslipDegrees)>.5,'Throttle must affect physical cornering balance');
    assert(power.speed>lift.speed+4,'Throttle must retain useful corner-exit acceleration');
    throttleControl.push({kind,profiles});

    const brakeRig=rig(kind);brakeRig.atSpeed(70);
    let coupledDuringBraking=false;
    for(let i=0;i<24;i++){
      brakeRig.step({...zero,steer:.2,brake:.4});
      if(Math.abs(brakeRig.v.diffTransfer.rear)>.1)coupledDuringBraking=true;
      assert(brakeRig.v.driveModel.demand===0,'Foot braking must inhibit propulsive engine torque');
    }
    assert(coupledDuringBraking,'The limited-slip differential must continue to work during foot braking');
    const inertia=brakeRig.v.wheels.map(w=>w.inertia);
    brakeRig.v.reset({x:0,y:8,z:0,tx:0,tz:-1,width:12,s:0,distance:0});
    for(let i=0;i<12;i++)brakeRig.step({...zero,throttle:.5});
    assert(brakeRig.v.wheels.every((w,i)=>!w.contact&&w.inertia===inertia[i]),'Airborne wheels retain their physical inertia');
    brakeRig.world.free();
  }

  const frameRates=[];
  for(const hz of [30,60,120]){
    const {world,v,step}=rig('suv');
    for(let i=0;i<hz*6;i++)step({...zero,throttle:.7,steer:i>=hz*3?.2:0},1/hz);
    frameRates.push({hz,x:v.position.x,z:v.position.z,speed:v.speed*3.6});
    world.free();
  }
  const reference=frameRates[1];
  for(const result of frameRates)assert(Math.hypot(result.x-reference.x,result.z-reference.z)<.01&&Math.abs(result.speed-reference.speed)<.05,'Render cadence must not change the physics outcome');
  console.log(JSON.stringify({launches,throttleControl,brakingCoupling:'passed',airborneInertia:'passed',frameRates},null,2));
}finally{rmSync(temp,{recursive:true,force:true});}
