import assert from 'node:assert/strict';
import {readFileSync,writeFileSync,mkdirSync,mkdtempSync,rmSync} from 'node:fs';
import {createRequire} from 'node:module';
import {dirname,join} from 'node:path';
import {fileURLToPath} from 'node:url';
import ts from 'typescript';

// Isolate the earlier dry-grip/engine-braking setup changes with the same driver
// and the current drivetrain. This is not a replay of an older physics build.
const root=dirname(dirname(fileURLToPath(import.meta.url))),cache=join(root,'node_modules/.cache');
mkdirSync(cache,{recursive:true});const temp=mkdtempSync(join(cache,'rally-grip-'));
const require=createRequire(import.meta.url),RAPIER=require('@dimforge/rapier3d-compat');
const dt=1/60,zero={steer:0,throttle:0,brake:0,handbrake:0};
try{
  for(const name of ['course','vehicle-config','vehicle','steering','simulation','drive-model','wheel-model','vendor/ecctrl/CurveLUT','vendor/stunt-rally/gravel','vendor/stunt-rally/pacejka','vendor/stunt-rally/abs','vendor/stunt-rally/engine-friction','vendor/stunt-rally/differential']){
    const {outputText}=ts.transpileModule(readFileSync(join(root,'lib/rally',name+'.ts'),'utf8'),{compilerOptions:{module:ts.ModuleKind.CommonJS,target:ts.ScriptTarget.ES2022,esModuleInterop:true}});
    mkdirSync(dirname(join(temp,name+'.cjs')),{recursive:true});
    writeFileSync(join(temp,name+'.cjs'),outputText.replace(/require\("\.\/(.*?)"\)/g,'require("./$1.cjs")'));
  }
  const course=require(join(temp,'course.cjs')),{VEHICLES}=require(join(temp,'vehicle-config.cjs')),{RallyVehicle}=require(join(temp,'vehicle.cjs'));
  const {advanceVehicle}=require(join(temp,'simulation.cjs'));
  const productionSurface=course.surfaceAt;
  await RAPIER.init();
  const beta=v=>Math.atan2(-v.velocity.x*v.forward.z+v.velocity.z*v.forward.x,v.forwardSpeed);
  function rig(kind,grip,rollingResistance){
    course.surfaceAt=()=>({grip,rollingResistance,rollingDrag:4,mud:grip<.4,label:grip<.4?'WET MUD':'LOOSE DIRT'});
    const world=new RAPIER.World({x:0,y:-9.81,z:0});world.timestep=dt;world.integrationParameters.numSolverIterations=8;
    world.createCollider(RAPIER.ColliderDesc.cuboid(2000,.5,2000).setTranslation(0,-.5,0));
    const v=new RallyVehicle(world,kind),step=(input=zero,enabled=true)=>{advanceVehicle(v,world,input,dt,enabled);};
    function atSpeed(kph){
      v.reset({x:0,y:0,z:0,tx:0,tz:-1,width:12,s:0,distance:0});for(let i=0;i<120;i++)step(zero,false);
      v.body.setLinvel({x:0,y:0,z:-kph/3.6},true);v.wheels.forEach(w=>w.angularSpeed=kph/3.6/v.config.radius);
      // Start in a plausible road gear, rather than injecting 80 km/h into first.
      const wheelRPM=kph/3.6/v.config.radius*30/Math.PI,c=v.config.powertrain;
      let gear=0;for(let i=0;i<c.gearRatios.length;i++)if(wheelRPM*c.gearRatios[i]*c.finalDriveRatio>c.shiftDownRPM+1200)gear=i;
      v.driveModel.gearIndex=gear;v.driveModel.driveRatio=c.gearRatios[gear]*c.finalDriveRatio;v.syncEngine();
    }
    return {world,v,step,atSpeed};
  }
  const comparison=[];
  for(const kind of ['suv','truck']){
    const config=VEHICLES[kind],tunedResponse={...config.tireLateralResponse},tunedBraking=config.engineBraking;
    const profiles=[];
    for(const profile of ['previous','revised']){
      config.tireLateralResponse=profile==='previous'?{front:1,rear:1}:tunedResponse;
      config.engineBraking=profile==='previous'?0:tunedBraking;
      const r=rig(kind,profile==='previous'?.55:.59,profile==='previous'?.016:.025),{v,step,atSpeed}=r;
      atSpeed(80);for(let i=0;i<75;i++)step({...zero,steer:.4,throttle:.7});
      const entry=Math.abs(beta(v));let recovery=0;
      for(let i=0;i<180;i++){
        const yaw=v.body.angvel().y;step({...zero,steer:-course.clamp(beta(v)*2+yaw*.35,-1,1),throttle:.15});
        if(Math.abs(beta(v))<5*Math.PI/180&&Math.abs(yaw)<.25&&recovery===0)recovery=(i+1)*dt;
      }
      atSpeed(80);for(let i=0;i<240;i++)step(zero);const liftSpeed=v.speed*3.6;
      atSpeed(60);for(let i=0;i<90;i++)step({...zero,steer:.15,throttle:.25});const gentleSlip=Math.abs(beta(v))*180/Math.PI;
      profiles.push({profile,cornerEntrySideslipDegrees:+(entry*180/Math.PI).toFixed(2),countersteerRecoverySeconds:+recovery.toFixed(2),speedAfterFourSecondLift:+liftSpeed.toFixed(2),gentleCornerSideslipDegrees:+gentleSlip.toFixed(2)});
      r.world.free();
    }
    config.tireLateralResponse=tunedResponse;config.engineBraking=tunedBraking;
    const [before,after]=profiles;
    assert(after.countersteerRecoverySeconds>0&&after.countersteerRecoverySeconds<before.countersteerRecoverySeconds,'The revised tires must catch the same slide sooner');
    assert(after.cornerEntrySideslipDegrees<before.cornerEntrySideslipDegrees,'Moderate cornering must produce less unintended sideslip');
    assert(after.gentleCornerSideslipDegrees<before.gentleCornerSideslipDegrees,'Gentle cornering must hold a tighter line');
    assert(after.speedAfterFourSecondLift<before.speedAfterFourSecondLift-3,'Lifting must add noticeable engine and gravel resistance');
    assert(after.cornerEntrySideslipDegrees>5,'The revised car must retain a useful rally slide');
    comparison.push({kind,profiles});
  }
  course.surfaceAt=productionSurface;
  const dry=productionSurface(course.courseAt(.1).x,course.courseAt(.1).z),wet=productionSurface(course.courseAt(.36).x,course.courseAt(.36).z);
  assert(wet.grip<dry.grip&&wet.rollingResistance>dry.rollingResistance,'Mud must retain lower grip and greater resistance than dry gravel');
  console.log(JSON.stringify({comparison,surfaceContrast:'passed'},null,2));
}finally{rmSync(temp,{recursive:true,force:true});}
