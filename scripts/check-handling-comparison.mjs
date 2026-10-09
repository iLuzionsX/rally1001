import assert from 'node:assert/strict';
import {readFileSync,writeFileSync,mkdirSync,mkdtempSync,rmSync} from 'node:fs';
import {createRequire} from 'node:module';
import {dirname,join} from 'node:path';
import {fileURLToPath} from 'node:url';
import ts from 'typescript';

// Identical scripted inputs on flat dry/wet ground, using the production solver.
// These compare regressions and response shape, not fidelity to DiRT Rally.
const root=dirname(dirname(fileURLToPath(import.meta.url))),cache=join(root,'node_modules/.cache');
mkdirSync(cache,{recursive:true});const temp=mkdtempSync(join(cache,'handling-comparison-'));
const require=createRequire(import.meta.url),RAPIER=require('@dimforge/rapier3d-compat');
const zero={steer:0,throttle:0,brake:0,handbrake:0},dt=1/240;
const round=n=>+n.toFixed(4);
try{
  for(const name of ['course','vehicle-config','vehicle','steering','simulation','drive-model','wheel-model','vendor/ecctrl/CurveLUT','vendor/stunt-rally/gravel','vendor/stunt-rally/pacejka','vendor/stunt-rally/abs','vendor/stunt-rally/engine-friction','vendor/stunt-rally/differential']){
    const {outputText}=ts.transpileModule(readFileSync(join(root,'lib/rally',name+'.ts'),'utf8'),{compilerOptions:{module:ts.ModuleKind.CommonJS,target:ts.ScriptTarget.ES2022,esModuleInterop:true}});
    mkdirSync(dirname(join(temp,name+'.cjs')),{recursive:true});
    writeFileSync(join(temp,name+'.cjs'),outputText.replace(/require\("\.\/(.*?)"\)/g,'require("./$1.cjs")'));
  }
  const course=require(join(temp,'course.cjs')),{RallyVehicle}=require(join(temp,'vehicle.cjs'));
  const {advanceVehicle}=require(join(temp,'simulation.cjs'));
  await RAPIER.init();
  const beta=v=>Math.atan2(-v.velocity.x*v.forward.z+v.velocity.z*v.forward.x,v.forwardSpeed)*180/Math.PI;
  function rig(kind,mode,wet=false){
    course.surfaceAt=()=>({type:wet?'mud':'gravel',grip:wet?.34:.59,rollingResistance:wet?.045:.025,rollingDrag:wet?45:4,mud:wet,label:wet?'WET MUD':'LOOSE DIRT'});
    const world=new RAPIER.World({x:0,y:-9.81,z:0});world.integrationParameters.numSolverIterations=8;
    world.createCollider(RAPIER.ColliderDesc.cuboid(2000,.5,2000).setTranslation(0,-.5,0));
    const v=new RallyVehicle(world,kind,mode);
    const step=(input=zero,frame=dt)=>{
      advanceVehicle(v,world,input,frame,true);
      assert(Number.isFinite(v.speed)&&Number.isFinite(v.position.y));
      assert(v.wheels.every(w=>Number.isFinite(w.angularSpeed)&&Number.isFinite(w.relaxedSlipAngle)));
      assert(1-2*(v.rotation.x**2+v.rotation.z**2)>.8,'Car must remain upright');
    };
    function atSpeed(kmh){
      v.reset({x:0,y:0,z:0,tx:0,tz:-1,width:12,s:0,distance:0});
      for(let i=0;i<480;i++)advanceVehicle(v,world,zero,dt,false);
      v.brake=0;
      v.body.setLinvel({x:0,y:0,z:-kmh/3.6},true);v.wheels.forEach(w=>{w.angularSpeed=kmh/3.6/v.config.radius;w.relaxedSlipAngle=0;});
      const c=v.config.powertrain,wheelRPM=kmh/3.6/v.config.radius*30/Math.PI;
      let gear=0;for(let i=0;i<c.gearRatios.length;i++)if(wheelRPM*c.gearRatios[i]*c.finalDriveRatio>c.shiftDownRPM+1200)gear=i;
      v.driveModel.gearIndex=gear;v.driveModel.driveRatio=c.gearRatios[gear]*c.finalDriveRatio;v.syncEngine();
    }
    return {world,v,step,atSpeed};
  }
  const comparison=[];
  for(const kind of ['suv','truck'])for(const wet of [false,true]){
    const profiles=[];
    for(const mode of ['baseline','refined']){
      const {world,v,step,atSpeed}=rig(kind,mode,wet);
      atSpeed(80);let stopTime=0;
      for(let i=0;i<240*6;i++){step({...zero,brake:1});if(v.speed<.15){stopTime=(i+1)*dt;break;}}
      assert(stopTime>0,'Full braking must stop the vehicle');
      const stopDistance=Math.hypot(v.position.x,v.position.z);
      atSpeed(70);let pressureJump=0,previousPressure=0,maxTrailSlip=0,peakFrontShare=0,maxYaw=0;
      for(let i=0;i<240*2;i++){
        const t=i*dt,brake=t<.35?.45:Math.max(0,.45*(1-(t-.35)/.65));
        step({...zero,steer:.22,brake,throttle:t>1.1?.35:0});
        pressureJump=Math.max(pressureJump,Math.abs(v.brake-previousPressure));previousPressure=v.brake;
        maxTrailSlip=Math.max(maxTrailSlip,Math.abs(beta(v)));maxYaw=Math.max(maxYaw,Math.abs(v.body.angvel().y));
        peakFrontShare=Math.max(peakFrontShare,(v.wheels[0].force+v.wheels[1].force)/v.wheels.reduce((sum,w)=>sum+w.force,0));
      }
      assert(maxTrailSlip<35&&v.speed>5,'Trail braking must retain controllable forward motion');
      const trailExitSpeed=v.speed*3.6;
      atSpeed(70);let maxSlalomSlip=0;
      for(let i=0;i<240*6;i++){
        step({...zero,steer:.25*Math.sin(i*dt*Math.PI),throttle:.3});
        maxSlalomSlip=Math.max(maxSlalomSlip,Math.abs(beta(v)));
      }
      assert(maxSlalomSlip<35&&v.speed>5,'Repeated steering reversals must remain controllable');
      // A contact slip step should build lateral response, then release it without
      // keeping a stale force after airborne/reset or mode changes.
      atSpeed(80);v.body.setLinvel({x:2,y:0,z:-80/3.6},true);step();
      const firstAngle=v.wheels[0].relaxedSlipAngle,rawAngle=v.wheels[0].slipAngle;
      if(mode==='refined')assert(Math.abs(firstAngle)>0&&Math.abs(firstAngle)<Math.abs(rawAngle)*.8,`New lateral response must build progressively: ${kind} wet=${wet} angle=${firstAngle} raw=${rawAngle} mode=${v.handlingMode}`);
      else assert(Math.abs(firstAngle-rawAngle)<1e-12,'Original response must remain immediate');
      v.body.setTranslation({x:0,y:8,z:0},true);step();
      assert(v.wheels.every(w=>!w.contact&&w.relaxedSlipAngle===0),'Airborne tires must release contact deformation');
      v.setHandlingMode(mode==='refined'?'baseline':'refined');
      assert(v.wheels.every(w=>w.handlingMode===v.handlingMode&&w.relaxedSlipAngle===0));
      v.setHandlingMode(mode);
      atSpeed(0);for(let i=0;i<480;i++)step();
      assert(v.speed<.02,'Relaxation must not cause parked creep');
      profiles.push({mode,stop80Metres:round(stopDistance),stop80Seconds:round(stopTime),largestBrakePressureStep:round(pressureJump),trailPeakSideslipDegrees:round(maxTrailSlip),trailPeakYawRate:round(maxYaw),trailPeakFrontLoadShare:round(peakFrontShare),trailExitKmh:round(trailExitSpeed),slalomPeakSideslipDegrees:round(maxSlalomSlip),firstSlipResponseRatio:round(firstAngle/rawAngle)});
      world.free();
    }
    const [old,current]=profiles;
    assert(current.stop80Metres<old.stop80Metres*1.06,'Full braking distance must stay within 6% of original');
    assert(current.largestBrakePressureStep<old.largestBrakePressureStep*.25,'Brake onset must avoid the original pressure step');
    assert(current.trailPeakSideslipDegrees<old.trailPeakSideslipDegrees+4,'Trail braking must not introduce a larger uncontrolled slide');
    assert(current.slalomPeakSideslipDegrees<old.slalomPeakSideslipDegrees+4,'Slalom must not develop added oscillation');
    comparison.push({kind,surface:wet?'wet mud':'dry gravel',profiles});
  }
  // Same input trace at different render cadences in both selectable modes.
  const cadence=[];
  for(const mode of ['baseline','refined']){
    const runs=[];
    for(const hz of [30,60,120]){
      const {world,v,step,atSpeed}=rig('suv',mode);atSpeed(70);
      for(let i=0;i<hz*4;i++)step({...zero,steer:i<hz*2?.2:-.2,brake:i<hz?.3:0,throttle:i>=hz?.4:0},1/hz);
      runs.push({hz,x:v.position.x,z:v.position.z,speed:v.speed});world.free();
    }
    for(const run of runs)assert(Math.hypot(run.x-runs[0].x,run.z-runs[0].z)<.01&&Math.abs(run.speed-runs[0].speed)<.01,'Render cadence must preserve the same outcome');
    cadence.push({mode,result:'passed at 30 / 60 / 120 Hz'});
  }
  const result={comparison,cadence,contactLifecycle:'airborne, reset, mode switch and parked stability passed',limitations:'Scripted production-physics checks; no hands-on DiRT Rally comparison or browser/GPU verification.'};
  writeFileSync(join(root,'reference/handling-refinement-validation.json'),JSON.stringify(result,null,2)+'\n');
  console.log(JSON.stringify(result,null,2));
}finally{rmSync(temp,{recursive:true,force:true});}
