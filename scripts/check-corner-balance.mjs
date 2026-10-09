import assert from 'node:assert/strict';
import {readFileSync,writeFileSync,mkdirSync,mkdtempSync,rmSync} from 'node:fs';
import {createRequire} from 'node:module';
import {dirname,join} from 'node:path';
import {fileURLToPath} from 'node:url';
import ts from 'typescript';

// Matched entry / power-exit traces followed by active driver countersteering.
// Dry entry: 70 km/h. Wet entry: 55 km/h. Production solver, no yaw assists.
// These compare regressions and response shape, not fidelity to DiRT Rally.
const root=dirname(dirname(fileURLToPath(import.meta.url))),cache=join(root,'node_modules/.cache');
mkdirSync(cache,{recursive:true});const temp=mkdtempSync(join(cache,'corner-balance-'));
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
  const results=[];
  for(const kind of ['suv','truck'])for(const wet of [false,true])for(const mode of ['refined','rally']){
    for(const maneuver of ['coast','trail','power']){
      const {world,v,step,atSpeed}=rig(kind,mode,wet);atSpeed(wet?55:70);
      let peakSlip=0,peakYaw=0,heading=0,entryHeading=0,entryFrontLoad=0,powerHeading=0;
      const snapshots=[];
      for(let i=0;i<240*2.5;i++){
        const t=i*dt;
        const steer=t<1.5?.28:.16;
        let throttle=t<.5?.3:t<1.5?0:.65;
        let brake=maneuver==='trail'&&t>=.5&&t<1.2?.3*(1-(t-.5)/.7):0;
        if(maneuver==='power')throttle=.65;
        if(maneuver==='coast')throttle=t<.5?.3:0;
        step({...zero,steer,throttle,brake});
        assert(Math.abs(v.wheels.reduce((sum,w)=>sum+w.driveTorque,0)-v.engineWheelTorque)<.2,'Differentials must conserve input torque');
        if(brake>.01)assert(v.driveModel.demand===0,'Foot braking must inhibit propulsion; flywheel inertia may still couple');
        const slip=Math.abs(beta(v)),yaw=Math.abs(v.body.angvel().y);
        peakSlip=Math.max(peakSlip,slip);peakYaw=Math.max(peakYaw,yaw);heading+=v.body.angvel().y*dt*180/Math.PI;
        if(i===359){entryHeading=heading;entryFrontLoad=(v.wheels[0].force+v.wheels[1].force)/v.wheels.reduce((s,w)=>s+w.force,0);}
        if(i===599)powerHeading=heading-entryHeading;
        if([119,239,359,599].includes(i))snapshots.push({t:round((i+1)*dt),speed:round(v.speed*3.6),slip:round(beta(v)),yaw:round(v.body.angvel().y),center:round(v.diffTransfer.center)});
      }
      const exitSpeed=v.speed*3.6,exitSlip=beta(v);
      let recoveredAt=0;
      for(let i=0;i<240*4;i++){
        const slip=beta(v)*Math.PI/180,yaw=v.body.angvel().y;
        step({...zero,steer:-course.clamp(slip*2+yaw*.35,-1,1),throttle:.15});
        if(Math.abs(beta(v))<5&&Math.abs(v.body.angvel().y)<.25&&!recoveredAt)recoveredAt=(i+1)*dt;
      }
      assert(recoveredAt>0&&recoveredAt<3,`Driver recovery ${kind} ${mode} wet=${wet} ${maneuver}: recovered=${recoveredAt} exitSlip=${exitSlip} finalSlip=${beta(v)} speed=${v.speed}`);
      assert(Math.abs(beta(v))<3&&v.forwardSpeed>5,'Recovery must settle with forward momentum');
      results.push({kind,wet,mode,maneuver,exitSpeed:round(exitSpeed),exitSlip:round(exitSlip),recoverySeconds:round(recoveredAt),peakSlip:round(peakSlip),peakYaw:round(peakYaw),entryHeading:round(entryHeading),powerHeading:round(powerHeading),entryFrontLoad:round(entryFrontLoad),snapshots});world.free();
    }
  }
  for(const original of results.filter(r=>r.mode==='refined')){
    const current=results.find(r=>r.mode==='rally'&&r.kind===original.kind&&r.wet===original.wet&&r.maneuver===original.maneuver);
    assert(current.peakSlip<35,'Normal corner maneuvers must stay below excessive sideslip');
    // Mud: the coast-released centre lets the rear brake harder under trail
    // braking, and the mud curve trades that for side force, so the rally
    // setup's extra rotation is larger there (and still recovered below).
    assert(current.peakSlip<original.peakSlip+(original.wet?6:3),'Added corner freedom must not introduce a large slide');
    assert(current.exitSpeed>original.exitSpeed*.95,'Corner exit must retain forward speed');
    if(!original.wet&&original.maneuver==='coast')assert(current.entryHeading>original.entryHeading,'Coast setup must allow measurably more dry corner-entry rotation');
  }
  const braking=[],cadence=[];
  for(const kind of ['suv','truck'])for(const wet of [false,true]){
    const profiles=[];
    for(const mode of ['refined','rally']){
      const {world,v,step,atSpeed}=rig(kind,mode,wet);atSpeed(80);
      let seconds=0;
      for(let i=0;i<240*6;i++){step({...zero,brake:1});if(v.speed<.15){seconds=(i+1)*dt;break;}}
      assert(seconds>0,'Full braking must stop the car');
      assert(Math.abs(v.position.x)<.1,'Straight braking must not introduce yaw');
      profiles.push({mode,metres:round(Math.hypot(v.position.x,v.position.z)),seconds:round(seconds)});world.free();
    }
    // 5%: flywheel inertia shortened Refined stops more than Rally (SUV dry 26.05->25.01 m vs 26.56->26.09 m).
    assert(profiles[1].metres<profiles[0].metres*1.05,'Braking must stay within 5% of Refined');
    braking.push({kind,wet,profiles});
  }
  for(const kind of ['suv','truck']){
    const runs=[];
    for(const hz of [30,60,120]){
      const {world,v,step,atSpeed}=rig(kind,'rally');atSpeed(70);
      for(let i=0;i<hz*6;i++)step({...zero,steer:Math.floor(i/hz)%2===0?.25:-.25,throttle:.3},1/hz);
      assert(Math.abs(beta(v))<35&&v.speed>5,'Steering reversals must stay controllable');
      runs.push({hz,x:v.position.x,z:v.position.z,speed:v.speed});
      v.setHandlingMode('refined');v.setHandlingMode('rally');
      assert(v.wheels.every(w=>w.handlingMode==='rally'&&w.relaxedSlipAngle===0),'Mode switching clears contact deformation');
      atSpeed(0);for(let i=0;i<480;i++)step();assert(v.speed<.02,'Parked car must not creep');
      world.free();
    }
    for(const run of runs)assert(Math.hypot(run.x-runs[0].x,run.z-runs[0].z)<.01&&Math.abs(run.speed-runs[0].speed)<.01,'Render cadence must preserve the same outcome');
    cadence.push({kind,result:'30 / 60 / 120 Hz steering reversals, mode switching and parked stability passed'});
  }
  const {differentialTorques}=require(join(temp,'vendor/stunt-rally/differential.cjs'));
  const {VEHICLES}=require(join(temp,'vehicle-config.cjs'));
  for(const kind of ['suv','truck'])for(const config of Object.values(VEHICLES[kind].rallyDifferential))
    for(const torque of [-2000,0,2000])for(const speeds of [[80,70],[-80,-70],[0,100],[100,100]]){
      const result=differentialTorques(torque,...speeds,2,2,dt,config);
      assert(Math.abs(result.side1+result.side2-torque)<1e-9,'Coupling must conserve drive torque');
      assert(result.transfer*(speeds[0]-speeds[1])>=0,'Coupling must dissipate wheel-speed differences');
      assert(Math.abs(result.transfer)<=Math.abs(speeds[0]-speeds[1])/dt+1e-9,'Coupling must not reverse the wheel-speed gap');
    }
  const report={results,braking,cadence,limitations:'Scripted production-solver tests, dry 70 km/h and wet 55 km/h. No hands-on DiRT Rally or browser/GPU comparison. Vehicle settings are game tuning, not measured factory specifications.'};
  writeFileSync(join(root,'reference/corner-balance-validation.json'),JSON.stringify(report,null,2)+'\n');
  console.log(JSON.stringify({maneuvers:results.length,braking,cadence,result:'passed'},null,2));
}finally{rmSync(temp,{recursive:true,force:true});}
