import {readFileSync,writeFileSync,mkdirSync,mkdtempSync,rmSync,readdirSync} from 'node:fs';
import {createRequire} from 'node:module';
import {dirname,join} from 'node:path';
import {fileURLToPath} from 'node:url';
import ts from 'typescript';

// Golden traces for porting the vehicle model: fixed scenarios on flat ground,
// recording the applied inputs and the full vehicle state every 60 Hz tick.
// Inputs are stored as applied, so a port can replay them open-loop and compare
// state tick by tick. Output: reference/golden-traces/<kind>-<scenario>.json.
const root=dirname(dirname(fileURLToPath(import.meta.url))),cache=join(root,'node_modules/.cache');
mkdirSync(cache,{recursive:true});const temp=mkdtempSync(join(cache,'rally-golden-'));
const require=createRequire(import.meta.url),RAPIER=require('@dimforge/rapier3d-compat');
const dt=1/60,zero={steer:0,throttle:0,brake:0,handbrake:0},out=join(root,'reference/golden-traces');
const r=x=>Math.round(x*1e6)/1e6,v3=v=>[r(v.x),r(v.y),r(v.z)],q4=q=>[r(q.x),r(q.y),r(q.z),r(q.w)];
try{
  for(const name of ['course','vehicle-config','vehicle','steering','simulation','drive-model','wheel-model','vendor/ecctrl/CurveLUT','vendor/stunt-rally/gravel','vendor/stunt-rally/pacejka','vendor/stunt-rally/abs','vendor/stunt-rally/engine-friction','vendor/stunt-rally/differential']){
    const {outputText}=ts.transpileModule(readFileSync(join(root,'lib/rally',name+'.ts'),'utf8'),{compilerOptions:{module:ts.ModuleKind.CommonJS,target:ts.ScriptTarget.ES2022,esModuleInterop:true}});
    mkdirSync(dirname(join(temp,name+'.cjs')),{recursive:true});
    writeFileSync(join(temp,name+'.cjs'),outputText.replace(/require\("\.\/(.*?)"\)/g,'require("./$1.cjs")'));
  }
  const course=require(join(temp,'course.cjs'));
  const {RallyVehicle}=require(join(temp,'vehicle.cjs')),{advanceVehicle,PHYSICS_STEP,DRIVELINE_STEP}=require(join(temp,'simulation.cjs'));
  await RAPIER.init();
  // Flat surfaces: bump 0 so the port needs no course geometry for these.
  const surface=name=>({...course.SURFACES[name],bump:0,mud:name==='mud'});

  function run(kind,{surfaceName='dirt',mode='rally',kmh=0,ticks,drive}){
    course.surfaceAt=()=>surface(surfaceName);
    const world=new RAPIER.World({x:0,y:-9.81,z:0});world.integrationParameters.numSolverIterations=8;
    world.createCollider(RAPIER.ColliderDesc.cuboid(2000,.5,2000).setTranslation(0,-.5,0));
    const v=new RallyVehicle(world,kind,mode);
    v.reset({x:0,y:0,z:0,tx:0,tz:-1,width:12,s:0,distance:0});
    for(let i=0;i<120;i++)advanceVehicle(v,world,zero,dt,false);
    if(kmh>0){
      v.body.setLinvel({x:0,y:0,z:-kmh/3.6},true);v.wheels.forEach(w=>w.angularSpeed=kmh/3.6/v.config.radius);
      const c=v.config.powertrain,wheelRPM=kmh/3.6/v.config.radius*30/Math.PI;
      let gear=0;for(let i=0;i<c.gearRatios.length;i++)if(wheelRPM*c.gearRatios[i]*c.finalDriveRatio>c.shiftDownRPM+1200)gear=i;
      v.driveModel.gearIndex=gear;v.driveModel.driveRatio=c.gearRatios[gear]*c.finalDriveRatio;v.syncEngine();
    }
    let held=.3;
    const hold=(target,extra={})=>{
      const error=target-v.speed;held=course.clamp(held+error*.01,0,1);
      return {...zero,throttle:course.clamp(held+error*.5,0,1),brake:course.clamp(-error*.3,0,1),...extra};
    };
    const state=()=>({
      position:v3(v.position),rotation:q4(v.rotation),velocity:v3(v.velocity),angvel:v3(v.body.angvel()),
      speed:r(v.speed),forwardSpeed:r(v.forwardSpeed),steer:r(v.steer),rpm:r(v.rpm),gear:v.gear,
      throttle:r(v.throttle),brake:r(v.brake),handbrake:r(v.handbrake),engineWheelTorque:r(v.engineWheelTorque),
      wheels:v.wheels.map(w=>({contact:w.contact,angularSpeed:r(w.angularSpeed),force:r(w.force),suspensionLength:r(w.suspensionLength),
        longSlip:r(w.longSlip),slipAngle:r(w.slipAngle),longVelocity:r(w.longVelocity),sideVelocity:r(w.sideVelocity),aligning:r(w.aligning??0),steer:r(w.steer)})),
    });
    const initial=state(),frames=[];
    for(let i=0;i<ticks;i++){
      const input=drive(i,v,hold);
      advanceVehicle(v,world,input,dt,true);
      frames.push({input:{steer:r(input.steer),throttle:r(input.throttle),brake:r(input.brake),handbrake:r(input.handbrake)},...state()});
    }
    world.free();
    return {initial,frames};
  }

  // Each scenario exercises a different part of the model.
  const scenarios={
    launch:{ticks:480,drive:()=>({...zero,throttle:1})},
    'brake-100':{kmh:100,ticks:360,drive:(i,v)=>({...zero,brake:v.forwardSpeed>.5?1:0})},
    'step-steer-60':{kmh:60,ticks:270,drive:(i,v,hold)=>hold(60/3.6,{steer:i<30?0:.25})},
    'step-steer-100-tarmac':{surfaceName:'tarmac',kmh:100,ticks:270,drive:(i,v,hold)=>hold(100/3.6,{steer:i<30?0:.15})},
    'handbrake-turn-60':{kmh:60,ticks:300,drive:(i)=>({...zero,steer:i>=20&&i<110?.8:0,handbrake:i>=30&&i<80?1:0,throttle:i>=80?.6:0})},
    'mud-launch':{surfaceName:'mud',ticks:360,drive:()=>({...zero,throttle:1})},
    'forest-slalom':{surfaceName:'forest',kmh:50,ticks:480,drive:(i,v,hold)=>hold(50/3.6,{steer:Math.sin(i/60*Math.PI*.8)*.7})},
    'reverse':{ticks:300,drive:(i)=>({...zero,brake:1,steer:i>150?.5:0})},
    'baseline-step-steer-60':{mode:'baseline',kmh:60,ticks:270,drive:(i,v,hold)=>hold(60/3.6,{steer:i<30?0:.25})},
  };
  // Replace only this script's traces: other recorders share the directory.
  mkdirSync(out,{recursive:true});
  for(const f of readdirSync(out))if(/^(sti|truck)-.*\.json$|^index\.json$/.test(f))rmSync(join(out,f));
  const index=[];
  for(const kind of ['sti','truck'])for(const [name,s] of Object.entries(scenarios)){
    const {initial,frames}=run(kind,s),file=`${kind}-${name}.json`;
    const setup={kind,scenario:name,mode:s.mode??'rally',surface:surface(s.surfaceName??'dirt'),initialKmh:s.kmh??0,ticks:s.ticks,dt,physicsStep:PHYSICS_STEP,drivelineStep:DRIVELINE_STEP,settleTicks:120};
    writeFileSync(join(out,file),JSON.stringify({setup,initial,frames}));
    const last=frames.at(-1);
    index.push({file,kind,scenario:name,finalKmh:+(last.speed*3.6).toFixed(2),finalPosition:last.position});
  }
  writeFileSync(join(out,'index.json'),JSON.stringify({rapier:JSON.parse(readFileSync(join(root,'node_modules/@dimforge/rapier3d-compat/package.json'),'utf8')).version,traces:index},null,2)+'\n');
  console.log(JSON.stringify(index.map(({file,finalKmh})=>({file,finalKmh}))));
}finally{rmSync(temp,{recursive:true,force:true});}
