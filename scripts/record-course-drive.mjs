import {readFileSync,writeFileSync,mkdirSync,mkdtempSync,rmSync} from 'node:fs';
import {createRequire} from 'node:module';
import {dirname,join} from 'node:path';
import {fileURLToPath} from 'node:url';
import ts from 'typescript';

// A drive on the real course: terrain and road trimesh colliders, production
// surfaceAt, a pure-pursuit driver on the centreline from the start line for
// 14 s per car. Inputs are recorded as applied for open-loop replay.
// Not part of `traces:record` (and so not diffed in CI): 14 s of chaotic driving
// amplifies last-bit differences between Node/V8 versions. Re-record it with
// `pnpm traces:record-course-drive` when the physics changes.
// Output: reference/golden-traces/course-drive.json.
const root=dirname(dirname(fileURLToPath(import.meta.url))),cache=join(root,'node_modules/.cache');
mkdirSync(cache,{recursive:true});const temp=mkdtempSync(join(cache,'rally-course-drive-'));
const require=createRequire(import.meta.url),RAPIER=require('@dimforge/rapier3d-compat');
try{
  for(const name of ['course','vehicle-config','vehicle','steering','simulation','drive-model','wheel-model','vendor/ecctrl/CurveLUT','vendor/stunt-rally/gravel','vendor/stunt-rally/pacejka','vendor/stunt-rally/abs','vendor/stunt-rally/engine-friction','vendor/stunt-rally/differential']){
    const {outputText}=ts.transpileModule(readFileSync(join(root,'lib/rally',name+'.ts'),'utf8'),{compilerOptions:{module:ts.ModuleKind.CommonJS,target:ts.ScriptTarget.ES2022,esModuleInterop:true}});
    mkdirSync(dirname(join(temp,name+'.cjs')),{recursive:true});
    writeFileSync(join(temp,name+'.cjs'),outputText.replace(/require\("\.\/(.*?)"\)/g,'require("./$1.cjs")'));
  }
  const course=require(join(temp,'course.cjs')),{RallyVehicle}=require(join(temp,'vehicle.cjs')),{advanceVehicle}=require(join(temp,'simulation.cjs'));
  await RAPIER.init();
  const terrain=course.makeTerrainData(),road=course.makeRoadData(),dt=1/60,zero={steer:0,throttle:0,brake:0,handbrake:0};
  const runs=[];
  for(const kind of ['sti','truck']){
    const world=new RAPIER.World({x:0,y:-9.81,z:0});world.timestep=1/60;world.integrationParameters.numSolverIterations=8;
    for(const data of [terrain,road])world.createCollider(RAPIER.ColliderDesc.trimesh(data.vertices,data.indices).setFriction(.9));
    const v=new RallyVehicle(world,kind,'rally');v.slideSteering=true;
    for(let i=0;i<50;i++)advanceVehicle(v,world,zero,dt,false); // as selectVehicle
    v.reset(course.spawnPoint(0));
    const frames=[];
    for(let i=0;i<840;i++){
      const p=v.position,n=course.nearestRoad(p.x,p.z),ahead=course.courseAt(n.point.s+(8+v.speed*.6)/course.COURSE_LENGTH);
      const ex=ahead.x-p.x,ez=ahead.z-p.z,left=v.forward.z*ex-v.forward.x*ez,demand=Math.atan(2.6*2*left/Math.max(1,ex*ex+ez*ez));
      const target=kind==='sti'?15:13,input={steer:course.clamp(demand*6,-1,1),throttle:v.speed<target?1:.25,brake:v.speed>target+4?.4:0,handbrake:0};
      advanceVehicle(v,world,input,dt,true);
      frames.push({input,position:[v.position.x,v.position.y,v.position.z],speed:v.speed,gear:v.gear,surface:v.surface});
    }
    world.free();
    runs.push({kind,frames});
  }
  mkdirSync(join(root,'reference/golden-traces'),{recursive:true});
  writeFileSync(join(root,'reference/golden-traces/course-drive.json'),JSON.stringify({runs})+'\n');
  for(const r of runs){const f=r.frames.at(-1),n=course.nearestRoad(f.position[0],f.position[2]);console.log(`${r.kind}: ${(n.point.s*course.COURSE_LENGTH).toFixed(0)} m along, ${(f.speed*3.6).toFixed(1)} km/h, ${n.distance.toFixed(1)} m off centre, surfaces ${[...new Set(r.frames.map(x=>x.surface))].join('/')}`);}
}finally{rmSync(temp,{recursive:true,force:true});}
