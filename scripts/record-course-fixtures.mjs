import {readFileSync,writeFileSync,mkdirSync,mkdtempSync,rmSync} from 'node:fs';
import {createRequire} from 'node:module';
import {dirname,join} from 'node:path';
import {fileURLToPath} from 'node:url';
import ts from 'typescript';

// Fixtures for porting the course: the centreline knots, and seeded samples of
// courseAt, nearestRoad, surfaceAt, roadHeight and terrainHeight (which together
// define the driven surface and its collision meshes).
// Output: reference/golden-traces/course.json.
const root=dirname(dirname(fileURLToPath(import.meta.url))),cache=join(root,'node_modules/.cache');
mkdirSync(cache,{recursive:true});const temp=mkdtempSync(join(cache,'rally-course-'));
const require=createRequire(import.meta.url);
try{
  const {outputText}=ts.transpileModule(readFileSync(join(root,'lib/rally/course.ts'),'utf8'),{compilerOptions:{module:ts.ModuleKind.CommonJS,target:ts.ScriptTarget.ES2022}});
  writeFileSync(join(temp,'course.cjs'),outputText);
  const c=require(join(temp,'course.cjs')),rand=c.random(7070);
  const point=p=>[p.x,p.y,p.z,p.tx,p.tz,p.width,p.s,p.distance];
  const report={length:c.COURSE_LENGTH,course:c.COURSE.map(point),courseAt:[],nearest:[],surface:[],terrain:[],road:[]};
  for(let i=0;i<300;i++){const s=rand()*1.2-.1;report.courseAt.push([s,...point(c.courseAt(s))]);}
  for(let i=0;i<1500;i++){
    // Mostly near the road (where the wheels are), some anywhere in the world.
    const p=c.courseAt(rand()),off=i%5===4?(rand()-.5)*500:(rand()-.5)*40;
    const x=i%5===4?(rand()-.5)*560:p.x-p.tz*off+(rand()-.5)*3,z=i%5===4?(rand()-.5)*560:p.z+p.tx*off+(rand()-.5)*3;
    const n=c.nearestRoad(x,z),s=c.surfaceAt(x,z);
    report.nearest.push([x,z,...point(n.point),n.distance,n.lateral]);
    report.surface.push([x,z,s.type,s.grip,s.rollingResistance,s.rollingDrag,s.loose,s.bump,s.mud?1:0,s.label]);
    report.terrain.push([x,z,c.terrainHeight(x,z)]);
  }
  for(let i=0;i<600;i++){const p=c.courseAt(rand()),lateral=(rand()-.5)*p.width;report.road.push([p.s,lateral,c.roadHeight(p,lateral)]);}
  mkdirSync(join(root,'reference/golden-traces'),{recursive:true});
  // 12 significant digits: Node/V8 versions differ in the last bits of Math.sin
  // and friends, and CI diffs this file. The port compares at 1e-9.
  const round=(_,v)=>typeof v==='number'&&Number.isFinite(v)?Number(v.toPrecision(12)):v;
  writeFileSync(join(root,'reference/golden-traces/course.json'),JSON.stringify(report,round)+'\n');
  console.log('course length',c.COURSE_LENGTH.toFixed(1),'m;',report.nearest.length,'samples');
}finally{rmSync(temp,{recursive:true,force:true});}
