import assert from 'node:assert/strict';
import {readFileSync,writeFileSync,mkdirSync,mkdtempSync,rmSync} from 'node:fs';
import {createRequire} from 'node:module';
import {dirname,join} from 'node:path';
import {fileURLToPath} from 'node:url';
import ts from 'typescript';

// Standard vehicle-dynamics tests on the shipped physics, with numeric targets:
// a constant-radius skidpad (understeer gradient, aligning torque and trail),
// step-steer (rack and yaw response), and a hands-off rack disturbance (shimmy).
// The baseline handling mode is reported alongside for comparison.
const root=dirname(dirname(fileURLToPath(import.meta.url))),cache=join(root,'node_modules/.cache');
mkdirSync(cache,{recursive:true});const temp=mkdtempSync(join(cache,'rally-steering-response-'));
const require=createRequire(import.meta.url),RAPIER=require('@dimforge/rapier3d-compat');
const zero={steer:0,throttle:0,brake:0,handbrake:0},dt=1/60,g=9.81,deg=180/Math.PI;
try{
  for(const name of ['course','vehicle-config','vehicle','steering','simulation','drive-model','wheel-model','vendor/ecctrl/CurveLUT','vendor/stunt-rally/gravel','vendor/stunt-rally/pacejka','vendor/stunt-rally/abs','vendor/stunt-rally/engine-friction','vendor/stunt-rally/differential']){
    const {outputText}=ts.transpileModule(readFileSync(join(root,'lib/rally',name+'.ts'),'utf8'),{compilerOptions:{module:ts.ModuleKind.CommonJS,target:ts.ScriptTarget.ES2022,esModuleInterop:true}});
    mkdirSync(dirname(join(temp,name+'.cjs')),{recursive:true});
    writeFileSync(join(temp,name+'.cjs'),outputText.replace(/require\("\.\/(.*?)"\)/g,'require("./$1.cjs")'));
  }
  const course=require(join(temp,'course.cjs'));
  const {RallyVehicle}=require(join(temp,'vehicle.cjs')),{advanceVehicle}=require(join(temp,'simulation.cjs'));
  await RAPIER.init();
  const surfaces={gravel:{type:'gravel',grip:.59,rollingResistance:.025,rollingDrag:4,loose:.55,bump:0,mud:false,label:'LOOSE DIRT'},tarmac:{...course.SURFACES.tarmac,bump:0,mud:false}};
  course.surfaceAt=()=>surfaces.gravel;
  // Body slip angle: + when the velocity points left of the heading.
  const beta=v=>Math.atan2(-(v.velocity.x*-v.forward.z+v.velocity.z*v.forward.x),v.forwardSpeed);
  function rig(kind,mode,surface='gravel'){
    course.surfaceAt=()=>surfaces[surface];
    const world=new RAPIER.World({x:0,y:-9.81,z:0});world.integrationParameters.numSolverIterations=8;
    world.createCollider(RAPIER.ColliderDesc.cuboid(2000,.5,2000).setTranslation(0,-.5,0));
    const v=new RallyVehicle(world,kind,mode);
    v.reset({x:0,y:0,z:0,tx:0,tz:-1,width:12,s:0,distance:0});
    for(let i=0;i<120;i++)advanceVehicle(v,world,zero,dt,false);
    const step=(input=zero)=>advanceVehicle(v,world,input,dt,true);
    const atSpeed=kmh=>{
      v.body.setLinvel({x:0,y:0,z:-kmh/3.6},true);v.wheels.forEach(w=>w.angularSpeed=kmh/3.6/v.config.radius);
      const c=v.config.powertrain,wheelRPM=kmh/3.6/v.config.radius*30/Math.PI;
      let gear=0;for(let i=0;i<c.gearRatios.length;i++)if(wheelRPM*c.gearRatios[i]*c.finalDriveRatio>c.shiftDownRPM+1200)gear=i;
      v.driveModel.gearIndex=gear;v.driveModel.driveRatio=c.gearRatios[gear]*c.finalDriveRatio;v.syncEngine();
    };
    // Speed hold: proportional throttle/brake around an integrated throttle.
    let held=.3;
    const hold=(target,steer)=>{
      const error=target-v.speed;held=course.clamp(held+error*.01,0,1);
      step({...zero,steer,throttle:course.clamp(held+error*.5,0,1),brake:course.clamp(-error*.3,0,1)});
    };
    return {world,v,step,atSpeed,hold};
  }
  const mean=list=>list.reduce((sum,x)=>sum+x,0)/list.length;

  // Constant-radius skidpad: a pure-pursuit driver holds a 30 m left circle while
  // speed rises in stages. Each stage settles for 3 s, then averages 2 s.
  const RADIUS=30;
  function skidpad(kind,mode,surface){
    const {world,v,hold}=rig(kind,mode,surface),c=v.config,wheelbase=c.back-c.front,stages=[];
    for(const kmh of [15,25,35,45,55]){
      const samples=[];
      for(let i=0;i<300;i++){
        const p=v.position,dx=p.x+RADIUS,dz=p.z,angle=Math.atan2(dz,dx)-(5+v.speed*.5)/RADIUS;
        const ex=-RADIUS+RADIUS*Math.cos(angle)-p.x,ez=RADIUS*Math.sin(angle)-p.z;
        const left=v.forward.z*ex-v.forward.x*ez,demand=Math.atan(wheelbase*2*left/Math.max(1,ex*ex+ez*ez));
        const limit=Math.max(.05,demand>0?v.steerLimits[1]:v.steerLimits[0]);
        hold(kmh/3.6,Math.sign(demand)*Math.min(1,Math.abs(demand)/limit)**(1/1.15));
        if(i>=180){
          const [a,b]=v.wheels;
          samples.push({speed:v.speed,yaw:v.body.angvel().y,radius:Math.hypot(dx,dz),steer:v.steer,beta:beta(v),aligning:a.aligning+b.aligning,trail:(a.trail+b.trail)/2});
        }
      }
      const speed=mean(samples.map(s=>s.speed)),radius=mean(samples.map(s=>s.radius));
      stages.push({kmh:+(speed*3.6).toFixed(1),radius:+radius.toFixed(1),lateralG:+(speed*Math.abs(mean(samples.map(s=>s.yaw)))/g).toFixed(3),
        steerDeg:+(mean(samples.map(s=>s.steer))*deg).toFixed(2),ackermannDeg:+(wheelbase/radius*deg).toFixed(2),
        slipDeg:+(mean(samples.map(s=>s.beta))*deg).toFixed(2),aligningNm:+mean(samples.map(s=>s.aligning)).toFixed(0),trailMm:+(mean(samples.map(s=>s.trail))*1000).toFixed(1)});
    }
    world.free();
    // Understeer gradient: extra steer beyond Ackermann per g, fitted over the
    // stages below 0.55 g (the tires' near-linear range).
    const linear=stages.filter(s=>s.lateralG>.05&&s.lateralG<.55),x=linear.map(s=>s.lateralG),y=linear.map(s=>s.steerDeg-s.ackermannDeg);
    const mx=mean(x),my=mean(y),gradient=x.reduce((sum,xi,i)=>sum+(xi-mx)*(y[i]-my),0)/x.reduce((sum,xi)=>sum+(xi-mx)**2,0);
    return {understeerDegPerG:+gradient.toFixed(2),stages};
  }

  // Step steer at constant speed: rack and yaw-rate 90% rise times, overshoot.
  function stepSteer(kind,mode,kmh,command,surface){
    const {world,v,atSpeed,hold}=rig(kind,mode,surface);atSpeed(kmh);
    for(let i=0;i<30;i++)hold(kmh/3.6,0);
    const yaw=[],steer=[];
    for(let i=0;i<240;i++){hold(kmh/3.6,command);yaw.push(v.body.angvel().y);steer.push(v.steer);}
    world.free();
    const steady=mean(yaw.slice(150,210)),steadySteer=mean(steer.slice(150,210));
    const rise=(list,target)=>(list.findIndex(x=>x>=.9*target)+1)*dt;
    return {kmh,command,steerDeg:+(steadySteer*deg).toFixed(2),rackRiseSeconds:+rise(steer,steadySteer).toFixed(3),
      yawRate:+steady.toFixed(3),yawRiseSeconds:+rise(yaw,steady).toFixed(3),yawOvershootPercent:+((Math.max(...yaw)/steady-1)*100).toFixed(1),finalSlipDeg:+(beta(v)*deg).toFixed(1)};
  }

  // Hands-off at 100 km/h on a straight: kick the rack and watch it settle.
  function shimmy(kind){
    const {world,v,atSpeed,hold}=rig(kind,'rally');v.slideSteering=true;atSpeed(100);
    for(let i=0;i<30;i++)hold(100/3.6,0);
    v.rack.rate=3;const angles=[];
    for(let i=0;i<120;i++){hold(100/3.6,0);angles.push(v.steer);}
    world.free();
    const peak=Math.max(...angles.map(Math.abs)),late=Math.max(...angles.slice(45).map(Math.abs));
    return {peakDeg:+(peak*deg).toFixed(2),after075sDeg:+(late*deg).toFixed(3),heading:+Math.abs(Math.atan2(v.velocity.x,-v.velocity.z)*deg).toFixed(1)};
  }

  const report={skidpad:[],stepSteer:[],shimmy:[],tarmac:[]};
  for(const kind of ['sti','truck']){
    for(const mode of ['rally','baseline']){
      const pad=skidpad(kind,mode);report.skidpad.push({kind,mode,...pad});
      for(const [kmh,command] of [[60,.25],[100,.15]])report.stepSteer.push({kind,mode,...stepSteer(kind,mode,kmh,command)});
      if(mode!=='rally')continue;
      const {stages}=pad,low=stages[0];
      // Geometry: at walking pace the car steers its Ackermann angle.
      assert(Math.abs(low.steerDeg-low.ackermannDeg)<1,`${kind}: low-speed steer must match Ackermann (${low.steerDeg} vs ${low.ackermannDeg})`);
      assert(stages.every(s=>Math.abs(s.radius-RADIUS)<RADIUS*.25),`${kind}: the skidpad driver must hold the circle`);
      // Mild, stable understeer through the linear range.
      assert(pad.understeerDegPerG>.3&&pad.understeerDegPerG<8,`${kind}: understeer gradient out of range (${pad.understeerDegPerG} deg/g)`);
      // The aligning torque resists the steer and rises with cornering force...
      assert(stages.every(s=>s.aligningNm<=0),`${kind}: a left turn's aligning torque must pull the wheels right`);
      assert(Math.abs(stages[3].aligningNm)>Math.abs(stages[0].aligningNm)*3,`${kind}: aligning torque must build with lateral force`);
      // ...and the trail collapses toward the caster as the tires near their limit.
      const trailDrop=stages.at(-1).trailMm/stages[0].trailMm;
      assert(trailDrop<.85,`${kind}: the steering must go light toward the limit (trail ${stages[0].trailMm} -> ${stages.at(-1).trailMm} mm)`);
      for(const s of report.stepSteer.filter(s=>s.kind===kind&&s.mode==='rally')){
        assert(s.rackRiseSeconds>0&&s.rackRiseSeconds<.35,`${kind}: rack must follow a step input within 0.35 s (${s.rackRiseSeconds})`);
        assert(s.yawRiseSeconds>0&&s.yawRiseSeconds<1.25,`${kind}: yaw must respond within 1.25 s at ${s.kmh} km/h (${s.yawRiseSeconds})`);
        assert(s.yawOvershootPercent<60,`${kind}: yaw overshoot at ${s.kmh} km/h too large (${s.yawOvershootPercent}%)`);
      }
    }
    // Tarmac (no stage on the course yet): the same tests on the tarmac curve.
    const pad=skidpad(kind,'rally','tarmac'),step=stepSteer(kind,'rally',100,.15,'tarmac');
    report.tarmac.push({kind,understeerDegPerG:pad.understeerDegPerG,peakLateralG:Math.max(...pad.stages.map(s=>s.lateralG)),stepSteer100:step});
    assert(pad.understeerDegPerG>.3&&pad.understeerDegPerG<8,`${kind}: tarmac understeer gradient out of range (${pad.understeerDegPerG} deg/g)`);
    assert(step.yawOvershootPercent<60,`${kind}: tarmac yaw overshoot too large (${step.yawOvershootPercent}%)`);
    const kick=shimmy(kind);report.shimmy.push({kind,...kick});
    assert(kick.after075sDeg<.5,`${kind}: a hands-off rack disturbance must settle without shimmy (${kick.after075sDeg} deg)`);
    assert(kick.heading<3,`${kind}: hands-off straight-line running must hold its heading (${kick.heading} deg)`);
  }
  console.log(JSON.stringify(report,null,2));
}finally{rmSync(temp,{recursive:true,force:true});}
