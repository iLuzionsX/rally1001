import assert from 'node:assert/strict';
import {readFileSync,writeFileSync,mkdirSync,mkdtempSync,rmSync} from 'node:fs';
import {createRequire} from 'node:module';
import {dirname,join} from 'node:path';
import {fileURLToPath} from 'node:url';
import ts from 'typescript';

// Steering rack (aligning torque, relaxed-hands countersteer, tire-derived
// lock) and bump-stop landings
// on the shipped physics. Internal acceptance checks, not a hands-on benchmark.
const root=dirname(dirname(fileURLToPath(import.meta.url))),cache=join(root,'node_modules/.cache');
mkdirSync(cache,{recursive:true});const temp=mkdtempSync(join(cache,'rally-steering-'));
const require=createRequire(import.meta.url),RAPIER=require('@dimforge/rapier3d-compat');
const zero={steer:0,throttle:0,brake:0,handbrake:0},dt=1/60;
try{
  for(const name of ['course','vehicle-config','vehicle','steering','simulation','drive-model','wheel-model','vendor/ecctrl/CurveLUT','vendor/stunt-rally/gravel','vendor/stunt-rally/pacejka','vendor/stunt-rally/abs','vendor/stunt-rally/engine-friction','vendor/stunt-rally/differential']){
    const {outputText}=ts.transpileModule(readFileSync(join(root,'lib/rally',name+'.ts'),'utf8'),{compilerOptions:{module:ts.ModuleKind.CommonJS,target:ts.ScriptTarget.ES2022,esModuleInterop:true}});
    mkdirSync(dirname(join(temp,name+'.cjs')),{recursive:true});
    writeFileSync(join(temp,name+'.cjs'),outputText.replace(/require\("\.\/(.*?)"\)/g,'require("./$1.cjs")'));
  }
  const course=require(join(temp,'course.cjs'));
  const {RallyVehicle}=require(join(temp,'vehicle.cjs')),{advanceVehicle}=require(join(temp,'simulation.cjs'));
  await RAPIER.init();
  course.surfaceAt=()=>({type:'gravel',grip:.59,rollingResistance:.025,rollingDrag:4,loose:.55,bump:0,mud:false,label:'LOOSE DIRT'});
  // Body slip angle: + when the velocity points left of the heading.
  const beta=v=>Math.atan2(-(v.velocity.x*-v.forward.z+v.velocity.z*v.forward.x),v.forwardSpeed);
  function rig(kind,slideSteering){
    const world=new RAPIER.World({x:0,y:-9.81,z:0});world.integrationParameters.numSolverIterations=8;
    world.createCollider(RAPIER.ColliderDesc.cuboid(2000,.5,2000).setTranslation(0,-.5,0));
    const v=new RallyVehicle(world,kind,'rally');v.slideSteering=slideSteering;
    v.reset({x:0,y:0,z:0,tx:0,tz:-1,width:12,s:0,distance:0});
    for(let i=0;i<120;i++)advanceVehicle(v,world,zero,dt,false);
    const step=(input=zero)=>advanceVehicle(v,world,input,dt,true);
    // Place the car travelling at kmh, its heading yawed `slip` rad left of travel.
    const slide=(kmh,slip,yawRate=0)=>{
      const speed=kmh/3.6;v.body.setRotation({x:0,y:Math.sin(slip/2),z:0,w:Math.cos(slip/2)},true);
      v.body.setLinvel({x:0,y:0,z:-speed},true);v.body.setAngvel({x:0,y:yawRate,z:0},true);
      v.wheels.forEach(w=>{w.angularSpeed=speed*Math.cos(slip)/v.config.radius;w.relaxedSlipAngle=0;});
      const c=v.config.powertrain,wheelRPM=speed/v.config.radius*30/Math.PI;
      let gear=0;for(let i=0;i<c.gearRatios.length;i++)if(wheelRPM*c.gearRatios[i]*c.finalDriveRatio>c.shiftDownRPM+1200)gear=i;
      v.driveModel.gearIndex=gear;v.driveModel.driveRatio=c.gearRatios[gear]*c.finalDriveRatio;v.syncEngine();v.steer=0;
    };
    return {world,v,step,slide};
  }
  const report={steering:[],landing:[]};
  for(const kind of ['suv','truck']){
    // Hands off in a slide: the tires' aligning torque trails the wheels toward
    // travel (countersteer) only with relaxed hands, and the slide recovers
    // sooner than with the wheels held centred.
    const recovery={};
    for(const assist of [false,true]){
      const {world,v,step,slide}=rig(kind,assist);
      // Heading 25 degrees left of travel, still rotating left: an oversteer slide.
      slide(70,.44,.6);step();step();step();step();step();step();
      const early=v.steer;let worst=0,settled=null;
      for(let i=0;i<150;i++){step({...zero,throttle:.3});const b=Math.abs(beta(v));worst=Math.max(worst,b);if(settled===null&&b<.08)settled=(i+1)*dt;}
      recovery[assist?'slide':'centred']={steerAfterTenthSecond:+early.toFixed(3),peakSlip:+worst.toFixed(3),recoveredIn:settled,finalSlip:+Math.abs(beta(v)).toFixed(3)};
      if(assist)assert(early<-.05,`${kind}: hands-off wheels must trail into countersteer in a slide (${early})`);
      else assert(Math.abs(early)<.02,`${kind}: firm hands hold a centred wheel against the aligning torque (${early})`);
      world.free();
    }
    assert(recovery.slide.peakSlip<=recovery.centred.peakSlip+.01,`${kind}: caster alignment must not deepen the slide`);
    assert(recovery.slide.finalSlip<recovery.centred.finalSlip||recovery.slide.recoveredIn!==null&&(recovery.centred.recoveredIn===null||recovery.slide.recoveredIn<=recovery.centred.recoveredIn),`${kind}: hands-off recovery must be no worse with slide steering`);

    // The lock is measured from the front axle's travel: in a slide full
    // countersteer reaches past the turn-in limit on grip; turn-in may not.
    const {world,v,step,slide}=rig(kind,true);
    slide(100,0,0);for(let i=0;i<20;i++)step({...zero,steer:1});
    const gripLock=v.steerLimits[1];
    assert(v.steer<=gripLock*1.05+.01,`${kind}: turn-in on grip keeps the tire-derived lock (${v.steer} vs ${gripLock})`);
    slide(140,0,0);step();const straightLock=v.steerLimits[0];
    assert(straightLock<v.config.steering*.95,`${kind}: at speed the lock is bounded by the tires' slip (${straightLock})`);
    slide(140,.4,.4);let counter=0,counterLock=0;
    for(let i=0;i<20;i++){step({...zero,steer:-1});counter=Math.max(counter,-v.steer);counterLock=Math.max(counterLock,v.steerLimits[0]);}
    assert(counterLock>straightLock*1.1&&counterLock<=v.config.steering+1e-6,`${kind}: the countersteer lock must open past the straight-line lock (${counterLock} vs ${straightLock})`);
    assert(counter>straightLock*.95,`${kind}: full countersteer input must use the opened lock (${counter})`);
    report.steering.push({kind,...recovery,countersteer:+counter.toFixed(3),countersteerLock:+counterLock.toFixed(3),straightLock:+straightLock.toFixed(3),cornerLock:+gripLock.toFixed(3)});
    world.free();

    // Landings: drop the parked car flat from a height. The bump stop, not the
    // chassis collider, must catch it, and it must settle without bouncing off.
    const chassisBottom=.18+.26;
    for(const drop of [1,2]){
      const {world,v,step}=rig(kind,true);
      const rest=v.position.y,p=v.position;
      v.body.setTranslation({x:p.x,y:rest+drop,z:p.z},true);v.body.setLinvel({x:0,y:0,z:0},true);v.body.setAngvel({x:0,y:0,z:0},true);
      let lowest=Infinity,landed=false,rebound=0,peakForce=0;
      for(let i=0;i<180;i++){
        step();lowest=Math.min(lowest,v.position.y);peakForce=Math.max(peakForce,...v.wheels.map(w=>w.force));
        if(v.wheels.some(w=>w.contact))landed=true;
        if(landed&&i>30)rebound=Math.max(rebound,v.position.y-rest);
      }
      const clearance=lowest-chassisBottom;
      report.landing.push({kind,drop,restHeight:+rest.toFixed(3),lowest:+lowest.toFixed(3),chassisClearance:+clearance.toFixed(3),peakWheelLoadG:+(peakForce/(v.config.mass*9.81)).toFixed(2),rebound:+rebound.toFixed(3),settledError:+Math.abs(v.position.y-rest).toFixed(3)});
      assert(clearance>.05,`${kind}: a ${drop} m drop must not ground the chassis (lowest ${lowest})`);
      assert(rebound<drop*.25,`${kind}: a ${drop} m drop must not bounce back up (${rebound})`);
      assert(Math.abs(v.position.y-rest)<.03,`${kind}: the car must settle back to ride height`);
      world.free();
    }
  }
  console.log(JSON.stringify(report,null,2));
}finally{rmSync(temp,{recursive:true,force:true});}
