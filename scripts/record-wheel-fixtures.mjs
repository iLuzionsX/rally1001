import {readFileSync,writeFileSync,mkdirSync,mkdtempSync,rmSync} from 'node:fs';
import {createRequire} from 'node:module';
import {dirname,join} from 'node:path';
import {fileURLToPath} from 'node:url';
import ts from 'typescript';

// Module-level fixtures for porting the wheel contact model. The chassis is put
// at a scripted pose and velocity every step (the world is never stepped after
// setup), so the wheel's raycasts, suspension, tire and impulses are isolated
// from chassis dynamics. Records the inputs, the wheel's state and every impulse
// it applies. Output: reference/golden-traces/wheel.json.
const root=dirname(dirname(fileURLToPath(import.meta.url))),cache=join(root,'node_modules/.cache');
mkdirSync(cache,{recursive:true});const temp=mkdtempSync(join(cache,'rally-wheel-'));
const require=createRequire(import.meta.url),RAPIER=require('@dimforge/rapier3d-compat');
try{
  for(const name of ['course','vehicle-config','wheel-model','vendor/stunt-rally/gravel','vendor/stunt-rally/pacejka']){
    const {outputText}=ts.transpileModule(readFileSync(join(root,'lib/rally',name+'.ts'),'utf8'),{compilerOptions:{module:ts.ModuleKind.CommonJS,target:ts.ScriptTarget.ES2022,esModuleInterop:true}});
    mkdirSync(dirname(join(temp,name+'.cjs')),{recursive:true});
    writeFileSync(join(temp,name+'.cjs'),outputText.replace(/require\("\.\/(.*?)"\)/g,'require("./$1.cjs")'));
  }
  const course=require(join(temp,'course.cjs')),{VEHICLES}=require(join(temp,'vehicle-config.cjs')),{RallyWheel}=require(join(temp,'wheel-model.cjs'));
  await RAPIER.init();
  const dt=1/240,STEPS=900,rand=course.random(4242),between=(a,b)=>a+(b-a)*rand();
  // Static geometry, shared with the Swift test: [halfX,halfY,halfZ, x,y,z, qx,qy,qz,qw, friction].
  const ramp=Math.sin(.06/2),rampW=Math.cos(.06/2);
  const scenery=[
    [60,.5,200, 0,-.5,0, 0,0,0,1, .9],
    [8,.3,5, 0,-.12,-24, ramp,0,0,rampW, .9],
    [8,.04,.6, 0,.04,-41, 0,0,0,1, .9],
  ];
  const surfaces=[course.SURFACES.dirt,course.SURFACES.mud,course.SURFACES.tarmac,course.SURFACES.forest];
  const report={dt,steps:STEPS,scenery,runs:[]};
  for(const kind of ['sti','truck'])for(const [mode,corner] of [['rally',0],['baseline',0],['rally',3]]){
    const c=VEHICLES[kind],world=new RAPIER.World({x:0,y:-9.81,z:0});
    for(const [hx,hy,hz,x,y,z,qx,qy,qz,qw,f] of scenery)world.createCollider(RAPIER.ColliderDesc.cuboid(hx,hy,hz).setTranslation(x,y,z).setRotation({x:qx,y:qy,z:qz,w:qw}).setFriction(f));
    // The chassis as RallyVehicle builds it, for the same centre of mass.
    const body=world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setCanSleep(false).setTranslation(0,5,0));
    body.setAdditionalMassProperties(c.mass,{x:0,y:c.centerOfMass,z:c.centerOfMassForward},{x:c.mass*(c.length*c.length+1.7)/12,y:c.mass*(c.length*c.length+c.width*c.width)/12,z:c.mass*(c.width*c.width+1.7)/12},{x:0,y:0,z:0,w:1},true);
    body.recomputeMassPropertiesFromColliders();
    world.timestep=dt;world.step(); // builds the scene-query structures
    const x=corner%2===0?-c.track/2:c.track/2,z=corner<2?c.front:c.back;
    const wheel=new RallyWheel(world,body,kind,x,z);wheel.handlingMode=mode;
    let surface=surfaces[0];course.surfaceAt=()=>surface;
    const impulses=[],apply=body.applyImpulseAtPoint.bind(body);
    body.applyImpulseAtPoint=(impulse,point,wake)=>{impulses.push([impulse.x,impulse.y,impulse.z,point.x,point.y,point.z]);apply(impulse,point,wake);};
    const steps=[];let distance=0;
    for(let i=0;i<STEPS;i++){
      const t=i*dt,speed=t<.4?t*4:t<2.4?1.6+(t-.4)*11:t<3?23.6-(t-2.4)*30:5.6+Math.sin(t*3)*2;
      distance+=speed*dt;
      if(i%150===0)surface={...surfaces[(i/150)%4],bump:0};
      surface={...surface,bump:Math.sin(distance*2.1)*.012*(surface.loose??0)};
      const yaw=Math.sin(t*1.3)*.25+(t>3?Math.sin(t*9)*.15:0),pitch=Math.sin(t*2.2)*.03,roll=Math.sin(t*1.7+1)*.05;
      // Ride height: a share of suspension length above the tire radius, sweeping
      // from near full droop into the bump stop, with an airborne hop at 2.0-2.3 s.
      const share=.3+.55*(.5+.5*Math.sin(t*4.3)),air=t>2&&t<2.3?Math.sin((t-2)/.3*Math.PI)*.9:0;
      const pos={x:Math.sin(t*.9)*1.5,y:c.radius-c.mount+c.suspension*share+air+(distance>19&&distance<29?(distance-19)*.06:0),z:-distance};
      const cy=Math.cos(yaw/2),sy=Math.sin(yaw/2),cp=Math.cos(pitch/2),sp=Math.sin(pitch/2),cr=Math.cos(roll/2),sr=Math.sin(roll/2);
      const rot={x:sp*cy*cr+cp*sy*sr,y:cp*sy*cr-sp*cy*sr,z:cp*cy*sr-sp*sy*cr,w:cp*cy*cr+sp*sy*sr}; // yaw(Y) * pitch(X) * roll(Z)
      const linvel={x:-Math.sin(yaw)*speed+between(-.6,.6),y:between(-1.2,1.2)+(air?Math.cos((t-2)/.3*Math.PI)*9:0),z:-Math.cos(yaw)*speed};
      const angvel={x:between(-.3,.3),y:Math.cos(t*1.3)*.33+between(-.2,.2),z:between(-.3,.3)};
      body.setTranslation(pos,true);body.setRotation(rot,true);body.setLinvel(linvel,true);body.setAngvel(angvel,true);
      const steer=corner<2?Math.sin(t*1.1)*.35+(t>3.2?Math.sin(t*11)*.2:0):0;
      const drive=t<.3?0:t<2.2?(t<1?900:400)+between(-80,80):t<2.6?0:t<3?0:between(-300,600);
      const brake=t>=2.4&&t<3?(t<2.7?3000:9000):t>3.5?between(0,400):0;
      wheel.steer=steer;impulses.length=0;
      wheel.contactStep(dt);
      const contact={contact:wheel.contact,force:wheel.force,suspensionLength:wheel.suspensionLength};
      wheel.refreshVelocity();wheel.solve(dt,drive,brake);
      steps.push({pos:[pos.x,pos.y,pos.z],rot:[rot.x,rot.y,rot.z,rot.w],linvel:[linvel.x,linvel.y,linvel.z],angvel:[angvel.x,angvel.y,angvel.z],
        surface:{type:surface.type,grip:surface.grip,rollingResistance:surface.rollingResistance,rollingDrag:surface.rollingDrag,loose:surface.loose,bump:surface.bump,mud:!!surface.mud},
        steer,drive,brake,contact,
        state:[wheel.contact?1:0,wheel.force,wheel.suspensionLength,wheel.y,wheel.longVelocity,wheel.sideVelocity,wheel.longSlip,wheel.slipAngle,wheel.relaxedSlip,wheel.relaxedSlipAngle,
          wheel.angularSpeed,wheel.rotation,wheel.aligning,wheel.trail,wheel.plough,wheel.bump,wheel.slip,wheel.loadMass,wheel.friction,
          wheel.normal.x,wheel.normal.y,wheel.normal.z,wheel.contactPoint.x,wheel.contactPoint.y,wheel.contactPoint.z],
        impulses:impulses.map(row=>[...row])});
    }
    world.free();
    report.runs.push({kind,mode,corner,x,z,fields:['contact','force','suspensionLength','y','longVelocity','sideVelocity','longSlip','slipAngle','relaxedSlip','relaxedSlipAngle','angularSpeed','rotation','aligning','trail','plough','bump','slip','loadMass','friction','nx','ny','nz','cx','cy','cz'],steps});
  }
  mkdirSync(join(root,'reference/golden-traces'),{recursive:true});
  writeFileSync(join(root,'reference/golden-traces/wheel.json'),JSON.stringify(report)+'\n');
  console.log(report.runs.map(r=>{
    const s=r.steps,air=s.filter(x=>!x.state[0]).length,stop=s.filter(x=>x.state[0]&&x.state[2]<VEHICLES[r.kind].suspension-VEHICLES[r.kind].suspensionTravel).length,plough=s.filter(x=>x.state[14]>0).length;
    return `${r.kind}/${r.mode}/${r.corner}: airborne ${air}, bump-stop ${stop}, plough ${plough}, impulses ${s.reduce((n,x)=>n+x.impulses.length,0)}`;
  }).join('\n'));
}finally{rmSync(temp,{recursive:true,force:true});}
