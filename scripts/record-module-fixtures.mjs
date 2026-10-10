import {readFileSync,writeFileSync,mkdirSync,mkdtempSync,rmSync} from 'node:fs';
import {createRequire} from 'node:module';
import {dirname,join} from 'node:path';
import {fileURLToPath} from 'node:url';
import ts from 'typescript';

// Module-level fixtures for porting the steering rack and drivetrain: each module
// driven by seeded input sequences, recording the inputs as applied and its state
// after every call, so a port can replay them exactly.
// Output: reference/golden-traces/modules.json.
const root=dirname(dirname(fileURLToPath(import.meta.url))),cache=join(root,'node_modules/.cache');
mkdirSync(cache,{recursive:true});const temp=mkdtempSync(join(cache,'rally-modules-'));
const require=createRequire(import.meta.url);
try{
  for(const name of ['course','vehicle-config','steering','drive-model','vendor/ecctrl/CurveLUT','vendor/stunt-rally/gravel','vendor/stunt-rally/pacejka','vendor/stunt-rally/abs','vendor/stunt-rally/engine-friction','vendor/stunt-rally/differential']){
    const {outputText}=ts.transpileModule(readFileSync(join(root,'lib/rally',name+'.ts'),'utf8'),{compilerOptions:{module:ts.ModuleKind.CommonJS,target:ts.ScriptTarget.ES2022,esModuleInterop:true}});
    mkdirSync(dirname(join(temp,name+'.cjs')),{recursive:true});
    writeFileSync(join(temp,name+'.cjs'),outputText.replace(/require\("\.\/(.*?)"\)/g,'require("./$1.cjs")'));
  }
  const {random,clamp}=require(join(temp,'course.cjs')),{VEHICLES}=require(join(temp,'vehicle-config.cjs'));
  const {SteeringRack,steerBounds}=require(join(temp,'steering.cjs')),{EcctrlDriveModel}=require(join(temp,'drive-model.cjs'));
  const {bakeCurveLUT,evaluateCurveLUT}=require(join(temp,'vendor/ecctrl/CurveLUT.cjs'));
  const {engineFrictionTorque}=require(join(temp,'vendor/stunt-rally/engine-friction.cjs'));
  const {differentialTorques}=require(join(temp,'vendor/stunt-rally/differential.cjs'));
  const {TireABS}=require(join(temp,'vendor/stunt-rally/abs.cjs')),{TIRES}=require(join(temp,'vendor/stunt-rally/pacejka.cjs'));
  const report={};

  // Ecctrl curves: the drive model's two baked tables, sampled on and between knots.
  const torque=bakeCurveLUT([{x:0,y:.82},{x:.2,y:1.06},{x:.5,y:1.22},{x:.75,y:1.14},{x:.94,y:1},{x:1.04,y:0}],128);
  const steer=bakeCurveLUT([{x:0,y:1,r_out:0},{x:.2,y:1,r_in:0,r_out:0},{x:1,y:.4,r_in:0}],50);
  const xs=Array.from({length:241},(_,i)=>i/200-.1);
  report.curves={torque:{lut:[...torque.lut],samples:xs.map(x=>[x,evaluateCurveLUT(x,torque)])},steer:{lut:[...steer.lut],samples:xs.map(x=>[x,evaluateCurveLUT(x,steer)])}};

  const rand=random(1001),between=(a,b)=>a+(b-a)*rand();
  report.engineFriction=Array.from({length:400},()=>{
    const args=[between(-900,900),rand()<.3?1:between(-15,15),between(5000,8000),between(600,1000),between(200,700),rand()<.2?0:rand()<.2?1:between(-.2,1.2),between(0,1)];
    return {args,out:engineFrictionTorque(...args)};
  });
  report.differential=Array.from({length:400},()=>{
    const config={antiSlip:between(0,1000),torqueSensitivity:rand()<.3?0:between(0,.5),coastFactor:between(0,1),split:between(.3,.7)};
    const args=[rand()<.1?0:between(-3000,3000),between(-200,200),between(-200,200),between(.5,8),between(.5,8),rand()<.5?1/240:1/960];
    if(rand()<.1)args[2]=args[1];
    return {args,config,out:differentialTorques(...args,config)};
  });
  // ABS: one wheel braking through a slip sweep on each surface.
  report.abs=['tarmac','gravel','mud'].map(surface=>{
    const abs=new TireABS(),steps=[];
    for(let i=0;i<300;i++){
      const demand=i<20?0:i>260?.05:clamp(between(-.2,1.3),0,1),slip=Math.sin(i*.07)*.45+between(-.05,.05),load=between(1500,6000),speed=i>230?between(0,8):between(4,40);
      steps.push({demand,slip,load,speed,out:abs.update(demand,slip,load,speed,TIRES[surface]),active:abs.active});
    }
    return {surface,steps};
  });

  report.steeringRack=[];report.steerBounds=[];report.driveModel=[];
  for(const kind of ['sti','truck']){
    const c=VEHICLES[kind],dt=1/240;
    // Rack: driver targets steps, ramps and releases against a tire torque that
    // follows the rack (an aligning spring with noise), with grip changes.
    const rack=new SteeringRack(c.steeringRack,c.steering),steps=[];
    let target=0,grip=1;
    for(let i=0;i<2400;i++){
      if(i%240===0)target=clamp(between(-1.2,1.2),-1,1)*c.steering;
      if(i%600===300)grip=rand()<.5?between(0,.3):1;
      if(i===1800)rack.rate=4;
      const aligning=-rack.angle*between(800,2500)+between(-150,150)+(i>2000?between(-3000,3000):0);
      const angle=rack.step(dt,target,aligning,grip);
      steps.push([target,aligning,grip,angle,rack.rate,rack.hands]);
    }
    report.steeringRack.push({kind,dt,fields:['target','aligning','grip','angle','rate','hands'],steps});
    for(let i=0;i<200;i++){const args=[between(-.8,.8),between(0,.6),c.steering];report.steerBounds.push({args,out:steerBounds(...args)});}

    // Drive model: a lumped driveline (four wheels on one shaft with road load)
    // through launch, upshifts, lift-off, kick-down, wheelspin, handbrake and
    // reverse. updateTransmission runs each chassis step, engineStep four times.
    const p=c.powertrain,model=new EcctrlDriveModel(p,c.radius,c.engineBraking),calls=[];
    const weights=[c.frontDrive/2,c.frontDrive/2,(1-c.frontDrive)/2,(1-c.frontDrive)/2],inertia=c.wheelInertia*4;
    let shaft=0,reverse=false,syncedAt=-1,syncShaft=0;
    for(let i=0;i<3600;i++){
      const t=i*dt;
      reverse=t>12.5;
      const throttle=t<6?1:t<7?0:t<9?(t<8?.35:1):t<10?0:t<12?clamp(between(-.2,1.2),0,1):t<12.5?0:.8;
      const disengaged=t>=10&&t<10.6;
      const spin=t>3&&t<3.6?between(1.05,1.6):1;
      const wheels=weights.map(weight=>({angularSpeed:shaft*spin*between(.97,1.03),weight,longSlip:spin>1.2?between(.8,2):between(-.05,.05)}));
      if(i===2400&&syncedAt<0){syncedAt=i;syncShaft=shaft;model.syncEngine(shaft,reverse);}
      model.updateTransmission(wheels,dt,reverse);
      const transmission={gearIndex:model.gearIndex,gearboxRPM:model.gearboxRPM,driveRatio:model.driveRatio,shiftCooldownTimer:model.shiftCooldownTimer};
      const engine=[];
      for(let n=0;n<4;n++){
        const h=dt/4,input=shaft,wheelTorque=model.engineStep(h,throttle,input,reverse,inertia,disengaged);
        const road=(t>=10&&t<10.6?2500:0)*Math.sign(shaft)+.43*(shaft*c.radius)**2*c.radius*Math.sign(shaft)+c.mass*9.81*.02*c.radius*Math.tanh(shaft*4);
        shaft+=(wheelTorque-road)/(inertia+c.mass*c.radius*c.radius)*h;
        engine.push([h,input,wheelTorque,model.engineSpeed,model.engineRPM,model.clutchTorque,model.clutchSlip,model.combustionTorque,model.shiftTimer,model.blip,model.limiterTimer]);
      }
      calls.push({throttle,reverse,disengaged,sync:syncedAt===i?syncShaft:null,wheels:wheels.map(w=>[w.angularSpeed,w.weight,w.longSlip]),transmission:[transmission.gearIndex,transmission.gearboxRPM,transmission.driveRatio,transmission.shiftCooldownTimer],engine,steeringLimit:model.steeringLimit(Math.abs(shaft*c.radius),c.maxSpeed,c.steering)});
    }
    report.driveModel.push({kind,dt,inertia,fields:{wheels:['angularSpeed','weight','longSlip'],transmission:['gearIndex','gearboxRPM','driveRatio','shiftCooldownTimer'],engine:['h','shaft','wheelTorque','engineSpeed','engineRPM','clutchTorque','clutchSlip','combustionTorque','shiftTimer','blip','limiterTimer']},calls});
  }
  mkdirSync(join(root,'reference/golden-traces'),{recursive:true});
  writeFileSync(join(root,'reference/golden-traces/modules.json'),JSON.stringify(report)+'\n');
  const gears=report.driveModel.map(d=>d.kind+': gears '+[...new Set(d.calls.map(c=>c.transmission[0]+1))].join(',')+', peak rpm '+Math.max(...d.calls.flatMap(c=>c.engine.map(e=>e[4]))).toFixed(0));
  console.log(gears.join(' | '));
}finally{rmSync(temp,{recursive:true,force:true});}
