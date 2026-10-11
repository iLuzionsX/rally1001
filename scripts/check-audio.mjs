import assert from 'node:assert/strict';
import {readFileSync,writeFileSync,mkdirSync,mkdtempSync,rmSync} from 'node:fs';
import {createRequire} from 'node:module';
import {dirname,join} from 'node:path';
import {fileURLToPath} from 'node:url';
import ts from 'typescript';
const root=dirname(dirname(fileURLToPath(import.meta.url))),cache=join(root,'node_modules/.cache');
mkdirSync(cache,{recursive:true});const temp=mkdtempSync(join(cache,'rally-audio-'));
class Param{value=0;timeConstant=0;setTargetAtTime(v,t,tau){this.value=v;this.timeConstant=tau;}}
class Node{gain=new Param();frequency=new Param();playbackRate=new Param();connect(){}disconnect(){}start(){}stop(){}}
class Context{state='running';currentTime=0;destination=new Node();createGain(){return new Node();}createDynamicsCompressor(){const n=new Node();for(const k of ['threshold','knee','ratio','attack','release'])n[k]=new Param();return n;}createBiquadFilter(){return new Node();}createBufferSource(){return new Node();}async close(){this.state='closed';}}
try{
 const src=readFileSync(join(root,'lib/rally/audio.ts'),'utf8').replace("import {clamp} from './course';",'const clamp=(v:number,a:number,b:number)=>Math.max(a,Math.min(b,v));');
 const output=ts.transpileModule(src,{compilerOptions:{module:ts.ModuleKind.CommonJS,target:ts.ScriptTarget.ES2022,esModuleInterop:true}}).outputText;
 writeFileSync(join(temp,'audio.cjs'),output);
 writeFileSync(join(temp,'engine-grains.json'),readFileSync(join(root,'lib/rally/engine-grains.json')));
 const {RallyAudio}=createRequire(import.meta.url)(join(temp,'audio.cjs'));
 globalThis.AudioContext=Context;
 const audio=new RallyAudio();audio.beginLoading=()=>{};await audio.start();
 // Exercise the real voice-loading graph using each committed WAV, not tone fixtures.
 audio.getRecording=async urls=>{
  const bytes=readFileSync(join(root,'public',urls[0]));
  assert.equal(bytes.toString('ascii',0,4),'RIFF');
  let rate=0,size=0;
  for(let at=12;at+8<=bytes.length;){const name=bytes.toString('ascii',at,at+4),n=bytes.readUInt32LE(at+4);if(name==='fmt ')rate=bytes.readUInt32LE(at+12);if(name==='data')size=n;at+=8+n+(n%2);}
  assert(rate>0&&size>0);assert(size/2/rate<.25,'engine must not contain a long rev phrase');
  return {sampleRate:rate,length:size/2,duration:size/2/rate};
 };
 await Promise.all(['sti','truck'].map(k=>audio.loadEngine(k)));
 for(const name of ['gravel','scrub','ambience'])audio.surface.set(name,{source:new Node(),gain:new Node()});
 for(const kind of ['sti','truck']){
  audio.kind=kind;
  const voices=()=>audio.voices.filter(v=>v.kind===kind),level=()=>voices().reduce((s,v)=>s+v.gain.gain.value,0);
  audio.update(3000,1,20,0,true);const low=voices().map(v=>v.source.playbackRate.value),loaded=level();
  audio.update(6000,1,20,0,true);voices().forEach((v,i)=>assert(Math.abs(v.source.playbackRate.value/low[i]-2)<.001,'pitch must track RPM, without plateau'));
  audio.update(3000,0,20,0,true);const coast=level();assert(loaded>coast*2,'load must sound stronger than lift-off');
  audio.update(3000,1,20,0,true,'LOOSE DIRT',0,1,true);assert(level()<loaded*.15,'upshift must audibly cut combustion');
  audio.update(3000,0,20,0,true,'LOOSE DIRT',0,1,false,.75);assert(level()>coast*2,'downshift blip must respond without pedal input');
  audio.update(3000,1,20,1,true,'LOOSE DIRT',0,1);assert(audio.surface.get('gravel').gain.gain.value>0);assert(audio.surface.get('scrub').gain.gain.value>0);
  audio.update(3000,1,20,1,true,'LOOSE DIRT',0,0);assert.equal(audio.surface.get('gravel').gain.gain.value,0);assert.equal(audio.surface.get('scrub').gain.gain.value,0);
  audio.update(950,0,0,0,true);assert(level()>0,'stationary idle must remain audible');assert.equal(audio.surface.get('gravel').gain.gain.value,0);
  assert(audio.voices.filter(v=>v.kind!==kind).every(v=>v.gain.gain.value===0),'other vehicle must be silent');
  assert(voices().every(v=>v.gain.gain.timeConstant<=.025),'controls must respond quickly');
  console.log(`${kind}: recorded grains, idle, RPM, throttle lift, shift cut, blip, wheel contact and vehicle isolation passed`);
 }
 const environment=audio.environmentBus.gain.value;audio.setEngineVolume(0);assert.equal(audio.engineBus.gain.value,0);assert.equal(audio.environmentBus.gain.value,environment);
 audio.setEnvironmentVolume(0);assert.equal(audio.environmentBus.gain.value,0);
 audio.enabled=false;audio.update(3000,1,20,0,true);assert.equal(audio.master.gain.value,0);
 audio.enabled=true;audio.update(3000,1,20,0,false);assert.equal(audio.master.gain.value,0);
 await audio.dispose();assert.equal(audio.context,null);assert.equal(audio.voices.length,0);
 console.log('Independent sliders, mute, pause and disposal passed');
}finally{rmSync(temp,{recursive:true,force:true});}
