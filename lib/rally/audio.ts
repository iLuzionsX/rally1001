/**
 * Recording-only rally audio. No oscillator, procedural engine tone, or generated noise.
 * The Subaru WRX and Ford pickup use separate real vehicle recordings.
 * Engine grains are prepared from credited real takes and served locally.
 * Asset credits and licenses: public/assets/audio/CREDITS.txt
 */
import {clamp} from './course';
import engineGrains from './engine-grains.json';
import type {VehicleKind} from './vehicle-config';

type EngineRole='idle'|'pull-low'|'pull-mid'|'pull-high'|'coast-low'|'coast-high';
type Voice={kind:VehicleKind;role:EngineRole;anchor:number;source:AudioBufferSourceNode;gain:GainNode;filter?:BiquadFilterNode};
type Loop={source:AudioBufferSourceNode;gain:GainNode;filter?:BiquadFilterNode};

const SURFACE_RECORDINGS={
 gravel:'https://raw.githubusercontent.com/yashimosh/border-run/main/public/sfx/tires_gravel.mp3',
 scrub:'/assets/audio/squeal.mp3',
 ambience:'https://raw.githubusercontent.com/pbojinov/jungle-sketchbook/main/public/audio/jungle-ambience.mp3',
};
const smooth=(value:number,min:number,max:number)=>{const x=clamp((value-min)/(max-min),0,1);return x*x*(3-2*x);};
// Independent recording buses retain the engine-forward mix without lifting road noise.
const ENGINE_BUS_GAIN=4.3;
const ENVIRONMENT_BUS_GAIN=.85;

/** Select steady, audible sections by recorded loudness, then splice a short
 * crossfade into each loop. Every output sample is derived from original audio. */
function makeRecordedLoop(context:AudioContext,recording:AudioBuffer,rank:number,seconds=1.85,earlyOnly=false):AudioBuffer{
 const rate=recording.sampleRate,n=Math.max(100,Math.min(recording.length,Math.floor(seconds*rate)));
 const signal=recording.getChannelData(0),candidates:{start:number;level:number;stability:number}[]=[];
 const stride=Math.max(1,Math.floor(rate*.42)),sampleStride=101;
 const lastStart=earlyOnly?Math.min(recording.length-n,Math.floor(rate*Math.min(20,recording.duration*.4))):recording.length-n;
 for(let start=0;start<=lastStart;start+=stride){
  let energy=0,a=0,b=0,first=0,last=0;
  for(let i=0;i<n;i+=sampleStride){
   const x=signal[start+i],power=x*x;energy+=power;
   if(i<n/2){a+=power;first++;}else{b+=power;last++;}
  }
  const level=Math.sqrt(energy/Math.ceil(n/sampleStride));
  const stability=Math.abs(Math.sqrt(a/Math.max(1,first))-Math.sqrt(b/Math.max(1,last)));
  if(level>.0015)candidates.push({start,level,stability});
 }
 // At most three seconds of low-level material from the original first section
 // are preferred for stationary idle. Motion ranks sample across the full take.
 const stable=candidates.filter(x=>x.stability<x.level*.7);
 const all=(stable.length>5?stable:candidates).sort((a,b)=>a.level-b.level);
 const position=all.length?all[Math.round(clamp(rank,0,1)*(all.length-1))].start:0;
 return spliceLoop(context,recording,position,n,Math.min(Math.floor(rate*.11),Math.floor(n/8)));
}

function spliceLoop(context:AudioContext,recording:AudioBuffer,position:number,n:number,fade:number){
 const rate=recording.sampleRate;
 const size=n-fade;
 const buffer=context.createBuffer(recording.numberOfChannels,size,rate);
 for(let channel=0;channel<recording.numberOfChannels;channel++){
  const input=recording.getChannelData(channel),output=buffer.getChannelData(channel);
  for(let i=0;i<size;i++){
   if(i<fade){
    const t=i/fade;
    output[i]=input[position+i]*t+input[position+size+i]*(1-t);
   }else output[i]=input[position+i];
  }
 }
 // Normalize different microphone levels without introducing generated sound.
 let sum=0,peak=0,count=0;
 for(let channel=0;channel<buffer.numberOfChannels;channel++){
  const data=buffer.getChannelData(channel);
  for(let i=0;i<data.length;i++){sum+=data[i]*data[i];peak=Math.max(peak,Math.abs(data[i]));count++;}
 }
 const rms=Math.sqrt(sum/Math.max(1,count));
 const gain=Math.min(12,.82/Math.max(peak,.0001),.18/Math.max(rms,.0001));
 for(let channel=0;channel<buffer.numberOfChannels;channel++){
  const data=buffer.getChannelData(channel);for(let i=0;i<data.length;i++)data[i]*=gain;
 }
 return buffer;
}

export class RallyAudio{
 context:AudioContext|null=null;
 master:GainNode|null=null;
 /** Dedicated recording-only engine bus; engine is dominant over the roadbed. */
 private engineBus:GainNode|null=null;
 private environmentBus:GainNode|null=null;
 enabled=true;
 engineVolume=1;
 environmentVolume=.7;
 kind:VehicleKind='sti';
 private voices:Voice[]=[];
 private surface=new Map<keyof typeof SURFACE_RECORDINGS,Loop>();
 private loading:Promise<void>|null=null;
 private decoded=new Map<string,Promise<AudioBuffer>>();
 private active=false;
 private lastRPM=900;
 private lastThrottle=0;
 private lastSpeed=0;
 private lastSlip=0;
 private lastSurface='LOOSE DIRT';
 private lastBrake=0;
 private contact=1;
 private shiftCut=false;
 private blip=0;
 private limiterCut=false;
 private failureCount=0;
 /** Number of failed remote recordings; useful when diagnosing unavailable hosts. */
 get unavailableRecordings(){return this.failureCount;}
 /** The engine remains silent rather than substituting fake synthesized engine audio. */
 get recordedEngineReady(){return this.voices.some(v=>v.kind===this.kind);}
 /** Live engine control; a zero value silences only the engine. */
 setEngineVolume(value:number){
  this.engineVolume=clamp(Number.isFinite(value)?value:1,0,1);
  if(this.context&&this.engineBus)this.engineBus.gain.setTargetAtTime(ENGINE_BUS_GAIN*this.engineVolume,this.context.currentTime,.045);
 }
 /** Controls recorded gravel, tire scrubbing and outdoor ambience. */
 setEnvironmentVolume(value:number){
  this.environmentVolume=clamp(Number.isFinite(value)?value:.7,0,1);
  if(this.context&&this.environmentBus)this.environmentBus.gain.setTargetAtTime(ENVIRONMENT_BUS_GAIN*this.environmentVolume,this.context.currentTime,.045);
 }
 private async getRecording(urls:string[]):Promise<AudioBuffer>{
  const context=this.context;
  if(!context)throw new Error('Audio context not initialized');
  const cacheKey=urls[0];
  const existing=this.decoded.get(cacheKey);
  if(existing)return existing;
  const promise=(async()=>{
   let lastError:unknown;
   for(const url of urls){
    try{
     const response=await fetch(url,{credentials:'omit'});
     if(!response.ok)throw new Error('Audio HTTP '+response.status);
     const data=await response.arrayBuffer();
     if(context.state==='closed')throw new Error('Audio stopped');
     return await context.decodeAudioData(data);
    }catch(error){lastError=error;}
   }
   throw lastError??new Error('Recorded audio unavailable');
  })();
  this.decoded.set(cacheKey,promise);
  return promise;
 }
 private playLoop(buffer:AudioBuffer,volume=0,filterType?:BiquadFilterType,cutoff=0,output?:AudioNode):Loop{
  const ctx=this.context!,source=ctx.createBufferSource(),gain=ctx.createGain();
  source.buffer=buffer;source.loop=true;gain.gain.value=volume;
  let filter:BiquadFilterNode|undefined;
  if(filterType){
   filter=ctx.createBiquadFilter();filter.type=filterType;filter.frequency.value=cutoff;
   source.connect(filter);filter.connect(gain);
  }else source.connect(gain);
  gain.connect(output??this.master!);
  source.start();
  return {source,gain,filter};
 }
 private async loadEngine(kind:VehicleKind){
  const context=this.context!;
  const layers=engineGrains[kind];
  const buffers=await Promise.all(layers.map(layer=>this.getRecording([layer.url])));
  if(context.state==='closed'||this.context!==context)return;
  for(let i=0;i<layers.length;i++){
   const layer=layers[i];
   const loop=this.playLoop(buffers[i],0,'lowpass',2500,this.engineBus!);
   this.voices.push({...layer,...loop,role:layer.role as EngineRole,kind});
  }
  this.mix();
 }
 private async loadSurface(name:keyof typeof SURFACE_RECORDINGS){
  const context=this.context!;
  const decoded=await this.getRecording([SURFACE_RECORDINGS[name]]);
  if(context.state==='closed')return;
  const looped=name==='ambience'?decoded:makeRecordedLoop(context,decoded,.65,3.5);
  const voice=this.playLoop(looped,0,name==='scrub'?'highpass':'lowpass',name==='scrub'?320:3700,this.environmentBus!);
  this.surface.set(name,voice);
  this.mix();
 }
 private beginLoading(){
  if(this.loading)return;
  const jobs:Promise<unknown>[]=[
   this.loadEngine('sti'),this.loadEngine('truck'),
   this.loadSurface('gravel'),this.loadSurface('scrub'),this.loadSurface('ambience'),
  ];
  this.loading=Promise.allSettled(jobs).then(results=>{
   this.failureCount=results.filter(r=>r.status==='rejected').length;
   for(const result of results)if(result.status==='rejected')console.warn('Recorded rally sound unavailable',result.reason);
  });
 }
 /** Called only after a direct user action so iOS/Safari can unlock WebAudio. */
 async start(){
  if(!this.context){
   const context=this.context=new AudioContext();
   const limiter=context.createDynamicsCompressor();
   limiter.threshold.value=-11;limiter.knee.value=9;limiter.ratio.value=4;
   limiter.attack.value=.003;limiter.release.value=.16;
   const master=this.master=context.createGain();
   master.gain.value=0;master.connect(limiter);limiter.connect(context.destination);
   // Louder default engine mix, routed independently from the environment.
   this.engineBus=context.createGain();this.engineBus.gain.value=ENGINE_BUS_GAIN*this.engineVolume;this.engineBus.connect(master);
   this.environmentBus=context.createGain();this.environmentBus.gain.value=ENVIRONMENT_BUS_GAIN*this.environmentVolume;this.environmentBus.connect(master);
  }
  if(this.context.state==='suspended')await this.context.resume();
  this.beginLoading();
 }
 /** Countdown chirp is UI feedback, not an engine tone. */
 cue(frequency=520){
  const ctx=this.context;if(!ctx||!this.enabled)return;
  const time=ctx.currentTime,osc=ctx.createOscillator(),gain=ctx.createGain();
  osc.type='sine';osc.frequency.value=frequency;
  gain.gain.setValueAtTime(.0001,time);
  gain.gain.linearRampToValueAtTime(.075,time+.012);
  gain.gain.exponentialRampToValueAtTime(.0001,time+.14);
  osc.connect(gain);gain.connect(ctx.destination);osc.start(time);osc.stop(time+.15);
  osc.onended=()=>{osc.disconnect();gain.disconnect();};
 }
 update(rpm:number,throttle:number,speed:number,slip:number,active:boolean,surface='LOOSE DIRT',brake=0,contact=1,shiftCut=false,blip=0,limiterCut=false){
  this.lastRPM=rpm;this.lastThrottle=throttle;this.lastSpeed=speed;this.lastSlip=slip;
  this.lastSurface=surface;this.lastBrake=brake;this.active=active;
  this.contact=clamp(contact,0,1);this.shiftCut=shiftCut;this.blip=blip;this.limiterCut=limiterCut;
  this.mix();
 }
 private mix(){
  const ctx=this.context,master=this.master;
  if(!ctx||!master)return;
  const t=ctx.currentTime;
  master.gain.setTargetAtTime(this.enabled&&this.active?.94:0,t,.13);
  const rpm=clamp(this.lastRPM,700,8500);
  const pedal=clamp(Math.max(this.lastThrottle,this.blip),0,1);
  const throttle=this.shiftCut||this.limiterCut?0:pedal;
  const brake=clamp(this.lastBrake,0,1),speed=Math.max(0,this.lastSpeed);
  // Normalize the RPM blend so crossings never pile up unrelated engine takes.
  // All voices follow crank RPM, including coasting and stationary throttle blips.
  const selected=this.voices.filter(v=>v.kind===this.kind);
  const weights=selected.map(v=>Math.exp(-Math.pow(Math.log(rpm/v.anchor)/.48,2)));
  const sum=weights.reduce((a,b)=>a+b,0)||1;
  const load=.22+.68*throttle+.08*brake;
  const cut=this.shiftCut||this.limiterCut?.3:1;
  for(const voice of this.voices){
   const index=selected.indexOf(voice);
   const pull=voice.role.startsWith('pull'),coast=voice.role.startsWith('coast');
   const mode=pull?.12+.88*throttle:coast?1-.86*throttle:1;
   // Normalize each load family separately; idle fades out above launch RPM.
   const family=selected.reduce((total,v,i)=>total+(v.role.startsWith('pull')===pull&&v.role.startsWith('coast')===coast?weights[i]:0),0)||sum;
   const idle=1-smooth(rpm,1100,1900);
   const weight=index<0?0:voice.role==='idle'?idle*.72:weights[index]/family*mode*load*(1-idle);
   voice.gain.gain.setTargetAtTime(weight*cut,t,.018);
   // No fixed-rate plateau: pitch keeps moving throughout the rev range.
   voice.source.playbackRate.setTargetAtTime(clamp(rpm/voice.anchor,.15,12),t,.018);
   voice.filter?.frequency.setTargetAtTime(950+rpm*.55+throttle*3300,t,.025);
  }
  // Road contact texture grows with ground speed. Sliding multiplies gravel
  // crunch, while recorded scrubbing activates only after tire saturation.
  const loose=/dirt|gravel|mud|sand|trail/i.test(this.lastSurface);
  const speedGain=smooth(speed,.6,18);
  const slip=clamp(this.lastSlip,0,3);
  const gravel=this.contact*speedGain*(loose?.25:.025)*(1+Math.min(1,slip)*.6);
  const scrub=this.contact*smooth(slip,.28,1.15)*smooth(speed,1.5,11)*(loose?.08:.27);
  this.surface.get('gravel')?.gain.gain.setTargetAtTime(gravel,t,.09);
  this.surface.get('scrub')?.gain.gain.setTargetAtTime(scrub,t,.065);
  this.surface.get('ambience')?.gain.gain.setTargetAtTime(.025*(1-.65*throttle),t,.25);
  const rate=this.surface.get('gravel')?.source.playbackRate;
  rate?.setTargetAtTime(clamp(.7+speed*.026,.78,1.55),t,.12);
 }
 dispose(){
  for(const voice of this.voices){try{voice.source.stop();}catch{}voice.source.disconnect();voice.gain.disconnect();voice.filter?.disconnect();}
  for(const loop of this.surface.values()){try{loop.source.stop();}catch{}loop.source.disconnect();loop.gain.disconnect();loop.filter?.disconnect();}
  this.voices=[];this.surface.clear();this.decoded.clear();this.loading=null;
  this.engineBus?.disconnect();this.engineBus=null;
  this.environmentBus?.disconnect();this.environmentBus=null;
  this.master?.disconnect();this.master=null;
  const ctx=this.context;this.context=null;if(ctx)void ctx.close();
 }
}
