/**
 * Recording-only rally audio. No oscillator, procedural engine tone, or generated noise.
 * The Subaru WRX and Ford pickup use separate real vehicle recordings.
 * Freesound preview URLs have a primary CDN and original-host fallback.
 * Asset credits and licenses: public/assets/audio/CREDITS.txt
 */
import {clamp} from './course';
import type {VehicleKind} from './vehicle-config';

type EngineRole='idle'|'pull-low'|'pull-mid'|'pull-high'|'coast-low'|'coast-high';
type VehicleSources={idle:string[];motion:string[]};
type Voice={kind:VehicleKind;role:EngineRole;anchor:number;source:AudioBufferSourceNode;gain:GainNode};
type Loop={source:AudioBufferSourceNode;gain:GainNode};

const VEHICLE_RECORDINGS:Record<VehicleKind,VehicleSources>={
 suv:{
  // Recorded 2003 Subaru Impreza WRX, ulose2piranha, Freesound #273334, CC0.
  idle:['https://cdn.freesound.org/previews/273/273334_4168822-hq.mp3','https://freesound.org/data/previews/273/273334_4168822-hq.mp3'],
  motion:['https://cdn.freesound.org/previews/273/273334_4168822-hq.mp3','https://freesound.org/data/previews/273/273334_4168822-hq.mp3'],
 },
 truck:{
  // Recorded Ford pickup: start/idle #332499 and driving away #332503,
  // rambler52, Freesound, CC BY 4.0. (Vintage pickup, not an F-150.)
  idle:['https://cdn.freesound.org/previews/332/332499_2291385-hq.mp3','https://freesound.org/data/previews/332/332499_2291385-hq.mp3'],
  motion:['https://cdn.freesound.org/previews/332/332503_2291385-hq.mp3','https://freesound.org/data/previews/332/332503_2291385-hq.mp3'],
 },
};
const SURFACE_RECORDINGS={
 gravel:'https://raw.githubusercontent.com/yashimosh/border-run/main/public/sfx/tires_gravel.mp3',
 scrub:'/assets/audio/squeal.mp3',
 ambience:'https://raw.githubusercontent.com/pbojinov/jungle-sketchbook/main/public/audio/jungle-ambience.mp3',
};
const ENGINE_LAYERS:{role:EngineRole;anchor:number;rank:number}[]=[
 {role:'idle',anchor:950,rank:.15},
 {role:'pull-low',anchor:2100,rank:.42},
 {role:'pull-mid',anchor:4000,rank:.7},
 {role:'pull-high',anchor:6100,rank:.95},
 {role:'coast-low',anchor:2350,rank:.30},
 {role:'coast-high',anchor:5000,rank:.62},
];
const smooth=(value:number,min:number,max:number)=>{const x=clamp((value-min)/(max-min),0,1);return x*x*(3-2*x);};

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
 const fade=Math.min(Math.floor(rate*.11),Math.floor(n/8));
 const size=n-fade;
 const buffer=context.createBuffer(recording.numberOfChannels,size,rate);
 for(let channel=0;channel<recording.numberOfChannels;channel++){
  const input=recording.getChannelData(channel),output=buffer.getChannelData(channel);
  for(let i=0;i<size;i++){
   if(i<fade){
    const t=i/fade;
    output[i]=input[position+i]*Math.sin(t*Math.PI/2)+input[position+size+i]*Math.cos(t*Math.PI/2);
   }else output[i]=input[position+i];
  }
 }
 // Normalize different microphone levels without introducing generated sound.
 let sum=0,peak=0,count=0;
 for(let channel=0;channel<buffer.numberOfChannels;channel++){
  const data=buffer.getChannelData(channel);
  for(let i=0;i<data.length;i+=61){sum+=data[i]*data[i];peak=Math.max(peak,Math.abs(data[i]));count++;}
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
 enabled=true;
 kind:VehicleKind='suv';
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
 private failureCount=0;
 /** Number of failed remote recordings; useful when diagnosing unavailable hosts. */
 get unavailableRecordings(){return this.failureCount;}
 /** The engine remains silent rather than substituting fake synthesized engine audio. */
 get recordedEngineReady(){return this.voices.some(v=>v.kind===this.kind);}
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
 private playLoop(buffer:AudioBuffer,volume=0,filterType?:BiquadFilterType,cutoff=0):Loop{
  const ctx=this.context!,source=ctx.createBufferSource(),gain=ctx.createGain();
  source.buffer=buffer;source.loop=true;gain.gain.value=volume;
  if(filterType){
   const filter=ctx.createBiquadFilter();filter.type=filterType;filter.frequency.value=cutoff;
   source.connect(filter);filter.connect(gain);
  }else source.connect(gain);
  gain.connect(this.master!);
  source.start();
  return {source,gain};
 }
 private async loadEngine(kind:VehicleKind){
  const context=this.context!;
  const recordings=VEHICLE_RECORDINGS[kind];
  const [idle,motion]=await Promise.all([
   this.getRecording(recordings.idle),this.getRecording(recordings.motion),
  ]);
  if(context.state==='closed')return;
  for(const layer of ENGINE_LAYERS){
   const take=layer.role==='idle'?idle:motion;
   const buffer=makeRecordedLoop(context,take,layer.rank,1.85,layer.role==='idle');
   const {source,gain}=this.playLoop(buffer,0,'lowpass',layer.role==='idle'?2500:5500);
   this.voices.push({...layer,kind,source,gain});
  }
  this.mix();
 }
 private async loadSurface(name:keyof typeof SURFACE_RECORDINGS){
  const context=this.context!;
  const decoded=await this.getRecording([SURFACE_RECORDINGS[name]]);
  if(context.state==='closed')return;
  const looped=name==='ambience'?decoded:makeRecordedLoop(context,decoded,.65,3.5);
  const voice=this.playLoop(looped,0,name==='scrub'?'highpass':'lowpass',name==='scrub'?320:3700);
  this.surface.set(name,voice);
  this.mix();
 }
 private beginLoading(){
  if(this.loading)return;
  const jobs:Promise<unknown>[]=[
   this.loadEngine('suv'),this.loadEngine('truck'),
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
   limiter.threshold.value=-7;limiter.ratio.value=5;
   limiter.attack.value=.003;limiter.release.value=.16;
   const master=this.master=context.createGain();
   master.gain.value=0;master.connect(limiter);limiter.connect(context.destination);
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
 update(rpm:number,throttle:number,speed:number,slip:number,active:boolean,surface='LOOSE DIRT',brake=0){
  this.lastRPM=rpm;this.lastThrottle=throttle;this.lastSpeed=speed;this.lastSlip=slip;
  this.lastSurface=surface;this.lastBrake=brake;this.active=active;
  this.mix();
 }
 private mix(){
  const ctx=this.context,master=this.master;
  if(!ctx||!master)return;
  const t=ctx.currentTime;
  master.gain.setTargetAtTime(this.enabled&&this.active?.78:0,t,.13);
  const rpm=clamp(this.lastRPM,700,7300),throttle=clamp(this.lastThrottle,0,1);
  const brake=clamp(this.lastBrake,0,1),speed=Math.max(0,this.lastSpeed);
  const idle=1-smooth(rpm,950,1900);
  const on=clamp(.06+throttle*.94,0,1),off=clamp((1-throttle)*(.65+brake*.25),0,1);
  for(const voice of this.voices){
   const selected=voice.kind===this.kind?1:0;
   let weight=0;
   if(voice.role==='idle')weight=idle*.65;
   else{
    const width=voice.role==='pull-low'?1900:voice.role==='pull-mid'?2200:voice.role==='pull-high'?2200:voice.role==='coast-low'?2400:2900;
    const rWeight=clamp(1-Math.abs(rpm-voice.anchor)/width,0,1);
    weight=rWeight*(voice.role.startsWith('pull')?on*.33:off*.20)*(1-idle*.7);
   }
   voice.gain.gain.setTargetAtTime(selected*weight,t,.09);
   voice.source.playbackRate.setTargetAtTime(clamp(rpm/voice.anchor,.68,1.38),t,.085);
  }
  // Road contact texture grows with ground speed. Sliding multiplies gravel
  // crunch, while recorded scrubbing activates only after tire saturation.
  const loose=/dirt|gravel|mud|sand|trail/i.test(this.lastSurface);
  const speedGain=smooth(speed,.6,18);
  const slip=clamp(this.lastSlip,0,3);
  const gravel=speedGain*(loose?.31:.07)*(1+Math.min(1,slip)*.6);
  const scrub=smooth(slip,.28,1.15)*smooth(speed,1.5,11)*.32;
  this.surface.get('gravel')?.gain.gain.setTargetAtTime(gravel,t,.09);
  this.surface.get('scrub')?.gain.gain.setTargetAtTime(scrub,t,.065);
  this.surface.get('ambience')?.gain.gain.setTargetAtTime(.19,t,.5);
  const rate=this.surface.get('gravel')?.source.playbackRate;
  rate?.setTargetAtTime(clamp(.7+speed*.026,.78,1.55),t,.12);
 }
 dispose(){
  for(const voice of this.voices){try{voice.source.stop();}catch{}voice.source.disconnect();voice.gain.disconnect();}
  for(const loop of this.surface.values()){try{loop.source.stop();}catch{}loop.source.disconnect();loop.gain.disconnect();}
  this.voices=[];this.surface.clear();this.decoded.clear();
  this.master?.disconnect();this.master=null;
  const ctx=this.context;this.context=null;if(ctx)void ctx.close();
 }
}
