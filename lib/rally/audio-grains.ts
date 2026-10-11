/** Extract sustained, periodic grains from real engine takes. No generated samples.
 * Loudness is a quality gate, never an RPM label. The detected repetition rate
 * calibrates each voice against the vehicle's idle recording. */
export function selectEngineGrain(signal:Float32Array,sampleRate:number,targetHz:number,idle=false){
 const step=Math.max(1,Math.round(sampleRate/4000)),rate=sampleRate/step;
 const frame=Math.min(signal.length,Math.floor(sampleRate*.24));
 const stride=Math.max(1,Math.floor(sampleRate*.18));
 const end=idle?Math.min(signal.length-frame,Math.floor(sampleRate*12)):signal.length-frame;
 const low=24,high=600;
 let best={start:0,frequency:targetHz,score:-Infinity};
 for(let start=0;start<=end;start+=stride){
  const samples:number[]=[];
  let mean=0;
  for(let i=0;i<frame;i+=step){const x=signal[start+i];samples.push(x);mean+=x;}
  mean/=samples.length;
  let energy=0,first=0,last=0;
  for(let i=0;i<samples.length;i++){const x=samples[i]-mean;samples[i]=x;energy+=x*x;if(i<samples.length/2)first+=x*x;else last+=x*x;}
  const rms=Math.sqrt(energy/samples.length);
  if(rms<.003)continue;
  const drift=Math.abs(first-last)/Math.max(energy,1e-9);
  // Reject a recorded rev sweep or pass-by envelope before considering pitch.
  if(drift>.16)continue;
  const correlations:number[]=[];
  const minLag=Math.max(2,Math.floor(rate/high)),maxLag=Math.min(Math.ceil(rate/low),Math.floor(samples.length/3));
  for(let lag=minLag-1;lag<=maxLag+1;lag++){
   let dot=0,a=0,b=0;
   for(let i=0;i<samples.length-lag;i++){dot+=samples[i]*samples[i+lag];a+=samples[i]*samples[i];b+=samples[i+lag]*samples[i+lag];}
   correlations[lag]=dot/Math.max(Math.sqrt(a*b),1e-9);
  }
  for(let lag=minLag;lag<=maxLag;lag++){
   const correlation=correlations[lag];
   if(correlation<correlations[lag-1]||correlation<correlations[lag+1])continue;
   const frequency=rate/lag;
   const score=correlation-drift*.8-Math.abs(Math.log(frequency/targetHz))*.25+Math.min(.06,rms*.3);
   if(score>best.score)best={start,frequency,score};
  }
 }
 // Whole recorded cycles and a one-cycle splice prevent a slow looping rev phrase.
 const period=Math.max(2,Math.round(sampleRate/best.frequency));
 const cycles=Math.max(4,Math.floor(sampleRate*.16/period));
 const length=Math.min(signal.length-best.start,period*(cycles+1));
 return {...best,length,fade:Math.min(period,Math.floor(length/5))};
}
