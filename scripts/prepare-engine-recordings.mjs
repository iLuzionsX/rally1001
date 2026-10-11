// Rebuild local sustained grains from the credited original takes. Requires ffmpeg.
// Usage: node scripts/prepare-engine-recordings.mjs <directory containing sti.mp3,
// truck-idle.mp3 and truck-motion.mp3>. Sources are listed in audio/CREDITS.txt.
import {readFileSync,writeFileSync,mkdirSync,mkdtempSync,rmSync} from 'node:fs';
import {execFileSync} from 'node:child_process';
import {createHash} from 'node:crypto';
import {createRequire} from 'node:module';
import {dirname,join,resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import ts from 'typescript';
const root=dirname(dirname(fileURLToPath(import.meta.url)));
const input=resolve(process.argv[2]??'');
if(!process.argv[2])throw Error('Pass the source-recording directory.');
mkdirSync(join(root,'node_modules/.cache'),{recursive:true});
const temp=mkdtempSync(join(root,'node_modules/.cache/engine-grains-'));
try{
 const compiled=ts.transpileModule(readFileSync(join(root,'lib/rally/audio-grains.ts'),'utf8'),{compilerOptions:{module:ts.ModuleKind.CommonJS,target:ts.ScriptTarget.ES2022}}).outputText;
 writeFileSync(join(temp,'grains.cjs'),compiled);
 const {selectEngineGrain}=createRequire(import.meta.url)(join(temp,'grains.cjs'));
 const rate=44100,decoded={};
 const sources={};
 for(const name of ['sti','truck-idle','truck-motion']){
  const path=join(input,name+'.mp3'),raw=join(temp,name+'.f32');
  sources[name]=createHash('sha256').update(readFileSync(path)).digest('hex');
  execFileSync('ffmpeg',['-v','error','-i',path,'-f','f32le','-ac','1','-ar',String(rate),raw]);
  const bytes=readFileSync(raw);decoded[name]=new Float32Array(bytes.buffer.slice(bytes.byteOffset,bytes.byteOffset+bytes.byteLength));
 }
 const layers=[['idle',950],['pull-low',2100],['pull-mid',4000],['pull-high',6100],['coast-low',2350],['coast-high',5000]];
 const manifest={};const output=join(root,'public/assets/audio/engines');mkdirSync(output,{recursive:true});
 for(const kind of ['sti','truck']){
  const idle=decoded[kind==='sti'?'sti':'truck-idle'],motion=decoded[kind==='sti'?'sti':'truck-motion'];
  const reference=selectEngineGrain(idle,rate,48,true).frequency;
  manifest[kind]=[];
  for(const [role,anchor] of layers){
   const signal=role==='idle'?idle:motion;
   const grain=selectEngineGrain(signal,rate,reference*anchor/950,role==='idle');
   if(!Number.isFinite(grain.score))throw Error(`No steady recorded grain for ${kind}/${role}`);
   const size=grain.length-grain.fade,data=new Float32Array(size);
   for(let i=0;i<size;i++){const t=i/grain.fade;data[i]=i<grain.fade?signal[grain.start+i]*t+signal[grain.start+size+i]*(1-t):signal[grain.start+i];}
   let energy=0,peak=0;for(const x of data){energy+=x*x;peak=Math.max(peak,Math.abs(x));}
   const gain=Math.min(12,.82/Math.max(peak,.0001),.18/Math.max(Math.sqrt(energy/size),.0001));
   for(let i=0;i<size;i++)data[i]*=gain;
   const raw=join(temp,'loop.f32');writeFileSync(raw,Buffer.from(data.buffer));
   const name=`${kind}-${role}.wav`;
   execFileSync('ffmpeg',['-y','-v','error','-f','f32le','-ar',String(rate),'-ac','1','-i',raw,'-c:a','pcm_s16le',join(output,name)]);
   manifest[kind].push({role,anchor:Math.round(950*grain.frequency/reference),url:'/assets/audio/engines/'+name});
   console.log(kind,role,`source ${grain.start/rate}s, loop ${(size/rate).toFixed(3)}s, anchor ${manifest[kind].at(-1).anchor}, score ${grain.score.toFixed(3)}`);
  }
 }
 writeFileSync(join(root,'lib/rally/engine-grains.json'),JSON.stringify(manifest,null,2)+'\n');
 writeFileSync(join(output,'SOURCES.json'),JSON.stringify({sampleRate:rate,sourceSHA256:sources},null,2)+'\n');
}finally{rmSync(temp,{recursive:true,force:true});}
