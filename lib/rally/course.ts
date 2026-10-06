export const WORLD_SIZE=600, GATE_COUNT=12;
export const COURSE_ID='sketch-loop-v1';
export const POND_CENTER={x:205,z:-30};
export type CoursePoint={x:number;y:number;z:number;tx:number;tz:number;width:number;s:number;distance:number};
export const clamp=(v:number,a:number,b:number)=>Math.max(a,Math.min(b,v));
export const damp=(v:number,t:number,k:number,dt:number)=>v+(t-v)*(1-Math.exp(-k*dt));
export function random(seed:number){return()=>{seed=(Math.imul(seed,1664525)+1013904223)|0;return(seed>>>0)/4294967296;};}
export function formatTime(t:number){return Math.floor(t/60).toString().padStart(2,'0')+':'+(t%60).toFixed(2).padStart(5,'0');}
// Trace the supplied sketch clockwise, beginning at its lower-left finish mark.
// Photo coordinates preserve the silhouette; metres keep the rally road driveable.
const points=[
 [315,927],[290,851],[271,790],[278,750],[312,739],[334,727],[335,709],[316,695],
 [282,682],[248,640],[240,601],[249,566],[269,530],[291,506],
 [318,503],[339,520],[350,550],[367,558],[400,546],[440,522],
 [469,503],[490,504],[522,522],[548,524],[581,513],[604,534],
 [616,568],[612,600],[594,614],[539,638],[482,663],[448,688],
 [432,716],[444,741],[481,760],[522,772],[550,784],[568,802],
 [564,825],[545,854],[512,892],[474,932],[433,969],[391,990],
 [355,992],[331,972],
].map(([x,z])=>[(x-429)*.8,(z-748)*.8]);
const wrap=(s:number)=>(s%1+1)%1;
function spline(s:number){const n=points.length,u=wrap(s)*n,i=Math.floor(u),t=u-i,a=points[(i+n-1)%n],b=points[i%n],c=points[(i+1)%n],d=points[(i+2)%n];return [0,1].map(k=>.5*((2*b[k])+(-a[k]+c[k])*t+(2*a[k]-5*b[k]+4*c[k]-d[k])*t*t+(-a[k]+3*b[k]-3*c[k]+d[k])*t*t*t));}
function baseHeight(x:number,z:number){return Math.sin(x*.016)*4.5+Math.cos(z*.018)*3.8+Math.sin((x+z)*.011)*3+Math.sin(x*.037-z*.028)*1.3;}
function ramp(s:number){let h=0;for(const center of [.24,.77]){const d=Math.abs(s-center);if(d<.013)h+=1.6*(1+Math.cos(d/.013*Math.PI))*.5;}return h;}
// Sample by distance so checkpoints, foliage and minimap spacing follow the
// road evenly despite the sketch's closely spaced points at its tight bends.
const trace=Array.from({length:4097},(_,i)=>spline(i/4096));
const traceDistance=[0];
for(let i=1;i<trace.length;i++)traceDistance[i]=traceDistance[i-1]+Math.hypot(trace[i][0]-trace[i-1][0],trace[i][1]-trace[i-1][1]);
const traceLength=traceDistance[4096];
let traceIndex=0;
const path=Array.from({length:721},(_,i)=>{
 const target=i/720*traceLength;
 while(traceIndex<4095&&traceDistance[traceIndex+1]<target)traceIndex++;
 const a=trace[traceIndex],b=trace[traceIndex+1],t=(target-traceDistance[traceIndex])/(traceDistance[traceIndex+1]-traceDistance[traceIndex]);
 return [a[0]+(b[0]-a[0])*t,a[1]+(b[1]-a[1])*t];
});
export const COURSE:CoursePoint[]=path.map((p,i)=>{
 const s=i/720,a=path[(i+719)%720],b=path[(i+1)%720],length=Math.hypot(b[0]-a[0],b[1]-a[1]);
 return {x:p[0],z:p[1],y:baseHeight(...p as [number,number])+ramp(s),tx:(b[0]-a[0])/length,tz:(b[1]-a[1])/length,width:12+Math.sin(s*12)*.65,s,distance:0};
});
for(let i=1;i<COURSE.length;i++)COURSE[i].distance=COURSE[i-1].distance+Math.hypot(COURSE[i].x-COURSE[i-1].x,COURSE[i].z-COURSE[i-1].z);
export const COURSE_LENGTH=COURSE[720].distance;
export function courseAt(s:number):CoursePoint{const u=wrap(s)*720,i=Math.floor(u),t=u-i,a=COURSE[i],b=COURSE[i+1];return {x:a.x+(b.x-a.x)*t,y:a.y+(b.y-a.y)*t,z:a.z+(b.z-a.z)*t,tx:a.tx,tz:a.tz,width:a.width,s:wrap(s),distance:a.distance+(b.distance-a.distance)*t};}
const bins=new Map<string,number[]>();COURSE.forEach((p,i)=>{const key=Math.floor(p.x/18)+','+Math.floor(p.z/18);const v=bins.get(key)??[];v.push(i);bins.set(key,v);});
export function nearestRoad(x:number,z:number){const bx=Math.floor(x/18),bz=Math.floor(z/18);let d=Infinity,index=0;for(let a=-2;a<=2;a++)for(let b=-2;b<=2;b++)for(const i of bins.get((bx+a)+','+(bz+b))??[]){const p=COURSE[i],d2=(p.x-x)**2+(p.z-z)**2;if(d2<d){d=d2;index=i;}}if(d===Infinity)for(let i=0;i<720;i++){const p=COURSE[i],d2=(p.x-x)**2+(p.z-z)**2;if(d2<d){d=d2;index=i;}}const p=COURSE[index];return {point:p,distance:Math.sqrt(d),lateral:(x-p.x)*(-p.tz)+(z-p.z)*p.tx};}
export function roadHeight(p:CoursePoint,lateral:number){const ruts=Math.exp(-(((Math.abs(lateral)-.72)/.18)**2))*.045;return p.y+.09+Math.max(0,1-Math.abs(lateral)/(p.width/2))*.07-ruts;}
export function terrainHeight(x:number,z:number,n=nearestRoad(x,z)){const d=n.distance,w=n.point.width/2,h=baseHeight(x,z);if(d<w+3){const blend=clamp((d-w)/3,0,1);return roadHeight(n.point,n.lateral)-.09+(h-roadHeight(n.point,n.lateral)+.09)*blend*blend*(3-2*blend);}const pond=Math.hypot(x-POND_CENTER.x,z-POND_CENTER.z);if(pond<31){const t=clamp((pond-22)/9,0,1);return -3.5+(h+3.5)*t*t*(3-2*t);}return h;}
export function surfaceAt(x:number,z:number){const n=nearestRoad(x,z),s=n.point.s,off=n.distance>n.point.width/2,mud=!off&&((s>.345&&s<.39)||(s>.64&&s<.682));return {grip:mud?.34:off?.46:.59,rollingResistance:mud?.045:off?.035:.025,rollingDrag:mud?45:off?25:4,mud,label:mud?'WET MUD':off?'FOREST FLOOR':'LOOSE DIRT'};}
export function makeTerrainData(){const count=240,step=WORLD_SIZE/count,vertices=new Float32Array((count+1)**2*3),colors=new Float32Array(vertices.length),indices=new Uint32Array(count**2*6);for(let z=0;z<=count;z++)for(let x=0;x<=count;x++){const px=x*step-WORLD_SIZE/2,pz=z*step-WORLD_SIZE/2,i=(z*(count+1)+x)*3,h=terrainHeight(px,pz);vertices.set([px,h,pz],i);const v=(Math.sin(px*.9+pz*1.3)+1)*.04;colors.set([.78+v,.84+v,.72+v*.5],i);}for(let z=0;z<count;z++)for(let x=0;x<count;x++){const a=z*(count+1)+x;indices.set([a,a+count+1,a+1,a+1,a+count+1,a+count+2],(z*count+x)*6);}return {vertices,colors,indices};}
export function makeRoadData(){const cols=12,vertices=new Float32Array(721*(cols+1)*3),colors=new Float32Array(vertices.length),uvs=new Float32Array(721*(cols+1)*2),indices=new Uint32Array(720*cols*6);for(let i=0;i<=720;i++){const p=COURSE[i];for(let j=0;j<=cols;j++){const lateral=(j/cols-.5)*p.width,k=(i*(cols+1)+j),x=p.x-p.tz*lateral,z=p.z+p.tx*lateral;vertices.set([x,roadHeight(p,lateral),z],k*3);const mud=surfaceAt(x,z).mud,edge=Math.abs(lateral)/(p.width/2),v=.93+.035*Math.sin(p.distance*.06)+.025*Math.sin(x*.5+z*.33)-Math.pow(edge,5)*.09;colors.set(mud?[.66*v,.63*v,.58*v]:[v,v*.98,v*.94],k*3);uvs.set([lateral/3,p.distance/3],k*2);if(i<720&&j<cols){const a=k;indices.set([a,a+1,a+cols+1,a+1,a+cols+2,a+cols+1],(i*cols+j)*6);}}}return {vertices,colors,uvs,indices};}
