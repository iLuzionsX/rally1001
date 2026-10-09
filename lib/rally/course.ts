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
// Numeric bin keys: this lookup runs per wheel per physics step, so it must not allocate strings.
const binKey=(bx:number,bz:number)=>(bx+32768)*65536+(bz+32768);
const bins=new Map<number,number[]>();COURSE.forEach((p,i)=>{const key=binKey(Math.floor(p.x/18),Math.floor(p.z/18));const v=bins.get(key)??[];v.push(i);bins.set(key,v);});
const NO_KNOTS:number[]=[];
function knotNearest(x:number,z:number){const bx=Math.floor(x/18),bz=Math.floor(z/18);let d=Infinity,index=0;for(let a=-2;a<=2;a++)for(let b=-2;b<=2;b++)for(const i of bins.get(binKey(bx+a,bz+b))??NO_KNOTS){const p=COURSE[i],d2=(p.x-x)**2+(p.z-z)**2;if(d2<d){d=d2;index=i;}}if(d===Infinity)for(let i=0;i<720;i++){const p=COURSE[i],d2=(p.x-x)**2+(p.z-z)**2;if(d2<d){d=d2;index=i;}}return index;}
function lerpPoint(a:CoursePoint,b:CoursePoint,t:number):CoursePoint{const tx=a.tx+(b.tx-a.tx)*t,tz=a.tz+(b.tz-a.tz)*t,l=Math.hypot(tx,tz)||1;return {x:a.x+(b.x-a.x)*t,y:a.y+(b.y-a.y)*t,z:a.z+(b.z-a.z)*t,tx:tx/l,tz:tz/l,width:a.width+(b.width-a.width)*t,s:a.s+(b.s-a.s)*t,distance:a.distance+(b.distance-a.distance)*t};}
// Project onto the closer neighbouring segment, so distance and lateral offset
// are continuous. Road detail is keyed on them and must not step at each knot.
export function nearestRoad(x:number,z:number){
 const k=knotNearest(x,z);let best:CoursePoint=COURSE[k],d=Infinity;
 for(const [i,j] of [[k===0?719:k-1,k===0?720:k],[k===720?0:k,k===720?1:k+1]]){
  const a=COURSE[i],b=COURSE[j],ex=b.x-a.x,ez=b.z-a.z,t=clamp(((x-a.x)*ex+(z-a.z)*ez)/(ex*ex+ez*ez||1),0,1);
  const p=lerpPoint(a,b,t),d2=(p.x-x)**2+(p.z-z)**2;if(d2<d){d=d2;best=p;}
 }
 return {point:best,distance:Math.sqrt(d),lateral:(x-best.x)*(-best.tz)+(z-best.z)*best.tx};
}
const smoothNoise=(u:number,seed:number)=>.5+.25*Math.sin(u*1.7+seed)+.25*Math.sin(u*.63+seed*2.1);
// Course distance restarts at the finish line; cross-fade the last stretch
// into the start so the loop has no step in the road.
const SEAM=30;
function seamless(f:(distance:number,lateral:number)=>number,distance:number,lateral:number){
 const w=clamp((distance-(COURSE_LENGTH-SEAM))/SEAM,0,1);
 if(w===0)return f(distance,lateral);
 const t=w*w*(3-2*w);return f(distance,lateral)*(1-t)+f(distance-COURSE_LENGTH,lateral)*t;
}
/** Road shape between the knots: crests/compressions and broken-up bumps.
 * Crests make the car go light; bumps differ left/right so the body rolls. */
export const roadDetail=(distance:number,lateral:number)=>seamless(roadShape,distance,lateral);
function roadShape(distance:number,lateral:number){
 const crestEnvelope=smoothNoise(distance/48,.7)**1.5,roughness=.35+.65*smoothNoise(distance/21,2.3);
 const crests=crestEnvelope*(.42*Math.sin(distance*2*Math.PI/46+1.3)+.15*Math.sin(distance*2*Math.PI/23+.4));
 const bumps=roughness*(.028*Math.sin(distance*1.93+lateral*.8)+.02*Math.sin(distance*1.17-lateral*1.4+2)+.012*Math.sin(distance*3.1+lateral*2.6+.5));
 return crests+bumps;
}
/** Sub-mesh corrugation and stones, sampled by the wheels only. Metres. */
export function surfaceBump(distance:number,lateral:number,off:boolean,mud:boolean,x:number,z:number){
 if(mud)return seamless((d,l)=>.003*Math.sin(d*2.1+l),distance,lateral);
 if(off)return .022*Math.sin(x*2.3+z*.7)*Math.sin(z*1.9-x*.4)+.01*Math.sin(x*4.1-z*3.3);
 return seamless(washboard,distance,lateral);
}
function washboard(distance:number,lateral:number){
 return (.4+.6*smoothNoise(distance/13,5.1))*(.007*Math.sin(distance*4.2+lateral*.5)+.004*Math.sin(distance*6.9-lateral*1.9+1.7));
}
export function roadHeight(p:CoursePoint,lateral:number){const ruts=Math.exp(-(((Math.abs(lateral)-.72)/.18)**2))*.045;return p.y+.09+Math.max(0,1-Math.abs(lateral)/(p.width/2))*.07-ruts+roadDetail(p.distance,lateral);}
export function terrainHeight(x:number,z:number,n=nearestRoad(x,z)){const d=n.distance,w=n.point.width/2,h=baseHeight(x,z);if(d<w+3){const blend=clamp((d-w)/3,0,1);return roadHeight(n.point,n.lateral)-.09+(h-roadHeight(n.point,n.lateral)+.09)*blend*blend*(3-2*blend);}const pond=Math.hypot(x-POND_CENTER.x,z-POND_CENTER.z);if(pond<31){const t=clamp((pond-22)/9,0,1);return -3.5+(h+3.5)*t*t*(3-2*t);}return h;}
// loose: how much material a sliding tire can dig into and push against.
export function surfaceAt(x:number,z:number){const n=nearestRoad(x,z),s=n.point.s,off=n.distance>n.point.width/2,mud=!off&&((s>.345&&s<.39)||(s>.64&&s<.682));return {grip:mud?.34:off?.46:.59,rollingResistance:mud?.045:off?.035:.025,rollingDrag:mud?45:off?25:4,loose:mud?.9:off?.8:.55,bump:surfaceBump(n.point.distance,n.lateral,off,mud,x,z),mud,label:mud?'WET MUD':off?'FOREST FLOOR':'LOOSE DIRT'};}
export function makeTerrainData(){const count=240,step=WORLD_SIZE/count,vertices=new Float32Array((count+1)**2*3),colors=new Float32Array(vertices.length),indices=new Uint32Array(count**2*6);for(let z=0;z<=count;z++)for(let x=0;x<=count;x++){const px=x*step-WORLD_SIZE/2,pz=z*step-WORLD_SIZE/2,i=(z*(count+1)+x)*3,h=terrainHeight(px,pz);vertices.set([px,h,pz],i);const v=(Math.sin(px*.9+pz*1.3)+1)*.04;colors.set([.78+v,.84+v,.72+v*.5],i);}for(let z=0;z<count;z++)for(let x=0;x<count;x++){const a=z*(count+1)+x;indices.set([a,a+count+1,a+1,a+1,a+count+1,a+count+2],(z*count+x)*6);}return {vertices,colors,indices};}
// Dense enough (~0.5 m x 0.75 m) for the collision mesh to carry roadDetail bumps.
export const ROAD_ROWS=2880,ROAD_COLS=16;
export function roadSample(i:number){const u=i/ROAD_ROWS*720,k=Math.min(719,Math.floor(u));return lerpPoint(COURSE[k],COURSE[k+1],u-k);}
export function makeRoadData(){const rows=ROAD_ROWS,cols=ROAD_COLS,vertices=new Float32Array((rows+1)*(cols+1)*3),colors=new Float32Array(vertices.length),uvs=new Float32Array((rows+1)*(cols+1)*2),indices=new Uint32Array(rows*cols*6);for(let i=0;i<=rows;i++){const p=roadSample(i);for(let j=0;j<=cols;j++){const lateral=(j/cols-.5)*p.width,k=(i*(cols+1)+j),x=p.x-p.tz*lateral,z=p.z+p.tx*lateral;vertices.set([x,roadHeight(p,lateral),z],k*3);const mud=surfaceAt(x,z).mud,edge=Math.abs(lateral)/(p.width/2),v=.93+.035*Math.sin(p.distance*.06)+.025*Math.sin(x*.5+z*.33)-Math.pow(edge,5)*.09;colors.set(mud?[.66*v,.63*v,.58*v]:[v,v*.98,v*.94],k*3);uvs.set([lateral/3,p.distance/3],k*2);if(i<rows&&j<cols){const a=k;indices.set([a,a+1,a+cols+1,a+1,a+cols+2,a+cols+1],(i*cols+j)*6);}}}return {vertices,colors,uvs,indices};}
