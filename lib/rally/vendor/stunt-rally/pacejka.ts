/*!
 * Stunt Rally / VDrift CARTIRE adapter.
 * Derived from src/vdrift/cartire.cpp at
 * https://github.com/stuntrally/stuntrally3/tree/fe3ecd73fff671c3d75ecaeadccea5c5e6afe0d8
 * Copyright Stunt Rally and VDrift contributors. GPL-3.0-or-later.
 * See reference/stunt-rally/LICENSE and the original downloaded sources.
 * Modified: C++ -> TypeScript; omit visual outputs and steering-wheel FFB;
 * symmetric zero-camber forces avoid the donor's small alignment bias;
 * one curve object per surface coefficient set; slip comes in already
 * relaxed by the caller's contact-patch model; finer optimum-slip search;
 * peak-force and slip-at-force-share lookups for the steering rack.
 */
import {stuntRally,tarmac,gravel,mud} from './gravel';

type Coefficients={lateral:number[];longitudinal:number[]};
// Load table rows: every half kN up to the donor's 30 kN bound.
const ROWS=60;
const kN=(load:number)=>Math.min(30,Math.max(0,load)*.001);
const SHARE_STEPS=[.5,.6,.7,.8,.85,.9,.95,.97,.99,1];

/** One tire on one surface: the donor's Pacejka '89 style pure-slip curves,
 * Beckman combined slip and traction cap, with load tables baked once. */
export class TireCurve{
  private hats:{sigma:number;alpha:number}[]=[];
  private shares:number[][]=[];
  constructor(private c:Coefficients){
    // Donor's optimum-slip search per half-kN load, over the full slip range,
    // and the pure slip angle (deg) at which side force reaches a share of peak.
    for(let i=0;i<ROWS;i++){
      const fz=(i+1)*.5,side=Array.from({length:1801},(_,n)=>this.fy(n*.05,fz,1));
      let sigma=.005,maxX=0,alpha=.05,maxY=0;
      for(let n=1;n<=400;n++){const x=this.fx(n*.005,fz,1);if(x>maxX){maxX=x;sigma=n*.005;}}
      for(let n=1;n<=1800;n++)if(side[n]>maxY){maxY=side[n];alpha=n*.05;}
      this.hats.push({sigma,alpha});
      this.shares.push(SHARE_STEPS.map(share=>{let n=0;while(n<1800&&side[n]<share*maxY-1e-9)n++;return n*.05;}));
    }
  }
  /** Pure longitudinal force (N) at slip ratio sigma, load fz (kN), friction mu. */
  fx(sigma:number,fz:number,mu:number){
    const b=this.c.longitudinal,d=(b[1]*fz+b[2])*fz*mu;
    const stiffness=(b[3]*fz+b[4])*Math.exp(-b[5]*fz)/(b[0]*(b[1]*fz+b[2]));
    const e=b[6]*fz*fz+b[7]*fz+b[8],s=100*sigma+b[9]*fz+b[10],sb=s*stiffness;
    return d*Math.sin(b[0]*Math.atan(sb+e*(Math.atan(sb)-sb)));
  }
  /** Pure lateral force (N) at slip angle alpha (deg), load fz (kN), friction mu. */
  fy(alpha:number,fz:number,mu:number){
    const a=this.c.lateral,d=(a[1]*fz+a[2])*fz*mu;
    const stiffness=a[3]*Math.sin(2*Math.atan(fz/a[4]))/(a[0]*(a[1]*fz+a[2])*fz);
    const e=a[6]*fz+a[7],sb=alpha*stiffness;
    return d*Math.sin(a[0]*Math.atan(sb+e*(Math.atan(sb)-sb)));
  }
  private row(load:number){
    const index=Math.max(0,Math.min(ROWS-1,kN(load)/.5-1)),lo=Math.floor(index);
    return {lo,hi:Math.min(lo+1,ROWS-1),t:index-lo};
  }
  /** Slip ratio and slip angle (deg) of peak pure force at this load. */
  optimumSlip(load:number){
    const {lo,hi,t}=this.row(load),h=this.hats;
    return {sigmaHat:h[lo].sigma*(1-t)+h[hi].sigma*t,alphaHat:h[lo].alpha*(1-t)+h[hi].alpha*t};
  }
  /** Slip angle (rad) at which the pure side force reaches `share` (.5-1) of
   * its peak at this load. The curve's shape is independent of friction. */
  slipAtShare(load:number,share:number){
    const {lo,hi,t}=this.row(load);
    const target=Math.max(SHARE_STEPS[0],Math.min(SHARE_STEPS[SHARE_STEPS.length-1],share));
    let k=0;while(k<SHARE_STEPS.length-2&&target>SHARE_STEPS[k+1])k++;
    const u=(target-SHARE_STEPS[k])/(SHARE_STEPS[k+1]-SHARE_STEPS[k]);
    const at=(row:number[])=>row[k]*(1-u)+row[k+1]*u;
    return (at(this.shares[lo])*(1-t)+at(this.shares[hi])*t)*Math.PI/180;
  }
  /** Peak pure side force (N) at this load and friction. */
  lateralPeak(load:number,mu:number){const a=this.c.lateral,fz=kN(load);return Math.abs((a[1]*fz+a[2])*fz*mu);}
  /** Peak pure longitudinal force (N) at this load and friction. */
  longitudinalPeak(load:number,mu:number){const b=this.c.longitudinal,fz=kN(load);return Math.abs((b[1]*fz+b[2])*fz*mu);}
  /** Small-slip stiffnesses at this load and friction: N per unit slip ratio
   * and N per radian of slip angle. */
  stiffness(load:number,mu:number){
    const fz=kN(load);
    return {longitudinal:this.fx(.001,fz,mu)/.001,lateral:this.fy(.05,fz,mu)/(.05*Math.PI/180)};
  }
  /** CARTIRE GetForce: Beckman combined slip, then the donor's traction cap.
   * `slipRatio` and `slipAngle` (rad) are the contact patch's slip. */
  force(load:number,mu:number,slipRatio:number,slipAngle:number){
    const fz=kN(load);
    if(fz<1e-6)return {long:0,side:0};
    const {sigmaHat,alphaHat}=this.optimumSlip(load);
    const s=slipRatio/sigmaHat,angle=slipAngle*180/Math.PI/alphaHat,rho=Math.max(Math.hypot(s,angle),.0001);
    let long=s/rho*this.fx(rho*sigmaHat,fz,mu),side=angle/rho*this.fy(rho*alphaHat,fz,mu);
    const sum=Math.abs(long)+Math.abs(side),longFactor=sum>1?Math.abs(long)/sum:1;
    const maximum=this.longitudinalPeak(load,mu)*longFactor+this.lateralPeak(load,mu)*(1-longFactor);
    if(sum>maximum){const scale=maximum/sum;long*=scale;side*=scale;}
    return {long,side};
  }
}

/** Tire curves per surface type. */
export const TIRES={tarmac:new TireCurve(tarmac),gravel:new TireCurve(gravel),mud:new TireCurve(mud)};
/** The donor's single gravel curve, kept for comparison checks. */
export const STUNT_RALLY_GRAVEL=new TireCurve(stuntRally);
