/*!
 * Stunt Rally / VDrift CARTIRE adapter.
 * Derived from src/vdrift/cartire.cpp at
 * https://github.com/stuntrally/stuntrally3/tree/fe3ecd73fff671c3d75ecaeadccea5c5e6afe0d8
 * Copyright Stunt Rally and VDrift contributors. GPL-3.0-or-later.
 * See reference/stunt-rally/LICENSE and the original downloaded sources.
 * Modified: C++ -> TypeScript; omit visual outputs and steering-wheel FFB;
 * symmetric zero-camber forces avoid the donor's small alignment bias;
 * configurable cornering stiffness preserves the donor's force ceiling;
 * optional relaxed lateral slip feeds combined forces; telemetry stays raw.
 */
import {lateral as a,longitudinal as b} from './gravel';

const fx=(sigma:number,fz:number,mu:number)=>{
  const d=(b[1]*fz+b[2])*fz*mu;
  const stiffness=(b[3]*fz+b[4])*Math.exp(-b[5]*fz)/(b[0]*(b[1]*fz+b[2]));
  const e=b[6]*fz*fz+b[7]*fz+b[8],s=100*sigma+b[9]*fz+b[10],sb=s*stiffness;
  return d*Math.sin(b[0]*Math.atan(sb+e*(Math.atan(sb)-sb)));
};
const fy=(alpha:number,fz:number,mu:number)=>{
  const d=(a[1]*fz+a[2])*fz*mu;
  const stiffness=a[3]*Math.sin(2*Math.atan(fz/a[4]))/(a[0]*(a[1]*fz+a[2])*fz);
  const e=a[6]*fz+a[7],sb=alpha*stiffness;
  return d*Math.sin(a[0]*Math.atan(sb+e*(Math.atan(sb)-sb)));
};

// Donor's optimum-slip search and half-kN load table, baked once at startup.
const hats=Array.from({length:60},(_,i)=>{
  const fz=(i+1)*.5;let sigma=.01,alpha=.1,maxX=0,maxY=0;
  for(let n=1;n<200;n++){
    const s=n*.01,angle=n*.1,x=fx(s,fz,1),y=fy(angle,fz,1);
    if(x>maxX){maxX=x;sigma=s;}if(y>maxY){maxY=y;alpha=angle;}
  }
  return {sigma,alpha};
});

export function optimumGravelSlip(load:number){
  const fz=Math.min(30,Math.max(0,load)*.001);
  const index=Math.max(0,Math.min(hats.length-1,fz/.5-1)),lo=Math.floor(index),hi=Math.min(lo+1,hats.length-1),t=index-lo;
  const sigmaHat=hats[lo].sigma*(1-t)+hats[hi].sigma*t,alphaHat=hats[lo].alpha*(1-t)+hats[hi].alpha*t;
  return {sigmaHat,alphaHat};
}

export function gravelForce(load:number,mu:number,longVelocity:number,sideVelocity:number,patchSpeed:number,lateralResponse=1,effectiveSlipAngle?:number,surface?:{longSlipScale:number;angleScale:number;combined:number}){
  const fz=Math.min(30,Math.max(0,load)*.001);
  if(fz<1e-6)return {long:0,side:0,slipRatio:0,slipAngle:0};
  const {sigmaHat,alphaHat}=optimumGravelSlip(load);
  const denominator=Math.max(Math.abs(longVelocity),.01);
  const sigma=(patchSpeed-longVelocity)/denominator,alpha=-Math.atan2(sideVelocity,denominator)*180/Math.PI;
  // CARTIRE GetForce: Beckman combined slip, then the donor's traction cap.
  const s=sigma/(sigmaHat*(surface?.longSlipScale??1)),angle=(effectiveSlipAngle===undefined?alpha:effectiveSlipAngle*180/Math.PI)*lateralResponse/(alphaHat*(surface?.angleScale??1)),rho=Math.max(Math.hypot(s,angle),.0001);
  let long=s/rho*fx(rho*sigmaHat,fz,mu),side=angle/rho*fy(rho*alphaHat,fz,mu);
  const sum=Math.abs(long)+Math.abs(side),longFactor=sum>1?Math.abs(long)/sum:1;
  const maximum=Math.abs((b[1]*fz+b[2])*fz*mu)*longFactor+Math.abs((a[1]*fz+a[2])*fz*mu)*(1-longFactor);
  const legacyScale=sum>maximum?maximum/sum:1;
  // Blend the donor's conservative diamond into an elliptical combined-force
  // budget. This rounds the brake/throttle-to-cornering handoff without adding
  // force beyond either pure-axis ceiling or applying artificial yaw torque.
  if(surface){
    const peakX=Math.abs((b[1]*fz+b[2])*fz*mu),peakY=Math.abs((a[1]*fz+a[2])*fz*mu);
    const ellipse=Math.hypot(long/Math.max(peakX,.001),side/Math.max(peakY,.001));
    const ellipseScale=ellipse>1?1/ellipse:1;
    const scale=legacyScale+(ellipseScale-legacyScale)*surface.combined;
    long*=scale;side*=scale;
  }else{long*=legacyScale;side*=legacyScale;}
  return {long,side,slipRatio:sigma,slipAngle:alpha*Math.PI/180};
}
