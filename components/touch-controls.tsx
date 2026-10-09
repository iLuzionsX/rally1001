"use client";
import {useEffect,useRef,useState,useSyncExternalStore,type CSSProperties,type PointerEvent as Pointer,type ReactNode} from 'react';
import type {RallyEngine} from '@/lib/rally/engine';

export type SteerInput='wheel'|'slider';
export type ControlId='steer'|'pedals'|'handbrake';
/** Control centres as fractions of the play area; missing ones keep the default CSS place. */
export type ControlLayout=Partial<Record<ControlId,{x:number;y:number}>>;
type TouchPrefs={steerInput:SteerInput;layout:ControlLayout;showPerf:boolean};
type Engine=React.RefObject<RallyEngine|null>;
const PREFS_KEY='wildtrail-touch-v1';
const clamp=(v:number,a:number,b:number)=>Math.max(a,Math.min(b,v));

function readPrefs():TouchPrefs{
  const prefs:TouchPrefs={steerInput:'wheel',layout:{},showPerf:false};
  try{
    const p=JSON.parse(localStorage.getItem(PREFS_KEY)??'{}');
    if(p.steerInput==='slider')prefs.steerInput='slider';if(p.showPerf===true)prefs.showPerf=true;
    for(const id of ['steer','pedals','handbrake'] as const){const v=p.layout?.[id];if(v&&Number.isFinite(v.x)&&Number.isFinite(v.y))prefs.layout[id]={x:clamp(v.x,0,1),y:clamp(v.y,0,1)};}
  }catch{}
  return prefs;
}
// One shared store so the settings dialog and the controls agree, read after
// hydration (the server renders the defaults).
const DEFAULT_PREFS:TouchPrefs={steerInput:'wheel',layout:{},showPerf:false};
let current:TouchPrefs|null=null;const listeners=new Set<()=>void>();
const subscribe=(listener:()=>void)=>{listeners.add(listener);return()=>{listeners.delete(listener);};};
const snapshot=()=>current??=readPrefs();
function update(change:Partial<TouchPrefs>){current={...snapshot(),...change};try{localStorage.setItem(PREFS_KEY,JSON.stringify(current));}catch{}listeners.forEach(l=>l());}
/** Steering style, on-screen control positions and the performance readout, remembered per device. */
export function useTouchPrefs(){
  const prefs=useSyncExternalStore(subscribe,snapshot,()=>DEFAULT_PREFS);
  return {...prefs,setSteerInput:(steerInput:SteerInput)=>update({steerInput}),setLayout:(layout:ControlLayout)=>update({layout}),setShowPerf:(showPerf:boolean)=>update({showPerf})};
}

function SteeringWheel({engine}:{engine:Engine}){const drag=useRef<{id:number;x:number}|null>(null),[turn,setTurn]=useState(0);const down=(e:Pointer<HTMLDivElement>)=>{e.preventDefault();e.currentTarget.setPointerCapture(e.pointerId);drag.current={id:e.pointerId,x:e.clientX};setTurn(0);engine.current?.setTouch({steer:0});};const move=(e:Pointer<HTMLDivElement>)=>{if(!drag.current||e.pointerId!==drag.current.id)return;const v=Math.max(-1,Math.min(1,(e.clientX-drag.current.x)/65));setTurn(v);engine.current?.setTouch({steer:-v});};const up=(e:Pointer<HTMLDivElement>)=>{if(drag.current?.id!==e.pointerId)return;drag.current=null;setTurn(0);engine.current?.setTouch({steer:0});};useEffect(()=>()=>{drag.current=null;engine.current?.setTouch({steer:0});},[engine]);return <div className="steering-wheel-body" role="group" aria-label="Steering wheel: drag left or right" onPointerDown={down} onPointerMove={move} onPointerUp={up} onPointerCancel={up} onLostPointerCapture={up}><div className="steering-wheel" style={{transform:`rotate(${turn*115}deg)`}}><svg viewBox="0 0 140 140" aria-hidden="true"><circle cx="70" cy="70" r="58" fill="#0c0e0d18" stroke="#f4f1e9d4" strokeWidth="2.5"/><circle cx="70" cy="70" r="49" fill="none" stroke="#f4f1e963" strokeWidth="1" strokeDasharray="1 7" strokeLinecap="round"/><path d="M70 70 L20 55 M70 70 L120 55 M70 70 L70 123" fill="none" stroke="#f4f1e9bd" strokeWidth="2.5"/><circle cx="70" cy="70" r="15" fill="#111313" stroke="#f4f1e9bd" strokeWidth="2"/><circle cx="70" cy="70" r="3" fill="#ff6056"/><path d="M70 9 L70 18" stroke="#ff6056" strokeWidth="4" strokeLinecap="round"/></svg></div><span className="steer-label">STEER</span></div>;}

/**
 * Horizontal steering slider. The thumb sits under the finger: where you touch
 * the track is how far you steer, so full lock is one swipe and centre is a
 * fixed, findable place. Releasing springs the thumb back to centre.
 */
function SteeringSlider({engine}:{engine:Engine}){
  const track=useRef<HTMLDivElement>(null),id=useRef<number|null>(null),[value,setValue]=useState(0),[held,setHeld]=useState(false);
  const apply=(clientX:number)=>{
    const r=track.current!.getBoundingClientRect(),half=r.width/2-26;
    let v=clamp((clientX-(r.left+r.width/2))/half,-1,1);
    // A small dead zone at centre so a resting thumb drives straight.
    v=Math.sign(v)*Math.max(0,(Math.abs(v)-.05)/.95);
    setValue(v);engine.current?.setTouch({steer:-v});
  };
  const down=(e:Pointer<HTMLDivElement>)=>{e.preventDefault();try{e.currentTarget.setPointerCapture(e.pointerId);}catch{}id.current=e.pointerId;setHeld(true);apply(e.clientX);};
  const move=(e:Pointer<HTMLDivElement>)=>{if(id.current===e.pointerId)apply(e.clientX);};
  const up=(e:Pointer<HTMLDivElement>)=>{if(id.current!==e.pointerId)return;id.current=null;setHeld(false);setValue(0);engine.current?.setTouch({steer:0});};
  useEffect(()=>()=>{engine.current?.setTouch({steer:0});},[engine]);
  return <div ref={track} className={`steering-slider ${held?'held':''}`} role="slider" aria-label="Steering: slide left or right" aria-valuemin={-1} aria-valuemax={1} aria-valuenow={Math.round(value*100)/100} onPointerDown={down} onPointerMove={move} onPointerUp={up} onPointerCancel={up} onLostPointerCapture={up}>
    <span className="slider-arrow left" aria-hidden="true">‹</span><span className="slider-centre" aria-hidden="true"/><span className="slider-arrow right" aria-hidden="true">›</span>
    <span className="slider-fill" aria-hidden="true" style={{left:value<0?`calc(50% + ${value*50}% - ${value*26}px)`:'50%',width:`calc(${Math.abs(value)*50}% - ${Math.abs(value)*26}px)`}}/>
    <span className="slider-thumb" aria-hidden="true" style={{left:`calc(50% + ${value*50}% - ${value*26}px)`}}/>
    <span className="steer-label">STEER</span>
  </div>;
}

function Pedal({kind,label,engine}:{kind:'throttle'|'brake';label:string;engine:Engine}){const [held,setHeld]=useState(false),id=useRef<number|null>(null);const down=(e:Pointer<HTMLButtonElement>)=>{e.preventDefault();id.current=e.pointerId;e.currentTarget.setPointerCapture(e.pointerId);setHeld(true);engine.current?.setTouch({[kind]:1});};const up=(e:Pointer<HTMLButtonElement>)=>{if(id.current!==e.pointerId)return;id.current=null;setHeld(false);engine.current?.setTouch({[kind]:0});};useEffect(()=>()=>{engine.current?.setTouch({[kind]:0});},[engine,kind]);return <button className={`pedal ${kind} ${held?'held':''}`} aria-label={kind==='throttle'?'Hold to accelerate':'Hold to brake; keep holding at rest to reverse'} onPointerDown={down} onPointerUp={up} onPointerCancel={up} onLostPointerCapture={up}><div className="pedal-grooves" aria-hidden="true"><span/><span/><span/></div><span>{label}</span></button>;}

/**
 * Places one on-screen control and, in layout editing, adds a handle over it;
 * dragging the handle moves the control's centre. Positions are kept as
 * fractions of the play area so they survive rotation and other screen sizes.
 */
function Slot({id,layout,editing,onMove,children}:{id:ControlId;layout:ControlLayout;editing:boolean;onMove:(id:ControlId,p:{x:number;y:number})=>void;children:(style:CSSProperties|undefined,handle:ReactNode)=>ReactNode}){
  const drag=useRef<{id:number;dx:number;dy:number}|null>(null);
  const place=layout[id],style:CSSProperties|undefined=place?{left:`${place.x*100}%`,top:`${place.y*100}%`,right:'auto',bottom:'auto',transform:'translate(-50%,-50%)'}:undefined;
  const control=(e:Pointer<HTMLDivElement>)=>e.currentTarget.parentElement!;
  const down=(e:Pointer<HTMLDivElement>)=>{e.preventDefault();e.stopPropagation();e.currentTarget.setPointerCapture(e.pointerId);const r=control(e).getBoundingClientRect();drag.current={id:e.pointerId,dx:e.clientX-(r.left+r.width/2),dy:e.clientY-(r.top+r.height/2)};};
  const move=(e:Pointer<HTMLDivElement>)=>{
    if(drag.current?.id!==e.pointerId)return;
    const el=control(e),area=el.closest('.touch-controls')!.getBoundingClientRect(),r=el.getBoundingClientRect();
    const hw=r.width/2/area.width,hh=r.height/2/area.height;
    onMove(id,{x:clamp((e.clientX-drag.current.dx-area.left)/area.width,hw,1-hw),y:clamp((e.clientY-drag.current.dy-area.top)/area.height,hh,1-hh)});
  };
  const up=(e:Pointer<HTMLDivElement>)=>{if(drag.current?.id===e.pointerId)drag.current=null;};
  return <>{children(style,editing&&<div className="control-drag" role="button" aria-label={`Move ${id==='steer'?'steering':id}`} onPointerDown={down} onPointerMove={move} onPointerUp={up} onPointerCancel={up}/>)}</>;
}

export function TouchControls({engine,steerInput,layout,editing=false,onLayout}:{engine:Engine;steerInput:SteerInput;layout:ControlLayout;editing?:boolean;onLayout?:(layout:ControlLayout)=>void}){
  const move=(id:ControlId,p:{x:number;y:number})=>onLayout?.({...layout,[id]:p});
  const hand=(v:number)=>engine.current?.setTouch({handbrake:v});
  return <div className={`touch-controls ${editing?'editing':''}`}>
    <Slot id="steer" layout={layout} editing={editing} onMove={move}>{(style,handle)=><div className={`steering-zone ${steerInput==='slider'?'slider-zone':''}`} style={style}>{steerInput==='slider'?<SteeringSlider engine={engine}/>:<SteeringWheel engine={engine}/>}{handle}</div>}</Slot>
    <Slot id="pedals" layout={layout} editing={editing} onMove={move}>{(style,handle)=><div className="pedals" style={style}><Pedal kind="brake" label="BRAKE" engine={engine}/><Pedal kind="throttle" label="GAS" engine={engine}/>{handle}</div>}</Slot>
    <Slot id="handbrake" layout={layout} editing={editing} onMove={move}>{(style,handle)=><button className="handbrake" style={style} aria-label="Hold handbrake to slide" onPointerDown={e=>{e.preventDefault();e.currentTarget.setPointerCapture(e.pointerId);hand(1);}} onPointerUp={()=>hand(0)} onPointerCancel={()=>hand(0)} onLostPointerCapture={()=>hand(0)}>SLIDE{handle}</button>}</Slot>
  </div>;
}
