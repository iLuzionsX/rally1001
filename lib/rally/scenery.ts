import * as THREE from 'three';
import RAPIER from '@dimforge/rapier3d-compat';
import {courseAt,nearestRoad,terrainHeight,roadHeight,random,roadSample,ROAD_ROWS} from './course';
import type {Quality} from './world';

type Placement={x:number;y:number;z:number;scale:number;yScale:number;angle:number};
type Cell={root:THREE.Group;x:number;z:number;range:number};
export type Scenery={cells:Cell[];wind:{value:number};props:number;update:(position:THREE.Vector3,time:number,quality:Quality)=>void};

/** Spatially batched photogrammetry, with matching wind in color and shadow passes. */
export function createScenery(scene:THREE.Scene,physics:RAPIER.World,models:THREE.Group[]):Scenery {
  const rand=random(82041),wind={value:0},cells:Cell[]=[];
  let props=0;
  const sets:Placement[][]=models.map(()=>[]);
  function place(index:number,s:number,offset:number,scale:number,yScale=1) {
    const p=courseAt(s),x=p.x-p.tz*offset,z=p.z+p.tx*offset,n=nearestRoad(x,z);
    if(n.distance<n.point.width/2+(index<2?3.5:.65))return;
    const y=terrainHeight(x,z,n);
    sets[index].push({x,y:y-.04,z,scale,yScale,angle:rand()*Math.PI*2});
    if(index<2&&n.distance<25){
      physics.createCollider(RAPIER.ColliderDesc.cylinder(scale*.35,scale*.07).setTranslation(x,y+scale*.35,z));
      props++;
    }
  }
  // Two planted layers: an arching canopy and smaller trees beneath it.
  for(let i=0;i<280;i++) {
    const s=i/280+rand()*.002,side=i%2?1:-1;
    place(i%2,s,side*(11+rand()*22),2.6+rand()*2.9,1.35+rand()*.3);
  }
  for(let i=0;i<110;i++)place(i%2,rand(),(i%2?1:-1)*(38+rand()*28),3.6+rand()*3.2,1.4);
  for(let i=0;i<1300;i++)place(2,rand(),(rand()>.5?1:-1)*(7.5+rand()*15),.65+rand()*1.3);
  for(let i=0;i<2200;i++)place(3,rand(),(rand()>.5?1:-1)*(6.9+rand()*12),.7+rand()*1.4);
  for(let i=0;i<220;i++)place(4,rand(),(rand()>.5?1:-1)*(9+rand()*22),20+rand()*45,.75+rand()*.5);

  const dummy=new THREE.Object3D();
  models.forEach((model,index)=>{
    if(index<2){const bounds=new THREE.Box3().setFromObject(model);const height=bounds.max.y-bounds.min.y;model.scale.setScalar(3.5/height);model.position.y=-bounds.min.y*3.5/height;}
    model.updateMatrixWorld(true);
    const partitions=new Map<string,Placement[]>();
    for(const p of sets[index]) {
      const key=Math.floor(p.x/48)+','+Math.floor(p.z/48);
      const bucket=partitions.get(key)??[];bucket.push(p);partitions.set(key,bucket);
    }
    const sources:THREE.Mesh[]=[];model.traverse(o=>{if(o instanceof THREE.Mesh)sources.push(o);});
    // Fern and grass assets include separate variants laid out for display.
    // Select one per cell rather than repeating that artificial grid.
    for(const [key,placements] of partitions) {
      const [cx,cz]=key.split(',').map(Number),root=new THREE.Group();
      const materialSources=index===2||index===3?[sources[(Math.abs(cx*7+cz*11))%sources.length]]:sources;
      for(const source of materialSources) {
        const geo=source.geometry.clone().applyMatrix4(source.matrixWorld);
        if(index===2||index===3){const box=new THREE.Box3().setFromBufferAttribute(geo.attributes.position as THREE.BufferAttribute);geo.translate(-(box.min.x+box.max.x)/2,-box.min.y,-(box.min.z+box.max.z)/2);}
        const base=(Array.isArray(source.material)?source.material[0]:source.material) as THREE.MeshStandardMaterial;
        const mat=base.clone();mat.envMapIntensity=.75;mat.metalness=0;
        const leaf=/leaves|fern|grass/i.test(mat.name);
        mat.transparent=false;mat.depthWrite=true;mat.alphaTest=leaf?.4:0;mat.side=leaf?THREE.DoubleSide:THREE.FrontSide;
        mat.color.multiplyScalar(index<2?.91:index===4?.88:1.05);
        mat.roughness=Math.max(.8,mat.roughness);
        const deform=(shader:{uniforms:Record<string,unknown>;vertexShader:string})=>{
          shader.uniforms.windTime=wind;
          shader.vertexShader='uniform float windTime;\n'+shader.vertexShader;
          shader.vertexShader=shader.vertexShader.replace('#include <begin_vertex>',`#include <begin_vertex>
            float phase=windTime*.8+instanceMatrix[3].x*.13+instanceMatrix[3].z*.17;
            float bend=sin(phase+position.y*1.4)*.028*max(position.y,0.);
            transformed.x+=bend; transformed.z+=bend*.43;`);
        };
        if(leaf)mat.onBeforeCompile=deform;
        const batch=new THREE.InstancedMesh(geo,mat,placements.length);
        batch.castShadow=index<3||index===4;batch.receiveShadow=true;
        if(leaf){const depth=new THREE.MeshDepthMaterial({depthPacking:THREE.RGBADepthPacking,map:mat.map,alphaTest:mat.alphaTest,side:THREE.DoubleSide});depth.onBeforeCompile=deform;batch.customDepthMaterial=depth;}
        placements.forEach((p,i)=>{
          dummy.position.set(p.x,p.y,p.z);dummy.rotation.set(0,p.angle,0);dummy.scale.set(p.scale,p.scale*p.yScale,p.scale);dummy.updateMatrix();batch.setMatrixAt(i,dummy.matrix);
          batch.setColorAt(i,new THREE.Color().setRGB(.83+rand()*.14,.86+rand()*.14,.8+rand()*.15));
        });
        batch.computeBoundingSphere();root.add(batch);
      }
      scene.add(root);cells.push({root,x:cx*48+24,z:cz*48+24,range:index<2?125:index===4?105:72});
    }
  });
  return {cells,wind,props,update:(p,time,quality)=>{
    wind.value=time;
    const factor=quality==='high'?1.15:quality==='battery'?.7:1;
    for(const c of cells)c.root.visible=Math.hypot(c.x-p.x,c.z-p.z)<c.range*factor+34;
  }};
}

/** Track furniture gives the road a designed circuit silhouette. */
export function addTrackside(scene:THREE.Scene,physics:RAPIER.World) {
  const metal=new THREE.MeshStandardMaterial({color:'#c5c7bc',metalness:.72,roughness:.44});
  const dark=new THREE.MeshStandardMaterial({color:'#242a26',metalness:.65,roughness:.55});
  const railGeometry=new THREE.BoxGeometry(1,.42,.11),postGeometry=new THREE.BoxGeometry(.08,1.12,.1);
  const stretches=[[.025,.067,-1],[.092,.16,1],[.205,.235,-1],[.407,.458,1],[.493,.545,-1],[.728,.767,1],[.865,.905,1],[.962,.999,-1]];
  const segments:ReturnType<typeof courseAt>[]=[];
  for(const [start,end,side] of stretches)for(let s=start;s<end;s+=.0028) {
    const p=courseAt(s),o=(p.width/2+1.25)*side;
    segments.push({...p,x:p.x-p.tz*o,z:p.z+p.tx*o});
  }
  const rails=new THREE.InstancedMesh(railGeometry,metal,segments.length),posts=new THREE.InstancedMesh(postGeometry,dark,segments.length);
  const dummy=new THREE.Object3D();
  segments.forEach((p,i)=>{
    dummy.rotation.y=Math.atan2(-p.tz,p.tx);dummy.position.set(p.x,p.y+.8,p.z);dummy.scale.set(3.25,1,1);dummy.updateMatrix();rails.setMatrixAt(i,dummy.matrix);
    const yaw=dummy.rotation.y;physics.createCollider(RAPIER.ColliderDesc.cuboid(1.625,.23,.09).setTranslation(p.x,p.y+.8,p.z).setRotation({x:0,y:Math.sin(yaw/2),z:0,w:Math.cos(yaw/2)}).setFriction(.5));
    dummy.position.y=p.y+.52;dummy.scale.setScalar(1);dummy.updateMatrix();posts.setMatrixAt(i,dummy.matrix);
  });
  for(const mesh of [rails,posts]){mesh.castShadow=mesh.receiveShadow=true;mesh.computeBoundingSphere();scene.add(mesh);}

  const rubber=new THREE.MeshStandardMaterial({color:'#161a18',roughness:.94});
  const tireGeometry=new THREE.TorusGeometry(.39,.13,8,16);tireGeometry.rotateX(Math.PI/2);
  const start=courseAt(0),camp=new THREE.Group();camp.position.set(start.x,start.y,start.z);camp.rotation.y=Math.atan2(-start.tx,-start.tz);
  for(const side of [-1,1])for(let i=0;i<7;i++)for(let j=0;j<3;j++) {
    const tire=new THREE.Mesh(tireGeometry,rubber);tire.position.set(side*8.2,j*.27+.17,5+i*.9);tire.castShadow=tire.receiveShadow=true;camp.add(tire);
  }
  const canvas=document.createElement('canvas');canvas.width=1024;canvas.height=256;
  const ctx=canvas.getContext('2d')!;ctx.fillStyle='#121917';ctx.fillRect(0,0,1024,256);ctx.fillStyle='#d5f75b';ctx.font='900 118px Arial';ctx.fillText('WILDTRAIL',55,165);ctx.fillRect(920,0,104,256);
  const map=new THREE.CanvasTexture(canvas);map.colorSpace=THREE.SRGBColorSpace;
  const wallMat=new THREE.MeshStandardMaterial({map,roughness:.85});
  for(const side of [-1,1])for(let i=0;i<3;i++) {
    const board=new THREE.Mesh(new THREE.BoxGeometry(.16,1.1,4.5),wallMat);board.position.set(side*9.6,.65,-1+i*4.7);board.castShadow=true;camp.add(board);
  }
  // An open service canopy with metal legs and a fabric roof.
  const tent=new THREE.Group();tent.position.set(14.5,0,9);
  const roof=new THREE.Mesh(new THREE.ConeGeometry(4.4,1.3,4),new THREE.MeshStandardMaterial({color:'#dedbc6',roughness:.95}));roof.rotation.y=Math.PI/4;roof.position.y=3.8;roof.scale.z=1.15;roof.castShadow=true;tent.add(roof);
  for(const x of [-2.85,2.85])for(const z of [-3.2,3.2]){const leg=new THREE.Mesh(new THREE.CylinderGeometry(.045,.045,3.2,8),metal);leg.position.set(x,1.6,z);leg.castShadow=true;tent.add(leg);}
  camp.add(tent);scene.add(camp);

  // Subtle worn-in racing lines, following height rather than floating decals.
  const vertices:number[]=[],uvs:number[]=[],indices:number[]=[];
  for(let i=0;i<=ROAD_ROWS;i++)for(const side of [-1,1]) {
    const p=roadSample(i),center=side*.8;
    for(const edge of [-.13,.13]){const d=center+edge;vertices.push(p.x-p.tz*d,roadHeight(p,d)+.018,p.z+p.tx*d);uvs.push(edge<0?0:1,p.distance*.7);}
    if(i<ROAD_ROWS){const a=i*4+(side===-1?0:2);indices.push(a,a+1,a+4,a+1,a+5,a+4);}
  }
  const geometry=new THREE.BufferGeometry();geometry.setAttribute('position',new THREE.Float32BufferAttribute(vertices,3));geometry.setAttribute('uv',new THREE.Float32BufferAttribute(uvs,2));geometry.setIndex(indices);geometry.computeVertexNormals();
  scene.add(new THREE.Mesh(geometry,new THREE.MeshStandardMaterial({color:'#514939',roughness:.92,transparent:true,opacity:.12,depthWrite:false,polygonOffset:true,polygonOffsetFactor:-1})));
}
