import * as THREE from 'three';
import {GLTFLoader} from 'three/addons/loaders/GLTFLoader.js';
import {HDRLoader} from 'three/addons/loaders/HDRLoader.js';
import {DAYLIGHT,makeSkybox} from './lighting';
import {createScenery,addTrackside,type Scenery} from './scenery';
import {Reflector} from 'three/addons/objects/Reflector.js';
import RAPIER from '@dimforge/rapier3d-compat';
import {random,courseAt,nearestRoad,terrainHeight,roadHeight,makeTerrainData,makeRoadData,GATE_COUNT,POND_CENTER,surfaceAt,ROAD_ROWS,ROAD_COLS,roadSample} from './course';
import {VEHICLES,type VehicleKind,type WheelPose} from './vehicle';
export type Quality='balanced'|'high'|'battery';
export type VisualVehicle={root:THREE.Group;wheels:THREE.Group[];brakeLights:THREE.Mesh[];dispose:()=>void};
export type WorldGraphics={scene:THREE.Scene;sun:THREE.DirectionalLight;gates:THREE.Group[];mirror:Reflector;models:Map<VehicleKind,THREE.Group>;scenery:Scenery;props:number;dispose:()=>void};
const files=['canopy-tree-03','canopy-tree-01','fern-real','grass-real','rock-real','subaru-rally','ford-rally'];
async function groundMaterial(asset:string,repeat:number,roughness:number){
 const loader=new THREE.TextureLoader();
 const [map,normalMap,roughnessMap]=await Promise.all(['diff','nor_gl','rough'].map(kind=>loader.loadAsync(`/assets/textures/${asset}_${kind}.jpg`)));
 for(const texture of [map,normalMap,roughnessMap]){texture.wrapS=texture.wrapT=THREE.RepeatWrapping;texture.repeat.setScalar(repeat);texture.anisotropy=8;}
 map.colorSpace=THREE.SRGBColorSpace;
 return new THREE.MeshStandardMaterial({map,normalMap,roughnessMap,normalScale:new THREE.Vector2(.65,.65),vertexColors:true,roughness});
}
function geometry(data:{vertices:Float32Array;indices:Uint32Array;colors:Float32Array;uvs?:Float32Array}){const g=new THREE.BufferGeometry();g.setAttribute('position',new THREE.BufferAttribute(data.vertices,3));g.setAttribute('color',new THREE.BufferAttribute(data.colors,3));g.setIndex(new THREE.BufferAttribute(data.indices,1));let uv=data.uvs;if(!uv){uv=new Float32Array(data.vertices.length/3*2);for(let i=0;i<data.vertices.length/3;i++)uv.set([data.vertices[i*3]/6,data.vertices[i*3+2]/6],i*2);}g.setAttribute('uv',new THREE.BufferAttribute(uv,2));g.computeVertexNormals();g.computeBoundingSphere();return g;}
function banner(text:string,small=false){const canvas=document.createElement('canvas');canvas.width=1024;canvas.height=small?128:256;const ctx=canvas.getContext('2d')!;ctx.fillStyle='#101b16';ctx.fillRect(0,0,1024,canvas.height);ctx.fillStyle='#d5f75b';ctx.fillRect(0,0,14,canvas.height);ctx.fillRect(1010,0,14,canvas.height);ctx.font=`900 ${small?60:116}px Arial`;ctx.textAlign='center';ctx.textBaseline='middle';ctx.fillText(text,512,canvas.height*.53);const tex=new THREE.CanvasTexture(canvas);tex.colorSpace=THREE.SRGBColorSpace;return new THREE.Mesh(new THREE.PlaneGeometry(small?7:12,small?.9:2.5),new THREE.MeshStandardMaterial({map:tex,side:THREE.DoubleSide,roughness:.8}));}
export async function createWorld(physics:RAPIER.World,progress:(n:number)=>void):Promise<WorldGraphics>{
 const scene=new THREE.Scene();scene.background=new THREE.Color('#82a6c3');scene.fog=new THREE.FogExp2(DAYLIGHT.fogColor,DAYLIGHT.fogDensity);scene.add(new THREE.HemisphereLight('#d9e7f4','#494837',DAYLIGHT.hemisphereIntensity));
 const sun=new THREE.DirectionalLight(DAYLIGHT.sunColor,DAYLIGHT.sunIntensity);sun.position.copy(DAYLIGHT.sunOffset);sun.castShadow=true;sun.shadow.mapSize.set(2048,2048);Object.assign(sun.shadow.camera,{left:-38,bottom:-38,right:38,top:38,near:3,far:235});sun.shadow.bias=-.0002;sun.shadow.normalBias=.035;sun.shadow.radius=3;scene.add(sun,sun.target);
 // A display-bounded panorama keeps sky/cloud pixels below the bloom threshold.
 // The matching HDR has a soft solar shoulder; direct sun supplies the shadows.
 const [skybox,hdr]=await Promise.all([
  new THREE.TextureLoader().loadAsync('/assets/textures/kloppenheim_03_puresky_4k.webp'),
  new HDRLoader().loadAsync('/assets/textures/kloppenheim_03_puresky_1k.hdr'),
 ]);
 scene.add(makeSkybox(skybox));hdr.mapping=THREE.EquirectangularReflectionMapping;
 scene.environment=hdr;scene.environmentIntensity=DAYLIGHT.environmentIntensity;scene.environmentRotation.y=DAYLIGHT.skyRotation;
 progress(.1);
 const [forestMaterial,dirtMaterial,mudMaterial]=await Promise.all([
 groundMaterial('brown_mud_leaves_01',1.1,.96),groundMaterial('brown_mud_02',1,.92),groundMaterial('brown_mud_02',1,.36),
 ]);
 const terrain=makeTerrainData(),road=makeRoadData(),terrainMesh=new THREE.Mesh(geometry(terrain),forestMaterial),roadGeometry=geometry(road);
 for(let i=0;i<ROAD_ROWS;i++){const p=roadSample(i+.5);roadGeometry.addGroup(i*ROAD_COLS*6,ROAD_COLS*6,surfaceAt(p.x,p.z).mud?1:0);}
 const roadMesh=new THREE.Mesh(roadGeometry,[dirtMaterial,mudMaterial]);
 for(const [mesh,data] of [[terrainMesh,terrain],[roadMesh,road]] as const){mesh.receiveShadow=true;scene.add(mesh);physics.createCollider(RAPIER.ColliderDesc.trimesh(data.vertices,data.indices).setFriction(.9));}progress(.18);
 const loader=new GLTFLoader();let loaded=0;
 const all=await Promise.all(files.map(async file=>{const gltf=await loader.loadAsync('/assets/models/'+file+'.glb');progress(.18+(++loaded)/files.length*.52);return gltf.scene;}));
 const models=new Map<VehicleKind,THREE.Group>([['suv',all[5]],['truck',all[6]]]);
 const scenery=createScenery(scene,physics,all.slice(0,5));const props=scenery.props;addTrackside(scene,physics);
 const rand=random(5924);progress(.79);
 const gates:THREE.Group[]=[],dark=new THREE.MeshStandardMaterial({color:'#263526',roughness:.75});for(let i=0;i<GATE_COUNT;i++){const p=courseAt(i/GATE_COUNT),g=new THREE.Group();g.position.set(p.x,p.y,p.z);g.rotation.y=Math.atan2(-p.tx,-p.tz);for(const side of [-1,1]){const pole=new THREE.Mesh(new THREE.CylinderGeometry(i===0?.11:.055,.1,i===0?7:3.5,7),dark);pole.position.set(side*(p.width/2+.65),i===0?3.5:1.7,0);pole.castShadow=true;g.add(pole);if(i>0){const flag=new THREE.Mesh(new THREE.PlaneGeometry(.85,1.85),new THREE.MeshStandardMaterial({color:i===1?'#d5f75b':'#efeee0',side:THREE.DoubleSide}));flag.position.set(pole.position.x,2.4,0);g.add(flag);}}if(i===0){const b=banner('WILDTRAIL');b.position.y=6.2;g.add(b);for(let row=0;row<2;row++)for(let col=0;col<12;col++){
 const tile=new THREE.PlaneGeometry(p.width/12,.65);tile.rotateX(-Math.PI/2);
 const pos=tile.attributes.position;
 for(let j=0;j<pos.count;j++){
  const lateral=(col+.5-6)*p.width/12+pos.getX(j),depth=(row-.5)*.65+pos.getZ(j);
  const x=p.x-p.tz*lateral-p.tx*depth,z=p.z+p.tx*lateral-p.tz*depth,n=nearestRoad(x,z);
  pos.setXYZ(j,lateral,roadHeight(n.point,n.lateral)-p.y+.025,depth);
 }
 tile.computeVertexNormals();
 g.add(new THREE.Mesh(tile,new THREE.MeshStandardMaterial({color:(row+col)%2?'#101b16':'#f2f2e5',roughness:.9})));
}}scene.add(g);gates.push(g);}
 for(let i=0;i<9;i++){const p=courseAt([.11,.20,.31,.43,.51,.57,.70,.79,.88][i]),side=i%2?1:-1,b=banner(i%2?'‹  ‹  ‹':'›  ›  ›',true);b.position.set(p.x-p.tz*(p.width/2+1.8)*side,p.y+1.5,p.z+p.tx*(p.width/2+1.8)*side);b.rotation.y=Math.atan2(-p.tx,-p.tz);scene.add(b);}
 const wet=new THREE.MeshPhysicalMaterial({color:'#5b5740',roughness:.05,metalness:.1,clearcoat:1,clearcoatRoughness:.03,transparent:true,opacity:.78,depthWrite:false});for(let i=0;i<26;i++){const p=courseAt(i<13?.348+rand()*.036:.643+rand()*.037),side=(rand()-.5)*(p.width-3),g=new THREE.CircleGeometry(.5+rand()*1.5,20);g.rotateX(-Math.PI/2);const v=g.attributes.position;for(let j=0;j<v.count;j++){const x=p.x-p.tz*side+v.getX(j),z=p.z+p.tx*side+v.getZ(j),n=nearestRoad(x,z);v.setXYZ(j,x,roadHeight(n.point,n.lateral)+.035,z);}g.computeVertexNormals();const mesh=new THREE.Mesh(g,wet);mesh.receiveShadow=true;scene.add(mesh);}
 const mirror=new Reflector(new THREE.CircleGeometry(25,64),{clipBias:.012,textureWidth:512,textureHeight:512,color:0x6a9180});mirror.rotation.x=-Math.PI/2;mirror.position.set(POND_CENTER.x,-1.4,POND_CENTER.z);scene.add(mirror);const water=new THREE.Mesh(new THREE.CircleGeometry(25.1,64),new THREE.MeshPhysicalMaterial({color:'#50775b',transparent:true,opacity:.28,roughness:.12,metalness:.1,depthWrite:false}));water.rotation.x=-Math.PI/2;water.position.set(POND_CENTER.x,-1.388,POND_CENTER.z);scene.add(water);
 const dispose=()=>{const geos=new Set<THREE.BufferGeometry>(),mats=new Set<THREE.Material>(),textures=new Set<THREE.Texture>();[scene,...all].forEach(root=>root.traverse(o=>{if(o instanceof THREE.Mesh){geos.add(o.geometry);if(o.customDepthMaterial)mats.add(o.customDepthMaterial);for(const m of Array.isArray(o.material)?o.material:[o.material]){mats.add(m);for(const v of Object.values(m))if(v instanceof THREE.Texture)textures.add(v);}}}));geos.forEach(g=>g.dispose());mats.forEach(m=>m.dispose());textures.forEach(t=>t.dispose());scene.background=null;scene.environment=null;hdr.dispose();mirror.getRenderTarget().dispose();sun.shadow.dispose();};progress(.9);return {scene,sun,gates,mirror,models,scenery,props,dispose};
}
export function makeVisualVehicle(kind:VehicleKind,model:THREE.Group):VisualVehicle{
 const root=new THREE.Group(),body=new THREE.Group();root.add(body);
 const config=VEHICLES[kind],wheels:THREE.Group[]=[],brakeLights:THREE.Mesh[]=[];
 const offset=-(config.radius+config.suspension-config.mount-config.mass*9.81/(4*config.springRate));
 const clone=model.clone(true);clone.updateMatrixWorld(true);
 clone.traverse(o=>{
  if(!(o instanceof THREE.Mesh))return;
  let ancestor:THREE.Object3D|null=o, wheelIndex:number|undefined;
  while(ancestor){if(typeof ancestor.userData.wheelIndex==='number'){wheelIndex=ancestor.userData.wheelIndex;break;}ancestor=ancestor.parent;}
  const g=o.geometry.clone().applyMatrix4(o.matrixWorld);
  const sources=Array.isArray(o.material)?o.material:[o.material];
  const materials=sources.map(source=>{
   const m=source.clone() as THREE.MeshStandardMaterial;
   m.envMapIntensity=1.35;
   if(/pearl|paint|body/i.test(m.name)&&!m.map){
    const paint=new THREE.MeshPhysicalMaterial({color:m.color,map:m.map,normalMap:m.normalMap,normalScale:m.normalScale,roughnessMap:m.roughnessMap,metalnessMap:m.metalnessMap,metalness:m.metalness,roughness:Math.max(.22,m.roughness),clearcoat:1,clearcoatRoughness:.16,envMapIntensity:1.35,side:m.side});paint.name=m.name;m.dispose();return paint;
   }
   return m;
  });
  const mesh=new THREE.Mesh(g,materials.length===1?materials[0]:materials);mesh.castShadow=mesh.receiveShadow=true;
  if(wheelIndex!==undefined&&ancestor){
   // Centre on the wheel's geometry, not its node origin: the Subaru's wheel vertices sit ~1.5 m from their node, which swung them out when steering.
   const center=new THREE.Box3().setFromObject(ancestor).getCenter(new THREE.Vector3());g.translate(-center.x,-center.y,-center.z);
   // Pick the side from where the mesh sits: the Subaru tags its left/right wheels the wrong way round, which put the spokes facing inward.
   wheelIndex=(wheelIndex&2)|(center.x>0?1:0);
   if(!wheels[wheelIndex]){const pivot=new THREE.Group();pivot.position.set(wheelIndex%2?config.track/2:-config.track/2,config.mount-config.suspension,wheelIndex<2?config.front:config.back);root.add(pivot);wheels[wheelIndex]=pivot;}
   wheels[wheelIndex].add(mesh);
  }else{
   g.translate(0,offset,0);body.add(mesh);
   // Illuminate the downloaded rear lenses rather than overlaying box lights.
   if(/rear lens|r_lights|red_bump|d_red/.test(materials[0].name))brakeLights.push(mesh);
  }
 });
 return {root,wheels,brakeLights,dispose:()=>root.traverse(o=>{if(o instanceof THREE.Mesh){o.geometry.dispose();for(const m of Array.isArray(o.material)?o.material:[o.material])m.dispose();}})};
}
export function poseWheels(v:VisualVehicle,poses:WheelPose[],brake:number){const steer=new THREE.Quaternion(),spin=new THREE.Quaternion(),up=new THREE.Vector3(0,1,0),axle=new THREE.Vector3(-1,0,0);poses.forEach((p,i)=>{const w=v.wheels[i];if(!w)return;w.position.set(p.x,p.y,p.z);steer.setFromAxisAngle(up,p.steer);spin.setFromAxisAngle(axle,p.rotation);w.quaternion.multiplyQuaternions(steer,spin);});v.brakeLights.forEach(l=>{for(const m of Array.isArray(l.material)?l.material:[l.material]){const lens=m as THREE.MeshStandardMaterial;lens.emissive.set('#e52616');lens.emissiveIntensity=.08+brake*1.4;}});}
