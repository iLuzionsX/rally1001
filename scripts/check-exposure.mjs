import {readFileSync,writeFileSync,mkdirSync,mkdtempSync,rmSync} from 'node:fs';
import {dirname,join} from 'node:path';
import {pathToFileURL,fileURLToPath} from 'node:url';
import {spawnSync} from 'node:child_process';
import ts from 'typescript';
import * as THREE from 'three';
import {HDRLoader} from 'three/addons/loaders/HDRLoader.js';

// Offscreen material probes, not screenshots of the complete game.
// Python test dependencies: moderngl, numpy, Pillow; EGL/Mesa on Linux.
const root=dirname(dirname(fileURLToPath(import.meta.url)));
const cache=join(root,'node_modules/.cache');mkdirSync(cache,{recursive:true});
const temp=mkdtempSync(join(cache,'exposure-'));
try {
  const compiled=ts.transpileModule(readFileSync(join(root,'lib/rally/lighting.ts'),'utf8'),{compilerOptions:{module:ts.ModuleKind.ESNext,target:ts.ScriptTarget.ES2022}}).outputText;
  writeFileSync(join(temp,'lighting.mjs'),compiled);
  const {DAYLIGHT}=await import(pathToFileURL(join(temp,'lighting.mjs')).href);
  const rendering=readFileSync(join(root,'lib/rally/rendering.ts'),'utf8');
  const profile={...DAYLIGHT,sunOffset:DAYLIGHT.sunOffset.toArray(),aoIntensity:DAYLIGHT.aoIntensity??Number(rendering.match(/blendIntensity = ([.\d]+)/)[1])};
  const buffer=readFileSync(join(root,'public/assets/textures/kloppenheim_03_puresky_1k.hdr'));
  const hdr=new HDRLoader().setDataType(THREE.FloatType).parse(buffer.buffer.slice(buffer.byteOffset,buffer.byteOffset+buffer.byteLength));
  writeFileSync(join(temp,'environment.f32'),Buffer.from(hdr.data.buffer));
  writeFileSync(join(temp,'input.json'),JSON.stringify({
    profile,hdrSize:[hdr.width,hdr.height],
    tone:THREE.ShaderChunk.tonemapping_pars_fragment,
    colors:THREE.ShaderChunk.colorspace_pars_fragment,
    grade:rendering.match(/fragmentShader: `([\s\S]*?)`/)[1],
  }));
  const result=spawnSync(process.env.CODEX_PRIMARY_RUNTIME_PYTHON??'python3',[
    join(root,'scripts/check-exposure.py'),'--root',root,'--data',temp,...process.argv.slice(2),
  ],{stdio:'inherit'});
  if(result.error)throw result.error;
  process.exitCode=result.status??1;
} finally {rmSync(temp,{recursive:true,force:true});}
