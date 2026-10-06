import * as THREE from 'three';
import {EffectComposer} from 'three/addons/postprocessing/EffectComposer.js';
import {RenderPass} from 'three/addons/postprocessing/RenderPass.js';
import {GTAOPass} from 'three/addons/postprocessing/GTAOPass.js';
import {UnrealBloomPass} from 'three/addons/postprocessing/UnrealBloomPass.js';
import {ShaderPass} from 'three/addons/postprocessing/ShaderPass.js';
import {OutputPass} from 'three/addons/postprocessing/OutputPass.js';
import {FXAAShader} from 'three/addons/shaders/FXAAShader.js';
import type {Quality} from './world';
import {DAYLIGHT} from './lighting';

/** Linear HDR pipeline. Tone mapping happens once, in OutputPass. */
export class RallyRendering {
  composer: EffectComposer;
  ao: GTAOPass;
  bloom: UnrealBloomPass;
  fxaa: ShaderPass;
  grade: ShaderPass;
  constructor(renderer: THREE.WebGLRenderer, scene: THREE.Scene, camera: THREE.Camera) {
    this.composer = new EffectComposer(renderer);
    this.composer.addPass(new RenderPass(scene, camera));
    this.ao = new GTAOPass(scene, camera, 512, 512);
    this.ao.blendIntensity = DAYLIGHT.aoIntensity;
    this.ao.updateGtaoMaterial({radius: 1.1, distanceExponent: 1.4, thickness: 1.4});
    this.composer.addPass(this.ao);
    // Keep bloom subtle and confined to the strongest HDR highlights.
    this.bloom = new UnrealBloomPass(new THREE.Vector2(512, 512), DAYLIGHT.bloomStrength, DAYLIGHT.bloomRadius, DAYLIGHT.bloomThreshold);
    this.composer.addPass(this.bloom);
    this.grade = new ShaderPass({
      uniforms: {tDiffuse: {value: null}},
      vertexShader: 'varying vec2 vUv; void main(){vUv=uv;gl_Position=projectionMatrix*modelViewMatrix*vec4(position,1.);}',
      fragmentShader: `uniform sampler2D tDiffuse; varying vec2 vUv;
        void main(){vec3 c=texture2D(tDiffuse,vUv).rgb;
          float l=dot(c,vec3(.2126,.7152,.0722));
          c=mix(vec3(l),c,1.035);
          c*=mix(vec3(.985,1.005,1.025),vec3(1.01,1.005,.99),smoothstep(.05,1.,l));
          vec2 q=(vUv-.5)*vec2(1.,.8); c*=1.-.23*dot(q,q);
          gl_FragColor=vec4(max(c,vec3(0.)),1.);}`,
    });
    this.composer.addPass(this.grade);
    this.composer.addPass(new OutputPass());
    this.fxaa = new ShaderPass(FXAAShader);
    this.composer.addPass(this.fxaa);
  }
  resize(w: number, h: number, dpr: number, quality: Quality) {
    this.composer.setPixelRatio(dpr);
    this.composer.setSize(w, h);
    // AO uses a separate normal/depth pass: reserve it for desktop high detail.
    this.ao.enabled = quality === 'high' && !matchMedia('(pointer:coarse)').matches;
    this.ao.setSize(Math.round(w*dpr*.55), Math.round(h*dpr*.55));
    this.bloom.enabled = quality !== 'battery';
    this.fxaa.uniforms.resolution.value.set(1/(w*dpr), 1/(h*dpr));
  }
  render(dt: number) { this.composer.render(dt); }
  dispose() {
    for (const pass of this.composer.passes) pass.dispose();
    this.composer.dispose();
  }
}
