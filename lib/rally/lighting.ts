import * as THREE from 'three';

// One photographed sun for the sky, reflections and moving shadow camera.
// The solar centroid is measured from Kloppenheim 03 Pure Sky's source HDR.
export const DAYLIGHT = {
  exposure: 1.1,
  skyRotation: 1.4147760083,
  skyIntensity: .85,
  environmentIntensity: 1.15,
  sunColor: '#fff1dd',
  sunIntensity: 3.1,
  sunOffset: new THREE.Vector3(46.0010876, 106.7370596, -46.0010876),
  hemisphereIntensity: .7,
  aoIntensity: .38,
  fogColor: '#a9bdc8',
  fogDensity: .0025,
  bloomStrength: .025,
  bloomRadius: .18,
  bloomThreshold: 2.4,
};

export function makeSkybox(texture: THREE.Texture) {
  texture.colorSpace = THREE.SRGBColorSpace;
  // SphereGeometry's horizontal UV direction is opposite equirectUv's.
  texture.repeat.x = -1;
  texture.offset.x = 1;
  const material = new THREE.MeshBasicMaterial({
    map: texture,
    color: new THREE.Color().setScalar(DAYLIGHT.skyIntensity),
    side: THREE.BackSide,
    depthTest: false,
    depthWrite: false,
    fog: false,
    allowOverride: false,
  });
  // Sample the 4K panorama directly. Native equirectangular backgrounds convert
  // it to six 2K cube faces, which adds unnecessary memory pressure on phones.
  const sky = new THREE.Mesh(new THREE.SphereGeometry(500, 64, 32), material);
  sky.name = 'Photographed cloud sky';
  sky.rotation.y = DAYLIGHT.skyRotation;
  sky.renderOrder = -1000;
  sky.frustumCulled = false;
  sky.onBeforeRender = (_renderer, _scene, camera) => {
    camera.getWorldPosition(sky.position);
    sky.updateMatrixWorld();
  };
  return sky;
}
