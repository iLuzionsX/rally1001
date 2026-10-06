"""Offscreen diffuse-material probes with the production grade and Three ACES.

This compiles and executes real GLSL under EGL. Cosine-integrated HDR diffuse
lighting is measured for the probe normal. It is not a full game/PMREM render:
canopy occlusion, specular reflections and actual GTAO geometry are not rebuilt.
"""
import argparse
import json
import math
from pathlib import Path

import moderngl
import numpy as np
from PIL import Image, ImageDraw, ImageFont

parser = argparse.ArgumentParser()
parser.add_argument('--root', type=Path, required=True)
parser.add_argument('--data', type=Path, required=True)
parser.add_argument('--sweep', action='store_true')
args = parser.parse_args()
root, data = args.root, args.data
source = json.loads((data / 'input.json').read_text())
current = source['profile']
baseline_path = root / 'reference/exposure-before.json'
baseline = json.loads(baseline_path.read_text()) if baseline_path.exists() else current
ctx = moderngl.create_context(standalone=True, backend='egl')
size = (160, 160)
vertex = '''#version 330
out vec2 vUv;
void main() {
    vec2 p = vec2((gl_VertexID << 1) & 2, gl_VertexID & 2);
    vUv = p; gl_Position = vec4(p * 2. - 1., 0., 1.);
}'''
colors = source['colors']
material = ctx.program(vertex_shader=vertex, fragment_shader='''#version 330
in vec2 vUv; out vec4 fragColor;
uniform sampler2D albedoMap, normalMap;
uniform vec3 sunDirection, sunColor, iblDiffuse;
uniform vec3 hemiSky, hemiGround, vertexTint;
uniform float sunIntensity, environmentIntensity, hemisphereIntensity;
uniform float sunVisibility, normalScale, ambientVisibility, aoMultiplier;
''' + colors + '''
void main() {
    vec3 albedo = sRGBTransferEOTF(vec4(texture(albedoMap, vUv).rgb, 1.)).rgb * vertexTint;
    vec3 tangentNormal = texture(normalMap, vUv).rgb * 2. - 1.;
    tangentNormal.xy *= normalScale;
    vec3 n = normalize(vec3(tangentNormal.x, tangentNormal.z, -tangentNormal.y));
    vec3 direct = max(dot(n, sunDirection), 0.) * sunColor * sunIntensity * sunVisibility;
    vec3 hemi = mix(hemiGround, hemiSky, .5 * n.y + .5) * hemisphereIntensity;
    vec3 indirect = (iblDiffuse * environmentIntensity * 3.14159265359 + hemi) * ambientVisibility;
    // Three's diffuse 1/pi convention and dielectric energy conservation.
    vec3 color = albedo * .96 * (direct + indirect) / 3.14159265359;
    fragColor = vec4(color * aoMultiplier, 1.);
}''')
grade_source = source['grade'].replace('varying vec2 vUv;', 'in vec2 vUv; out vec4 fragColor;')
grade_source = grade_source.replace('texture2D(', 'texture(').replace('gl_FragColor', 'fragColor')
grade = ctx.program(vertex_shader=vertex, fragment_shader='#version 330\n' + grade_source)
output = ctx.program(vertex_shader=vertex, fragment_shader='''#version 330
in vec2 vUv; out vec4 fragColor; uniform sampler2D tDiffuse;
''' + source['tone'] + colors + '''
void main() { fragColor = sRGBTransferOETF(vec4(ACESFilmicToneMapping(texture(tDiffuse, vUv).rgb), 1.)); }
''')
sky_program = ctx.program(vertex_shader=vertex, fragment_shader='''#version 330
in vec2 vUv; out vec4 fragColor;
uniform sampler2D sky; uniform float intensity;
''' + colors + '''
void main(){fragColor = vec4(sRGBTransferEOTF(vec4(texture(sky,vUv).rgb,1.)).rgb * intensity,1.);}
''')
vaos = {p: ctx.vertex_array(p, []) for p in [material, grade, output, sky_program]}
targets = [ctx.texture(size, 4, dtype='f4') for _ in range(3)]
fbos = [ctx.framebuffer([t]) for t in targets]
weights = np.array([.2126, .7152, .0722])

def eotf(rgb):
    rgb = np.asarray(rgb, dtype=np.float64)
    return np.where(rgb <= .04045, rgb / 12.92, ((rgb + .055) / 1.055) ** 2.4)

def linear_hex(value):
    return tuple(eotf([int(value[i:i+2], 16) / 255 for i in [1, 3, 5]]))

w, h = source['hdrSize']
environment = np.fromfile(data / 'environment.f32', dtype=np.float32).reshape(h, w, 4)[..., :3]
latitude = (.5 - (np.arange(h) + .5) / h) * math.pi
cosine = np.maximum(np.sin(latitude), 0) * np.cos(latitude)
# Integral L(w)*max(N.w,0)*d(w)/pi for a horizontal road surface.
ibl = (environment * cosine[:, None, None]).sum(axis=(0, 1)) * (2 * math.pi / w) * (math.pi / h) / math.pi

def texture(path=None, value=None):
    if path:
        pixels = np.asarray(Image.open(path).convert('RGB').resize(size, Image.Resampling.LANCZOS), dtype=np.uint8)
    else:
        pixels = np.broadcast_to(np.asarray(value, dtype=np.uint8), (*size, 3)).copy()
    t = ctx.texture(size, 3, pixels.tobytes()); t.filter = (moderngl.LINEAR, moderngl.LINEAR)
    return t

dirt = texture(root / 'public/assets/textures/brown_mud_02_diff.jpg')
normal = texture(root / 'public/assets/textures/brown_mud_02_nor_gl.jpg')
forest = texture(root / 'public/assets/textures/brown_mud_leaves_01_diff.jpg')
forest_normal = texture(root / 'public/assets/textures/brown_mud_leaves_01_nor_gl.jpg')
gray = texture(value=[118, 118, 118])  # approximately 18% linear reflectance
white = texture(value=[242, 242, 242])
flat = texture(value=[128, 128, 255])
sky_texture = texture(root / 'public/assets/textures/kloppenheim_03_puresky_4k.webp')
cases = [
    ('Dirt / sun', dirt, normal, [ .94, .92, .88 ], 1., 1., 1.),
    ('Dirt / shade', dirt, normal, [ .94, .92, .88 ], 0., 1., 1.),
    ('Canopy / contact shade', forest, forest_normal, [ .82, .88, .74 ], 0., .75, .6),
    ('18% gray / sun', gray, flat, [1.,1.,1.], 1., 1., 1.),
    ('18% gray / shade', gray, flat, [1.,1.,1.], 0., 1., 1.),
    ('White / sun', white, flat, [1.,1.,1.], 1., 1., 1.),
]

def finish(profile):
    targets[0].use(0);grade['tDiffuse'].value=0
    fbos[1].use();vaos[grade].render(moderngl.TRIANGLES, vertices=3)
    targets[1].use(0);output['tDiffuse'].value=0;output['toneMappingExposure'].value=profile['exposure']
    fbos[2].use();vaos[output].render(moderngl.TRIANGLES, vertices=3)
    pixels=np.frombuffer(fbos[2].read(components=4,dtype='f4'),dtype=np.float32).reshape(*size,4)[...,:3].copy()
    assert np.isfinite(pixels).all(), 'Non-finite shader output'
    luma=pixels@weights
    return pixels, {'median':round(float(np.median(luma)),4), 'p10':round(float(np.quantile(luma,.1)),4),
                    'clippedFraction':round(float(np.mean(np.all(pixels>.98,axis=-1))),6)}

def render(profile, quality='high'):
    results={};images={}
    sun=np.array(profile['sunOffset']);sun/=np.linalg.norm(sun)
    for label, albedo, normals, tint, visibility, ambient, ao in cases:
        albedo.use(0);normals.use(1);material['albedoMap'].value=0;material['normalMap'].value=1
        for key,value in {'sunDirection':tuple(sun),'sunColor':linear_hex(profile['sunColor']),
                          'iblDiffuse':tuple(ibl),'hemiSky':linear_hex('#d9e7f4'),
                          'hemiGround':linear_hex('#494837'),'vertexTint':tuple(tint),
                          'sunIntensity':profile['sunIntensity'],'environmentIntensity':profile['environmentIntensity'],
                          'hemisphereIntensity':profile['hemisphereIntensity'],'sunVisibility':visibility,
                          'normalScale':.65,'ambientVisibility':ambient,
                          'aoMultiplier':1-profile['aoIntensity']*(1-ao) if quality=='high' else 1.}.items():
            material[key].value=value
        fbos[0].use();vaos[material].render(moderngl.TRIANGLES,vertices=3)
        images[label],results[label]=finish(profile)
    sky_texture.use(0);sky_program['sky'].value=0;sky_program['intensity'].value=profile['skyIntensity']
    fbos[0].use();vaos[sky_program].render(moderngl.TRIANGLES,vertices=3)
    images['Sky'],results['Sky']=finish(profile)
    return results,images

before, before_images=render(baseline)
after, after_images=render(current)
if args.sweep:
    options=[]
    for exposure in [.9,1.,1.1]:
        for fill in [.95,1.15,1.35]:
            profile={**current,'exposure':exposure,'environmentIntensity':fill,'hemisphereIntensity':.7,
                     'skyIntensity':.85,'aoIntensity':.38}
            scores,_=render(profile)
            error=abs(scores['18% gray / shade']['median']-.46)+abs(scores['18% gray / sun']['median']-.65)
            options.append({'profile':{k:profile[k] for k in ['exposure','environmentIntensity','hemisphereIntensity','skyIntensity','aoIntensity']},
                            'graySun':scores['18% gray / sun']['median'],'grayShade':scores['18% gray / shade']['median'],
                            'roadShade':scores['Dirt / shade']['median'],'error':round(error,5)})
    print(json.dumps({'renderer':ctx.info['GL_RENDERER'],'before':before,'sweep':sorted(options,key=lambda x:x['error'])},indent=2))
else:
    checks={
        'shadowGrayReadable':.40<=after['18% gray / shade']['median']<=.58,
        'sunlitGrayBounded':.57<=after['18% gray / sun']['median']<=.74,
        'shadedDirtBrighter':after['Dirt / shade']['median']>=before['Dirt / shade']['median']*1.2,
        'contactShadeBrighter':after['Canopy / contact shade']['median']>=before['Canopy / contact shade']['median']*1.2,
        'skyHighlightsRetained':after['Sky']['clippedFraction']<.001,
        'whiteHighlightsRetained':after['White / sun']['clippedFraction']<.001,
        'skyCannotTriggerBloom':current['skyIntensity']<current['bloomThreshold'],
    }
    quality_scores={q:render(current,q)[0] for q in ['high','balanced','battery']}
    checks['allQualityProfilesBounded']=all(
        .40 <= scores['18% gray / shade']['median'] <= .58
        and scores['Sky']['clippedFraction'] < .001
        and scores['White / sun']['clippedFraction'] < .001
        for scores in quality_scores.values()
    )
    report={'test':'Offscreen GLSL diffuse material probes using actual textures, cosine-integrated source HDR, production grade and Three ACES/sRGB functions',
            'renderer':ctx.info['GL_RENDERER'],'api':ctx.info['GL_VERSION'],'probeResolution':list(size),
            'baseline':baseline,'current':current,'iblDiffuse':list(map(float,ibl)),
            'before':before,'after':after,'qualityProfiles':quality_scores,'checks':checks,
            'limits':['Material probes are not full-game screenshots','Environment diffuse integration approximates production PMREM',
                      'Specular reflections, actual tree visibility and geometry-derived GTAO are not rendered','Phone/browser playtesting remains unverified']}
    (root/'reference/exposure-regression.json').write_text(json.dumps(report,indent=2)+'\n')
    canvas=Image.new('RGB',(7*184+24,2*244+96),'#121815');draw=ImageDraw.Draw(canvas)
    font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',12)
    title=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf',17)
    draw.text((16,12),'Exposure test — offscreen material probes',font=title,fill='#e5e9df')
    draw.text((16,37),'Production color shaders; not a screenshot of the complete game. Numbers are median display brightness.',font=font,fill='#a7b6ad')
    for row,(name,images,scores) in enumerate([('Before',before_images,before),('After',after_images,after)]):
        y=72+row*244;draw.text((16,y),name,font=title,fill='#d5f75b')
        for column,label in enumerate(images):
            x=16+column*184
            pixels=(np.clip(images[label],0,1)*255).astype(np.uint8)
            canvas.paste(Image.fromarray(pixels),(x,y+28))
            draw.text((x,y+192),label,font=font,fill='#e5e9df')
            draw.text((x,y+208),f"{scores[label]['median']:.3f}",font=font,fill='#a7b6ad')
    canvas.save(root/'reference/exposure-probes.png')
    print(json.dumps({'renderer':report['renderer'],'before':before,'after':after,'checks':checks},indent=2))
    assert all(checks.values()), 'Exposure acceptance failed: '+', '.join(k for k,v in checks.items() if not v)
ctx.release()
