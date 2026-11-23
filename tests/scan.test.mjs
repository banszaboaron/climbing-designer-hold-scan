// Climbing Designer 1.3.0 · hold-scan tests (exported from the app repository without its UI test).
import test from 'node:test';import assert from 'node:assert/strict';
import {readFileSync,readdirSync,existsSync} from 'node:fs';
import path from 'node:path';
import {parseCameraText,parseDepthBlob,depthBlob,cameraText,captureFrames,parsePly} from '../src/domain/scan-capture.ts';
import {scanHold,plyHold,colorize,meshReport,framePoints,rigidInverse,apply,lookAtPoint,fitPlane,renderDepth} from '../src/domain/hold-scan.ts';
import {TRUTH_HOLDS,syntheticCapture,truthMesh,compareMeshes,topDownDisparity} from '../src/domain/scan-synthetic.ts';
import {roomanticOccupancy} from '../src/domain/roomantic-occupancy.ts';
import {holdMask,reliefMesh} from '../src/domain/photo-hold.ts';
const read=f=>readFileSync(f,'utf8');
const jug=TRUTH_HOLDS.find(h=>h.name==='jug');
/** Frames' depth agreement: points of frame a projected into frame b, |depth_b − z| median (metres). */
function consistency(A,B){const inv=rigidInverse(B.cameraToWorld),d=[];for(const p of framePoints(A,3)){const q=apply(inv,p);if(q[2]<.1)continue;const [fx,fy,cx,cy]=B.k,x=fx*q[0]/q[2]+cx,y=fy*q[1]/q[2]+cy;
  if(x<0||y<0||x>=B.imageWidth||y>=B.imageHeight)continue;const z=B.depth[Math.floor(y*B.depthHeight/B.imageHeight)*B.depthWidth+Math.floor(x*B.depthWidth/B.imageWidth)];if(Number.isFinite(z))d.push(Math.abs(z-q[2]));}
  d.sort((a,b)=>a-b);return {median:d[d.length>>1],n:d.length};}
/** Crossings of the vertical line (x, y) with the mesh (Möller–Trumbore on every triangle). */
function crossings(m,x,y){const p=m.positions,hits=[];for(let t=0;t<m.indices.length;t+=3){const [a,b,c]=[m.indices[t]*3,m.indices[t+1]*3,m.indices[t+2]*3];
  const e1=[p[b]-p[a],p[b+1]-p[a+1],p[b+2]-p[a+2]],e2=[p[c]-p[a],p[c+1]-p[a+1],p[c+2]-p[a+2]],det=e1[0]*e2[1]-e1[1]*e2[0];if(Math.abs(det)<1e-14)continue;
  const s=[x-p[a],y-p[a+1]],u=(s[0]*e2[1]-s[1]*e2[0])/det,v=(e1[0]*s[1]-e1[1]*s[0])/det;if(u<0||v<0||u+v>1)continue;hits.push(p[a+2]+u*e1[2]+v*e2[2]);}
  return [...new Set(hits.map(z=>Math.round(z*1e5)))].length;}

test('capture files: the Roomantic camera text (column-major simd) and depth blob are read as the app writes them',()=>{
  // camera_0001.txt of the Roomantic test capture (RoomanticLibTest/images), verbatim.
  const text=`extrinsics:\n-0.06119539216 -0.8633244038 0.5009253025 0 \n0.9943152666 -0.008917040192 0.1061020419 0 \n-0.08713374287 0.504570663 0.858962357 0 \n0.3192898035 -0.04087369516 -0.2977176905 1 \nprojection:\n1.402249098 0 0 0 \n0 1.869665504 0 0 \n-0.006250023842 -0.001376152039 -0.9999997616 -1 \n0 0 -0.0009999998147 0 \ncamera_transform:\n-0.06119538471 -0.8633242249 0.5009253025 0 \n-0.9943151474 0.008917035535 -0.1061020195 0 \n0.08713372797 -0.5045706034 -0.8589621782 0 \n0.3192898035 -0.04087371007 -0.2977176607 1 \nintrinsics:\n1346.15918 0 0 \n0 1346.15918 0 \n965.500061 718.5091553 1 \nimage_size: 1920 1440\ntimestamp: 310220.8306\n`;
  const c=parseCameraText(text);assert.deepEqual(c.k,[1346.15918,1346.15918,965.500061,718.5091553]);assert.equal(c.imageWidth,1920);assert.equal(c.imageHeight,1440);
  assert.deepEqual(c.cameraToWorld.slice(12,15),[.3192898035,-.04087371007,-.2977176607],'translation = 4th column');
  // the rotation is orthonormal (a misread row/column order would still be, so the multi-view test below checks the order)
  const R=[0,1,2].map(i=>c.cameraToWorld.slice(i*4,i*4+3));for(const [i,j] of [[0,0],[1,1],[2,2],[0,1],[1,2]])assert.ok(Math.abs(R[i].reduce((s,v,k)=>s+v*R[j][k],0)-(i===j?1:0))<1e-5);
  const depth=Float32Array.from({length:12},(_,i)=>i===5?NaN:.5+i/100),blob=depthBlob(4,3,depth),back=parseDepthBlob(blob);
  assert.equal(blob.length,12+48);assert.equal(back.width,4);assert.ok(Number.isNaN(back.depth[5]));assert.ok(Math.abs(back.depth[7]-.57)<1e-6);
  assert.throws(()=>parseDepthBlob(new Uint8Array(8)),/rövid/);const bad=depthBlob(4,3,depth);bad[10]=1;assert.throws(()=>parseDepthBlob(bad),/float32/);
  const back2=parseCameraText(cameraText({k:c.k,cameraToWorld:c.cameraToWorld,imageWidth:1920,imageHeight:1440}));assert.ok(back2.cameraToWorld.every((v,i)=>Math.abs(v-c.cameraToWorld[i])<1e-8),'writer ↔ reader');
  // a "videó" share: frames sorted by number, JPEG optional, macOS resource forks ignored
  const files=new Map([['capture/depth_0002.bin',blob],['capture/camera_0002.txt',new TextEncoder().encode(text)],['capture/depth_0001.bin',blob],['capture/camera_0001.txt',new TextEncoder().encode(text)],['capture/frame_0001.jpg',new Uint8Array(3)],['capture/._depth_0003.bin',new Uint8Array(1)]]);
  const frames=captureFrames(files);assert.deepEqual(frames.map(f=>f.index),[1,2]);assert.equal(frames[0].image,'frame_0001.jpg');assert.equal(frames[1].image,undefined);
  assert.throws(()=>captureFrames(new Map([['x.txt',new Uint8Array(1)]])),/nincs mélységkép/);
});

test('conventions: depth back-projected through camera_transform agrees across views; the transposed reading does not',()=>{
  const cap=syntheticCapture(jug,{frames:16,noise:0,smoothing:false});
  for(const [a,b] of [[0,3],[4,9],[2,14]]){const r=consistency(cap[a],cap[b]);assert.ok(r.n>2000&&r.median<.0005,`frames ${a}→${b}: ${r.median}`);}
  const T=m=>[m[0],m[4],m[8],m[3],m[1],m[5],m[9],m[7],m[2],m[6],m[10],m[11],m[12],m[13],m[14],m[15]],tr=f=>({...f,cameraToWorld:T(f.cameraToWorld)});
  assert.ok(consistency(tr(cap[0]),tr(cap[3])).median>.005,'a row/column mix-up is caught');
  const look=lookAtPoint(cap);assert.ok(Math.hypot(look[0],look[1]-.015,look[2])<.005,`the look-at point is the circled target: ${look}`);
});

test('a real hold shape from a noisy capture: metric size, watertight, flat back, the undercut kept (the photo relief cannot)',()=>{
  const cap=syntheticCapture(jug,{noise:.0015}),r=scanHold(cap),truth=truthMesh(jug),c=compareMeshes(r.mesh,truth);
  assert.ok(r.report.watertight&&r.report.components===1&&r.report.degenerate===0,JSON.stringify(r.report));assert.ok(r.report.volume>0,'outward normals');
  assert.ok(c.accuracyMm<1.2&&c.completenessMm<1.2&&c.fScore2mm>.85,JSON.stringify(c));
  for(const e of c.sizeErrorMm)assert.ok(Math.abs(e)<3,`size within 3 mm: ${c.sizeErrorMm}`);
  const zs=r.mesh.positions.filter((_,i)=>i%3===2);assert.equal(Math.min(...zs),0,'back on Z = 0');assert.ok(zs.filter(z=>z===0).length>30,'a flat back');
  const xs=r.mesh.positions.filter((_,i)=>i%3===0);assert.ok(Math.abs(Math.min(...xs)+Math.max(...xs))<1e-9,'centred');
  assert.ok(r.seconds<20,`processing time ${r.seconds}s`);
  // The lip overhangs the body: a vertical line through it crosses the surface four times (back, body top, lip bottom, lip top).
  const lipX=(c.rotation?-1:1)*.051;assert.ok(crossings(r.mesh,lipX,0)>=4,'undercut kept');
  const disp=topDownDisparity(jug),relief=reliefMesh(holdMask(disp.depth,disp.width,disp.height,.5),{lengthM:.114,thicknessM:.048});
  assert.ok(crossings(relief,.051,0)<=2,'a single-photo relief is a height field: no undercut');
  assert.throws(()=>scanHold(cap.slice(0,2)),/Legalább 3 képkocka/);
});

test('vertex colours come from the photos that see a vertex head-on and unoccluded',()=>{
  const cap=syntheticCapture(TRUTH_HOLDS[0],{frames:24,noise:0}),r=scanHold(cap);
  // a "photo" that is red on the hold's +e1 half (world), blue elsewhere
  const colors=colorize(r.mesh,r.basis,cap,(i,x,y)=>{const f=cap[i],[fx,fy,cx,cy]=f.k,d=f.depth[Math.floor(y*f.depthHeight/f.imageHeight)*f.depthWidth+Math.floor(x*f.depthWidth/f.imageWidth)];if(!Number.isFinite(d))return null;
    const w=apply(f.cameraToWorld,[(x-cx)/fx*d,(y-cy)/fy*d,d]),side=(w[0]-r.basis.origin[0])*r.basis.e1[0]+(w[1]-r.basis.origin[1])*r.basis.e1[1]+(w[2]-r.basis.origin[2])*r.basis.e1[2];return side>0?[1,0,0]:[0,0,1];});
  assert.equal(colors.length,r.mesh.positions.length);let right=0,checked=0;
  for(let v=0;v<colors.length/3;v++){const x=r.mesh.positions[v*3],z=r.mesh.positions[v*3+2];if(Math.abs(x)<.015||z<.004)continue;checked++;if((x>0)===(colors[v*3]>colors[v*3+2]))right++;}
  assert.ok(checked>200&&right/checked>.95,`${right}/${checked}`);
});

test('PLY: ascii (Roomantic object.ply) and binary are read; a clean mesh goes through the same TSDF path',()=>{
  const ascii=new TextEncoder().encode('ply\nformat ascii 1.0\ncomment Created with Roomantic version 4.0.2\nelement vertex 4\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nelement face 2\nproperty list uchar int vertex_indices\nend_header\n0 0 0 255 0 0\n1 0 0 0 255 0\n1 1 0 0 0 255\n0 1 0 10 10 10\n3 0 1 2\n4 0 2 3 1\n');
  const a=parsePly(ascii);assert.deepEqual(a.indices,[0,1,2,0,2,3,0,3,1],'quads fan-triangulated');assert.deepEqual(a.colors.slice(0,3),[1,0,0]);
  const head='ply\nformat binary_little_endian 1.0\nelement vertex 3\nproperty float x\nproperty float y\nproperty float z\nelement face 1\nproperty list uchar uint vertex_indices\nend_header\n',buf=new Uint8Array(head.length+36+13),dv=new DataView(buf.buffer);
  buf.set(new TextEncoder().encode(head));[0,0,0,1,0,0,0,1,0].forEach((v,i)=>dv.setFloat32(head.length+i*4,v,true));dv.setUint8(head.length+36,3);[0,1,2].forEach((v,i)=>dv.setUint32(head.length+37+i*4,v,true));
  const b=parsePly(buf);assert.deepEqual(b.positions,[0,0,0,1,0,0,0,1,0]);assert.deepEqual(b.indices,[0,1,2]);assert.equal(b.colors,undefined);
  assert.throws(()=>parsePly(new TextEncoder().encode('ply\nformat ascii 1.0\nelement vertex 1\nproperty float x\nproperty float y\nproperty float z\nelement face 1\nproperty list uchar int vertex_indices\nend_header\n0 0 0\n3 0 1 2\n')),/nem létező csúcsra/);
  assert.throws(()=>parsePly(new TextEncoder().encode('obj')),/Nem PLY/);
  // the TSDF result back in the world, on a 60 cm table quad, as an exported scan would be
  const scan=scanHold(syntheticCapture(jug,{frames:32})),bs=scan.basis,w=(x,y,z)=>[0,1,2].map(k=>bs.origin[k]+bs.e1[k]*x+bs.e2[k]*y+bs.n[k]*z),pos=[];
  for(let i=0;i<scan.mesh.positions.length;i+=3)pos.push(...w(scan.mesh.positions[i],scan.mesh.positions[i+1],scan.mesh.positions[i+2]));const base=pos.length/3;for(const [x,y] of [[-.3,-.3],[.3,-.3],[.3,.3],[-.3,.3]])pos.push(...w(x,y,0));
  const r=plyHold({positions:pos,indices:[...scan.mesh.indices,base,base+1,base+2,base,base+2,base+3]});
  assert.ok(r.report.watertight,'watertight');for(let k=0;k<3;k++)assert.ok(Math.abs(r.report.size[k]-scan.report.size[k])<.003,`same size: ${r.report.size} vs ${scan.report.size}`);
  assert.ok(Math.abs(r.resolution-.002)<.001,'the file resolution is reported');
  // virtual depth of a single square seen straight on
  const sq={positions:[-.1,-.1,0,.1,-.1,0,.1,.1,0,-.1,.1,0],indices:[0,1,2,0,2,3]},pose=[1,0,0,0,0,-1,0,0,0,0,-1,0,0,0,.5,1],d=renderDepth(sq,pose,[100,100,50,50],100,100,100,100);
  assert.ok(Math.abs(d[50*100+50]-.5)<1e-9);assert.ok(Number.isNaN(d[0]),'outside the square: no depth');
});

test('the Roomantic library mesh (1 cm occupancy, iso = 1 on blurred hit counts) inflates well-seen surfaces: why the raw capture is used',()=>{
  const cap=syntheticCapture(jug,{frames:24}),occ=roomanticOccupancy(cap),rep=meshReport(occ);
  // board top is the world plane y = 0; take the occupancy surface's highest point over a board patch 12–20 cm from the hold
  let top=-Infinity;for(let i=0;i<occ.positions.length;i+=3){const x=occ.positions[i],z=occ.positions[i+2],r=Math.hypot(x,z);if(r>.12&&r<.2)top=Math.max(top,occ.positions[i+1]);}
  assert.ok(top>.01,`board surface raised by ${(top*1000).toFixed(0)} mm (≥ 1 voxel)`);assert.ok(rep.triangles>1000);
});

const sample=process.env.ROOMANTIC_SAMPLE;
test('real capture (Roomantic test set): multi-view depth agrees within LiDAR accuracy and the pipeline runs',{skip:!sample||!existsSync(sample)?'ROOMANTIC_SAMPLE not set':false},()=>{
  const files=new Map();for(const n of readdirSync(sample))if(/\.(bin|txt)$/.test(n))files.set(n,new Uint8Array(readFileSync(path.join(sample,n))));
  const frames=captureFrames(files);assert.equal(frames.length,105);assert.equal(frames[0].depthWidth,256);
  for(const [a,b] of [[0,5],[10,20],[50,70]]){const r=consistency(frames[a],frames[b]);assert.ok(r.median<.012,`${a}→${b}: ${r.median}`);}
  const r=scanHold(frames,{voxel:.008,radius:.45,cut:.012,margin:.03});assert.ok(r.report.watertight);assert.ok(Math.abs(r.plane.normal[1])>.99,'the floor is found (ARKit Y up)');
});
