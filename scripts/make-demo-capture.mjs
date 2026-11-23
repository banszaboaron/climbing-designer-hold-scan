// 1.3.0 · A demo capture in the Roomantic "videó" share format, without an iPhone: node scripts/make-demo-capture.mjs [jug|sloper|crimp] [out.zip] [frames]
// Simulated LiDAR depth (see src/domain/scan-synthetic.ts) + shaded colour frames (PNG bytes under .jpg names: browsers sniff the format)
// + camera_NNNN.txt as the app writes it. For trying "Fogás szkennelése" and for the public demo; it is NOT a real scan.
import {writeFile} from 'node:fs/promises';import {deflateSync} from 'node:zlib';
import {zipSync} from 'three/addons/libs/fflate.module.js';
import {TRUTH_HOLDS,syntheticCapture} from '../src/domain/scan-synthetic.ts';
import {depthBlob,cameraText} from '../src/domain/scan-capture.ts';

const [name='jug',out='demo-capture.zip',count='48']=process.argv.slice(2),hold=TRUTH_HOLDS.find(h=>h.name===name);
if(!hold)throw new Error(`Ismeretlen fogás: ${name} (${TRUTH_HOLDS.map(h=>h.name).join(', ')})`);
const frames=syntheticCapture(hold,{frames:Number(count)}),files={},W=480,H=360;
const crcTable=Array.from({length:256},(_,n)=>{let c=n;for(let k=0;k<8;k++)c=c&1?0xedb88320^(c>>>1):c>>>1;return c>>>0;});
const crc=b=>{let c=0xffffffff;for(const x of b)c=crcTable[(c^x)&255]^(c>>>8);return (c^0xffffffff)>>>0;};
function png(rgb,w,h){const raw=Buffer.alloc((w*3+1)*h);for(let y=0;y<h;y++){raw[y*(w*3+1)]=0;rgb.copy(raw,y*(w*3+1)+1,y*w*3,(y+1)*w*3);}
  const chunk=(type,data)=>{const len=Buffer.alloc(4);len.writeUInt32BE(data.length);const td=Buffer.concat([Buffer.from(type),data]),c=Buffer.alloc(4);c.writeUInt32BE(crc(td));return Buffer.concat([len,td,c]);};
  const ihdr=Buffer.alloc(13);ihdr.writeUInt32BE(w,0);ihdr.writeUInt32BE(h,4);ihdr[8]=8;ihdr[9]=2;return Buffer.concat([Buffer.from([137,80,78,71,13,10,26,10]),chunk('IHDR',ihdr),chunk('IDAT',deflateSync(raw)),chunk('IEND',Buffer.alloc(0))]);}
// Colour: sphere tracing of the same scene, Lambert shading; the hold is orange with darker grip texture, the board pale with a 2 cm grid.
const toHold=p=>[p[0],-p[2],p[1]],scene=p=>{const q=toHold(p);return Math.min(hold.sdf(q[0],q[1],q[2]),Math.max(Math.abs(q[0])-.3,Math.abs(q[1])-.3,q[2],-q[2]-.02));};
for(const f of frames){const rgb=Buffer.alloc(W*H*3),m=f.cameraToWorld,eye=[m[12],m[13],m[14]],[fx,fy,cx,cy]=f.k;
  for(let j=0;j<H;j++)for(let i=0;i<W;i++){const px=(i+.5)*f.imageWidth/W,py=(j+.5)*f.imageHeight/H,c=[(px-cx)/fx,(py-cy)/fy,1];let d=[m[0]*c[0]+m[4]*c[1]+m[8],m[1]*c[0]+m[5]*c[1]+m[9],m[2]*c[0]+m[6]*c[1]+m[10]];const l=Math.hypot(...d);d=d.map(v=>v/l);
    let t=.05,hit=false;for(let it=0;it<200&&t<2;it++){const v=scene([eye[0]+d[0]*t,eye[1]+d[1]*t,eye[2]+d[2]*t]);if(v<5e-5){hit=true;break;}t+=Math.max(v*.9,1e-5);}
    let col=[40,44,48];if(hit){const p=[eye[0]+d[0]*t,eye[1]+d[1]*t,eye[2]+d[2]*t],e=1e-4,n=[0,1,2].map(k=>{const a=p.slice(),b=p.slice();a[k]+=e;b[k]-=e;return scene(a)-scene(b);}),nl=Math.hypot(...n),q=toHold(p);
      const shade=.35+.65*Math.max(0,(n[0]*.3+n[1]*.8+n[2]*.5)/nl/Math.hypot(.3,.8,.5)),isHold=hold.sdf(q[0],q[1],q[2])<.0005&&q[2]>.0005,grain=(Math.sin(q[0]*900)*Math.sin(q[1]*900)>.6)?.85:1;
      const base=isHold?[232,116,59].map(v=>v*grain):(Math.abs(q[0]%.02)<.0008||Math.abs(q[1]%.02)<.0008?[150,150,145]:[222,218,206]);col=base.map(v=>Math.min(255,v*shade));}
    rgb[(j*W+i)*3]=col[0];rgb[(j*W+i)*3+1]=col[1];rgb[(j*W+i)*3+2]=col[2];}
  const n=String(f.index).padStart(4,'0');files[`demo_capture/frame_${n}.jpg`]=png(rgb,W,H);files[`demo_capture/depth_${n}.bin`]=depthBlob(f.depthWidth,f.depthHeight,f.depth);
  files[`demo_capture/camera_${n}.txt`]=new TextEncoder().encode(cameraText(f));}
await writeFile(out,zipSync(files,{level:6}));console.log(`${out}: ${frames.length} képkocka (${name}), szintetikus demó, nem valódi szkennelés.`);
