// 1.3.0 · Hold scan from the command line: node scripts/scan-cli.mjs <capture.zip | capture-folder | mesh.ply> [out.ply] [--voxel mm] [--unit m|cm|mm]
// Same pipeline as Út → Fogás szkennelése… (src/domain/hold-scan.ts). Writes a binary PLY in metres (X long axis, Z off the wall, flat back
// at Z = 0) and prints the mesh report as JSON. Colours need the photos decoded, which only the app does; the CLI output is geometry.
import {readFile,readdir,writeFile,stat} from 'node:fs/promises';import path from 'node:path';
import {unzipSync} from 'three/addons/libs/fflate.module.js';
import {captureFrames,parsePly} from '../src/domain/scan-capture.ts';
import {scanHold,plyHold} from '../src/domain/hold-scan.ts';

const args=process.argv.slice(2),flag=(name,d)=>{const i=args.indexOf(name);if(i<0)return d;const v=args[i+1];args.splice(i,2);return v;};
const voxel=Number(flag('--voxel','2'))/1000,unit=flag('--unit','auto'),[input,out='scan.ply']=args;
if(!input){console.error('Használat: node scripts/scan-cli.mjs <felvétel.zip | mappa | háló.ply> [ki.ply] [--voxel mm] [--unit m|cm|mm]');process.exit(2);}
let result,kind;
if(/\.ply$/i.test(input)){kind='PLY import (finished mesh, not a scan)';const mesh=parsePly(new Uint8Array(await readFile(input))),scale={m:1,cm:.01,mm:.001}[unit];
  if(scale)mesh.positions=mesh.positions.map(v=>v*scale);result=plyHold(mesh,{voxel});}
else{kind='LiDAR capture, TSDF fusion';const files=new Map();
  if((await stat(input)).isDirectory()){for(const name of await readdir(input))if(/\.(bin|txt)$/i.test(name))files.set(name,new Uint8Array(await readFile(path.join(input,name))));}
  else for(const [name,bytes] of Object.entries(unzipSync(new Uint8Array(await readFile(input)))))if(/\.(bin|txt)$/i.test(name))files.set(name,bytes);
  result=scanHold(captureFrames(files),{voxel});}
const {positions,indices}=result.mesh,n=positions.length/3,f=indices.length/3;
const header=`ply\nformat binary_little_endian 1.0\ncomment ${kind}; metres, X long axis, Z off the wall\nelement vertex ${n}\nproperty float x\nproperty float y\nproperty float z\nelement face ${f}\nproperty list uchar int vertex_indices\nend_header\n`;
const body=Buffer.alloc(n*12+f*13);let o=0;for(let i=0;i<n*3;i++){body.writeFloatLE(positions[i],o);o+=4;}
for(let t=0;t<f;t++){body.writeUInt8(3,o++);for(let k=0;k<3;k++){body.writeInt32LE(indices[t*3+k],o);o+=4;}}
await writeFile(out,Buffer.concat([Buffer.from(header),body]));
const size=[0,1,2].map(k=>{let lo=Infinity,hi=-Infinity;for(let i=k;i<positions.length;i+=3){lo=Math.min(lo,positions[i]);hi=Math.max(hi,positions[i]);}return +((hi-lo)*1000).toFixed(1);});
console.log(JSON.stringify({input,output:out,kind,sizeMm:size,seconds:+result.seconds.toFixed(2),noiseMm:result.noise===undefined?undefined:+(result.noise*1000).toFixed(2),cutMm:result.cut===undefined?undefined:+(result.cut*1000).toFixed(2),...result.report},null,1));
