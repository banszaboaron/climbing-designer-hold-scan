// 1.3.0 · Repeatable scan benchmark on analytic holds (ground truth known): node scripts/scan-benchmark.mjs [--quick] [--json out.json]
// Methods: depth-capture TSDF (this release), the Roomantic library's occupancy mesh at 1 cm (re-implemented, then cleaned by the same
// PLY path), and the single-photo relief of 1.2.0 fed PERFECT relative depth and the TRUE length/thickness (its best case).
// Simulated LiDAR: 256×192 z-depth, Gaussian noise σ (at 32 cm, growing with d²), 3×3 smoothing; pose drift per frame.
import {writeFile} from 'node:fs/promises';
import {TRUTH_HOLDS,syntheticCapture,truthMesh,compareMeshes,topDownDisparity} from '../src/domain/scan-synthetic.ts';
import {scanHold,plyHold} from '../src/domain/hold-scan.ts';
import {roomanticOccupancy} from '../src/domain/roomantic-occupancy.ts';
import {holdMask,reliefMesh} from '../src/domain/photo-hold.ts';

const quick=process.argv.includes('--quick'),jsonAt=process.argv.indexOf('--json'),rows=[];
const scenarios=quick?[{label:'σ 1.5 mm, 48 frames',noise:.0015,frames:48,drift:0}]:[
  {label:'σ 1.5 mm, 48 frames',noise:.0015,frames:48,drift:0},
  {label:'σ 4 mm, 48 frames',noise:.004,frames:48,drift:0},
  {label:'σ 1.5 mm, 12 frames',noise:.0015,frames:12,drift:0},
  {label:'σ 1.5 mm, drift 1 mm',noise:.0015,frames:48,drift:.001},
];
const f=(x,d=1)=>Number.isFinite(x)?x.toFixed(d):'–';
for(const hold of TRUTH_HOLDS){const truth=truthMesh(hold),bb=[0,1,2].map(k=>{let lo=Infinity,hi=-Infinity;for(let i=k;i<truth.positions.length;i+=3){lo=Math.min(lo,truth.positions[i]);hi=Math.max(hi,truth.positions[i]);}return hi-lo;});
  for(const s of scenarios){const t0=performance.now(),capture=syntheticCapture(hold,{noise:s.noise,frames:s.frames,drift:s.drift}),simulate=(performance.now()-t0)/1000;
    const tsdf=scanHold(capture);rows.push({hold:hold.name,scenario:s.label,method:'TSDF 2 mm (1.3.0)',seconds:tsdf.seconds,watertight:tsdf.report.watertight,triangles:tsdf.report.triangles,...compareMeshes(tsdf.mesh,truth),simulate});
    if(s===scenarios[0]){
      // PLY path on a clean mesh: the TSDF result put back into the world on a 60 cm table quad (checks the import, not a scan).
      const b=tsdf.basis,w=(x,y,z)=>[0,1,2].map(k=>b.origin[k]+b.e1[k]*x+b.e2[k]*y+b.n[k]*z),pos=[];for(let i=0;i<tsdf.mesh.positions.length;i+=3)pos.push(...w(tsdf.mesh.positions[i],tsdf.mesh.positions[i+1],tsdf.mesh.positions[i+2]));
      const base=pos.length/3;for(const [x,y] of [[-.3,-.3],[.3,-.3],[.3,.3],[-.3,.3]])pos.push(...w(x,y,0));const t3=performance.now(),clean=plyHold({positions:pos,indices:[...tsdf.mesh.indices,base,base+1,base+2,base,base+2,base+3]});
      rows.push({hold:hold.name,scenario:s.label,method:'TSDF mesh as PLY + table → PLY path',seconds:(performance.now()-t3)/1000,watertight:clean.report.watertight,triangles:clean.report.triangles,...compareMeshes(clean.mesh,truth)});}
    if(s===scenarios[0]||!quick){const t1=performance.now(),occ=roomanticOccupancy(capture);
      try{const ply=plyHold({positions:occ.positions,indices:occ.indices},{voxel:.003}),sec=(performance.now()-t1)/1000;
        rows.push({hold:hold.name,scenario:s.label,method:'Roomantic occupancy 1 cm → PLY path',seconds:sec,watertight:ply.report.watertight,triangles:ply.report.triangles,...compareMeshes(ply.mesh,truth)});}
      catch(e){rows.push({hold:hold.name,scenario:s.label,method:'Roomantic occupancy 1 cm → PLY path',failed:e.message,seconds:(performance.now()-t1)/1000});}}
  }
  const t2=performance.now(),disp=topDownDisparity(hold),mask=holdMask(disp.depth,disp.width,disp.height,.5),relief=reliefMesh(mask,{lengthM:Math.max(bb[0],bb[1]),thicknessM:bb[2]}),sec=(performance.now()-t2)/1000;
  rows.push({hold:hold.name,scenario:'one top-down photo, perfect depth, true size given',method:'Single-photo relief (1.2.0), oracle',seconds:sec,watertight:null,triangles:relief.indices.length/3,...compareMeshes(relief,truth)});
}
const header='| Hold | Capture | Method | Mean dist. mm (acc / compl.) | 95% mm | F@2mm | Size error mm (x/y/z) | Volume % | Time s | Watertight |\n|---|---|---|---|---|---|---|---|---|---|';
const table=rows.map(r=>r.failed?`| ${r.hold} | ${r.scenario} | ${r.method} | failed: ${r.failed} | | | | | ${f(r.seconds,2)} | |`:`| ${r.hold} | ${r.scenario} | ${r.method} | ${f(r.accuracyMm,2)} / ${f(r.completenessMm,2)} | ${f(r.accuracy95Mm,1)} | ${f(r.fScore2mm,3)} | ${r.sizeErrorMm.map(x=>f(x,1)).join(' / ')} | ${f(r.volumeErrorPct,1)} | ${f(r.seconds,2)} | ${r.watertight===null?'–':r.watertight?'yes':'NO'} |`).join('\n');
console.log(header+'\n'+table);
if(jsonAt>0)await writeFile(process.argv[jsonAt+1],JSON.stringify({node:process.version,date:new Date().toISOString(),rows},null,1));
