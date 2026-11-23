// 1.3.0 · Ground truth for the scan benchmark and tests: analytic holds on a board, seen by a simulated LiDAR capture in the
// Roomantic format (256×192 z-depth, 1920×1440 intrinsics, ARKit world: metres, Y up). Sphere tracing of a signed distance function;
// optional Gaussian depth noise, 3×3 depth smoothing (as smoothedSceneDepth mixes edges) and per-frame pose drift.
import type {CaptureFrame} from './scan-capture.ts';
import {marchingCubes,rng,type ScanMesh,type V3} from './hold-scan.ts';

/** Hold coordinates: x long axis, y short axis, z up from the board (back on z = 0). */
export interface TruthHold {name:string;sdf:(x:number,y:number,z:number)=>number;bounds:[V3,V3]}
const ellipsoid=(x:number,y:number,z:number,a:number,b:number,c:number)=>{const k0=Math.hypot(x/a,y/b,z/c),k1=Math.hypot(x/(a*a),y/(b*b),z/(c*c));return k1?k0*(k0-1)/k1:-Math.min(a,b,c);};
const sphere=(x:number,y:number,z:number,r:number)=>Math.hypot(x,y,z)-r;
const smin=(a:number,b:number,k:number)=>{const h=Math.max(k-Math.abs(a-b),0)/k;return Math.min(a,b)-h*h*k/4;};
/** Three shapes of increasing difficulty: a smooth sloper, a jug with a real undercut, a small crimp with a 6 mm edge. */
export const TRUTH_HOLDS:TruthHold[]=[
  {name:'sloper',sdf:(x,y,z)=>Math.max(ellipsoid(x,y,z,.07,.05,.03),-z),bounds:[[-.07,-.05,0],[.07,.05,.03]]},
  {name:'jug',sdf:(x,y,z)=>Math.max(smin(ellipsoid(x,y,z,.06,.04,.03),sphere(x-.038,y,z-.032,.016),.006),-z),bounds:[[-.06,-.04,0],[.054,.04,.048]]},
  {name:'crimp',sdf:(x,y,z)=>{const box=Math.max(Math.abs(x)-.035,Math.abs(y)-.012,Math.abs(z-.009)-.009);return Math.max(smin(box,ellipsoid(x,y+.006,z,.03,.018,.024),.004),-z);},bounds:[[-.035,-.024,0],[.035,.012,.024]]},
];
/** Hold coordinates ↔ world (ARKit-like, Y up): world = (x, z, −y). e1 = +X, e2 = −Z, n = +Y (right-handed). */
const toHold=(p:V3):V3=>[p[0],-p[2],p[1]];

export interface CaptureOptions {frames?:number;distance?:number;elevation?:[number,number];noise?:number;smoothing?:boolean;drift?:number;seed?:number;turns?:number}
/** A capture circling the hold: a spiral of `frames` views from `elevation` (degrees above the board). */
export function syntheticCapture(hold:TruthHold,{frames=48,distance=.32,elevation=[30,70],noise=.0015,smoothing=true,drift=0,seed=3,turns=1.25}:CaptureOptions={}):CaptureFrame[]{
  const random=rng(seed),gauss=()=>{let u=0,v=0;while(!u)u=random();while(!v)v=random();return Math.sqrt(-2*Math.log(u))*Math.cos(2*Math.PI*v);};
  const scene=(p:V3)=>{const q=toHold(p),board=Math.max(Math.abs(q[0])-.3,Math.abs(q[1])-.3,q[2],-q[2]-.02);return Math.min(hold.sdf(q[0],q[1],q[2]),board);};
  const target:V3=[0,.015,0],out:CaptureFrame[]=[],W=1920,H=1440,dw=256,dh=192,k:[number,number,number,number]=[1340,1340,W/2,H/2];
  for(let n=0;n<frames;n++){const s=frames>1?n/(frames-1):0,az=2*Math.PI*turns*s,el=(elevation[0]+(elevation[1]-elevation[0])*(.5-.5*Math.cos(Math.PI*2*s)))*Math.PI/180;
    const eye:V3=[target[0]+distance*Math.cos(el)*Math.cos(az),target[1]+distance*Math.sin(el),target[2]+distance*Math.cos(el)*Math.sin(az)];
    const f=norm3(sub3(target,eye)),r=norm3(cross3(f,[0,1,0])),d=cross3(f,r);
    const pose=[r[0],r[1],r[2],0,d[0],d[1],d[2],0,f[0],f[1],f[2],0,eye[0],eye[1],eye[2],1];
    let depth=new Float32Array(dw*dh);
    for(let j=0;j<dh;j++)for(let i=0;i<dw;i++){const px=(i+.5)*W/dw,py=(j+.5)*H/dh,c:V3=[(px-k[2])/k[0],(py-k[3])/k[1],1],dir=norm3([r[0]*c[0]+d[0]*c[1]+f[0],r[1]*c[0]+d[1]*c[1]+f[1],r[2]*c[0]+d[2]*c[1]+f[2]]);
      let t=.05,hit=false;for(let it=0;it<300&&t<2;it++){const p:V3=[eye[0]+dir[0]*t,eye[1]+dir[1]*t,eye[2]+dir[2]*t],v=scene(p);if(v<2e-5){hit=true;break;}t+=Math.max(v*.9,1e-5);}
      depth[j*dw+i]=hit?t*(dir[0]*f[0]+dir[1]*f[1]+dir[2]*f[2]):NaN;}
    if(smoothing){const s2=new Float32Array(depth.length);for(let j=0;j<dh;j++)for(let i=0;i<dw;i++){let a=0,m=0;for(let b=-1;b<=1;b++)for(let c=-1;c<=1;c++){const x=i+c,y=j+b;if(x<0||y<0||x>=dw||y>=dh)continue;const v=depth[y*dw+x];if(Number.isFinite(v)){a+=v;m++;}}s2[j*dw+i]=Number.isFinite(depth[j*dw+i])&&m?a/m:NaN;}depth=s2;}
    if(noise>0)for(let i=0;i<depth.length;i++)if(Number.isFinite(depth[i]))depth[i]+=noise*(depth[i]/distance)**2*gauss();
    if(drift>0){pose[12]+=drift*gauss();pose[13]+=drift*gauss();pose[14]+=drift*gauss();}
    out.push({index:n+1,depth,depthWidth:dw,depthHeight:dh,imageWidth:W,imageHeight:H,k,cameraToWorld:pose,timestamp:n/30});}
  return out;
}
/** One straight-down view at Depth-Anything-like resolution, as relative inverse depth (larger = closer): the photo baseline's input with perfect depth. */
export function topDownDisparity(hold:TruthHold,{width=518,height=392,distance=.32}:{width?:number;height?:number;distance?:number}={}){
  const fx=width*.7,depth=new Float32Array(width*height);let lo=Infinity,hi=-Infinity;
  for(let j=0;j<height;j++)for(let i=0;i<width;i++){const x=(i+.5-width/2)/fx,y=-(j+.5-height/2)/fx,dir=norm3([x,y,-1]);
    let t=.05,hit=false;for(let it=0;it<300&&t<2;it++){const p:V3=[dir[0]*t,dir[1]*t,distance+dir[2]*t],v=Math.min(hold.sdf(p[0],p[1],p[2]),p[2]);if(v<2e-5){hit=true;break;}t+=Math.max(v*.9,1e-5);}
    const z=hit?t*-dir[2]:distance,disp=1/z;depth[j*width+i]=disp;lo=Math.min(lo,disp);hi=Math.max(hi,disp);}
  for(let i=0;i<depth.length;i++)depth[i]=(depth[i]-lo)/((hi-lo)||1);return {depth,width,height};
}
/** The truth as a dense mesh (0.5 mm), mounted like the scans: centred on X/Y, back on Z = 0. */
export function truthMesh(hold:TruthHold,voxel=.0005):ScanMesh{const [a,b]=hold.bounds,m=.003,min:V3=[a[0]-m,a[1]-m,-m],nx=Math.ceil((b[0]-a[0]+2*m)/voxel)+1,ny=Math.ceil((b[1]-a[1]+2*m)/voxel)+1,nz=Math.ceil((b[2]+2*m)/voxel)+1;
  const field=new Float32Array(nx*ny*nz);for(let k=0;k<nz;k++)for(let j=0;j<ny;j++)for(let i=0;i<nx;i++){const z=min[2]+k*voxel;field[(k*ny+j)*nx+i]=z<0?1:hold.sdf(min[0]+i*voxel,min[1]+j*voxel,z);}
  const mesh=marchingCubes({field,nx,ny,nz,origin:min,voxel});const p=mesh.positions;let x0=Infinity,x1=-Infinity,y0=Infinity,y1=-Infinity;
  for(let i=0;i<p.length;i+=3){x0=Math.min(x0,p[i]);x1=Math.max(x1,p[i]);y0=Math.min(y0,p[i+1]);y1=Math.max(y1,p[i+1]);if(p[i+2]<1e-4)p[i+2]=0;}
  for(let i=0;i<p.length;i+=3){p[i]-=(x0+x1)/2;p[i+1]-=(y0+y1)/2;}return mesh;}

export interface Comparison {accuracyMm:number;accuracy95Mm:number;completenessMm:number;fScore2mm:number;sizeErrorMm:V3;volumeErrorPct:number;rotation:number}
/**
 * Mesh vs truth, both mounted: symmetric nearest-surface-sample distances (dense area-weighted samples, spatial hash),
 * best of the two in-plane rotations (0°, 180°: the long axis has no preferred direction). F-score at 2 mm as in reconstruction benchmarks.
 */
export function compareMeshes(mesh:ScanMesh,truth:ScanMesh):Comparison{
  const sample=(m:ScanMesh,spacing:number)=>{const out:number[]=[],seen=new Set<string>(),p=m.positions,f=m.indices;for(let t=0;t<f.length;t+=3){const a=f[t]*3,b=f[t+1]*3,c=f[t+2]*3;
    const l=Math.max(Math.hypot(p[b]-p[a],p[b+1]-p[a+1],p[b+2]-p[a+2]),Math.hypot(p[c]-p[a],p[c+1]-p[a+1],p[c+2]-p[a+2])),s=Math.max(1,Math.ceil(l/spacing));
    for(let i=0;i<=s;i++)for(let j=0;j<=s-i;j++){const u=i/s,v=j/s,w=1-u-v,x=p[a]*w+p[b]*u+p[c]*v,y=p[a+1]*w+p[b+1]*u+p[c+1]*v,z=p[a+2]*w+p[b+2]*u+p[c+2]*v,k=`${Math.round(x*2e4)},${Math.round(y*2e4)},${Math.round(z*2e4)}`;if(!seen.has(k)){seen.add(k);out.push(x,y,z);}}}return out;};
  // Distances are clipped at CAP (10 mm): beyond that a point counts as a gross error, its exact distance does not matter.
  const CAP=.01,hashOf=(pts:number[],_cell?:number)=>kdTree(pts),nearest=(t:KdTree,_p:number[],_c:number,x:number,y:number,z:number)=>t.nearest(x,y,z,CAP);
  const cell=.002,tp=sample(truth,.001),th=hashOf(tp,cell),sizeOf=(p:number[])=>{const mn=[Infinity,Infinity,Infinity],mx=[-Infinity,-Infinity,-Infinity];for(let i=0;i<p.length;i+=3)for(let k=0;k<3;k++){mn[k]=Math.min(mn[k],p[i+k]);mx[k]=Math.max(mx[k],p[i+k]);}return [mx[0]-mn[0],mx[1]-mn[1],mx[2]-mn[2]] as V3;};
  let best:Comparison|null=null;
  for(const rotation of [0,180]){const s=rotation?-1:1,m0=sample(mesh,.001),mp=m0.map((v,i)=>i%3===2?v:v*s),mh=hashOf(mp,cell);
    const acc:number[]=[];for(let i=0;i<mp.length;i+=3)acc.push(nearest(th,tp,cell,mp[i],mp[i+1],mp[i+2]));
    const comp:number[]=[];for(let i=0;i<tp.length;i+=3)comp.push(nearest(mh,mp,cell,tp[i],tp[i+1],tp[i+2]));
    const mean=(a:number[])=>a.reduce((x,y)=>x+y,0)/a.length,pct=(a:number[],q:number)=>a.slice().sort((x,y)=>x-y)[Math.floor(a.length*q)];
    const prec=acc.filter(d=>d<.002).length/acc.length,rec=comp.filter(d=>d<.002).length/comp.length,ms=sizeOf(mesh.positions),ts=sizeOf(truth.positions);
    const vol=(m:ScanMesh)=>{let v=0;const p=m.positions,f=m.indices;for(let t=0;t<f.length;t+=3){const a=f[t]*3,b=f[t+1]*3,c=f[t+2]*3;v+=p[a]*(p[b+1]*p[c+2]-p[b+2]*p[c+1])-p[a+1]*(p[b]*p[c+2]-p[b+2]*p[c])+p[a+2]*(p[b]*p[c+1]-p[b+1]*p[c]);}return v/6;};
    const c:Comparison={accuracyMm:mean(acc)*1000,accuracy95Mm:pct(acc,.95)*1000,completenessMm:mean(comp)*1000,fScore2mm:prec+rec?2*prec*rec/(prec+rec):0,sizeErrorMm:[0,1,2].map(k=>(ms[k]-ts[k])*1000) as V3,volumeErrorPct:(vol(mesh)/vol(truth)-1)*100,rotation};
    if(!best||c.accuracyMm+c.completenessMm<best.accuracyMm+best.completenessMm)best=c;}
  return best!;
}
interface KdTree {nearest:(x:number,y:number,z:number,within?:number)=>number}
/** Static 3-d tree over a flat xyz array (median splits, implicit layout): exact nearest-neighbour distance. */
export function kdTree(pts:number[]):KdTree{const n=pts.length/3,order=new Int32Array(n),axis=new Uint8Array(n);for(let i=0;i<n;i++)order[i]=i;
  const build=(lo:number,hi:number,depth:number)=>{if(hi-lo<=1){if(hi>lo)axis[lo]=depth%3;return;}const mid=(lo+hi)>>1;
    // split along the widest extent of this subset (flat point sets would otherwise waste levels on a zero-width axis)
    const mn=[Infinity,Infinity,Infinity],mx=[-Infinity,-Infinity,-Infinity];for(let i=lo;i<hi;i++)for(let k=0;k<3;k++){const v=pts[order[i]*3+k];if(v<mn[k])mn[k]=v;if(v>mx[k])mx[k]=v;}
    const ext=[mx[0]-mn[0],mx[1]-mn[1],mx[2]-mn[2]],a=ext[0]>=ext[1]&&ext[0]>=ext[2]?0:ext[1]>=ext[2]?1:2;
    // nth_element by quickselect on the axis
    let l=lo,h=hi-1;while(l<h){const pivot=pts[order[(l+h)>>1]*3+a];let i=l,j=h;while(i<=j){while(pts[order[i]*3+a]<pivot)i++;while(pts[order[j]*3+a]>pivot)j--;if(i<=j){const t=order[i];order[i]=order[j];order[j]=t;i++;j--;}}if(mid<=j)h=j;else if(mid>=i)l=i;else break;}
    axis[mid]=a;build(lo,mid,depth+1);build(mid+1,hi,depth+1);};
  build(0,n,0);
  return {nearest(x,y,z,within=Infinity){let best=within;const q=[x,y,z];const visit=(lo:number,hi:number)=>{if(hi<=lo)return;const mid=(lo+hi)>>1,p=order[mid]*3,d=Math.hypot(pts[p]-x,pts[p+1]-y,pts[p+2]-z);if(d<best)best=d;if(hi-lo===1)return;
    const a=axis[mid],diff=q[a]-pts[p+a];if(diff<0){visit(lo,mid);if(-diff<best)visit(mid+1,hi);}else{visit(mid+1,hi);if(diff<best)visit(lo,mid);}};visit(0,n);return best;}};}
const sub3=(a:V3,b:V3):V3=>[a[0]-b[0],a[1]-b[1],a[2]-b[2]],cross3=(a:V3,b:V3):V3=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]],norm3=(a:V3):V3=>{const l=Math.hypot(a[0],a[1],a[2])||1;return [a[0]/l,a[1]/l,a[2]/l];};
