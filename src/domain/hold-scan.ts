// 1.3.0 · Fogás szkennelése: a real hold from a LiDAR depth capture (ARKit smoothedSceneDepth + camera poses, see scan-capture.ts).
// The hold lies on a table (or hangs on a wall); the support plane becomes its flat back, as in "Fogás fotóból".
// Steps (pure geometry over typed arrays, Node-testable):
//   1. look-at point: least-squares closest point to all optical axes (the user keeps the hold in the middle while circling it);
//   2. support plane: RANSAC + least-squares refit on back-projected depth around that point, normal towards the cameras;
//   3. footprint: what rises above the plane, as a 2D grid component nearest the look-at point → region of interest (metric);
//   4. fusion: projective TSDF in plane coordinates with space carving: seen-free = outside, never-seen-but-occluded = solid
//      ("what is not seen is filled/flat"), below the cut height = outside, so the surface closes with a flat back;
//   5. marching cubes (three.js tables) with shared edge vertices → indexed, watertight mesh; largest component; light Taubin smoothing;
//   6. mount convention of the editor: metres, X = longest footprint axis, Y = n × X, Z = plane normal, back on Z = 0, centred.
// Scale is metric (ARKit depth), nothing is resized. Mesh validity is reported (boundary / non-manifold edges, volume).
import {edgeTable,triTable} from 'three/addons/objects/MarchingCubes.js';
import type {CaptureFrame,Mat4,PlyMesh} from './scan-capture.ts';

export type V3=[number,number,number];
export interface Plane {point:V3;normal:V3}
export interface Basis {origin:V3;e1:V3;e2:V3;n:V3}
export interface ScanMesh {positions:number[];indices:number[];colors?:number[]}
export interface ScanOptions {voxel?:number;truncation?:number;cut?:number;margin?:number;smooth?:number;radius?:number;pixelStep?:number;maxFrames?:number}
export interface ScanResult {mesh:ScanMesh;basis:Basis;report:MeshReport;lookAt:V3;plane:Plane;frames:number;grid:[number,number,number];voxel:number;cut:number;noise:number;seconds:number}
export interface MeshReport {vertices:number;triangles:number;boundaryEdges:number;nonManifoldEdges:number;degenerate:number;components:number;volume:number;size:V3;watertight:boolean}

const sub=(a:V3,b:V3):V3=>[a[0]-b[0],a[1]-b[1],a[2]-b[2]],add=(a:V3,b:V3):V3=>[a[0]+b[0],a[1]+b[1],a[2]+b[2]],mul=(a:V3,s:number):V3=>[a[0]*s,a[1]*s,a[2]*s];
const dot=(a:V3,b:V3)=>a[0]*b[0]+a[1]*b[1]+a[2]*b[2],cross=(a:V3,b:V3):V3=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
const norm=(a:V3)=>Math.hypot(a[0],a[1],a[2]),unit=(a:V3):V3=>mul(a,1/(norm(a)||1));
/** Column-major 4×4 applied to a point. */
export const apply=(m:Mat4,p:V3):V3=>[m[0]*p[0]+m[4]*p[1]+m[8]*p[2]+m[12],m[1]*p[0]+m[5]*p[1]+m[9]*p[2]+m[13],m[2]*p[0]+m[6]*p[1]+m[10]*p[2]+m[14]];
/** Inverse of a rigid column-major transform (rotation part orthonormal). */
export function rigidInverse(m:Mat4):Mat4{const r=[m[0],m[1],m[2],m[4],m[5],m[6],m[8],m[9],m[10]],t:V3=[m[12],m[13],m[14]];
  // R^T: column j of R^T = row j of R.
  const o=[r[0],r[3],r[6],0,r[1],r[4],r[7],0,r[2],r[5],r[8],0,0,0,0,1];const ti=apply(o,t);o[12]=-ti[0];o[13]=-ti[1];o[14]=-ti[2];return o;}
/** 3×3 symmetric solve (Cramer); null when singular. */
function solve3(a:number[][],b:V3):V3|null{const det=(m:number[][])=>m[0][0]*(m[1][1]*m[2][2]-m[1][2]*m[2][1])-m[0][1]*(m[1][0]*m[2][2]-m[1][2]*m[2][0])+m[0][2]*(m[1][0]*m[2][1]-m[1][1]*m[2][0]);
  const d=det(a);if(Math.abs(d)<1e-12)return null;return [0,1,2].map(k=>det(a.map((row,i)=>row.map((v,j)=>j===k?b[i]:v)))/d) as V3;}
/** Smallest-eigenvalue eigenvector of a symmetric 3×3 (Jacobi). */
export function smallestEigen(c:number[][]):{vector:V3;values:V3}{const a=c.map(r=>r.slice()),v=[[1,0,0],[0,1,0],[0,0,1]];
  for(let sweep=0;sweep<50;sweep++){let off=0;for(let p=0;p<3;p++)for(let q=p+1;q<3;q++)off+=a[p][q]**2;if(off<1e-20)break;
    for(let p=0;p<3;p++)for(let q=p+1;q<3;q++){if(Math.abs(a[p][q])<1e-30)continue;const th=(a[q][q]-a[p][p])/(2*a[p][q]),t=Math.sign(th||1)/(Math.abs(th)+Math.sqrt(th*th+1)),cs=1/Math.sqrt(t*t+1),sn=t*cs;
      for(let k=0;k<3;k++){const akp=a[k][p],akq=a[k][q];a[k][p]=cs*akp-sn*akq;a[k][q]=sn*akp+cs*akq;}
      for(let k=0;k<3;k++){const apk=a[p][k],aqk=a[q][k];a[p][k]=cs*apk-sn*aqk;a[q][k]=sn*apk+cs*aqk;}
      for(let k=0;k<3;k++){const vkp=v[k][p],vkq=v[k][q];v[k][p]=cs*vkp-sn*vkq;v[k][q]=sn*vkp+cs*vkq;}}}
  const values:V3=[a[0][0],a[1][1],a[2][2]],i=values.indexOf(Math.min(...values));return {vector:unit([v[0][i],v[1][i],v[2][i]]),values};}

/** World points of one frame's depth map (every `step`-th depth pixel), metres. */
export function framePoints(f:CaptureFrame,step=2,maxDepth=2):V3[]{const out:V3[]=[],[fx,fy,cx,cy]=f.k,sx=f.imageWidth/f.depthWidth,sy=f.imageHeight/f.depthHeight;
  for(let j=0;j<f.depthHeight;j+=step)for(let i=0;i<f.depthWidth;i+=step){const d=f.depth[j*f.depthWidth+i];if(!(d>0&&d<=maxDepth))continue;
    const px=(i+.5)*sx,py=(j+.5)*sy;out.push(apply(f.cameraToWorld,[(px-cx)/fx*d,(py-cy)/fy*d,d]));}return out;}
export const cameraCentre=(f:CaptureFrame):V3=>apply(f.cameraToWorld,[0,0,0]);
export const cameraAxis=(f:CaptureFrame):V3=>unit(sub(apply(f.cameraToWorld,[0,0,1]),cameraCentre(f)));

/** The point all optical axes pass closest to; falls back to the median centre-pixel depth point when the axes are nearly parallel. */
export function lookAtPoint(frames:CaptureFrame[]):V3{
  const a=[[0,0,0],[0,0,0],[0,0,0]],b:V3=[0,0,0];
  for(const f of frames){const o=cameraCentre(f),d=cameraAxis(f);for(let i=0;i<3;i++){for(let j=0;j<3;j++)a[i][j]+=(i===j?1:0)-d[i]*d[j];b[i]+=o[i]-d[i]*dot(d,o);}}
  const e=smallestEigen(a),spread=Math.min(...e.values)/Math.max(...e.values);
  const p=spread>.02?solve3(a,b):null;if(p&&frames.every(f=>dot(sub(p,cameraCentre(f)),cameraAxis(f))>0))return p;
  const centre=frames.map(f=>{const i=Math.floor(f.depthWidth/2),j=Math.floor(f.depthHeight/2),d=f.depth[j*f.depthWidth+i];return Number.isFinite(d)?add(cameraCentre(f),mul(cameraAxis(f),d)):null;}).filter((x):x is V3=>!!x);
  if(!centre.length)throw new Error('A felvétel közepén nincs mélységadat.');const med=(k:number)=>centre.map(p=>p[k]).sort((x,y)=>x-y)[centre.length>>1];return [med(0),med(1),med(2)];
}

/** Deterministic PRNG (tests and repeated runs give the same plane). */
export function rng(seed=1){let s=seed>>>0||1;return ()=>{s^=s<<13;s>>>=0;s^=s>>17;s^=s<<5;s>>>=0;return s/4294967296;};}
/** Dominant plane through the points (RANSAC, then least-squares refit on the inliers). */
export function fitPlane(points:V3[],{threshold=.004,iterations=400,seed=7}:{threshold?:number;iterations?:number;seed?:number}={}):{plane:Plane;inliers:number}{
  if(points.length<3)throw new Error('Túl kevés mélységpont a sík illesztéséhez.');const random=rng(seed);let best:Plane|null=null,count=-1;
  for(let it=0;it<iterations;it++){const p=points[Math.floor(random()*points.length)],q=points[Math.floor(random()*points.length)],r=points[Math.floor(random()*points.length)];
    const n=cross(sub(q,p),sub(r,p)),l=norm(n);if(l<1e-9)continue;const u=mul(n,1/l);let c=0;for(const x of points)if(Math.abs(dot(sub(x,p),u))<threshold)c++;if(c>count){count=c;best={point:p,normal:u};}}
  if(!best)throw new Error('Nem található sík a felvételen.');
  const inl=points.filter(x=>Math.abs(dot(sub(x,best!.point),best!.normal))<threshold*1.5),m:V3=[0,0,0];for(const x of inl){m[0]+=x[0];m[1]+=x[1];m[2]+=x[2];}const c=mul(m,1/inl.length),cov=[[0,0,0],[0,0,0],[0,0,0]];
  for(const x of inl){const d=sub(x,c);for(let i=0;i<3;i++)for(let j=0;j<3;j++)cov[i][j]+=d[i]*d[j];}
  return {plane:{point:c,normal:smallestEigen(cov).vector},inliers:inl.length};
}

/** Orthonormal plane frame: n towards `towards`, e1 arbitrary in-plane (replaced by the footprint's long axis later). */
export function planeBasis(plane:Plane,towards:V3):Basis{let n=plane.normal;if(dot(sub(towards,plane.point),n)<0)n=mul(n,-1);
  const helper:V3=Math.abs(n[0])<.9?[1,0,0]:[0,1,0],e1=unit(cross(helper,n)),e2=cross(n,e1);return {origin:plane.point,e1,e2,n};}
export const toPlane=(b:Basis,p:V3):V3=>{const d=sub(p,b.origin);return [dot(d,b.e1),dot(d,b.e2),dot(d,b.n)];};
export const fromPlane=(b:Basis,q:V3):V3=>add(b.origin,add(add(mul(b.e1,q[0]),mul(b.e2,q[1])),mul(b.n,q[2])));

export interface Footprint {basis:Basis;min:[number,number];max:[number,number];height:number;cells:number;mask:Uint8Array;cell:number;gx:number;gy:number;gmin:[number,number]}
/**
 * What rises above the plane near the look-at point: a 2D occupancy grid in plane coordinates (5 mm cells), the connected region
 * nearest that point. The basis is turned so e1 is the region's long (principal) axis; the bounds are in that basis.
 */
export function holdFootprint(points:V3[],basis0:Basis,lookAt:V3,{radius=.25,minHeight=.005,maxHeight=.4,cell=.005,quantile=.995}:{radius?:number;minHeight?:number;maxHeight?:number;cell?:number;quantile?:number}={}):Footprint{
  const c=toPlane(basis0,lookAt),raised:V3[]=[];for(const p of points){const q=toPlane(basis0,p);if(q[2]>minHeight&&q[2]<maxHeight&&Math.hypot(q[0]-c[0],q[1]-c[1])<radius)raised.push(q);}
  if(raised.length<20)throw new Error('A síkból nem emelkedik ki fogás a felvétel közepén. Tartsd középen a fogást, és járd körbe.');
  const g=Math.ceil(2*radius/cell),count=new Uint16Array(g*g),x0=c[0]-radius,y0=c[1]-radius;
  for(const q of raised){const i=Math.floor((q[0]-x0)/cell),j=Math.floor((q[1]-y0)/cell);if(i>=0&&j>=0&&i<g&&j<g&&count[j*g+i]<65535)count[j*g+i]++;}
  const on=new Uint8Array(g*g);for(let i=0;i<on.length;i++)on[i]=count[i]>=2?1:0;
  // Connected components (8-neighbour); score = size, discounted by distance from the look-at point.
  const label=new Int32Array(g*g).fill(-1);let best=-1,bestScore=-1,id=0;
  for(let s=0;s<on.length;s++){if(!on[s]||label[s]>=0)continue;const stack=[s];label[s]=id;let n=0,mx=0,my=0;
    while(stack.length){const k=stack.pop()!,i=k%g,j=(k-i)/g;n++;mx+=i;my+=j;for(let dj=-1;dj<=1;dj++)for(let di=-1;di<=1;di++){const ii=i+di,jj=j+dj;if(ii<0||jj<0||ii>=g||jj>=g)continue;const t=jj*g+ii;if(on[t]&&label[t]<0){label[t]=id;stack.push(t);}}}
    const d=Math.hypot(mx/n-g/2,my/n-g/2)*cell,score=n/(1+(d/.05)**2);if(score>bestScore){bestScore=score;best=id;}id++;}
  // Principal axis of the region's cells → e1.
  let sx=0,sy=0,sxx=0,syy=0,sxy=0,n=0;for(let k=0;k<label.length;k++)if(label[k]===best){const x=x0+(k%g+.5)*cell,y=y0+(Math.floor(k/g)+.5)*cell;sx+=x;sy+=y;sxx+=x*x;syy+=y*y;sxy+=x*y;n++;}
  const mx=sx/n,my=sy/n,cxx=sxx/n-mx*mx,cyy=syy/n-my*my,cxy=sxy/n-mx*my,angle=.5*Math.atan2(2*cxy,cxx-cyy),ca=Math.cos(angle),sa=Math.sin(angle);
  const e1=unit(add(mul(basis0.e1,ca),mul(basis0.e2,sa))),basis:Basis={origin:basis0.origin,e1,e2:cross(basis0.n,e1),n:basis0.n};
  const min:[number,number]=[Infinity,Infinity],max:[number,number]=[-Infinity,-Infinity],heights:number[]=[];
  for(let k=0;k<label.length;k++)if(label[k]===best){const x=x0+(k%g+.5)*cell,y=y0+(Math.floor(k/g)+.5)*cell,q=toPlane(basis,fromPlane(basis0,[x,y,0]));for(const t of [0,1]){min[t]=Math.min(min[t],q[t]-cell/2);max[t]=Math.max(max[t],q[t]+cell/2);}}
  for(const q of raised){const i=Math.floor((q[0]-x0)/cell),j=Math.floor((q[1]-y0)/cell);if(i>=0&&j>=0&&i<g&&j<g&&label[j*g+i]===best)heights.push(q[2]);}
  heights.sort((a,b)=>a-b);const mask=new Uint8Array(g*g);for(let k=0;k<label.length;k++)mask[k]=label[k]===best?1:0;
  return {basis,min,max,height:heights[Math.min(heights.length-1,Math.floor(heights.length*quantile))]??minHeight,cells:n,mask,cell,gx:g,gy:g,gmin:[x0,y0]};
}

/** The footprint mask (in the original look-at grid) as a test on plane coordinates of `basis`, grown by `grow` metres. */
function footprintTest(fp:Footprint,basis0:Basis,grow:number){const g=fp.gx,r=Math.ceil(grow/fp.cell),grown=new Uint8Array(g*g);
  for(let k=0;k<fp.mask.length;k++)if(fp.mask[k]){const i=k%g,j=(k-i)/g;for(let dj=-r;dj<=r;dj++)for(let di=-r;di<=r;di++){const ii=i+di,jj=j+dj;if(ii>=0&&jj>=0&&ii<g&&jj<g&&di*di+dj*dj<=r*r+r)grown[jj*g+ii]=1;}}
  return (q:V3)=>{const p=toPlane(basis0,fromPlane(fp.basis,q)),i=Math.floor((p[0]-fp.gmin[0])/fp.cell),j=Math.floor((p[1]-fp.gmin[1])/fp.cell);return i>=0&&j>=0&&i<g&&j<g&&grown[j*g+i]===1;};}

/**
 * Projective TSDF over a box in plane coordinates. Returns the field ready for marching cubes (negative = solid):
 * observed voxels keep their averaged truncated distance; voxels only ever occluded count as solid; voxels never in view,
 * outside the grown footprint or at/below the cut height are empty, so the surface closes with a flat back.
 */
export function fuseTsdf(frames:CaptureFrame[],basis:Basis,box:{min:V3;max:V3},{voxel=.002,truncation=.008,cut=.003,inside}:{voxel?:number;truncation?:number;cut?:number;inside?:(q:V3)=>boolean}){
  const nx=Math.max(2,Math.ceil((box.max[0]-box.min[0])/voxel)+1),ny=Math.max(2,Math.ceil((box.max[1]-box.min[1])/voxel)+1),nz=Math.max(2,Math.ceil((box.max[2]-box.min[2])/voxel)+1);
  if(nx*ny*nz>12e6)throw new Error('Túl nagy kivágás ehhez a felbontáshoz. Csökkentsd a sugarat vagy növeld a voxelméretet.');
  const N=nx*ny*nz,tsdf=new Float32Array(N),weight=new Float32Array(N),occluded=new Uint8Array(N);
  for(const f of frames){
    // plane coords → camera coords as one affine map: cam = inv(C)·(origin + x e1 + y e2 + z n)
    const inv=rigidInverse(f.cameraToWorld),o=apply(inv,basis.origin),r=(v:V3):V3=>sub(apply(inv,add(basis.origin,v)),o);
    const ax=r(basis.e1),ay=r(basis.e2),az=r(basis.n),[fx,fy,cx,cy]=f.k,dw=f.depthWidth,dh=f.depthHeight,sx=dw/f.imageWidth,sy=dh/f.imageHeight;
    for(let k=0;k<nz;k++){const z=box.min[2]+k*voxel;for(let j=0;j<ny;j++){const y=box.min[1]+j*voxel;
      let px=o[0]+ay[0]*y+az[0]*z+ax[0]*box.min[0],py=o[1]+ay[1]*y+az[1]*z+ax[1]*box.min[0],pz=o[2]+ay[2]*y+az[2]*z+ax[2]*box.min[0];const dx=ax[0]*voxel,dy=ax[1]*voxel,dz=ax[2]*voxel;
      for(let i=0;i<nx;i++,px+=dx,py+=dy,pz+=dz){if(pz<.05)continue;const u=(fx*px/pz+cx)*sx,v=(fy*py/pz+cy)*sy;if(u<0||v<0||u>=dw||v>=dh)continue;
        const d=f.depth[Math.floor(v)*dw+Math.floor(u)];if(!(d>0))continue;const sdf=d-pz,idx=(k*ny+j)*nx+i;
        if(sdf<-truncation){occluded[idx]=1;continue;}const val=sdf>truncation?1:sdf/truncation,w=weight[idx];tsdf[idx]=(tsdf[idx]*w+val)/(w+1);weight[idx]=w+1;}}}}
  const field=new Float32Array(N);
  for(let k=0;k<nz;k++){const z=box.min[2]+k*voxel;for(let j=0;j<ny;j++)for(let i=0;i<nx;i++){const idx=(k*ny+j)*nx+i;
    const border=i===0||j===0||k===0||i===nx-1||j===ny-1||k===nz-1;
    field[idx]=border||z<=cut||(inside&&!inside([box.min[0]+i*voxel,box.min[1]+j*voxel,z]))?1:weight[idx]>0?tsdf[idx]:occluded[idx]?-1:1;}}
  return {field,nx,ny,nz,origin:box.min,voxel};
}

/** Marching cubes on a scalar grid (iso 0, negative = solid). Shared edge vertices → indexed mesh; outward winding. */
export function marchingCubes({field,nx,ny,nz,origin,voxel}:{field:Float32Array;nx:number;ny:number;nz:number;origin:V3;voxel:number},iso=0):ScanMesh{
  const positions:number[]=[],indices:number[]=[],cache=new Map<number,number>(),at=(i:number,j:number,k:number)=>field[(k*ny+j)*nx+i];
  // Bourke corner order (three.js uses the same tables): 0 (0,0,0) 1 (1,0,0) 2 (1,1,0) 3 (0,1,0) 4..7 the same at z+1.
  const corner=[[0,0,0],[1,0,0],[1,1,0],[0,1,0],[0,0,1],[1,0,1],[1,1,1],[0,1,1]],edges=[[0,1],[1,2],[3,2],[0,3],[4,5],[5,6],[7,6],[4,7],[0,4],[1,5],[2,6],[3,7]];
  const vertex=(i:number,j:number,k:number,e:number,val:number[])=>{const [a,b]=edges[e],ca=corner[a],cb=corner[b],ii=i+ca[0],jj=j+ca[1],kk=k+ca[2],axis=cb[0]-ca[0]?0:cb[1]-ca[1]?1:2,key=((kk*ny+jj)*nx+ii)*3+axis;
    let v=cache.get(key);if(v===undefined){const t=(iso-val[a])/((val[b]-val[a])||1e-12);v=positions.length/3;
      positions.push(origin[0]+(ii+(axis===0?t:0))*voxel,origin[1]+(jj+(axis===1?t:0))*voxel,origin[2]+(kk+(axis===2?t:0))*voxel);cache.set(key,v);}return v;};
  const val=new Array<number>(8),ids=new Array<number>(12);
  for(let k=0;k<nz-1;k++)for(let j=0;j<ny-1;j++)for(let i=0;i<nx-1;i++){let index=0;
    for(let c=0;c<8;c++){val[c]=at(i+corner[c][0],j+corner[c][1],k+corner[c][2]);if(val[c]<iso)index|=1<<c;}
    const mask=edgeTable[index];if(!mask)continue;for(let e=0;e<12;e++)if(mask&(1<<e))ids[e]=vertex(i,j,k,e,val);
    for(let t=index*16;triTable[t]!==-1;t+=3)indices.push(ids[triTable[t]],ids[triTable[t+1]],ids[triTable[t+2]]);}
  const mesh={positions,indices};if(signedVolume(mesh)<0)for(let t=0;t<indices.length;t+=3){const x=indices[t+1];indices[t+1]=indices[t+2];indices[t+2]=x;}
  return dropDegenerate(mesh);
}
export function signedVolume({positions:p,indices:f}:ScanMesh){let v=0;for(let t=0;t<f.length;t+=3){const a=f[t]*3,b=f[t+1]*3,c=f[t+2]*3;
  v+=p[a]*(p[b+1]*p[c+2]-p[b+2]*p[c+1])-p[a+1]*(p[b]*p[c+2]-p[b+2]*p[c])+p[a+2]*(p[b]*p[c+1]-p[b+1]*p[c]);}return v/6;}
/** Removes triangles with a repeated vertex and the vertices nothing uses any more. */
export function dropDegenerate({positions,indices,colors}:ScanMesh):ScanMesh{const keep:number[]=[];for(let t=0;t<indices.length;t+=3){const a=indices[t],b=indices[t+1],c=indices[t+2];if(a!==b&&b!==c&&a!==c)keep.push(a,b,c);}
  const map=new Map<number,number>(),p:number[]=[],col:number[]=[];const idx=keep.map(i=>{let j=map.get(i);if(j===undefined){j=p.length/3;map.set(i,j);p.push(positions[i*3],positions[i*3+1],positions[i*3+2]);if(colors)col.push(colors[i*3],colors[i*3+1],colors[i*3+2]);}return j;});
  return colors?{positions:p,indices:idx,colors:col}:{positions:p,indices:idx};}
/** Merges vertices closer than `tolerance` (a triangle-soup PLY becomes an indexed mesh). */
export function weld(m:ScanMesh,tolerance=1e-6):ScanMesh{const map=new Map<string,number>(),remap=new Int32Array(m.positions.length/3),p:number[]=[],col:number[]=[];
  for(let i=0;i<remap.length;i++){const key=`${Math.round(m.positions[i*3]/tolerance)},${Math.round(m.positions[i*3+1]/tolerance)},${Math.round(m.positions[i*3+2]/tolerance)}`;let j=map.get(key);
    if(j===undefined){j=p.length/3;map.set(key,j);p.push(m.positions[i*3],m.positions[i*3+1],m.positions[i*3+2]);if(m.colors)col.push(m.colors[i*3],m.colors[i*3+1],m.colors[i*3+2]);}remap[i]=j;}
  return dropDegenerate({positions:p,indices:m.indices.map(i=>remap[i]),colors:m.colors?col:undefined});}
/** Keeps the connected component with the most triangles. */
export function largestComponent(m:ScanMesh):ScanMesh{const n=m.positions.length/3,parent=new Int32Array(n);for(let i=0;i<n;i++)parent[i]=i;
  const find=(x:number):number=>{while(parent[x]!==x){parent[x]=parent[parent[x]];x=parent[x];}return x;};
  for(let t=0;t<m.indices.length;t+=3){const a=find(m.indices[t]),b=find(m.indices[t+1]),c=find(m.indices[t+2]);parent[b]=a;parent[find(c)]=a;}
  const size=new Map<number,number>();for(let t=0;t<m.indices.length;t+=3){const r=find(m.indices[t]);size.set(r,(size.get(r)??0)+1);}
  let best=-1,most=-1;for(const [r,s] of size)if(s>most){most=s;best=r;}
  return dropDegenerate({...m,indices:m.indices.filter((_,i,a)=>find(a[i-i%3])===best)});}
/** Taubin λ/μ smoothing (no shrinkage); vertices on the back plane (z ≤ fixedZ) stay in it. */
export function taubin(m:ScanMesh,iterations=4,fixedZ=-Infinity,lambda=.5,mu=-.53):ScanMesh{const n=m.positions.length/3,nb:Set<number>[]=Array.from({length:n},()=>new Set());
  for(let t=0;t<m.indices.length;t+=3)for(let e=0;e<3;e++){const a=m.indices[t+e],b=m.indices[t+(e+1)%3];nb[a].add(b);nb[b].add(a);}
  let p=Float64Array.from(m.positions);const step=(f:number)=>{const q=Float64Array.from(p);for(let i=0;i<n;i++){if(!nb[i].size)continue;let x=0,y=0,z=0;for(const j of nb[i]){x+=p[j*3];y+=p[j*3+1];z+=p[j*3+2];}const k=nb[i].size;
    q[i*3]+=f*(x/k-p[i*3]);q[i*3+1]+=f*(y/k-p[i*3+1]);q[i*3+2]=p[i*3+2]<=fixedZ?p[i*3+2]:p[i*3+2]+f*(z/k-p[i*3+2]);}p=q;};
  for(let i=0;i<iterations;i++){step(lambda);step(mu);}return {...m,positions:Array.from(p)};}

/** Edge-based validity: a closed 2-manifold has every edge in exactly two triangles. */
export function meshReport(m:ScanMesh):MeshReport{const edges=new Map<string,number>();let degenerate=0;const p=m.positions;
  for(let t=0;t<m.indices.length;t+=3){const [a,b,c]=[m.indices[t],m.indices[t+1],m.indices[t+2]];
    const ab:V3=[p[b*3]-p[a*3],p[b*3+1]-p[a*3+1],p[b*3+2]-p[a*3+2]],ac:V3=[p[c*3]-p[a*3],p[c*3+1]-p[a*3+1],p[c*3+2]-p[a*3+2]];if(norm(cross(ab,ac))<1e-14)degenerate++;
    for(const [x,y] of [[a,b],[b,c],[c,a]]){const k=x<y?`${x},${y}`:`${y},${x}`;edges.set(k,(edges.get(k)??0)+1);}}
  let boundaryEdges=0,nonManifoldEdges=0;for(const c of edges.values()){if(c===1)boundaryEdges++;else if(c>2)nonManifoldEdges++;}
  const n=p.length/3,parent=new Int32Array(n);for(let i=0;i<n;i++)parent[i]=i;const find=(x:number):number=>{while(parent[x]!==x){parent[x]=parent[parent[x]];x=parent[x];}return x;};
  for(let t=0;t<m.indices.length;t+=3){const a=find(m.indices[t]);parent[find(m.indices[t+1])]=a;parent[find(m.indices[t+2])]=a;}
  const roots=new Set<number>();for(const i of m.indices)roots.add(find(i));
  const min=[Infinity,Infinity,Infinity],max=[-Infinity,-Infinity,-Infinity];for(let i=0;i<p.length;i+=3)for(let k=0;k<3;k++){min[k]=Math.min(min[k],p[i+k]);max[k]=Math.max(max[k],p[i+k]);}
  return {vertices:n,triangles:m.indices.length/3,boundaryEdges,nonManifoldEdges,degenerate,components:roots.size,volume:signedVolume(m),size:[max[0]-min[0],max[1]-min[1],max[2]-min[2]],watertight:boundaryEdges===0&&nonManifoldEdges===0};}

/** Plane coordinates → mount convention: centred on X/Y, back (plane) on Z = 0. Returns the shift used. */
export function mountScan(m:ScanMesh):{mesh:ScanMesh;shift:V3}{const p=m.positions;let x0=Infinity,x1=-Infinity,y0=Infinity,y1=-Infinity;
  for(let i=0;i<p.length;i+=3){x0=Math.min(x0,p[i]);x1=Math.max(x1,p[i]);y0=Math.min(y0,p[i+1]);y1=Math.max(y1,p[i+1]);}
  const shift:V3=[-(x0+x1)/2,-(y0+y1)/2,0],out=p.slice();for(let i=0;i<out.length;i+=3){out[i]+=shift[0];out[i+1]+=shift[1];}return {mesh:{...m,positions:out},shift};}

/**
 * The full capture → hold pipeline. `cut` (m above the support plane) is where the hold is separated from the table; the
 * flat back is put back on the plane (Z = 0), so the thickness is not reduced by it.
 */
export function scanHold(frames:CaptureFrame[],options:ScanOptions={}):ScanResult{
  const started=performance.now(),{voxel=.002,truncation:trunc0=Math.max(.006,voxel*3),cut:cut0=.004,margin=.012,smooth=3,radius=.25,pixelStep=2,maxFrames=120}=options;
  const used=frames.length>maxFrames?frames.filter((_,i)=>i%Math.ceil(frames.length/maxFrames)===0):frames;
  if(used.length<3)throw new Error('Legalább 3 képkocka kell: járd körbe a fogást.');
  const lookAt=lookAtPoint(used),near:V3[]=[];for(const f of used.filter((_,i)=>i%Math.max(1,Math.floor(used.length/24))===0))for(const p of framePoints(f,pixelStep))if(norm(sub(p,lookAt))<radius*1.2)near.push(p);
  const towards=mul(used.reduce((s,f)=>add(s,cameraCentre(f)),[0,0,0] as V3),1/used.length);
  const {plane}=fitPlane(near.filter((_,i)=>i%Math.max(1,Math.floor(near.length/6000))===0),{threshold:Math.max(.003,voxel*1.5)});
  // Depth noise measured on the table itself: below the plane there is only noise (the hold is above), so the RMS of the
  // negative side is an unbiased σ. The cut and the TSDF truncation follow it, else table noise would count as hold.
  const basisN=planeBasis(plane,towards);let ss=0,sn=0;for(const p of near){const h=dot(sub(p,basisN.origin),basisN.n);if(h<0&&h>-.03){ss+=h*h;sn++;}}
  const noise=sn>50?Math.sqrt(ss/sn):0,cut=Math.max(cut0,2.5*noise),truncation=Math.max(trunc0,3*noise);
  const basis0=basisN,fp=holdFootprint(near,basis0,lookAt,{radius,minHeight:cut+.001});
  const basis=fp.basis,box={min:[fp.min[0]-margin,fp.min[1]-margin,0] as V3,max:[fp.max[0]+margin,fp.max[1]+margin,fp.height+margin] as V3};
  const grid=fuseTsdf(used,basis,box,{voxel,truncation,cut,inside:footprintTest(fp,basis0,margin*.75)});
  let mesh=largestComponent(marchingCubes(grid));
  // The flat back: everything at the cut layer goes onto the plane itself.
  const flat=cut+voxel*.75,p=mesh.positions;for(let i=2;i<p.length;i+=3)if(p[i]<=flat)p[i]=0;
  mesh=dropDegenerate(mesh);if(smooth>0)mesh=taubin(mesh,smooth,1e-9);
  const mounted=mountScan(mesh),shift=mounted.shift,report=meshReport(mounted.mesh);
  if(report.triangles<20)throw new Error('A fogás nem rakható össze a felvételből. Járd körbe lassabban, 25–40 cm-ről.');
  const b:Basis={...basis,origin:fromPlane(basis,[-shift[0],-shift[1],0])};
  return {mesh:mounted.mesh,basis:b,report,lookAt,plane,frames:used.length,grid:[grid.nx,grid.ny,grid.nz],voxel,cut,noise,seconds:(performance.now()-started)/1000};
}

/**
 * Vertex colours from the capture's photos: each vertex takes the colour of the frames that see it head-on and unoccluded
 * (depth agrees within `tolerance`), weighted by how frontal the view is. `sample` reads an RGB (0…1) at colour-image pixels.
 */
export function colorize(m:ScanMesh,basis:Basis,frames:CaptureFrame[],sample:(frame:number,x:number,y:number)=>V3|null,tolerance=.01):number[]{
  const n=m.positions.length/3,normals=new Float64Array(n*3),p=m.positions;
  for(let t=0;t<m.indices.length;t+=3){const [a,b,c]=[m.indices[t],m.indices[t+1],m.indices[t+2]],ab:V3=[p[b*3]-p[a*3],p[b*3+1]-p[a*3+1],p[b*3+2]-p[a*3+2]],ac:V3=[p[c*3]-p[a*3],p[c*3+1]-p[a*3+1],p[c*3+2]-p[a*3+2]],f=cross(ab,ac);for(const v of [a,b,c])for(let k=0;k<3;k++)normals[v*3+k]+=f[k];}
  const sum=new Float64Array(n*3),wsum=new Float64Array(n),cams=frames.map(f=>({f,inv:rigidInverse(f.cameraToWorld),c:cameraCentre(f)}));
  for(let v=0;v<n;v++){const world=fromPlane(basis,[p[v*3],p[v*3+1],p[v*3+2]]),nw=unit(sub(fromPlane(basis,add([p[v*3],p[v*3+1],p[v*3+2]],unit([normals[v*3],normals[v*3+1],normals[v*3+2]]))),world));
    for(const [fi,{f,inv,c}] of cams.entries()){const cos=dot(nw,unit(sub(c,world)));if(cos<.2)continue;const q=apply(inv,world);if(q[2]<.05)continue;const [fx,fy,cx,cy]=f.k,x=fx*q[0]/q[2]+cx,y=fy*q[1]/q[2]+cy;
      if(x<0||y<0||x>=f.imageWidth||y>=f.imageHeight)continue;const d=f.depth[Math.floor(y*f.depthHeight/f.imageHeight)*f.depthWidth+Math.floor(x*f.depthWidth/f.imageWidth)];if(!(Math.abs(d-q[2])<tolerance))continue;
      const rgb=sample(fi,x,y);if(!rgb)continue;const w=cos*cos;for(let k=0;k<3;k++)sum[v*3+k]+=rgb[k]*w;wsum[v]+=w;}}
  const out:number[]=[];let gray=0,seen=0;for(let v=0;v<n;v++)if(wsum[v]){gray+=(sum[v*3]+sum[v*3+1]+sum[v*3+2])/3/wsum[v];seen++;}const fill=seen?gray/seen:.55;
  for(let v=0;v<n;v++)for(let k=0;k<3;k++)out.push(wsum[v]?sum[v*3+k]/wsum[v]:fill);return out;
}

/** Software z-buffer: the mesh's z-depth as a camera with pose `cameraToWorld` and intrinsics `k` would measure it (NaN = nothing). */
export function renderDepth(m:ScanMesh,cameraToWorld:Mat4,k:[number,number,number,number],imageWidth:number,imageHeight:number,dw:number,dh:number){
  const inv=rigidInverse(cameraToWorld),[fx,fy,cx,cy]=k,sx=dw/imageWidth,sy=dh/imageHeight,depth=new Float32Array(dw*dh).fill(Infinity),p=m.positions,n=p.length/3;
  const u=new Float64Array(n),v=new Float64Array(n),z=new Float64Array(n);for(let i=0;i<n;i++){const q=apply(inv,[p[i*3],p[i*3+1],p[i*3+2]]);z[i]=q[2];u[i]=(fx*q[0]/q[2]+cx)*sx;v[i]=(fy*q[1]/q[2]+cy)*sy;}
  for(let t=0;t<m.indices.length;t+=3){const a=m.indices[t],b=m.indices[t+1],c=m.indices[t+2];if(z[a]<.02||z[b]<.02||z[c]<.02)continue;
    const area=(u[b]-u[a])*(v[c]-v[a])-(u[c]-u[a])*(v[b]-v[a]);if(Math.abs(area)<1e-12)continue;
    const x0=Math.max(0,Math.floor(Math.min(u[a],u[b],u[c]))),x1=Math.min(dw-1,Math.ceil(Math.max(u[a],u[b],u[c]))),y0=Math.max(0,Math.floor(Math.min(v[a],v[b],v[c]))),y1=Math.min(dh-1,Math.ceil(Math.max(v[a],v[b],v[c])));
    for(let y=y0;y<=y1;y++)for(let x=x0;x<=x1;x++){const px=x+.5,py=y+.5,w0=((u[b]-px)*(v[c]-py)-(u[c]-px)*(v[b]-py))/area,w1=((u[c]-px)*(v[a]-py)-(u[a]-px)*(v[c]-py))/area,w2=1-w0-w1;
      if(w0<0||w1<0||w2<0)continue;const d=1/(w0/z[a]+w1/z[b]+w2/z[c]),i=y*dw+x;if(d<depth[i])depth[i]=d;}}
  for(let i=0;i<depth.length;i++)if(depth[i]===Infinity)depth[i]=NaN;return depth;
}

/**
 * A finished mesh (e.g. the Roomantic app's object.ply, ARKit world, metres): support plane and hold region are found on the mesh,
 * then the mesh is "re-captured" by 48 virtual depth cameras circling the hold and goes through the same TSDF path as a real capture
 * (so undercuts, the flat back and the validity checks behave the same). Coarse input stays coarse: no detail is added.
 */
export function plyHold(ply:PlyMesh,{voxel=.002,cut:cut0=.004,margin=.012,smooth=2,radius=.25}:{voxel?:number;cut?:number;margin?:number;smooth?:number;radius?:number}={}):Omit<ScanResult,'lookAt'>&{resolution:number}{
  const started=performance.now(),mesh0=weld(ply),pts:V3[]=[];for(let i=0;i<mesh0.positions.length;i+=3)pts.push([mesh0.positions[i],mesh0.positions[i+1],mesh0.positions[i+2]]);
  if(pts.length<10||mesh0.indices.length<3)throw new Error('A PLY-fájlban túl kevés háromszög van.');
  // The file's own resolution (median edge): a 1 cm voxel mesh has a wavy table and cannot be cut closer than about one cell.
  const lengths:number[]=[];for(let t=0;t<mesh0.indices.length&&lengths.length<20000;t+=3)lengths.push(norm(sub(pts[mesh0.indices[t]],pts[mesh0.indices[t+1]])));
  const resolution=lengths.sort((a,b)=>a-b)[lengths.length>>1]??voxel,cut=Math.max(cut0,resolution*1.2);
  // Surface samples (triangle interiors) for the plane and the footprint.
  const samples:V3[]=[],f=mesh0.indices,spacing=Math.max(voxel,.002);for(let t=0;t<f.length;t+=3){const a=pts[f[t]],b=pts[f[t+1]],c=pts[f[t+2]],l=Math.max(norm(sub(b,a)),norm(sub(c,a)),norm(sub(c,b))),s=Math.max(1,Math.ceil(l/spacing));
    for(let i=0;i<=s;i++)for(let j=0;j<=s-i;j++){const u=i/s,v=j/s;samples.push(add(a,add(mul(sub(b,a),u),mul(sub(c,a),v))));}
    if(samples.length>4e6)throw new Error('A PLY-fájl túl nagy ehhez a felbontáshoz.');}
  const {plane}=fitPlane(samples.filter((_,i)=>i%Math.max(1,Math.floor(samples.length/6000))===0),{threshold:Math.max(.004,voxel*2,resolution*.8)});
  // Without camera poses the cameras were on the side the surface faces: the plane normal points to the mesh's raised side.
  const raised=(n:V3)=>samples.reduce((s,p)=>{const h=dot(sub(p,plane.point),n);return h>cut*2&&h<.4?s+1:s;},0);
  const up=raised(plane.normal)>=raised(mul(plane.normal,-1))?plane.normal:mul(plane.normal,-1);
  // A voxel mesh of a table is a thin closed shell: two parallel layers. The hold stands on the upper one.
  {const span=Math.max(.01,resolution*2),bins=new Float64Array(Math.ceil(3*span/.0005));for(const p of samples){const h=dot(sub(p,plane.point),up);if(h>-span&&h<2*span)bins[Math.floor((h+span)/.0005)]++;}
    const peak=Math.max(...bins);let top=-1;for(let i=bins.length-1;i>=0;i--)if(bins[i]>=peak*.35&&(i===0||bins[i]>=bins[i-1])&&(i===bins.length-1||bins[i]>=bins[i+1])){top=i;break;}
    if(top>=0){let s=0,w=0;for(let i=Math.max(0,top-2);i<=Math.min(bins.length-1,top+2);i++){s+=bins[i]*(-span+(i+.5)*.0005);w+=bins[i];}plane.point=add(plane.point,mul(up,s/w));}}
  const centreGuess=(()=>{const r=samples.filter(p=>{const h=dot(sub(p,plane.point),up);return h>cut*2&&h<.4;});if(!r.length)throw new Error('A PLY-fájlban nincs a síkból kiemelkedő tárgy.');
    const m=(k:number)=>r.map(p=>p[k]).sort((x,y)=>x-y)[r.length>>1];return [m(0),m(1),m(2)] as V3;})();
  const basis0=planeBasis({point:plane.point,normal:up},add(plane.point,up)),fp=holdFootprint(samples,basis0,centreGuess,{radius,minHeight:cut+.001,cell:Math.max(.005,voxel*2),quantile:1});
  // Virtual capture: a spiral of views 25…80° above the plane, far enough to see the whole hold, 1.6 mm pixels at 30 cm.
  const b=fp.basis,size=Math.hypot(fp.max[0]-fp.min[0],fp.max[1]-fp.min[1],fp.height),target=fromPlane(b,[(fp.min[0]+fp.max[0])/2,(fp.min[1]+fp.max[1])/2,fp.height*.4]);
  const distance=Math.max(.25,size*2.4),W=1920,H=1440,k:[number,number,number,number]=[1340,1340,W/2,H/2],dw=320,dh=240,frames:CaptureFrame[]=[];
  for(let i=0;i<48;i++){const s=i/47,az=2*Math.PI*1.25*s,el=(25+55*(.5-.5*Math.cos(2*Math.PI*s)))*Math.PI/180;
    const eye=add(target,add(add(mul(b.e1,distance*Math.cos(el)*Math.cos(az)),mul(b.e2,distance*Math.cos(el)*Math.sin(az))),mul(b.n,distance*Math.sin(el))));
    const fwd=unit(sub(target,eye)),right=unit(cross(fwd,b.n)),down=cross(fwd,right),pose=[...right,0,...down,0,...fwd,0,...eye,1];
    frames.push({index:i+1,depth:renderDepth(mesh0,pose,k,W,H,dw,dh),depthWidth:dw,depthHeight:dh,imageWidth:W,imageHeight:H,k,cameraToWorld:pose,timestamp:i/30});}
  const r=scanHold(frames,{voxel,cut,margin,smooth,radius:Math.max(radius,size)});
  return {...r,resolution,seconds:(performance.now()-started)/1000};
}
