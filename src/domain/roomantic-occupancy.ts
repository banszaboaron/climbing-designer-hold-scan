// 1.3.0 · Baseline for the scan benchmark: the reconstruction of the Roomantic library (github.com/Yuropa/Roomantic, RoomanticLib:
// ObjectScanner::add_frame + PointCloud + Mesh, joint work of its authors with Áron Bán-Szabó), re-expressed in TypeScript so the
// same captures can be compared without OpenCV. Behaviour kept: 100×100 normalised samples per frame (bilinear depth, the first
// frame only primes the cache), points beyond 2 m dropped, voxel hash with hit counts, dense count field over the bounds, separable
// Gaussian blur (σ = 1 voxel, radius 3), marching cubes at iso = 1 (POINT_FILTERING_COUNT). Default voxel: the "fine" 1 cm cloud.
// Not copied code: the original is C++/OpenCV; the marching-cubes tables here are three.js's.
import type {CaptureFrame} from './scan-capture.ts';
import {apply,marchingCubes,type ScanMesh,type V3} from './hold-scan.ts';

export function roomanticOccupancy(frames:CaptureFrame[],{voxel=.01,maxDepth=2}:{voxel?:number;maxDepth?:number}={}):ScanMesh{
  const cells=new Map<string,number>();let min:V3=[Infinity,Infinity,Infinity],max:V3=[-Infinity,-Infinity,-Infinity];
  for(const [n,f] of frames.entries()){if(n===0)continue;const [fx,fy,cx,cy]=f.k;
    for(let u=0;u<1;u+=.01)for(let v=0;v<1;v+=.01){const x=u*f.depthWidth,y=v*f.depthHeight,x0=Math.floor(x),y0=Math.floor(y),x1=Math.min(x0+1,f.depthWidth-1),y1=Math.min(y0+1,f.depthHeight-1);
      const wx=x-x0,wy=y-y0,at=(i:number,j:number)=>f.depth[j*f.depthWidth+i],d=(1-wx)*(1-wy)*at(x0,y0)+wx*(1-wy)*at(x1,y0)+(1-wx)*wy*at(x0,y1)+wx*wy*at(x1,y1);
      if(!(d<=maxDepth))continue;const px=u*f.imageWidth,py=v*f.imageHeight,p=apply(f.cameraToWorld,[(px-cx)/fx*d,(py-cy)/fy*d,d]);
      const key=`${Math.floor(p[0]/voxel)},${Math.floor(p[1]/voxel)},${Math.floor(p[2]/voxel)}`;if(!cells.has(key))for(let k=0;k<3;k++){min[k]=Math.min(min[k],p[k]);max[k]=Math.max(max[k],p[k]);}cells.set(key,(cells.get(key)??0)+1);}}
  if(!cells.size)throw new Error('Nincs pont a felvételben.');
  const nx=Math.floor((max[0]-min[0])/voxel)+1,ny=Math.floor((max[1]-min[1])/voxel)+1,nz=Math.floor((max[2]-min[2])/voxel)+1;if(nx*ny*nz>40e6)throw new Error('Túl nagy jelenet a Roomantic-alapmódszerhez.');
  let field=new Float32Array(nx*ny*nz);
  for(let k=0;k<nz;k++)for(let j=0;j<ny;j++)for(let i=0;i<nx;i++){const p=[min[0]+i*voxel,min[1]+j*voxel,min[2]+k*voxel];const c=cells.get(`${Math.floor(p[0]/voxel)},${Math.floor(p[1]/voxel)},${Math.floor(p[2]/voxel)}`);if(c)field[(k*ny+j)*nx+i]=c;}
  const kernel=Array.from({length:7},(_,i)=>Math.exp(-((i-3)**2)/2)),sum=kernel.reduce((a,b)=>a+b,0);for(let i=0;i<7;i++)kernel[i]/=sum;
  let tmp=new Float32Array(field.length);
  for(const [s,n] of [[1,nx],[nx,ny],[nx*ny,nz]] as [number,number][]){for(let i=0;i<field.length;i++){const c=Math.floor(i/s)%n;let acc=0;for(let r=-3;r<=3;r++){const cc=Math.min(n-1,Math.max(0,c+r));acc+=field[i+(cc-c)*s]*kernel[r+3];}tmp[i]=acc;}[field,tmp]=[tmp,field];}
  // Solid = count above the filtering level: negate so the shared marching cubes (negative = solid) applies.
  for(let i=0;i<field.length;i++)field[i]=1-field[i];
  return marchingCubes({field,nx,ny,nz,origin:min,voxel});
}
