// 1.2.0 · A hold from photos. Whatever the photo does not show is flat: the back is one plane (Z=0, against the wall).
// Pure geometry over plain arrays, so Node tests run it; the browser feeds it a depth map (offline) or a Meshy mesh.
// Output convention = editor mount convention: metres, Y up, back on Z=0, centred on X/Y.

export interface HoldMask {mask:Uint8Array;height:Float32Array;width:number;height_:number;coverage:number}
export interface MeshData {positions:number[];indices:number[]}

/** Least-squares plane z = a·x + b·y + c through the given samples. */
function fitPlane(samples:[number,number,number][]):[number,number,number]{
  let sxx=0,sxy=0,sx=0,syy=0,sy=0,n=0,sxz=0,syz=0,sz=0;
  for(const [x,y,z] of samples){sxx+=x*x;sxy+=x*y;sx+=x;syy+=y*y;sy+=y;n++;sxz+=x*z;syz+=y*z;sz+=z;}
  const m=[[sxx,sxy,sx],[sxy,syy,sy],[sx,sy,n]],r=[sxz,syz,sz],det=(a:number[][])=>a[0][0]*(a[1][1]*a[2][2]-a[1][2]*a[2][1])-a[0][1]*(a[1][0]*a[2][2]-a[1][2]*a[2][0])+a[0][2]*(a[1][0]*a[2][1]-a[1][1]*a[2][0]);
  const d=det(m);if(Math.abs(d)<1e-12)return [0,0,n?sz/n:0];
  return [0,1,2].map(k=>det(m.map((row,i)=>row.map((v,j)=>j===k?r[i]:v)))/d) as [number,number,number];
}
/** Otsu threshold of values in [0,1]. */
function otsu(values:Float32Array|number[]){const bins=new Array(64).fill(0);let total=0;for(const v of values){if(v<=0)continue;bins[Math.min(63,Math.floor(v*64))]++;total++;}
  if(!total)return .5;let sum=0;bins.forEach((c,i)=>sum+=c*(i+.5));let wb=0,sb=0,best=0,t=.5;
  for(let i=0;i<64;i++){wb+=bins[i];if(!wb)continue;const wf=total-wb;if(!wf)break;sb+=bins[i]*(i+.5);const mb=sb/wb,mf=(sum-sb)/wf,between=wb*wf*(mb-mf)**2;if(between>best){best=between;t=(i+1)/64;}}return t;}

/**
 * The hold in a depth map (larger = closer, as Depth Anything gives it). The photo shows the hold on a flat surface
 * (table or wall): that surface is fitted from the image border and taken as the back plane; what stands out of it is
 * the hold. sensitivity 0…1: higher keeps more of the low edges.
 */
export function holdMask(depth:Float32Array,width:number,height:number,sensitivity=.5):HoldMask{
  const band=Math.max(2,Math.round(Math.min(width,height)*.06)),border:[number,number,number][]=[];
  for(let y=0;y<height;y+=2)for(let x=0;x<width;x+=2)if(x<band||y<band||x>=width-band||y>=height-band)border.push([x/width,y/height,depth[y*width+x]]);
  // Two passes: the second ignores border samples far from the first plane (a hold touching the frame, a shadow).
  let plane=fitPlane(border);const residual=(p:[number,number,number])=>p[2]-(plane[0]*p[0]+plane[1]*p[1]+plane[2]);
  const spread=Math.sqrt(border.reduce((s,p)=>s+residual(p)**2,0)/Math.max(1,border.length));plane=fitPlane(border.filter(p=>Math.abs(residual(p))<=2*spread+1e-9));
  const rise=new Float32Array(width*height);let top=0;
  for(let y=0;y<height;y++)for(let x=0;x<width;x++){const v=depth[y*width+x]-(plane[0]*x/width+plane[1]*y/height+plane[2]);rise[y*width+x]=v;}
  const sorted=Float32Array.from(rise).sort();top=sorted[Math.floor(sorted.length*.995)]||1;if(top<=0)top=1;
  for(let i=0;i<rise.length;i++)rise[i]=Math.max(0,Math.min(1,rise[i]/top));
  const threshold=otsu(rise)*(1.6-1.2*Math.max(0,Math.min(1,sensitivity)));
  let mask=new Uint8Array(width*height);for(let i=0;i<mask.length;i++)mask[i]=rise[i]>threshold?1:0;
  mask=largestCentral(mask,width,height);mask=fillHoles(mask,width,height);
  let count=0;for(const m of mask)count+=m;
  return {mask,height:rise,width,height_:height,coverage:count/mask.length};
}
/** The biggest connected region, preferring one near the image centre (the photographed hold). */
function largestCentral(mask:Uint8Array,w:number,h:number){
  const label=new Int32Array(w*h).fill(-1);let best=-1,bestScore=0,id=0;const sizes:number[]=[];
  for(let start=0;start<mask.length;start++){if(!mask[start]||label[start]>=0)continue;const stack=[start];label[start]=id;let size=0,cx=0,cy=0;
    while(stack.length){const i=stack.pop()!,x=i%w,y=(i-x)/w;size++;cx+=x;cy+=y;for(const j of [x>0?i-1:-1,x<w-1?i+1:-1,y>0?i-w:-1,y<h-1?i+w:-1])if(j>=0&&mask[j]&&label[j]<0){label[j]=id;stack.push(j);}}
    const d=Math.hypot(cx/size/w-.5,cy/size/h-.5),score=size*(1.2-d);sizes.push(size);if(score>bestScore){bestScore=score;best=id;}id++;}
  const out=new Uint8Array(w*h);if(best>=0)for(let i=0;i<out.length;i++)out[i]=label[i]===best?1:0;return out;
}
function fillHoles(mask:Uint8Array,w:number,h:number){
  const outside=new Uint8Array(w*h),stack:number[]=[];
  for(let x=0;x<w;x++)for(const y of [0,h-1])if(!mask[y*w+x]){outside[y*w+x]=1;stack.push(y*w+x);}
  for(let y=0;y<h;y++)for(const x of [0,w-1])if(!mask[y*w+x]&&!outside[y*w+x]){outside[y*w+x]=1;stack.push(y*w+x);}
  while(stack.length){const i=stack.pop()!,x=i%w,y=(i-x)/w;for(const j of [x>0?i-1:-1,x<w-1?i+1:-1,y>0?i-w:-1,y<h-1?i+w:-1])if(j>=0&&!mask[j]&&!outside[j]){outside[j]=1;stack.push(j);}}
  const out=new Uint8Array(w*h);for(let i=0;i<out.length;i++)out[i]=outside[i]?0:1;return out;
}

/**
 * A closed relief solid: the photographed surface on top, a flat back (Z=0) below, the outline as a rounded edge.
 * lengthM is the hold's longest side, thicknessM its height off the wall; edge (0….5) how far the rounding reaches in.
 */
export function reliefMesh(m:HoldMask,{lengthM,thicknessM,grid=110,edge=.12,relief=.75,smooth=2}:{lengthM:number;thicknessM:number;grid?:number;edge?:number;relief?:number;smooth?:number}):MeshData{
  const {mask,height,width:w,height_:h}=m;let x0=w,y0=h,x1=-1,y1=-1;
  for(let y=0;y<h;y++)for(let x=0;x<w;x++)if(mask[y*w+x]){x0=Math.min(x0,x);x1=Math.max(x1,x);y0=Math.min(y0,y);y1=Math.max(y1,y);}
  if(x1<0)throw new Error('A fotón nem található fogás. Fotózd sima, egyszínű alapon, közelről.');
  const span=Math.max(x1-x0+1,y1-y0+1),step=span/grid,cols=Math.max(2,Math.ceil((x1-x0+1)/step))+2,rows=Math.max(2,Math.ceil((y1-y0+1)/step))+2;
  // Cell sampling (one cell of padding all round keeps the outline closed).
  const inside=new Uint8Array(cols*rows),rise=new Float32Array(cols*rows);
  for(let r=1;r<rows-1;r++)for(let c=1;c<cols-1;c++){let n=0,hit=0,sum=0;
    for(let y=Math.floor(y0+(r-1)*step);y<Math.min(h,Math.floor(y0+r*step));y++)for(let x=Math.floor(x0+(c-1)*step);x<Math.min(w,Math.floor(x0+c*step));x++){n++;if(mask[y*w+x]){hit++;sum+=height[y*w+x];}}
    if(n&&hit*2>=n){inside[r*cols+c]=1;rise[r*cols+c]=sum/hit;}}
  // Two light smoothing passes: depth estimates carry a fine grain that a moulded hold does not have.
  for(let pass=0;pass<smooth;pass++){const next=Float32Array.from(rise);for(let r=1;r<rows-1;r++)for(let c=1;c<cols-1;c++){const i=r*cols+c;if(!inside[i])continue;let sum=0,n=0;
    for(let dr=-1;dr<=1;dr++)for(let dc=-1;dc<=1;dc++){const j=i+dr*cols+dc;if(inside[j]){sum+=rise[j];n++;}}next[i]=sum/n;}rise.set(next);}
  // Chamfer distance to the outline, in cells: the edge rounds off like a real hold's rim.
  const dist=new Float32Array(cols*rows).fill(1e9);for(let i=0;i<dist.length;i++)if(!inside[i])dist[i]=0;
  for(let pass=0;pass<2;pass++)for(let k=0;k<dist.length;k++){const i=pass?dist.length-1-k:k,c=i%cols,r=(i-c)/cols,s=pass?1:-1;
    for(const [dc,dr,cost] of [[s,0,1],[0,s,1],[s,s,1.414],[-s,s,1.414]]){const cc=c+dc,rr=r+dr;if(cc>=0&&cc<cols&&rr>=0&&rr<rows)dist[i]=Math.min(dist[i],dist[rr*cols+cc]+cost);}}
  let top=0;for(let i=0;i<rise.length;i++)if(inside[i])top=Math.max(top,rise[i]);top||=1;
  const reach=Math.max(1,edge*grid),cellZ=new Float32Array(cols*rows);
  for(let i=0;i<cellZ.length;i++)if(inside[i]){const t=Math.min(1,dist[i]/reach),round=Math.sqrt(1-(1-t)**2);cellZ[i]=round*((1-relief)+relief*rise[i]/top);}
  let peak=0;for(const z of cellZ)peak=Math.max(peak,z);peak||=1;
  const cell=lengthM/grid,positions:number[]=[],indices:number[]=[],topIndex=new Int32Array((cols+1)*(rows+1)),bottomIndex=new Int32Array((cols+1)*(rows+1));
  const cx=cols/2,cy=rows/2;
  for(let r=0;r<=rows;r++)for(let c=0;c<=cols;c++){
    const around=[[c-1,r-1],[c,r-1],[c-1,r],[c,r]].filter(([a,b])=>a>=0&&b>=0&&a<cols&&b<rows).map(([a,b])=>b*cols+a);
    const open=around.length<4||around.some(i=>!inside[i]),z=open?0:around.reduce((s,i)=>s+cellZ[i],0)/4/peak*thicknessM,v=r*(cols+1)+c;
    topIndex[v]=positions.length/3;positions.push((c-cx)*cell,(cy-r)*cell,z);
    if(z>0){bottomIndex[v]=positions.length/3;positions.push((c-cx)*cell,(cy-r)*cell,0);}else bottomIndex[v]=topIndex[v];
  }
  for(let r=0;r<rows;r++)for(let c=0;c<cols;c++){if(!inside[r*cols+c])continue;const a=r*(cols+1)+c,b=a+1,d=a+cols+1,e=d+1;
    // Image rows run downwards, Y upwards: (a,d,b)/(b,d,e) faces +Z.
    indices.push(topIndex[a],topIndex[d],topIndex[b],topIndex[b],topIndex[d],topIndex[e]);
    indices.push(bottomIndex[a],bottomIndex[b],bottomIndex[d],bottomIndex[b],bottomIndex[e],bottomIndex[d]);}
  return compact({positions,indices});
}
/** Drops unused vertices and triangles that collapsed to a line. */
export function compact({positions,indices}:MeshData):MeshData{
  const keep:number[]=[];for(let i=0;i<indices.length;i+=3){const [a,b,c]=[indices[i],indices[i+1],indices[i+2]];if(a!==b&&b!==c&&a!==c)keep.push(a,b,c);}
  const map=new Map<number,number>(),out:number[]=[];const idx=keep.map(i=>{let j=map.get(i);if(j===undefined){j=out.length/3;map.set(i,j);out.push(positions[i*3],positions[i*3+1],positions[i*3+2]);}return j;});
  return {positions:out,indices:idx};
}

const bounds=(p:number[])=>{const min=[Infinity,Infinity,Infinity],max=[-Infinity,-Infinity,-Infinity];for(let i=0;i<p.length;i+=3)for(let k=0;k<3;k++){min[k]=Math.min(min[k],p[i+k]);max[k]=Math.max(max[k],p[i+k]);}return {min,max};};
/**
 * Where to cut the invented back of an AI mesh (fraction of its depth from the back): the slab with the widest
 * outline, i.e. where a hold sits on the wall. Searched within the back 70 %.
 */
export function autoBackCut(positions:number[]){
  const {min,max}=bounds(positions),depth=max[2]-min[2]||1,slabs=28;let best=.3,area=-1;
  for(let s=0;s<slabs*.7;s++){const lo=min[2]+depth*s/slabs,hi=lo+depth/slabs;let x0=Infinity,x1=-Infinity,y0=Infinity,y1=-Infinity;
    for(let i=0;i<positions.length;i+=3){const z=positions[i+2];if(z<lo||z>hi)continue;x0=Math.min(x0,positions[i]);x1=Math.max(x1,positions[i]);y0=Math.min(y0,positions[i+1]);y1=Math.max(y1,positions[i+1]);}
    const a=x1>x0&&y1>y0?(x1-x0)*(y1-y0):0;if(a>area*1.02){area=a;best=(s+.5)/slabs;}}
  return Math.round(best*100)/100;
}
/** Everything behind the cut plane is pressed onto it: the unseen back becomes one flat face. */
export function flattenBack({positions,indices}:MeshData,cut:number):MeshData{
  const {min,max}=bounds(positions),plane=min[2]+(max[2]-min[2])*Math.max(0,Math.min(.9,cut)),out=positions.slice();
  for(let i=2;i<out.length;i+=3)out[i]=Math.max(out[i],plane);
  // Triangles lying entirely in the plane stay (they are the flat back); none of them faces outwards from the wall.
  return {positions:out,indices:indices.slice()};
}
/** Editor mount: centred on X/Y, back on Z=0, longest side of the outline = lengthM; thicknessM (optional) sets the depth. */
export function mountMesh({positions,indices}:MeshData,lengthM:number,thicknessM?:number):MeshData{
  const {min,max}=bounds(positions),long=Math.max(max[0]-min[0],max[1]-min[1])||1,scale=lengthM/long,depth=(max[2]-min[2])*scale,zs=thicknessM&&depth>0?thicknessM/depth:1;
  const cx=(min[0]+max[0])/2,cy=(min[1]+max[1])/2,out:number[]=[];
  for(let i=0;i<positions.length;i+=3)out.push((positions[i]-cx)*scale,(positions[i+1]-cy)*scale,(positions[i+2]-min[2])*scale*zs);
  return {positions:out,indices:indices.slice()};
}
export const meshSize=(positions:number[])=>{const {min,max}=bounds(positions);return [max[0]-min[0],max[1]-min[1],max[2]-min[2]];};
