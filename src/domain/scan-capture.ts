// 1.3.0 · Fogás szkennelése: the files a depth capture arrives in. Pure parsing, no DOM, so Node tests and the browser share it.
// Capture format = the Roomantic iOS app's "video capture" share (github.com/Yuropa/Roomantic, joint work with Áron Bán-Szabó):
//   frame_NNNN.jpg   colour image as ARKit captured it (landscape sensor, e.g. 1920×1440)
//   depth_NNNN.bin   ImageBlob: u32 width, u32 height, u16 channels, u16 type (0 = float32), then ARKit smoothedSceneDepth in metres
//   camera_NNNN.txt  column-major simd matrices: extrinsics (ARCamera.transform), projection, camera_transform, intrinsics; image_size; timestamp
// camera_transform maps camera coordinates (OpenCV style: x right, y down, z forward, pixels of the colour image through K) to the
// ARKit world (metres, Y up). Only that and K are needed here. The Roomantic mesh export (object.ply, ASCII) is read by parsePly.

export type Mat4=number[]; // column-major, 16 values (as simd / three.js store them)
export interface CaptureFrame {index:number;depth:Float32Array;depthWidth:number;depthHeight:number;imageWidth:number;imageHeight:number;
  /** fx, fy, cx, cy in colour-image pixels */ k:[number,number,number,number];cameraToWorld:Mat4;timestamp:number;image?:string}
export interface PlyMesh {positions:number[];indices:number[];colors?:number[]}

const numbers=(line:string)=>line.trim().split(/\s+/).filter(Boolean).map(Number);

/** camera_NNNN.txt → K and camera→world. Each matrix line is one simd column. */
export function parseCameraText(text:string){
  const lines=text.split(/\r?\n/);const block=(name:string,size:number)=>{const at=lines.findIndex(l=>l.trim().startsWith(name+':'));if(at<0)throw new Error(`Hiányzó kameraadat: ${name}`);
    const out:number[]=[];for(let c=0;c<size;c++){const v=numbers(lines[at+1+c]??'');if(v.length<size||v.some(x=>!Number.isFinite(x)))throw new Error(`Hibás kameraadat: ${name}`);out.push(...v.slice(0,size));}return out;};
  const intr=block('intrinsics',3),cameraToWorld=block('camera_transform',4),extrinsics=block('extrinsics',4);
  const size=lines.find(l=>l.startsWith('image_size:')),time=lines.find(l=>l.startsWith('timestamp:'));
  const [w,h]=size?numbers(size.slice(11)):[0,0];if(!(w>0&&h>0))throw new Error('Hiányzó képméret a kameraadatban.');
  // Column-major 3×3: fx = col0.x, fy = col1.y, cx = col2.x, cy = col2.y.
  return {k:[intr[0],intr[4],intr[6],intr[7]] as [number,number,number,number],cameraToWorld,extrinsics,imageWidth:w,imageHeight:h,timestamp:time?Number(time.slice(10)):0};
}
/** depth_NNNN.bin (Roomantic ImageBlob) → metres. Non-finite / non-positive samples become NaN. */
export function parseDepthBlob(bytes:Uint8Array){
  if(bytes.length<12)throw new Error('Túl rövid mélységfájl.');const view=new DataView(bytes.buffer,bytes.byteOffset,bytes.byteLength);
  const width=view.getUint32(0,true),height=view.getUint32(4,true),channels=view.getUint16(8,true),type=view.getUint16(10,true);
  if(type!==0||channels!==1)throw new Error('Csak egycsatornás, float32 mélységkép támogatott.');
  if(!(width>0&&height>0&&width*height<=4096*4096)||bytes.length<12+width*height*4)throw new Error('Hibás mélységkép-méret.');
  const depth=new Float32Array(width*height);for(let i=0;i<depth.length;i++){const d=view.getFloat32(12+i*4,true);depth[i]=Number.isFinite(d)&&d>0?d:NaN;}
  return {width,height,depth};
}
export function depthBlob(width:number,height:number,depth:Float32Array){const out=new Uint8Array(12+depth.length*4),view=new DataView(out.buffer);
  view.setUint32(0,width,true);view.setUint32(4,height,true);view.setUint16(8,1,true);view.setUint16(10,0,true);for(let i=0;i<depth.length;i++)view.setFloat32(12+i*4,Number.isFinite(depth[i])?depth[i]:0,true);return out;}
/** The camera_NNNN.txt text the Roomantic app writes (extrinsics/projection kept plausible, only camera_transform and K are read). */
export function cameraText(f:{k:[number,number,number,number];cameraToWorld:Mat4;imageWidth:number;imageHeight:number;timestamp?:number}){
  const rows=(m:number[],n:number)=>Array.from({length:n},(_,c)=>m.slice(c*n,c*n+n).map(v=>+v.toPrecision(10)).join(' ')+' ').join('\n');
  const [fx,fy,cx,cy]=f.k;
  return `extrinsics:\n${rows(f.cameraToWorld,4)}\nprojection:\n${rows([1,0,0,0,0,1,0,0,0,0,-1,-1,0,0,-.001,0],4)}\ncamera_transform:\n${rows(f.cameraToWorld,4)}\nintrinsics:\n${rows([fx,0,0,0,fy,0,cx,cy,1],3)}\nimage_size: ${f.imageWidth} ${f.imageHeight}\ntimestamp: ${f.timestamp??0}\n`;
}

/** All frames of a capture given as name → bytes (a zip's or a folder's files). Frames need depth + camera; the JPEG is optional. */
export function captureFrames(files:Map<string,Uint8Array>):CaptureFrame[]{
  const base=new Map<string,Uint8Array>();for(const [name,bytes] of files){const short=name.split(/[\\/]/).pop()!;if(!short.startsWith('._'))base.set(short,bytes);}
  const frames:CaptureFrame[]=[],decoder=new TextDecoder();
  for(const [name,bytes] of base){const m=/^depth_(\d+)\.bin$/.exec(name);if(!m)continue;const camera=base.get(`camera_${m[1]}.txt`);if(!camera)continue;
    const c=parseCameraText(decoder.decode(camera)),d=parseDepthBlob(bytes);
    frames.push({index:Number(m[1]),depth:d.depth,depthWidth:d.width,depthHeight:d.height,imageWidth:c.imageWidth,imageHeight:c.imageHeight,k:c.k,cameraToWorld:c.cameraToWorld,timestamp:c.timestamp,image:base.has(`frame_${m[1]}.jpg`)?`frame_${m[1]}.jpg`:undefined});}
  if(!frames.length)throw new Error('A felvételben nincs mélységkép (depth_NNNN.bin + camera_NNNN.txt). A Roomantic alkalmazás „videó” megosztását add meg.');
  return frames.sort((a,b)=>a.index-b.index);
}

/** PLY (ascii or binary little-endian): vertex x/y/z (+ red/green/blue), triangle or polygon faces (fan-triangulated). */
export function parsePly(bytes:Uint8Array):PlyMesh{
  const head=new TextDecoder('latin1').decode(bytes.subarray(0,Math.min(bytes.length,65536))),end=head.indexOf('end_header');
  if(!head.startsWith('ply')||end<0)throw new Error('Nem PLY-fájl.');
  let bodyAt=end+'end_header'.length;while(bodyAt<head.length&&head[bodyAt]!=='\n')bodyAt++;bodyAt++;
  const lines=head.slice(0,end).split(/\r?\n/).map(l=>l.trim());const format=lines.find(l=>l.startsWith('format '))?.split(/\s+/)[1];
  if(format!=='ascii'&&format!=='binary_little_endian')throw new Error('Csak ASCII vagy little-endian bináris PLY támogatott.');
  interface Prop {name:string;type:string;list?:[string,string]} interface Element {name:string;count:number;props:Prop[]}
  const elements:Element[]=[];for(const l of lines){const p=l.split(/\s+/);if(p[0]==='element')elements.push({name:p[1],count:Number(p[2]),props:[]});
    else if(p[0]==='property'&&elements.length)elements[elements.length-1].props.push(p[1]==='list'?{name:p[4],type:'list',list:[p[2],p[3]]}:{name:p[2],type:p[1]});}
  const size:Record<string,number>={char:1,uchar:1,int8:1,uint8:1,short:2,ushort:2,int16:2,uint16:2,int:4,uint:4,int32:4,uint32:4,float:4,float32:4,double:8,float64:8};
  const view=new DataView(bytes.buffer,bytes.byteOffset,bytes.byteLength);let at=bodyAt;
  const readBin=(type:string)=>{const s=size[type];if(!s)throw new Error(`Ismeretlen PLY-típus: ${type}`);let v:number;
    switch(type){case 'char':case 'int8':v=view.getInt8(at);break;case 'uchar':case 'uint8':v=view.getUint8(at);break;case 'short':case 'int16':v=view.getInt16(at,true);break;case 'ushort':case 'uint16':v=view.getUint16(at,true);break;
      case 'int':case 'int32':v=view.getInt32(at,true);break;case 'uint':case 'uint32':v=view.getUint32(at,true);break;case 'float':case 'float32':v=view.getFloat32(at,true);break;default:v=view.getFloat64(at,true);}at+=s;return v;};
  let tokens:string[]=[],ti=0;if(format==='ascii')tokens=new TextDecoder('latin1').decode(bytes.subarray(bodyAt)).split(/\s+/).filter(Boolean);
  const read=format==='ascii'?()=>{if(ti>=tokens.length)throw new Error('Csonka PLY-fájl.');return Number(tokens[ti++]);}:readBin;
  const positions:number[]=[],colors:number[]=[],indices:number[]=[];let hasColor=false;
  for(const e of elements){
    if(e.name==='vertex'){const ix=e.props.findIndex(p=>p.name==='x'),iy=e.props.findIndex(p=>p.name==='y'),iz=e.props.findIndex(p=>p.name==='z');
      const ir=e.props.findIndex(p=>p.name==='red'||p.name==='r'),ig=e.props.findIndex(p=>p.name==='green'||p.name==='g'),ib=e.props.findIndex(p=>p.name==='blue'||p.name==='b');
      if(ix<0||iy<0||iz<0)throw new Error('A PLY-csúcsoknak nincs x/y/z koordinátája.');hasColor=ir>=0&&ig>=0&&ib>=0;
      const scale=hasColor&&['float','float32','double','float64'].includes(e.props[ir].type)?255:1;
      for(let i=0;i<e.count;i++){const v=e.props.map(p=>{if(p.list){const n=format==='ascii'?read(''):readBin(p.list[0]);for(let k=0;k<n;k++)format==='ascii'?read(''):readBin(p.list[1]);return 0;}return format==='ascii'?read(''):readBin(p.type);});
        positions.push(v[ix],v[iy],v[iz]);if(hasColor)colors.push(v[ir]*scale/255,v[ig]*scale/255,v[ib]*scale/255);}}
    else if(e.name==='face'){const li=e.props.findIndex(p=>p.list&&(p.name==='vertex_indices'||p.name==='vertex_index'));
      for(let i=0;i<e.count;i++)for(const [k,p] of e.props.entries()){
        if(p.list){const n=format==='ascii'?read(''):readBin(p.list[0]),poly:number[]=[];for(let j=0;j<n;j++)poly.push(format==='ascii'?read(''):readBin(p.list[1]));if(k===li)for(let j=1;j+1<poly.length;j++)indices.push(poly[0],poly[j],poly[j+1]);}
        else format==='ascii'?read(''):readBin(p.type);}}
    else for(let i=0;i<e.count;i++)for(const p of e.props){if(p.list){const n=format==='ascii'?read(''):readBin(p.list[0]);for(let j=0;j<n;j++)format==='ascii'?read(''):readBin(p.list[1]);}else format==='ascii'?read(''):readBin(p.type);}
  }
  const count=positions.length/3;if(!count)throw new Error('A PLY-fájl üres.');
  for(let i=0;i<positions.length;i++)if(!Number.isFinite(positions[i]))throw new Error('A PLY-fájlban érvénytelen koordináta van.');
  for(const i of indices)if(!(i>=0&&i<count&&Number.isInteger(i)))throw new Error('A PLY-fájl lapjai nem létező csúcsra mutatnak.');
  return hasColor?{positions,indices,colors}:{positions,indices};
}
