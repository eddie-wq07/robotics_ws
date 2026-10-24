import { Viewer, THREE } from './viewer.js';
import { loadVisual, registerDisplayModel, applyJointPositions } from './robot_visuals.js';

const app = document.querySelector('#app');
app.innerHTML = `<div class="assembly-page">
  <header class="assembly-header">
    <a class="brand" href="/">3D<span>Pen</span></a>
    <select id="marker" aria-label="Input image"><option>Loading…</option></select>
    <button id="upload">Upload</button><input id="file" type="file" accept="image/jpeg,image/png" hidden>
    <select id="surface" aria-label="Surface"><option value="plane">Flat surface</option><option value="sphere">80 mm sphere</option></select>
    <label>Size <input id="size" type="number" value="20" min="1" max="80" step="1" style="width:52px"> mm</label>
    <span id="print-status" class="compact-status" role="status">Loading…</span>
    <button id="expand" aria-pressed="false">Expand cell</button>
    <details id="components-menu"><summary>Components</summary>
      <div class="components-popover">
        <div class="popover-heading">Visible parts <span id="part-count"></span></div>
        <div id="inventory"></div>
        <p id="geometry-status"></p>
        <details class="remaining-inputs"><summary>Remaining hardware inputs</summary><ul id="missing"></ul></details>
        <a id="detail-link" href="#" target="_blank">Job data ↗</a>
      </div>
    </details>
    <span id="mode" class="jig-link"></span>
  </header>
  <main class="assembly-panels">
    <section class="picture-panel"><h2>0 · Picture</h2>
      <div class="compact-photo"><img id="artwork" alt="Selected tactile artwork" hidden><span id="empty-art">No print selected</span>
        <figure id="heightmap-preview" hidden><img id="heightmap" alt="Relief heightmap"><figcaption>Relief heights</figcaption></figure>
      </div><p class="panel-note" id="picture-note">Choose a saved print.</p>
    </section>
    <section class="relief-panel"><h2>1 · Relief</h2><div id="relief-view" class="view"></div><p class="panel-note" id="relief-note">Raised artwork on the surface.</p></section>
    <section class="strokes-panel"><h2>2 · Pen strokes<label class="toggle"><input id="show-surface" type="checkbox" checked> Surface</label></h2>
      <div id="strokes-view" class="view"><div class="legend" id="legend"></div></div><p class="panel-note" id="strokes-note"></p>
    </section>
    <section class="robot-panel"><h2>3 · Robot &amp; toolpath<span id="scene-title"></span></h2><div id="assembly-view" class="view"></div>
      <p class="visually-hidden" id="assembly-status" role="status">Loading assembly…</p>
    </section>
  </main>
  <footer class="assembly-playback">
    <button id="preview" class="primary">Preview</button><button id="execute" class="primary" disabled>Execute</button><button id="stop" disabled>Stop</button>
    <button id="play" class="primary" disabled>Play</button>
    <select id="rate" aria-label="Playback speed"><option value="1">1×</option><option value="10" selected>10×</option><option value="30">30×</option></select>
    <input id="seek" aria-label="Playback progress" type="range" min="0" max="1" step="any" value="0" disabled>
    <span id="clock" class="mono">0:00 / 0:00</span>
    <button id="home" disabled>Reset</button><button id="frame" disabled>Fit cell</button><button id="close" disabled>Focus surface</button>
    <button id="mount">Tool mount</button><button id="travel" aria-pressed="true">Travel</button>
  </footer>
</div>`;
const $ = s => app.querySelector(s);
const views = {relief: new Viewer($('#relief-view')), strokes: new Viewer($('#strokes-view')), robot: new Viewer($('#assembly-view'), {studio: true})};
const clock = s => `${Math.floor(s/60)}:${String(Math.floor(s%60)).padStart(2,'0')}`;
let current = null, compiled = null, previewToken = null, active = false, mode = 'preview';
let robotPreview = null, robotFlange = null, visualTipOffset = null;
let generation = 0, busy = false, playing = false, at = 0, previous = 0;
let robotRoot = null, robotJoints = [], robotSurfaces = [], needsFrame = false, toolMarker = null, pathGroup = null, deposit = null;
let drawRows = [], surfaceObject = null;
let modelHash = null;
let modelSurfaceType = null;
let jointSnapshot = null;
let travelPath = null, toolMount = null, strandRadius = .0005;
async function api(path, body) {
  const response = await fetch(path, body === undefined ? {} : {method:'POST', headers:{'Content-Type':'application/json','X-Pen-Request':'1'}, body:JSON.stringify(body)});
  const value = await response.json();
  if (!response.ok) throw new Error(value.error || response.statusText);
  return value;
}
function message(text) { $('#print-status').textContent = text; }
function controls() {
  $('#preview').disabled = busy || active || playing || !$('#marker').value;
  $('#execute').disabled = !previewToken || busy || active || mode === 'preview';
  $('#stop').disabled = !active && !busy && !playing;
  $('#play').disabled = !robotPreview || busy || active;
  $('#seek').disabled = !robotPreview || busy || active;
  for (const id of ['#marker','#surface','#size','#upload']) $(id).disabled = active;
}
async function invalidate() {
  generation++; previewToken = null; current = null; compiled = null; robotPreview = null; playing = false;
  $('#play').textContent='Play';
  $('#play').disabled = $('#seek').disabled = true;
  await api('/api/invalidate', {}); message('Press Preview to prepare the current image.'); controls();
}
function setTransform(object, value) {
  object.position.set(...(value.xyz || [0,0,0]));
  object.quaternion.setFromEuler(new THREE.Euler(...(value.rpy || [0,0,0]), 'ZYX'));
}
function material(color=0xb8c7d4, opacity=1) { return new THREE.MeshStandardMaterial({color,roughness:.65,transparent:opacity<1,opacity}); }
async function loadModel() {
  const model = await api('/api/model');
  if (!model.available) { $('#assembly-status').textContent = 'Assembly will appear when the description is available.'; return; }
  if(robotRoot){robotRoot.traverse(o=>o.geometry?.dispose());views.robot.content.remove(robotRoot);}
  robotJoints=[];robotSurfaces=[];modelHash=model.profile_source_sha256;
  modelSurfaceType=model.surface?.type;
  if(generation===0 && modelSurfaceType)$('#surface').value=modelSurfaceType;
  jointSnapshot=model.visualization?.joint_snapshot || null;
  robotRoot = new THREE.Group(); views.robot.content.add(robotRoot);
  const links = new Map(model.links.map(link => [link.name, new THREE.Group()]));
  const childNames = new Set(model.joints.map(j => j.child));
  for (const link of model.links) {
    const group = links.get(link.name); group.name = link.name;
    if (link.name === 'pen_surface') robotSurfaces.push(group);
    if (!childNames.has(link.name)) robotRoot.add(group);
    await Promise.all(link.visuals.map(async visual => {
      try {
        const groups=link.name==='pen_holder' ? model.visualization?.holder_mount?.mesh_material_groups : null;
        const shape = await loadVisual(visual, link.name, groups);
        if (shape) group.add(shape);
      } catch (error) { console.warn('Mesh unavailable',visual.url,error); }
    }));
  }
  for (const joint of model.joints) {
    const origin = new THREE.Group(); setTransform(origin,joint);
    links.get(joint.parent)?.add(origin); origin.add(links.get(joint.child));
    robotJoints.push({joint,group:links.get(joint.child)});
  }
  registerDisplayModel(links, model);
  robotFlange=links.get(`${model.robot_sn}_flange`);
  visualTipOffset=new THREE.Vector3(...model.flange_to_pen_tip.xyz_m);
  applyJointPositions(robotJoints,jointSnapshot?.joints);
  toolMount = links.get('pen_holder');
  $('#inventory').replaceChildren();
  for (const [name,group] of links) {
    if (!group.children.length) continue;
    const label = document.createElement('label'); label.className='component-row';
    const checkbox = document.createElement('input'); checkbox.type='checkbox'; checkbox.checked=true;
    checkbox.onchange=()=>{group.visible=checkbox.checked;};
    const title=document.createElement('span'); title.textContent=name.replace(`${model.robot_sn}_`,'');
    label.append(checkbox,title); $('#inventory').append(label);
  }
  $('#part-count').textContent = `${model.links.length} links`;
  $('#geometry-status').textContent = model.synthetic ? 'Vendor gray body and white covers. Holder: four CAD holes registered on the 50 mm flange pattern. Clocking and pen seating need physical confirmation. Tooling colors are neutral previews.' : 'Model uses the selected calibration.';
  $('#assembly-status').textContent = model.synthetic ? 'CAD mount · four holes aligned · physical surface registration pending' : 'Calibrated assembly';
  $('#scene-title').textContent='Rizon4s'; needsFrame=true; fitCell();
}
function fitCell() { views.robot.resize(); views.robot.frame(views.robot.content,{margin:1.12,direction:[1.3,-1.5,.9]}); }
function surface(spec) {
  let mesh;
  if (spec.type === 'sphere') { mesh=new THREE.Mesh(new THREE.SphereGeometry(spec.radius_m||.04,64,40),material(0xe9edf1,.7)); }
  else { const size=spec.size_m || [.08,.10,.0025]; mesh=new THREE.Mesh(new THREE.BoxGeometry(...size),material(0xe9edf1,.7)); mesh.position.z=-size[2]/2; }
  return mesh;
}
function worldGroup(job) {
  const group=new THREE.Group(); const t=job.profile.T_world_surface;
  group.position.set(...t.xyz_m); group.quaternion.set(...t.quaternion_xyzw); return group;
}
async function showJob(jobInfo, version) {
  const base=`/api/job/${jobInfo.id}`;
  const [job,path,report,relief,motion]=await Promise.all(['job.json','toolpath.json','report.json','relief.json','robot_preview.json'].map(name=>api(`${base}/${name}`)));
  if(job.profile_source_sha256 && job.profile_source_sha256!==modelHash)await loadModel();
  if (version!==generation) return;
  if(motion.kind!=='pen_robot_preview' || motion.frames.length<2)throw new Error('Robot motion preview is unavailable');
  compiled=job; robotPreview=motion; current=jobInfo; previewToken=jobInfo.preview_token; at=0;
  strandRadius = path.strand_width_mm / 2000;
  // Retain the registered CAD workpiece, including its mounting tabs and holes,
  // when it coincides with this job's analytical surface.
  views.robot.content.updateMatrixWorld(true);
  const expectedPosition=new THREE.Vector3(...job.profile.T_world_surface.xyz_m);
  const expectedRotation=new THREE.Quaternion(...job.profile.T_world_surface.quaternion_xyzw);
  const cadSurface=job.surface.type===modelSurfaceType ? robotSurfaces.find(group=>
    group.getWorldPosition(new THREE.Vector3()).distanceTo(expectedPosition)<1e-7 &&
    group.getWorldQuaternion(new THREE.Quaternion()).angleTo(expectedRotation)<1e-7) : null;
  for (const group of robotSurfaces) group.visible=group===cadSurface;
  $('#artwork').src=`${base}/picture.png`; $('#artwork').hidden=false; $('#empty-art').hidden=true;
  $('#heightmap').src=`${base}/relief.png`; $('#heightmap-preview').hidden=false;
  $('#picture-note').textContent=`${jobInfo.size_mm} mm · extracted 2D drawing`;
  $('#relief-note').textContent=`${path.surface.type} · ${path.layers} layer${path.layers===1?'':'s'}`;
  $('#strokes-note').textContent=`${report.stroke_count} strokes · ${clock(job.duration_s)} · travel with extrusion off`;
  for (const v of [views.relief,views.strokes]) { v.clear(); v.content.add(surface(path.surface)); }
  const raised=views.relief.strokes(path,{travels:false});
  const strokes=views.strokes.strokes(path);
  for (const [v,g] of [[views.relief,raised],[views.strokes,strokes]]) v.frame(g,{margin:3.2,direction:[1,-1,2]});
  $('#show-surface').onchange=()=>{for(const v of [views.relief,views.strokes])v.content.children[0].visible=$('#show-surface').checked;};
  if(pathGroup) { pathGroup.traverse(o=>{o.geometry?.dispose();}); views.robot.content.remove(pathGroup); }
  pathGroup=new THREE.Group(); views.robot.content.add(pathGroup);
  const placement=worldGroup(job); surfaceObject=surface(job.surface); placement.add(surfaceObject); pathGroup.add(placement);
  surfaceObject.visible=!cadSurface;
  const drawPoints=[], travelPoints=[];
  for(let i=1;i<job.samples.length;i++) {
    const points=job.samples[i].extrude && job.samples[i-1].extrude ? drawPoints : travelPoints;
    points.push(...job.samples[i-1].pose.slice(0,3),...job.samples[i].pose.slice(0,3));
  }
  const geometry=points=>new THREE.BufferGeometry().setAttribute('position',new THREE.Float32BufferAttribute(points,3));
  pathGroup.add(new THREE.LineSegments(geometry(drawPoints),new THREE.LineBasicMaterial({color:0x70b7ff,transparent:true,opacity:.75,toneMapped:false})));
  travelPath=new THREE.LineSegments(geometry(travelPoints),new THREE.LineDashedMaterial({color:0xf0bb75,dashSize:.002,gapSize:.001,transparent:true,opacity:.8,toneMapped:false}));
  travelPath.computeLineDistances(); travelPath.visible=$('#travel').getAttribute('aria-pressed')==='true'; pathGroup.add(travelPath);
  toolMarker=new THREE.Mesh(new THREE.SphereGeometry(.0016,24,16),new THREE.MeshBasicMaterial({color:0x63d4da,toneMapped:false})); pathGroup.add(toolMarker);
  drawRows=[];
  for(let i=1;i<job.samples.length;i++) if(job.samples[i].extrude && job.samples[i-1].extrude) drawRows.push({a:job.samples[i-1],b:job.samples[i]});
  deposit=Viewer.tubes([],path.strand_width_mm/2000,drawRows.length); pathGroup.add(deposit);
  $('#detail-link').href=`${base}/robot_preview.json`; $('#seek').max=motion.duration_s;
  $('#play').disabled=$('#seek').disabled=false;
  message(mode==='preview'?'Preview ready · preview-only mode':'Preview ready · Execute is available');
  $('#assembly-status').textContent='Joint motion preview · approach, draw, retract and return · collision checking pending';
  draw(0); controls();
  if(!robotRoot) fitCell();
}
function draw(seconds) {
  if(!compiled || !robotPreview) return;
  at=Math.min(robotPreview.duration_s,Math.max(0,seconds)); $('#seek').value=at;
  $('#clock').textContent=`${clock(at)} / ${clock(robotPreview.duration_s)}`;
  const frames=robotPreview.frames;
  let a=0,b=frames.length-1;
  while(a<b){const m=Math.floor((a+b+1)/2);if(frames[m].time_s<=at)a=m;else b=m-1;}
  const first=frames[a],next=frames[Math.min(a+1,frames.length-1)];
  const fraction=next.time_s>first.time_s?(at-first.time_s)/(next.time_s-first.time_s):0;
  const joints=Object.fromEntries(robotPreview.joint_names.map((name,i)=>[name,first.positions[i]+fraction*(next.positions[i]-first.positions[i])]));
  applyJointPositions(robotJoints,joints);
  robotRoot.updateMatrixWorld(true);
  toolMarker.position.copy(robotFlange.localToWorld(visualTipOffset.clone()));
  const timeFor=frame=>frame.job_time_s ?? (frame.phase==='APPROACH'?0:compiled.duration_s);
  const jobAt=timeFor(first)+fraction*(timeFor(next)-timeFor(first));
  if(playing)message(`${first.phase.toLowerCase()} · ${Math.round(at/robotPreview.duration_s*100)}%`);
  let low=0,high=compiled.samples.length-1;
  while(low<high){const mid=Math.floor((low+high+1)/2);if(compiled.samples[mid].time_s<=jobAt)low=mid;else high=mid-1;}
  const rows=[];
  for(const row of drawRows){if(row.a.time_s>jobAt)break;rows.push([row.a.pose.slice(0,3),row.b.time_s<=jobAt?row.b.pose.slice(0,3):compiled.samples[low].pose.slice(0,3),0x4f86df]);}
  Viewer.placeTubes(deposit,rows,strandRadius);
}
$('#mount').onclick=()=>{if(toolMount)views.robot.frame(toolMount,{margin:1.25,direction:[1,-1,.8]});};
$('#travel').onclick=()=>{const visible=$('#travel').getAttribute('aria-pressed')!=='true';$('#travel').setAttribute('aria-pressed',visible);if(travelPath)travelPath.visible=visible;};
$('#preview').onclick=async()=>{
  busy=true; previewToken=null; robotPreview=null; compiled=null; current=null; playing=false;
  $('#play').textContent='Play';controls(); const version=++generation; message('Preparing preview…');
  try {
    let info=await api('/api/preview',{image:$('#marker').value,surface:$('#surface').value,size_mm:Number($('#size').value)});
    while(info.state==='preparing'){await new Promise(r=>setTimeout(r,350));info=await api(`/api/job/${info.id}`);if(version!==generation)return;}
    if(info.state!=='ready')throw new Error(info.message);
    await showJob(info,version);
  } catch(error){message(error.message);} finally{busy=false;controls();}
};
$('#execute').onclick=async()=>{
  if(!current||!previewToken)return;
  try {await api('/api/execute',{job_id:current.id,preview_token:previewToken});previewToken=null;active=true;playing=false;controls();}
  catch(error){message(error.message);}
};
$('#stop').onclick=async()=>{
  if(playing){playing=false;$('#play').textContent='Play';message('Playback stopped');controls();if(!active)return;}
  try{message((await api('/api/stop',{})).message);}catch(error){message(error.message);}
};
$('#upload').onclick=()=>$('#file').click();
$('#file').onchange=async()=>{
  const file=$('#file').files[0];if(!file)return;
  if(file.size>12*1024*1024){message('Choose an image smaller than 12 MB');return;}
  try{const encoded=await new Promise((resolve,reject)=>{const reader=new FileReader();reader.onload=()=>resolve(reader.result.split(',')[1]);reader.onerror=reject;reader.readAsDataURL(file);});
    const result=await api('/api/upload',{name:file.name,data:encoded}); await refresh();$('#marker').value=result.image;await invalidate();}
  catch(error){message(error.message);}
};
for(const id of ['#marker','#surface','#size']) $(id).onchange=()=>invalidate().catch(e=>message(e.message));
$('#play').onclick=()=>{if(!robotPreview)return;playing=!playing;if(at>=robotPreview.duration_s)draw(0);$('#play').textContent=playing?'Pause':'Play';controls();};
$('#seek').oninput=()=>{playing=false;$('#play').textContent='Play';draw(Number($('#seek').value));controls();};
$('#home').onclick=()=>{playing=false;$('#play').textContent='Play';draw(0);controls();}; $('#frame').onclick=fitCell;
$('#close').onclick=()=>{if(surfaceObject)views.robot.frame(surfaceObject,{margin:3,direction:[1,-1,2]});};
for(const id of ['#home','#frame','#close'])$(id).disabled=false;
$('#expand').onclick=()=>{const expanded=app.firstElementChild.classList.toggle('cell-expanded');$('#expand').textContent=expanded?'Show pipeline':'Expand cell';$('#expand').setAttribute('aria-pressed',expanded);requestAnimationFrame(fitCell);};
async function refresh(){
  const state=await api('/api/state'); mode=state.mode;
  $('#mode').textContent=jointSnapshot ? `${mode} · arm snapshot` : `${mode} mode`;
  $('#mode').title=jointSnapshot ? `${jointSnapshot.robot_sn} read ${jointSnapshot.read_started_utc}; saved pose, not a live stream` : '';
  const selected=$('#marker').value;
  if(JSON.stringify([...$('#marker').options].map(o=>o.value))!==JSON.stringify(state.inputs)){
    $('#marker').replaceChildren(...state.inputs.map(name=>{const option=document.createElement('option');option.value=option.textContent=name;return option;}));
    if(state.inputs.includes(selected))$('#marker').value=selected;
  }
  const latest=current?state.jobs.find(j=>j.id===current.id):null;
  const wasActive=active;active=Boolean(state.active_job);
  if(latest && ['executing','complete','stopped'].includes(latest.state)){
    message(`${latest.message} · ${Math.round(latest.progress*100)}%`);
    if(robotPreview && !playing && (active || wasActive)){
      const jobTime=latest.progress*compiled.duration_s;
      const frame=robotPreview.frames.find(f=>f.job_time_s!==null && f.job_time_s>=jobTime);
      if(frame)draw(frame.time_s);
    }
  }
  const displayedJoints=jointSnapshot?.joints || state.robot?.joints;
  if(!robotPreview || (mode==='real' && active))applyJointPositions(robotJoints,displayedJoints);
  if(needsFrame && Object.keys(displayedJoints || {}).length){fitCell();needsFrame=false;}
  controls();
}
function animate(now){if(previous&&playing&&robotPreview){draw(at+(now-previous)/1000*Number($('#rate').value));if(at>=robotPreview.duration_s){playing=false;$('#play').textContent='Play';controls();}}previous=now;requestAnimationFrame(animate);}
requestAnimationFrame(animate);
refresh().then(()=>message('Select an image, then Preview.')).catch(e=>message(e.message));
loadModel().catch(e=>{$('#assembly-status').textContent=e.message;});
setInterval(()=>refresh().catch(e=>{message(`Connection lost: ${e.message}`);previewToken=null;controls();}),400);
