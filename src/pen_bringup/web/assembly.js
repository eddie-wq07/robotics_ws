import { Viewer, THREE } from './viewer.js';
import { loadVisual, registerDisplayModel, applyJointPositions } from './robot_visuals.js';
import { cloneSurfaceVisual, displayDrawRows } from './display_geometry.js';
import { preparationDisplay } from './preparation_progress.js';
import { executionDisplay } from './execution_progress.js';
import { executionAvailability } from './execution_availability.js';

const app = document.querySelector('#app');
app.innerHTML = `<div class="assembly-page">
  <header class="assembly-header">
    <a class="brand" href="/">3D<span>Pen</span></a>
    <select id="marker" aria-label="Input image"><option>Loading…</option></select>
    <button id="upload">Upload</button><input id="file" type="file" accept="image/jpeg,image/png" hidden>
    <select id="surface" aria-label="Surface"><option value="plane">Flat surface</option><option value="sphere">80 mm sphere</option><option value="cylinder">Tylenol + holder</option><option value="photo_board" hidden>Photo + cardboard</option></select>
    <select id="profile" aria-label="Drawing profile" title="Saved YAML profile" hidden></select>
    <details id="profile-settings" hidden><summary>Profile flags</summary>
      <div class="profile-flags-popover">
        <label><input id="profile-calibrated" type="checkbox" role="switch"> Calibrated</label>
        <label><input id="profile-fake-only" type="checkbox" role="switch"> Fake only</label>
        <p>Saved profile flags; changing them does not perform physical calibration.</p>
        <span id="profile-flags-status" role="status"></span>
      </div>
    </details>
    <label>Size <input id="size" type="number" value="20" min="1" max="100" step="0.5" style="width:52px"> mm</label>
    <label class="toggle"><input id="fill" type="checkbox"> Fill dark areas</label>
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
        <div id="picture-loading" class="panel-loading" hidden><span></span><progress aria-label="Loading selected image"></progress></div>
        <figure id="heightmap-preview" hidden><img id="heightmap" alt="Relief heightmap"><figcaption>Relief heights</figcaption></figure>
      </div><p class="panel-note" id="picture-note">Choose a saved print.</p>
    </section>
    <section class="camera-panel"><h2>1 · Robot camera<span id="camera-state">Connecting…</span></h2>
      <div class="camera-view"><img id="camera-feed" alt="Live view from the robot D405 camera" hidden><p id="camera-empty">Connecting to the robot camera…</p></div>
      <p class="panel-note">Live physical camera · available during simulation and printing</p></section>
    <section class="strokes-panel"><h2>2 · Pen strokes<label class="toggle"><input id="show-surface" type="checkbox" checked> Surface</label></h2>
      <div id="strokes-view" class="view"><div class="legend" id="legend"></div>
        <div id="strokes-loading" class="panel-loading" hidden><span></span><progress aria-label="Preparing pen strokes"></progress></div>
      </div><p class="panel-note" id="strokes-note"></p>
    </section>
    <section class="robot-panel"><h2>3 · Robot &amp; toolpath<span id="scene-title"></span></h2><div id="assembly-view" class="view"></div>
      <p class="visually-hidden" id="assembly-status" role="status">Loading assembly…</p>
    </section>
  </main>
  <section id="operation-status" class="operation-status" hidden aria-label="Preparation and robot operation progress">
    <div class="operation-heading"><strong id="operation-title"></strong><span id="operation-elapsed"></span></div>
    <div id="operation-detail" role="status" aria-live="polite"></div>
    <progress id="operation-progress" max="1" aria-label="Operation progress"></progress>
  </section>
  <footer class="assembly-playback">
    <button id="warmup" title="Press and release the pen button, then wait 30 seconds. An already-hot pen may start extruding.">Warm up pen · 30s</button>
    <span class="warmup-note">Optional · may extrude if hot</span>
    <button id="execute" class="primary" disabled aria-describedby="execute-reason">Execute</button><span id="execute-reason" role="status"></span><button id="stop" disabled>Stop</button>
    <button id="load-clearance-check" hidden>Load approved air check</button>
    <button id="play" class="primary" disabled>Play</button>
    <select id="rate" aria-label="Playback speed"><option value="1">1×</option><option value="10" selected>10×</option><option value="30">30×</option></select>
    <input id="seek" aria-label="Playback progress" type="range" min="0" max="1" step="any" value="0" disabled>
    <span id="clock" class="mono">0:00 / 0:00</span>
    <button id="home" disabled>Reset</button><button id="frame" disabled>Fit cell</button><button id="close" disabled>Focus surface</button>
    <button id="go-home" title="Return to the recorded Home using the joint controller at 3× speed">Go home · 3×</button>
    <button id="mount">Tool mount</button><button id="travel" aria-pressed="true">Travel</button>
  </footer>
</div>`;
const $ = s => app.querySelector(s);
const views = {strokes: new Viewer($('#strokes-view')), robot: new Viewer($('#assembly-view'), {studio: true})};
const clock = s => `${Math.floor(s/60)}:${String(Math.floor(s%60)).padStart(2,'0')}`;
let current = null, compiled = null, previewToken = null, active = false, mode = 'preview';
let robotPreview = null, robotFlange = null, visualTipOffset = null;
let generation = 0, busy = false, playing = false, at = 0, previous = 0;
let prepareTimer = null, preparationWanted = false, initialized = false;
let partialPreviewId = null;
let invalidation = Promise.resolve();
let executePending = false;
let warmupPending = false;
let homePending = false, homeJobId = null;
let operation = null;
let robotRoot = null, robotJoints = [], robotSurfaces = [], needsFrame = false, toolMarker = null, pathGroup = null, deposit = null;
let drawRows = [], surfaceObject = null;
let strokeDeposit = null, strokeMarker = null, localTips = [];
let worldToDrawing = null;
let modelHash = null;
let registeredWorkpiece = null;
let fillChoices={};
try{fillChoices=JSON.parse(localStorage.getItem('pen-fill-selections')) || {};}catch{}
function restoreFill(){ $('#fill').checked=fillChoices[$('#marker').value]===true; }
let modelSurfaceType = null;
let jointSnapshot = null;
let catalogProfiles = [], catalogKey = '', appliedProfileHash = null;
let savingFlags=false, executorReady=false;
let clearanceCheckAvailable=false;
let changingSelection = false, selectionRevision = 0, selectionTask = Promise.resolve();
let travelPath = null, toolMount = null, strandRadius = .0005;
async function api(path, body) {
  const response = await fetch(path, body === undefined ? {} : {method:'POST', headers:{'Content-Type':'application/json','X-Pen-Request':'1'}, body:JSON.stringify(body)});
  const value = await response.json();
  if (!response.ok) throw new Error(value.error || response.statusText);
  return value;
}
function message(text) { $('#print-status').textContent = text; }
function strokeViewDirection() { return $('#surface').value==='photo_board'?[-1,1,2]:[1,-1,2]; }
function panelLoading(panel, text, failed=false) {
  const box=$(`#${panel}-loading`);box.hidden=!text;
  box.querySelector('span').textContent=text || '';
  box.querySelector('progress').hidden=failed;
  box.querySelector('progress').removeAttribute('value');
  box.setAttribute('role',failed?'alert':'status');
}
function showSelectedImage(version) {
  const name=$('#marker').value;
  $('#artwork').hidden=true;$('#heightmap-preview').hidden=true;$('#empty-art').hidden=true;
  $('#picture-note').textContent=name || 'Choose a saved print.';
  panelLoading('picture','Loading selected image…');
  const image=new Image();
  image.onload=()=>{if(version!==generation)return;
    $('#artwork').src=image.src;$('#artwork').hidden=false;panelLoading('picture','');};
  image.onerror=()=>{if(version===generation)panelLoading('picture','Selected image could not be loaded.',true);};
  image.src=`/api/input/${encodeURIComponent(name)}`;
}
async function showPreparation(info, version) {
  if(version!==generation)return;
  const progress=preparationDisplay(info);
  if(operation?.kind==='Prepare drawing'){
    operation.detail=progress.text;operation.progress=progress.value;renderOperation();
  }
  panelLoading('strokes',progress.text);
  if(progress.value!==null){
    const bar=$('#strokes-loading progress');bar.max=1;bar.value=progress.value;
    bar.setAttribute('aria-valuetext',progress.text);
  }
  $('#strokes-note').textContent='Preview is being prepared; execution remains unavailable until checks finish.';
  if(info.preview_files?.includes('toolpath.json') && partialPreviewId!==info.id){
    const path=await api(`/api/job/${info.id}/toolpath.json`);
    if(version!==generation)return;
    views.strokes.clear();views.strokes.content.add(strokeSurface(path.surface));
    const strokes=views.strokes.strokes(path);
    views.strokes.frame(path.surface.type==='cylinder'?views.strokes.content:strokes,{margin:3.2,direction:strokeViewDirection()});
    partialPreviewId=info.id;
    $('#show-surface').onchange=()=>{views.strokes.content.children[0].visible=$('#show-surface').checked;views.strokes.requestRender();};
  }
}
function startOperation(kind, detail, jobId=null) {
  operation={kind, detail, jobId, started:performance.now(), running:true, progress:null};
  renderOperation();
}
function renderOperation() {
  if(!operation)return;
  $('#operation-status').hidden=false;
  $('#operation-status').dataset.state=operation.error?'error':operation.running?'running':'complete';
  $('#operation-title').textContent=operation.kind;
  const elapsed=(operation.ended ?? performance.now())-operation.started;
  $('#operation-elapsed').textContent=`${clock(elapsed/1000)} elapsed`;
  if($('#operation-detail').textContent!==operation.detail)$('#operation-detail').textContent=operation.detail;
  const bar=$('#operation-progress');
  if(operation.progress===null)bar.removeAttribute('value');
  else bar.value=operation.progress;
  bar.setAttribute('aria-label',`${operation.kind}: ${operation.detail}`);
}
function finishOperation(detail, error=false) {
  if(!operation)return;
  Object.assign(operation,{detail,error,running:false,ended:performance.now(),progress:error?null:1});
  // A stopped/error state must not look like an ongoing loading animation.
  renderOperation();
}
function updateOperation(state) {
  if(!operation?.running || !operation.jobId)return;
  const job=state.jobs.find(j=>j.id===operation.jobId);
  if(!job)return;
  if(['complete','stopped','error'].includes(job.state)){
    finishOperation(job.message,job.state!=='complete');return;
  }
  const display=executionDisplay(job,state.robot);
  operation.detail=display.text;operation.progress=display.value;
  renderOperation();
}
setInterval(()=>{if(operation?.running)renderOperation();},250);
function controls() {
  app.classList.toggle('photo-view',$('#surface').value==='photo_board');
  $('#size').max=$('#surface').value==='photo_board'?'200':'100';
  const profile=catalogProfiles.find(p=>p.id===$('#profile').value);
  const availability=executionAvailability({mode, profile, job:current, previewToken,
    executorReady, active, busy, changingSelection, preparationWanted,
    savingFlags, homePending, warmupPending, executePending});
  $('#execute').disabled=!availability.enabled;
  $('#execute').textContent=current?.clearance_check?'Execute air check':'Execute';
  $('#execute').title=availability.reason;
  $('#execute-reason').textContent=availability.reason;
  $('#load-clearance-check').hidden=!clearanceCheckAvailable;
  $('#load-clearance-check').disabled=busy || active || changingSelection || homePending || warmupPending || executePending;
  $('#profile-settings').hidden=!profile;
  for(const [id,key] of [['#profile-calibrated','calibrated'],['#profile-fake-only','fake_only']]){
    if(!savingFlags)$(id).checked=profile?.[key]===true;
    $(id).disabled=!profile || savingFlags || changingSelection || busy || active || homePending || warmupPending || executePending;
  }
  $('#warmup').disabled = false;
  $('#go-home').disabled = false;
  $('#stop').disabled = !active && !busy && !playing && !warmupPending && !homePending && !executePending;
  $('#play').disabled = !robotPreview || busy || active;
  $('#seek').disabled = !robotPreview || busy || active;
  for (const id of ['#marker','#surface','#profile','#size','#fill','#upload']) $(id).disabled = savingFlags || active || warmupPending || executePending || homePending;
}
function invalidate() {
  try{
    fillChoices[$('#marker').value]=$('#fill').checked;
    localStorage.setItem('pen-fill-selections',JSON.stringify(fillChoices));
    localStorage.setItem('pen-selection',JSON.stringify({image:$('#marker').value,surface:$('#surface').value,profile:$('#profile').value || undefined,size_mm:Number($('#size').value)}));
  }catch{}
  clearTimeout(prepareTimer); preparationWanted = true;
  generation++; previewToken = null; current = null; compiled = null; robotPreview = null; playing = false;
  partialPreviewId=null;showSelectedImage(generation);
  views.strokes.clear();strokeDeposit=null;strokeMarker=null;
  if(pathGroup)pathGroup.visible=false;
  views.robot.requestRender();
  panelLoading('strokes','Waiting to prepare pen strokes…');
  $('#strokes-note').textContent='Preparing the selected image.';
  $('#play').textContent='Play';
  $('#play').disabled = $('#seek').disabled = true;
  message('Preparing drawing…'); controls();
  invalidation = invalidation.catch(()=>{}).then(()=>api('/api/invalidate', {}));
  prepareTimer = setTimeout(prepareCurrent, 400);
  return invalidation;
}
function setTransform(object, value) {
  object.position.set(...(value.xyz || [0,0,0]));
  object.quaternion.setFromEuler(new THREE.Euler(...(value.rpy || [0,0,0]), 'ZYX'));
}
function material(color=0xb8c7d4, opacity=1) { return new THREE.MeshStandardMaterial({color,roughness:.65,transparent:opacity<1,opacity}); }
async function loadModel(selectedSurface = null, selectedProfile = $('#profile').value) {
  const query = new URLSearchParams();
  if(selectedSurface)query.set('surface',selectedSurface);
  if(selectedProfile)query.set('profile',selectedProfile);
  const model = await api('/api/model' + (query.size ? `?${query}` : ''));
  if (!model.available) { $('#assembly-status').textContent = 'Assembly will appear when the description is available.'; return; }
  views.strokes.clear(); registeredWorkpiece = null;
  if(robotRoot){robotRoot.traverse(o=>o.geometry?.dispose());views.robot.content.remove(robotRoot);}
  robotJoints=[];robotSurfaces=[];modelHash=model.profile_source_sha256;
  modelSurfaceType=model.surface?.type;
  if(generation===0 && modelSurfaceType)$('#surface').value=selectedSurface || modelSurfaceType;
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
  registeredWorkpiece = links.get('pen_surface');
  robotFlange=links.get(`${model.robot_sn}_flange`);
  visualTipOffset=new THREE.Vector3(...model.flange_to_pen_tip.xyz_m);
  applyJointPositions(robotJoints,jointSnapshot?.joints);
  toolMount = links.get('pen_holder');
  $('#inventory').replaceChildren();
  for (const [name,group] of links) {
    if (!group.children.length) continue;
    const label = document.createElement('label'); label.className='component-row';
    const checkbox = document.createElement('input'); checkbox.type='checkbox'; checkbox.checked=true;
    checkbox.onchange=()=>{group.visible=checkbox.checked;views.robot.requestRender();};
    const title=document.createElement('span'); title.textContent=name.replace(`${model.robot_sn}_`,'');
    label.append(checkbox,title); $('#inventory').append(label);
  }
  $('#part-count').textContent = `${model.links.length} links`;
  $('#geometry-status').textContent = model.synthetic ? 'Vendor gray body and white covers. Holder: four CAD holes registered on the 50 mm flange pattern. Clocking and pen seating need physical confirmation. Tooling colors are neutral previews.' : 'Model uses the selected calibration.';
  $('#assembly-status').textContent = model.synthetic ? 'CAD mount · four holes aligned · physical surface registration pending' : 'Calibrated assembly';
  $('#scene-title').textContent=jointSnapshot?.source ? 'Rizon4s · recorded pose' : 'Rizon4s'; needsFrame=true; fitCell();
}
function fitCell() { views.robot.resize(); views.robot.frame(views.robot.content,{margin:1.12,direction:[1.3,-1.5,.9]}); }
function surface(spec) {
  let mesh;
  if (spec.type === 'sphere') { mesh=new THREE.Mesh(new THREE.SphereGeometry(spec.radius_m||.04,64,40),material(0xe9edf1,.7)); }
  else if (spec.type === 'cylinder') {
    mesh=new THREE.Mesh(new THREE.CylinderGeometry(spec.radius_m,spec.radius_m,spec.height_m,64),material(0xe9edf1,.7));
    mesh.rotation.z=Math.PI/2;
  }
  else { const size=spec.size_m || [.08,.10,.0025]; mesh=new THREE.Mesh(new THREE.BoxGeometry(...size),material(0xe9edf1,.7)); mesh.position.z=-size[2]/2; }
  return mesh;
}
function worldGroup(job) {
  const group=new THREE.Group(); const t=job.profile.T_world_surface;
  group.position.set(...t.xyz_m); group.quaternion.set(...t.quaternion_xyzw); return group;
}
function strokeSurface(spec) {
  if (spec.type === modelSurfaceType && registeredWorkpiece?.children.length) {
    return cloneSurfaceVisual(registeredWorkpiece, $('#show-surface').checked);
  }
  const copy = surface(spec); copy.visible = $('#show-surface').checked; return copy;
}
async function showJob(jobInfo, version) {
  const base=`/api/job/${jobInfo.id}`;
  const [job,path,report,motion]=await Promise.all(['job.json','toolpath.json','report.json','robot_preview.json'].map(name=>api(`${base}/${name}`)));
  if(job.profile_source_sha256 && job.profile_source_sha256!==modelHash)await loadModel(jobInfo.surface || job.surface.type, jobInfo.profile || '');
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
  if(job.motion_settings){
    const setting=job.motion_settings;
    $('#picture-note').textContent=`${jobInfo.size_mm} mm · setup ${setting.approach_speed_scale || 1}× · draw ${setting.draw_speed_m_s*1000} mm/s · travel ${setting.travel_speed_m_s*1000} mm/s · tip ${(setting.stand_off_m*1000).toFixed(2)} mm above surface${setting.maximum_surface_normal_angle_deg ? ` · ≤${setting.maximum_surface_normal_angle_deg}° from surface normal` : ''}${job.air_run?' · AIR TEST · extrusion OFF':''}${job.pen.recycle_interval_s ? ` · recycle ${job.pen.recycle_interval_s}s` : ''}`;
  }
  $('#strokes-note').textContent=`${report.stroke_count} strokes · ${clock(job.duration_s)} · travel with extrusion off`;
  views.strokes.clear(); views.strokes.content.add(strokeSurface(path.surface));
  const strokes=views.strokes.strokes(path);
  // Filled artwork should show the actual hatch widths before playback, too.
  // Deposited material uses a darker colour as simulation or execution advances.
  Viewer.tint(strokes.children[0], {color:jobInfo.fill?0xffffff:0x91a4b8,
    opacity:jobInfo.fill?1:.2,vertexColors:false});
  views.strokes.frame(path.surface.type==='cylinder'?views.strokes.content:strokes,{margin:3.2,direction:strokeViewDirection()});
  $('#show-surface').onchange=()=>{views.strokes.content.children[0].visible=$('#show-surface').checked;views.strokes.requestRender();};
  if(pathGroup) { pathGroup.traverse(o=>{o.geometry?.dispose();}); views.robot.content.remove(pathGroup); }
  pathGroup=new THREE.Group(); views.robot.content.add(pathGroup);
  const placement=worldGroup(job); surfaceObject=surface(job.surface); placement.add(surfaceObject); pathGroup.add(placement);
  surfaceObject.visible=!cadSurface;
  if(jobInfo.fill){
    const plannedFill=strokes.children[0].clone();
    plannedFill.geometry=plannedFill.geometry.clone();
    plannedFill.material=plannedFill.material.clone();
    plannedFill.name='planned-infill';
    placement.add(plannedFill);
  }
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
  const transformMatrix=value=>new THREE.Matrix4().compose(
    new THREE.Vector3(...value.xyz_m),new THREE.Quaternion(...value.quaternion_xyzw),new THREE.Vector3(1,1,1));
  const worldToSurface=transformMatrix(job.profile.T_world_surface).invert();
  worldToDrawing=worldToSurface;
  const tcpToTip=transformMatrix(job.profile.T_flange_rdk_tcp).invert().multiply(transformMatrix(job.profile.T_flange_pen_tip));
  const worldTips=job.samples.map(sample=>new THREE.Vector3().setFromMatrixPosition(
    new THREE.Matrix4().compose(new THREE.Vector3(...sample.pose.slice(0,3)),
      new THREE.Quaternion(...sample.pose.slice(3)),new THREE.Vector3(1,1,1)).multiply(tcpToTip)));
  localTips=worldTips.map(point=>point.clone().applyMatrix4(worldToSurface));
  drawRows=displayDrawRows(job.samples,worldTips,localTips);
  deposit=Viewer.tubes([],path.strand_width_mm/2000,drawRows.length); pathGroup.add(deposit);
  strokeDeposit=Viewer.tubes([],strandRadius,drawRows.length); views.strokes.content.add(strokeDeposit);
  strokeMarker=new THREE.Mesh(new THREE.SphereGeometry(strandRadius*1.8,20,12),
    new THREE.MeshBasicMaterial({color:0x009b91,toneMapped:false}));
  views.strokes.content.add(strokeMarker);
  $('#detail-link').href=`${base}/robot_preview.json`; $('#seek').max=motion.duration_s;
  $('#play').disabled=$('#seek').disabled=false;
  message(mode==='preview'?`Ready to play · preview-only mode${jobInfo.cache_hit?' · saved trajectory':''}`:'Ready · Play or Execute');
  $('#assembly-status').textContent='Joint motion preview · approach, draw, retract and return · collision checking pending';
  draw(0); controls();
  panelLoading('strokes','');
  if(!robotRoot) fitCell();
}
function draw(seconds, live=null) {
  if(!compiled || !robotPreview) return;
  at=Math.min(robotPreview.duration_s,Math.max(0,seconds)); $('#seek').value=at;
  $('#clock').textContent=`${clock(at)} / ${clock(robotPreview.duration_s)}`;
  const frames=robotPreview.frames;
  let a=0,b=frames.length-1;
  while(a<b){const m=Math.floor((a+b+1)/2);if(frames[m].time_s<=at)a=m;else b=m-1;}
  const first=frames[a],next=frames[Math.min(a+1,frames.length-1)];
  const fraction=next.time_s>first.time_s?(at-first.time_s)/(next.time_s-first.time_s):0;
  const joints=Object.fromEntries(robotPreview.joint_names.map((name,i)=>[name,first.positions[i]+fraction*(next.positions[i]-first.positions[i])]));
  const freshJoints=live?.joints && Date.now()/1000-live.joints_received_at<1;
  if(!live || freshJoints)applyJointPositions(robotJoints,freshJoints?live.joints:joints);
  robotRoot.updateMatrixWorld(true);
  toolMarker.position.copy(robotFlange.localToWorld(visualTipOffset.clone()));
  const timeFor=frame=>frame.job_time_s ?? (frame.phase==='APPROACH'?0:compiled.duration_s);
  const jobAt=live?.jobTime ?? (timeFor(first)+fraction*(timeFor(next)-timeFor(first)));
  if(playing)message(`${first.phase.toLowerCase()} · ${Math.round(at/robotPreview.duration_s*100)}%`);
  let low=0,high=compiled.samples.length-1;
  while(low<high){const mid=Math.floor((low+high+1)/2);if(compiled.samples[mid].time_s<=jobAt)low=mid;else high=mid-1;}
  const rows=[], localRows=[];
  for(const row of drawRows){
    if(row.a.time_s>jobAt)break;
    const u=Math.min(1,Math.max(0,(jobAt-row.a.time_s)/(row.b.time_s-row.a.time_s)));
    const colour=current.fill?0x124caa:0x4f86df;
    rows.push([row.worldA.toArray(),row.worldA.clone().lerp(row.worldB,u).toArray(),colour]);
    localRows.push([row.localA.toArray(),row.localA.clone().lerp(row.localB,u).toArray(),colour]);
  }
  Viewer.placeTubes(deposit,rows,strandRadius);
  Viewer.placeTubes(strokeDeposit,localRows,strandRadius);
  const sample=compiled.samples[low],nextSample=compiled.samples[Math.min(low+1,compiled.samples.length-1)];
  const u=nextSample.time_s>sample.time_s?Math.min(1,Math.max(0,(jobAt-sample.time_s)/(nextSample.time_s-sample.time_s))):0;
  strokeMarker.position.copy(localTips[low]).lerp(localTips[Math.min(low+1,localTips.length-1)],u);
  if(freshJoints)strokeMarker.position.copy(toolMarker.position).applyMatrix4(worldToDrawing);
  $('#strokes-note').textContent=`${Math.round(jobAt/compiled.duration_s*100)}% drawn · ${sample.extrude?'pen down':'pen up'} · travel with extrusion off`;
  $('#scene-title').textContent=live?(freshJoints?'Live robot':'Waiting for live joints'):(jointSnapshot?.source?'Rizon4s · recorded pose':'Rizon4s');
  views.strokes.requestRender(); views.robot.requestRender();
}
$('#mount').onclick=()=>{if(toolMount)views.robot.frame(toolMount,{margin:1.25,direction:[1,-1,.8]});};
$('#travel').onclick=()=>{const visible=$('#travel').getAttribute('aria-pressed')!=='true';$('#travel').setAttribute('aria-pressed',visible);if(travelPath)travelPath.visible=visible;views.robot.requestRender();};
async function prepareCurrent() {
  if(changingSelection || busy || active || warmupPending || executePending || homePending || !preparationWanted || !$('#marker').value)return;
  preparationWanted=false;
  busy=true; previewToken=null; robotPreview=null; compiled=null; current=null; playing=false;
  $('#play').textContent='Play';controls(); const version=generation; message('Preparing drawing…');
  startOperation('Prepare drawing','Extracting image and generating trajectory…');
  try {
    await invalidation;
    if(version!==generation)return;
    let info=await api('/api/preview',{image:$('#marker').value,surface:$('#surface').value,size_mm:Number($('#size').value),fill:$('#fill').checked,profile:$('#profile').value || undefined});
    // Finish the pending request before starting the latest selection; the
    // server has one preparation slot. Stale results never unlock Execute.
    while(info.state==='preparing'){
      await showPreparation(info,version);
      await new Promise(r=>setTimeout(r,350));info=await api(`/api/job/${info.id}`);
    }
    if(version!==generation)return;
    if(info.state!=='ready')await showPreparation(info,version);
    if(info.state!=='ready')throw new Error(info.message);
    panelLoading('strokes','Loading drawing preview…');
    operation.detail='Loading the checked trajectory and drawing preview…';operation.progress=null;renderOperation();
    await showJob(info,version);
    if(version===generation && operation?.kind==='Prepare drawing')
      finishOperation('Prepared and saved locally. Ready to Play or Execute.');
  } catch(error){if(version===generation){message(error.message);
    if(operation?.kind==='Prepare drawing')finishOperation(`Preparation failed: ${error.message}`,true);
    panelLoading('strokes',`Preparation failed: ${error.message}`,true);
    $('#strokes-note').textContent=partialPreviewId?'Surface strokes only — robot trajectory was not approved.':'No checked pen-stroke preview is available.';}}
  finally{busy=false;controls();if(preparationWanted)prepareTimer=setTimeout(prepareCurrent,0);}
}
$('#load-clearance-check').onclick=async()=>{
  if(busy || active || changingSelection || homePending || warmupPending || executePending)return;
  clearTimeout(prepareTimer);preparationWanted=false;generation++;previewToken=null;playing=false;
  const version=generation;busy=true;controls();
  try{
    const info=await api('/api/clearance-check',{});
    await showJob(info,version);
    message('Approved air clearance check — extrusion OFF, 20 mm above photo. Press Execute air check to start.');
  }catch(error){message(error.message);}
  finally{busy=false;controls();}
};
$('#execute').onclick=async()=>{
  if(warmupPending || (operation?.running && operation.kind==='Pen warmup')){message('Wait for warmup to finish, or press Stop.');return;}
  if(executePending||homePending||active){message('The robot is already executing. Use Stop to end this run.');return;}
  if(changingSelection||busy||preparationWanted){message('Preparing the drawing. Press Execute when it is ready.');return;}
  if(!current||!previewToken){message('No ready drawing. Preparing the current selection…');await invalidate();return;}
  if(mode==='preview'){message('This server is preview-only. Real execution requires the connected real-mode server.');return;}
  executePending=true;playing=false;$('#play').textContent='Play';controls();
  startOperation('Execute','Verifying saved files and requesting execution…',current.id);
  try {const result=await api('/api/execute',{job_id:current.id,preview_token:previewToken});previewToken=null;active=result.state==='executing';
    operation.detail=result.message || 'Waiting for robot validation…';renderOperation();
    if(['complete','stopped','error'].includes(result.state))finishOperation(result.message,result.state!=='complete');}
  catch(error){message(error.message);finishOperation(error.message,true);}
  finally{executePending=false;controls();}
};
$('#warmup').onclick=async()=>{
  if(warmupPending){message('Pen warmup is already starting.');return;}
  if(active || homePending || executePending){message('Stop the current operation before warming the pen.');return;}
  if(busy){message('Wait for preparation to finish, or press Stop, then warm the pen.');return;}
  if(mode==='preview'){message('Pen warmup requires the connected real or simulated executor.');return;}
  warmupPending=true;playing=false;$('#play').textContent='Play';controls();
  startOperation('Pen warmup','Pressing and releasing the pen button, then waiting 30 seconds.');
  try{
    const result=await api('/api/warmup',{});
    operation.jobId=result.id;operation.detail=result.message;active=result.state==='executing';renderOperation();
    if(['complete','stopped','error'].includes(result.state))finishOperation(result.message,result.state!=='complete');
  }catch(error){message(error.message);finishOperation(error.message,true);}
  finally{warmupPending=false;controls();}
};
$('#go-home').onclick=async()=>{
  if(homePending){message('Home is already being prepared.');return;}
  if(active || warmupPending || executePending){message('Press Stop and wait for the current operation to end before going Home.');return;}
  if(mode!=='real'){message('Go home requires the connected real robot.');return;}
  clearTimeout(prepareTimer);preparationWanted=false;
  // Discard pending preview results while the server cancels their worker.
  generation++;
  homePending=true;playing=false;$('#play').textContent='Play';controls();
  startOperation('Go home','Checking current pose and planning a safe Home route…');
  try{
    const result=await api('/api/home',{surface:$('#surface').value,profile:$('#profile').value || undefined});
    message(result.message);
    if(result.state==='executing'){homeJobId=result.id;operation.jobId=result.id;operation.detail=result.message;active=true;renderOperation();}
    else {finishOperation(result.message,result.state!=='complete');if(result.state==='complete')await invalidate();}
  }catch(error){message(error.message);finishOperation(error.message,true);}
  finally{homePending=false;controls();}
};
$('#stop').onclick=async()=>{
  clearTimeout(prepareTimer); preparationWanted=false;
  if(busy){generation++;previewToken=null;}
  if(playing){playing=false;$('#play').textContent='Play';message('Playback stopped');controls();if(!active)return;}
  if(operation?.running){operation.detail='Stop requested; waiting for confirmation…';renderOperation();}
  try{message((await api('/api/stop',{})).message);}catch(error){message(error.message);}
};
$('#upload').onclick=()=>$('#file').click();
$('#file').onchange=async()=>{
  const file=$('#file').files[0];if(!file)return;
  if(file.size>12*1024*1024){message('Choose an image smaller than 12 MB');return;}
  try{const encoded=await new Promise((resolve,reject)=>{const reader=new FileReader();reader.onload=()=>resolve(reader.result.split(',')[1]);reader.onerror=reject;reader.readAsDataURL(file);});
    const result=await api('/api/upload',{name:file.name,data:encoded}); await refresh();$('#marker').value=result.image;
    if(catalogProfiles.length)await selectProfile({image:result.image,fallback_profile:$('#profile').value});
    else await invalidate();}
  catch(error){message(error.message);}
};
$('#marker').onchange=()=>{
  if(catalogProfiles.length)selectProfile({image:$('#marker').value,fallback_profile:$('#profile').value});
  else {restoreFill();invalidate().catch(e=>message(e.message));}
};
$('#profile').onchange=()=>selectProfile({profile:$('#profile').value});
async function saveProfileFlag(key, value){
  const profile=catalogProfiles.find(p=>p.id===$('#profile').value);
  if(!profile || savingFlags)return;
  savingFlags=true;controls();$('#profile-flags-status').textContent='Saving…';
  try{
    await api('/api/profile-flags',{profile:profile.id,profile_sha256:profile.sha256,flags:{[key]:value}});
    previewToken=null;current=null;
    await refresh();
    savingFlags=false;
    await selectProfile({profile:profile.id,image:$('#marker').value},
      {size_mm:Number($('#size').value),fill:$('#fill').checked});
    $('#profile-flags-status').textContent=`Saved to ${profile.id}. Drawing preparation refreshed.`;
  }catch(error){$('#profile-flags-status').textContent=error.message;message(error.message);}
  finally{savingFlags=false;controls();}
}
$('#profile-calibrated').onchange=()=>saveProfileFlag('calibrated',$('#profile-calibrated').checked);
$('#profile-fake-only').onchange=()=>saveProfileFlag('fake_only',$('#profile-fake-only').checked);

$('#fill').onchange=()=>invalidate().catch(e=>message(e.message));
$('#surface').onchange=async()=>{
  const selected=$('#surface').value;
  if(catalogProfiles.length){selectProfile({image:$('#marker').value,surface:selected});return;}
  try {
    await invalidate(); clearTimeout(prepareTimer);
    await loadModel(selected);
    if(preparationWanted)prepareTimer=setTimeout(prepareCurrent,0);
  } catch(error) {message(error.message);}
};
$('#size').oninput=()=>invalidate().catch(e=>message(e.message));
$('#play').onclick=()=>{if(!robotPreview)return;playing=!playing;if(at>=robotPreview.duration_s)draw(0);$('#play').textContent=playing?'Pause':'Play';controls();};
$('#seek').oninput=()=>{playing=false;$('#play').textContent='Play';draw(Number($('#seek').value));controls();};
$('#home').onclick=()=>{playing=false;$('#play').textContent='Play';draw(0);controls();}; $('#frame').onclick=fitCell;
$('#close').onclick=()=>{if(surfaceObject)views.robot.frame(surfaceObject,{margin:3,direction:[1,-1,2]});};
for(const id of ['#home','#frame','#close'])$(id).disabled=false;
$('#expand').onclick=()=>{const expanded=app.firstElementChild.classList.toggle('cell-expanded');$('#expand').textContent=expanded?'Show pipeline':'Expand cell';$('#expand').setAttribute('aria-pressed',expanded);requestAnimationFrame(fitCell);};
async function refresh(){
  const state=await api('/api/state'); mode=state.mode;
  executorReady=state.executor_ready ?? Boolean(Object.keys(state.robot?.status || {}).length);
  clearanceCheckAvailable=state.clearance_check_available===true;
  $('#surface option[value="photo_board"]').hidden=mode!=='preview';
  if(Array.isArray(state.profiles)){
    const key=JSON.stringify(state.profiles);
    if(key!==catalogKey){
      const selected=$('#profile').value;
      catalogKey=key;catalogProfiles=state.profiles;
      $('#profile').replaceChildren(...catalogProfiles.map(profile=>{
        const option=document.createElement('option');option.value=profile.id;
        option.textContent=profile.id;
        option.title=profile.unavailable_reason || profile.label;return option;
      }));
      if(catalogProfiles.some(p=>p.id===selected))$('#profile').value=selected;
      $('#profile').hidden=!catalogProfiles.length;
      if(initialized && selected && !catalogProfiles.some(p=>p.id===selected) && !active)
        selectProfile({image:$('#marker').value});
    }
    const chosen=catalogProfiles.find(p=>p.id===$('#profile').value);
    if(initialized && chosen && appliedProfileHash && chosen.sha256!==appliedProfileHash &&
       !savingFlags && !changingSelection && !active && !busy && !homePending && !warmupPending && !executePending){
      selectProfile({profile:chosen.id,image:$('#marker').value});
    }
  }
  updateOperation(state);
  const backend=state.robot?.status?.execution_backend;
  const modeLabel=backend==='joint_simulation' ? 'simulation · joint controller' : backend==='joint' ? `${mode} · joint controller` : mode;
  $('#mode').textContent=jointSnapshot ? `${modeLabel} · ${jointSnapshot.source?'recorded pose':'arm snapshot'}` : `${modeLabel} mode`;
  $('#mode').title=jointSnapshot ? `${jointSnapshot.robot_sn} read ${jointSnapshot.read_started_utc}; saved pose, not a live stream` : '';
  const selected=$('#marker').value;
  if(JSON.stringify([...$('#marker').options].map(o=>o.value))!==JSON.stringify(state.inputs)){
    $('#marker').replaceChildren(...state.inputs.map(name=>{const option=document.createElement('option');option.value=option.textContent=name;return option;}));
    if(state.inputs.includes(selected))$('#marker').value=selected;
    if(initialized && $('#marker').value!==selected){restoreFill();invalidate().catch(e=>message(e.message));}
    else if(!initialized && $('#marker').value){
      showSelectedImage(generation);panelLoading('strokes','Loading robot model before preparing strokes…');
    }
  }
  const latest=current?state.jobs.find(j=>j.id===current.id):null;
  const wasActive=active;active=Boolean(state.active_job);
  const homing=homeJobId?state.jobs.find(j=>j.id===homeJobId):null;
  if(homing){
    message(`${homing.message} · ${Math.round(homing.progress*100)}%`);
    const robot=state.robot;
    if(robot?.joints && Date.now()/1000-robot.joints_received_at<1){
      applyJointPositions(robotJoints,robot.joints);robotRoot.updateMatrixWorld(true);views.robot.requestRender();
      if(toolMarker)toolMarker.position.copy(robotFlange.localToWorld(visualTipOffset.clone()));
      if(strokeMarker && worldToDrawing)strokeMarker.position.copy(toolMarker.position).applyMatrix4(worldToDrawing);
      $('#scene-title').textContent='Live · joint home';
    }
    if(['complete','stopped'].includes(homing.state)){
      homeJobId=null;
      if(homing.state==='complete')await invalidate();
    }
    controls();return;
  }
  if(latest && ['executing','complete','stopped'].includes(latest.state) && (active || wasActive)){
    message(`${latest.message} · ${Math.round(latest.progress*100)}%`);
    if(robotPreview && !playing && (active || wasActive)){
      const jobTime=latest.state==='complete'?compiled.duration_s:
        (compiled.samples[latest.sample_index]?.time_s ?? latest.progress*compiled.duration_s);
      const frame=robotPreview.frames.find(f=>f.job_time_s!==null && f.job_time_s>=jobTime);
      if(['joint','joint_simulation'].includes(state.robot?.status?.execution_backend)){
        draw(robotPreview.duration_s*latest.progress,{...state.robot,jobTime});
      }else if(frame)draw(frame.time_s,{...state.robot,jobTime});
    }
  }
  const liveJoints=state.robot?.joints;
  const displayedJoints=Object.keys(liveJoints || {}).length ? liveJoints : jointSnapshot?.joints;
  if(!robotPreview && applyJointPositions(robotJoints,displayedJoints))views.robot.requestRender();
  if(active){$('#mode').textContent=`${modeLabel} · live execution`;$('#mode').title='Execution feedback and live joint states';}
  if(needsFrame && Object.keys(displayedJoints || {}).length){fitCell();needsFrame=false;}
  controls();
  return state;
}
function animate(now){if(previous&&playing&&robotPreview){draw(at+(now-previous)/1000*Number($('#rate').value));if(at>=robotPreview.duration_s){playing=false;$('#play').textContent='Play';controls();}}previous=now;requestAnimationFrame(animate);}
requestAnimationFrame(animate);
// Selection changes are serialized; stale responses never unlock a previous job.
function selectProfile(request, overrides={}) {
  const revision=++selectionRevision;changingSelection=true;
  const invalidated=invalidate();clearTimeout(prepareTimer);
  selectionTask=selectionTask.catch(()=>{}).then(async()=>{
    if(revision!==selectionRevision)return;
    try{
      await invalidated;
      const query=new URLSearchParams(Object.entries(request).filter(([,value])=>value));
      const resolved=await api(`/api/selection?${query}`);
      if(revision!==selectionRevision)return;
      const selected={...resolved,...overrides};
      if(selected.image && [...$('#marker').options].some(o=>o.value===selected.image))$('#marker').value=selected.image;
      $('#profile').value=selected.profile;$('#surface').value=selected.surface;
      $('#size').value=selected.size_mm;$('#fill').checked=selected.fill===true;
      showSelectedImage(generation);controls();
      await loadModel(selected.surface,selected.profile);
      if(revision!==selectionRevision)return;
      appliedProfileHash=resolved.profile_sha256;
      changingSelection=false;
      await invalidate();
    }catch(error){
      if(revision!==selectionRevision)return;
      changingSelection=false;preparationWanted=false;clearTimeout(prepareTimer);
      message(error.message);panelLoading('strokes',error.message,true);controls();
    }
  });
  return selectionTask;
}
async function initialize(){
  const state=await refresh();
  let saved=null;const query=new URLSearchParams(location.search);
  try{saved=(query.has('image') || query.has('profile')) ? {
    image:query.get('image'),surface:query.get('surface'),profile:query.get('profile'),
    ...(query.has('size_mm')?{size_mm:Number(query.get('size_mm'))}:{}),
  } : JSON.parse(localStorage.getItem('pen-selection'));}catch{}
  if(!saved || (saved.image && !state.inputs.includes(saved.image)) || (!saved.image && !saved.profile))
    saved=state.default_selection || {};
  if(catalogProfiles.length){
    const request={image:saved.image};
    if(saved.profile && catalogProfiles.some(p=>p.id===saved.profile))request.profile=saved.profile;
    else if(saved.surface)request.surface=saved.surface;
    const overrides={};
    if(Number.isFinite(saved.size_mm) && saved.size_mm>=1 && saved.size_mm<=200)overrides.size_mm=saved.size_mm;
    await selectProfile(request,overrides);
  }else{
    // Servers started before profile discovery retain their existing surface API.
    if(saved.image && state.inputs.includes(saved.image))$('#marker').value=saved.image;
    if(['plane','sphere','cylinder',...(mode==='preview'?['photo_board']:[])].includes(saved.surface))$('#surface').value=saved.surface;
    if(Number.isFinite(saved.size_mm) && saved.size_mm>=1 && saved.size_mm<=200)$('#size').value=saved.size_mm;
    await loadModel($('#surface').value);
    restoreFill();if(!active && $('#marker').value)await invalidate();
  }
  initialized=true;
  if(query.get('clearance_check')==='1' && state.clearance_check_available){
    clearTimeout(prepareTimer);preparationWanted=false;
    await $('#load-clearance-check').onclick();
  }
  if(query.size && query.get('clearance_check')!=='1')history.replaceState(null,'',location.pathname+location.hash);
}
initialize().catch(error=>message(error.message));
let refreshPending=false;
setInterval(async()=>{
  if(refreshPending)return;
  refreshPending=true;
  try{await refresh();}catch(e){message(`Connection lost: ${e.message}`);previewToken=null;
    if(operation?.running){operation.detail='Connection lost; robot state unconfirmed. Reconnecting…';renderOperation();}
    controls();}finally{refreshPending=false;}
},100);

let cameraObjectURL=null;
async function refreshCamera(){
  const controller=new AbortController();const timeout=setTimeout(()=>controller.abort(),2000);
  try{
    const response=await fetch('/api/camera/frame.jpg',{cache:'no-store',signal:controller.signal});
    if(!response.ok)throw new Error('Camera unavailable · reconnecting');
    const next=URL.createObjectURL(await response.blob());
    const loaded=new Image();loaded.src=next;
    try{await loaded.decode();}catch(error){URL.revokeObjectURL(next);throw error;}
    $('#camera-feed').src=next;$('#camera-feed').hidden=false;$('#camera-empty').hidden=true;
    $('#camera-feed').dataset.frameId=response.headers.get('X-Frame-Id') || '';
    $('#camera-state').textContent='Live';
    if(cameraObjectURL)URL.revokeObjectURL(cameraObjectURL);cameraObjectURL=next;
  }catch(error){
    $('#camera-feed').hidden=true;$('#camera-empty').hidden=false;
    $('#camera-empty').textContent='Camera unavailable · reconnecting';$('#camera-state').textContent='Offline';
  }finally{clearTimeout(timeout);setTimeout(refreshCamera,$('#camera-feed').hidden?1000:100);}
}
refreshCamera();
