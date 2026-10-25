// Presentation of known execution gates; the backend still validates each request.
export function executionAvailability(state) {
  if(state.mode==='preview')return {enabled:false, reason:'Preview-only server — use Play; physical Execute is unavailable.'};
  if(state.savingFlags)return {enabled:false, reason:'Saving profile flags…'};
  if(state.active || state.homePending || state.warmupPending || state.executePending)
    return {enabled:false, reason:'An operation is running; wait or press Stop.'};
  if(!state.job?.clearance_check && state.profile?.execution_blockers?.length)
    return {enabled:false, reason:state.profile.execution_blockers.join(' ')};
  if(state.busy || state.changingSelection || state.preparationWanted)
    return {enabled:false, reason:'Preparing the selected drawing…'};
  if(!state.job || state.job.state!=='ready' || !state.previewToken)
    return {enabled:false, reason:'Prepare the selected drawing before executing.'};
  if(state.profile && state.profile.sha256!==state.job.profile_sha256)
    return {enabled:false, reason:'Profile changed — prepare the drawing again.'};
  if(!state.executorReady)return {enabled:false, reason:'ROS execution server is unavailable.'};
  return {enabled:true, reason:state.job?.clearance_check ?
    'Air check only — extrusion OFF, 20 mm above photo, one use.' :
    'Ready to Execute; final robot checks run when pressed.'};
}
