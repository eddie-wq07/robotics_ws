// Loading a reviewed file is separate from generating its trajectory.
export function executionDisplay(job, robot, nowSeconds = Date.now() / 1000) {
  const status = robot?.status;
  const matches = job.sha256 && (status?.job_sha256 === job.sha256 ||
    (status?.phase === 'VALIDATING' && status.requested_job_sha256 === job.sha256));
  const fresh = status?.busy === 'true' && matches && nowSeconds - robot.received_at < 1;
  const phase = fresh ? status.phase : job.message;
  if (fresh && phase === 'VALIDATING') {
    const labels = {READING: 'Reading saved trajectory', HASHING: 'Verifying saved file checksum',
      PARSING: 'Loading saved trajectory into memory', SAMPLES: 'Checking saved trajectory samples',
      TIMING: 'Checking saved extrusion timing'};
    const fraction = Number(status.load_progress);
    const value = Number.isFinite(fraction) && fraction >= 0 ? Math.min(1, fraction) : null;
    const percent = value === null ? '' : ` · ${Math.floor(value * 100)}% of this check`;
    return {text: `${labels[status.load_stage] || 'Loading and validating saved trajectory'}${percent}…`, value};
  }
  const labels = {VALIDATING: 'Loading and validating saved trajectory…',
    PREFLIGHT: 'Checking controller and current robot state…',
    JOINT_HOME: 'Returning to recorded Home…', JOINT_RETREAT: 'Retracting from surface…',
    JOINT_APPROACH: 'Moving to drawing start…', JOINT_HOME_TO_APPROACH: 'Moving to approach route…',
    SWITCHING_TO_CARTESIAN: 'Activating Cartesian controller…', SWITCHING_TO_JOINT: 'Activating joint controller…',
    DRAW: 'Drawing…', HOLD: 'Holding position…', LOWER: 'Lowering pen…', LIFT: 'Lifting pen…',
    TRAVEL: 'Moving between strokes…', RELEASE: 'Pausing with extrusion off…',
    JOINT_PARK: 'Returning to park…', STOPPING: 'Stopping; waiting for robot confirmation…',
    WARMUP_STARTING: 'Checking pen warmup conditions…',
    WARMUP_PRESS: 'Pressing and releasing the button to start heating…',
    WARMUP_HEATING: `Heating pen — ${Math.ceil(30 * (1 - Number(job.progress || 0)))} seconds remaining. Arm stays stationary.`};
  const progress = Number(fresh ? status.progress : job.progress);
  const waiting = ['VALIDATING', 'PREFLIGHT', 'SWITCHING_TO_CARTESIAN', 'SWITCHING_TO_JOINT', 'STOPPING'].includes(phase);
  const value = !waiting && Number.isFinite(progress) && progress > 0 ? Math.min(1, progress) : null;
  return {text: labels[phase] || phase || 'Waiting for robot…', value};
}
