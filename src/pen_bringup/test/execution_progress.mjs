import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
const source = readFileSync(new URL('../web/execution_progress.js', import.meta.url), 'utf8');
const { executionDisplay } = await import('data:text/javascript,' + encodeURIComponent(source));
const job = {sha256: 'reviewed', message: 'Requesting execution', progress: 0};
const robot = {received_at: 100, status: {busy: 'true', phase: 'VALIDATING',
  job_sha256: '', requested_job_sha256: 'reviewed', load_stage: 'PARSING', load_progress: '-1'}};
// Loading must be visible before the executor has verified the reviewed digest.
assert.deepEqual(executionDisplay(job, robot, 100.1),
  {text: 'Loading saved trajectory into memory…', value: null});
robot.status.load_stage = 'SAMPLES'; robot.status.load_progress = '.35';
assert.equal(executionDisplay(job, robot, 100.1).value, .35);
assert.match(executionDisplay(job, robot, 100.1).text, /35% of this check/);
// Other jobs and stale status must never update this job's loading bar.
robot.status.requested_job_sha256 = 'other';
assert.equal(executionDisplay(job, robot, 100.1).text, job.message);
robot.status.requested_job_sha256 = 'reviewed';
assert.equal(executionDisplay(job, robot, 102).text, job.message);
Object.assign(robot.status, {job_sha256: 'reviewed', phase: 'PREFLIGHT', progress: '.35'});
assert.equal(executionDisplay(job, robot, 100.1).value, null);
Object.assign(robot.status, {phase: 'DRAW', progress: '.42'});
assert.deepEqual(executionDisplay(job, robot, 100.1), {text: 'Drawing…', value: .42});
console.log('Execution loading, stage percentages, stale job isolation and drawing progress passed.');
