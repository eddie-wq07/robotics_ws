import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
const source = readFileSync(new URL('../web/preparation_progress.js', import.meta.url), 'utf8');
const { preparationDisplay } = await import('data:text/javascript,' + encodeURIComponent(source));
let result = preparationDisplay({preparation_stage: 'Checking robot trajectory',
  preparation_progress: {fraction: .425, eta_seconds: 65.2}});
assert.equal(result.value, .425);
assert.match(result.text, /42%.*~1m 6s remaining/);
assert.equal(preparationDisplay({}).value, null);
assert.match(preparationDisplay({preparation_progress: {fraction: .1, eta_seconds: null}}).text, /estimating/);
assert.match(preparationDisplay({preparation_progress: {fraction: .1, eta_seconds: 119.9}}).text, /2m 0s/);
console.log('Progress label, percentage, indeterminate state and ETA formatting passed.');
