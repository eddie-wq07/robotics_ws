import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
const source=readFileSync(new URL('../web/execution_availability.js',import.meta.url),'utf8');
const {executionAvailability:check}=await import('data:text/javascript,'+encodeURIComponent(source));
const ready={mode:'real',executorReady:true,job:{state:'ready',profile_sha256:'a'},previewToken:'token',profile:{sha256:'a',execution_blockers:[]}};
assert.equal(check(ready).enabled,true);
assert.equal(check({...ready,mode:'preview'}).enabled,false);
assert.match(check({...ready,mode:'preview'}).reason,/Preview-only/);
for(const flag of ['active','busy','savingFlags','changingSelection','preparationWanted','homePending','warmupPending','executePending'])
  assert.equal(check({...ready,[flag]:true}).enabled,false,flag);
assert.equal(check({...ready,executorReady:false}).enabled,false);
assert.equal(check({...ready,previewToken:null}).enabled,false);
assert.equal(check({...ready,profile:{sha256:'changed'}}).enabled,false);
assert.match(check({...ready,profile:{sha256:'a',execution_blockers:['Missing physical qualification']}}).reason,/qualification/);
console.log('Execute readiness, disabled reasons, stale profiles and operation gates passed.');
