#!/usr/bin/env python3
"""Exercise only a running fake-mode browser application; requires Playwright."""
import argparse
import json
from pathlib import Path
import time
from playwright.sync_api import sync_playwright


parser=argparse.ArgumentParser()
parser.add_argument('--url', default='http://127.0.0.1:8080')
parser.add_argument('--out', type=Path, default=Path.home()/'robotics_setup/validation/pen/browser')
args=parser.parse_args()
args.out.mkdir(parents=True,exist_ok=True)
report={}
with sync_playwright() as p:
    browser=p.chromium.launch(executable_path='/usr/bin/google-chrome',headless=True,
        args=['--no-sandbox','--use-gl=angle','--use-angle=swiftshader','--enable-unsafe-swiftshader'])
    page=browser.new_page(viewport={'width':1500,'height':950})
    def wait_for_state(predicate, timeout):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            state=page.evaluate("fetch('/api/state').then(r=>r.json())")
            if predicate(state):
                return state
            page.wait_for_timeout(100)
        raise TimeoutError(f'Workflow did not reach expected state: {state}')
    errors=[]; page.on('pageerror',lambda error:errors.append(str(error)))
    page.goto(args.url,wait_until='domcontentloaded')
    page.wait_for_function("document.querySelector('#marker')?.options.length > 1")
    state=page.evaluate("fetch('/api/state').then(r=>r.json())")
    assert state['mode']=='fake', 'Browser integration tests require explicit fake mode'
    assert page.locator('#execute').is_enabled()
    page.select_option('#marker','apple.png')
    page.wait_for_function("!document.querySelector('#play').disabled",timeout=60000)
    report['automatic_preparation_unlocks_execute']=True
    page.locator('#size').fill('18'); page.locator('#size').blur()
    page.wait_for_function("document.querySelector('#play').disabled")
    report['edit_invalidates_preview']=True
    print('Automatic preparation and edit invalidation passed',flush=True)
    page.wait_for_function("!document.querySelector('#play').disabled",timeout=60000)
    page.screenshot(path=str(args.out/'preview.png'))
    page.click('#execute')
    page.wait_for_function("!document.querySelector('#stop').disabled")
    state=wait_for_state(lambda s:s['jobs'][-1]['state'] in ('complete','stopped'),90)
    last=state['jobs'][-1]
    report['execution']=last
    (args.out/'result.json').write_text(json.dumps(report,indent=2))
    assert last['state']=='complete', last['message']
    report['full_fake_execution']=True
    print('Browser to ROS fake execution completed',flush=True)
    page.screenshot(path=str(args.out/'completed.png'))
    page.select_option('#surface','plane')
    page.wait_for_function("!document.querySelector('#play').disabled",timeout=60000)
    report['plane_preview']=True
    page.click('#execute')
    page.wait_for_function("!document.querySelector('#stop').disabled")
    page.click('#stop')
    wait_for_state(lambda s:not s['active_job'] and s['jobs'][-1]['state']=='stopped',20)
    report['cancel_stops']=True
    report['page_errors']=errors
    assert not errors, errors
    (args.out/'result.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2))
    browser.close()
