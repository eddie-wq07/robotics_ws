"""Isolated scientific worker: request JSON in, inspectable artifacts out."""
import argparse
import json
from pathlib import Path
import sys
import hashlib
import yaml

from pen_image_processing.api import extract_drawing
from pen_image_processing.io import write_json
from .compiler import compile_drawing


def prepare(request, output):
    output = Path(output); output.mkdir(parents=True, exist_ok=True)
    image = request.get('image_path')
    drawing_path = request.get('drawing_path')
    if drawing_path:
        drawing = json.loads(Path(drawing_path).read_text())
        write_json(output/'drawing.json', drawing)
    elif image:
        drawing = extract_drawing(image, output, size_mm=request.get('size_mm',20), fill=request.get('fill',False))
    else:
        raise ValueError('request requires image_path or drawing_path')
    if request.get('stage') == 'image':
        return {'success':True,'artifact_path':str((output/'drawing.json').resolve())}
    settings = dict(request.get('settings') or {})
    for key in ['speed_m_s','sample_rate_hz','stand_off_m','lift_m','clearance_margin_m','air_offset_m']:
        if key in request: settings[key] = request[key]
    profile=request.get('profile')
    profile_file=request.get('profile_path')
    source_bytes=None
    if profile_file:
        profile_file=Path(profile_file).resolve()
        source_bytes=profile_file.read_bytes()
        profile=yaml.safe_load(source_bytes)
    job, preview, report = compile_drawing(drawing, surface=request.get('surface','sphere'),
        profile=profile,
        execution_mode=request.get('execution_mode','preview'),settings=settings)
    if request.get('robot_preview'):
        from .robot_preview import build_preview
        motion = build_preview(job, request['robot_preview'])
        write_json(output/'robot_preview.json', motion, compact=True)
        report['robot_preview'] = {key: value for key, value in motion.items() if key != 'frames'}
    if profile_file:
        job['profile_path']=str(profile_file)
        job['profile_source_sha256']=hashlib.sha256(source_bytes).hexdigest()
    write_json(output/'job.json',job,compact=True)
    write_json(output/'toolpath.json',preview,compact=True)
    write_json(output/'report.json',report)
    write_json(output/'request.json',request)
    result={'success':True,'artifact_path':str((output/'job.json').resolve()),
            'sha256':hashlib.sha256((output/'job.json').read_bytes()).hexdigest(),**report}
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--request',required=True,type=Path)
    parser.add_argument('--out',required=True,type=Path)
    args=parser.parse_args()
    try:
        result=prepare(json.loads(args.request.read_text()),args.out)
    except Exception as error:
        print(json.dumps({'success':False,'message':str(error)}),file=sys.stderr)
        return 1
    print(json.dumps(result,allow_nan=False))
    return 0


if __name__=='__main__':
    raise SystemExit(main())
