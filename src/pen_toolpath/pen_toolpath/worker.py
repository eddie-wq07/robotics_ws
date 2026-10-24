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
from .progress import ProgressWriter


def prepare(request, output):
    output = Path(output); output.mkdir(parents=True, exist_ok=True)
    progress = ProgressWriter(output)
    progress('image')
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
    for key in ['speed_m_s','travel_speed_m_s','sample_rate_hz','stand_off_m','lift_m','clearance_margin_m','air_offset_m']:
        if key in request: settings[key] = request[key]
    profile=request.get('profile')
    profile_file=request.get('profile_path')
    source_bytes=None
    if profile_file:
        profile_file=Path(profile_file).resolve()
        source_bytes=profile_file.read_bytes()
        profile=yaml.safe_load(source_bytes)
    # Preserve smooth continuous strokes with either hardware controller.
    if request.get('execution_backend', 'cartesian') in ('cartesian', 'joint'):
        settings.setdefault('continuous_strokes', True)
    routes=(profile or {}).get('routes',{})
    if request.get('robot_preview') and routes.get('approach') and routes.get('cartesian_entry_pose'):
        # Start the checked Cartesian prefix at the actual saved joint endpoint,
        # then connect to the raised Cartesian entry. Preview and execution must
        # share this connecting segment instead of preview alone moving it.
        import numpy as np
        from scipy.spatial.transform import Rotation
        from .robot_preview import Arm, transform
        context=dict(request['robot_preview'])
        if profile.get('preview_kinematic_correction'):
            context['base_correction']=profile['preview_kinematic_correction']
        arm=Arm(context)
        end=np.asarray(routes['approach']['points'][-1]['positions'],float)
        tcp=arm.forward(end)@np.linalg.inv(transform(profile['T_flange_pen_tip']))@transform(profile['T_flange_rdk_tcp'])
        settings['approach_start_pose']={'xyz_m':tcp[:3,3].tolist(),
                                        'quaternion_xyzw':Rotation.from_matrix(tcp[:3,:3]).as_quat().tolist()}
    if request.get('execution_backend') == 'joint':
        settings['continuous_strokes'] = True
        settings['air_offset_m'] = profile.get('joint_execution', {}).get('air_offset_m', 0.)
        if not settings['air_offset_m'] and not profile.get('joint_execution', {}).get('drawing_enabled', False):
            raise ValueError('Joint drawing is disabled until the raised air test is reviewed')
    for timing_attempt in range(4):
        progress('surface')
        job, preview, report = compile_drawing(drawing, surface=request.get('surface','sphere'),
            profile=profile,
            execution_mode=request.get('execution_mode','preview'),settings=settings)
        # Display geometry independently of the slower robot trajectory checks.
        # Only the final job/ready state authorizes execution.
        write_json(output/'toolpath.json',preview,compact=True)
        if request.get('robot_preview'):
            from .robot_preview import build_preview
            preview_context={**request['robot_preview'],
                             'joint_execution':request.get('execution_backend') == 'joint'}
            motion = build_preview(job, preview_context, progress=progress)
            if request.get('execution_backend') == 'joint':
                from .joint_trajectory import build_joint_trajectory, TimingLimitError
                write_json(output/'unretimed_robot_preview.json', motion, compact=True)
                try:
                    trajectory = build_joint_trajectory(job, request['robot_preview'], motion)
                except TimingLimitError as error:
                    lead=max(job['pen']['press_delay_s'],job['pen']['recycle_press_delay_s'])
                    planned=settings.get('recycle_planning_interval_s',job['pen']['recycle_interval_s'])
                    ratio=(error.duration_s-lead)/max(.01,planned-lead)
                    settings['recycle_planning_interval_s']=lead+(job['pen']['recycle_interval_s']-.01-lead)/ratio
                    continue
                write_json(output/'joint_trajectory.json', trajectory, compact=True)
                job['execution_backend'] = 'joint'
                job['joint_trajectory_sha256'] = hashlib.sha256((output/'joint_trajectory.json').read_bytes()).hexdigest()
                motion = {**trajectory, 'kind':'pen_robot_preview', 'simulation_only':True}
            write_json(output/'robot_preview.json', motion, compact=True)
            job['robot_preview_sha256']=hashlib.sha256((output/'robot_preview.json').read_bytes()).hexdigest()
            report['robot_preview'] = {key: value for key, value in motion.items() if key != 'frames'}
        break
    else:
        raise ValueError("Unable to meet the 15-second pause schedule after joint retiming")
    if profile_file:
        job['profile_path']=str(profile_file)
        job['profile_source_sha256']=hashlib.sha256(source_bytes).hexdigest()
    progress('saving')
    write_json(output/'job.json',job,compact=True)
    write_json(output/'toolpath.json',preview,compact=True)
    write_json(output/'report.json',report)
    write_json(output/'request.json',request)
    progress('complete', 1, 1)
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
