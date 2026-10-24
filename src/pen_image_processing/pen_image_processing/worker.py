"""Independent image extraction worker for the image-processing ROS node."""
import argparse
import json
from pathlib import Path
import sys
from .api import extract_drawing


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--request',type=Path,required=True)
    parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args()
    try:
        request=json.loads(args.request.read_text())
        extract_drawing(request['image_path'],args.out,size_mm=request.get('size_mm',20),fill=request.get('fill',False))
        print(json.dumps({'success':True,'artifact_path':str((args.out/'drawing.json').resolve())}))
        return 0
    except Exception as error:
        print(json.dumps({'success':False,'message':str(error)}),file=sys.stderr)
        return 1

if __name__=='__main__':
    raise SystemExit(main())
