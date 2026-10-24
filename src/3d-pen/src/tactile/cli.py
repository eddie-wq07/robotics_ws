"""Prepare a picture for the jig-mounted hemisphere; MoveIt plans the robot."""
import argparse
import math
import os
from pathlib import Path
import yaml

from .io import write_json
from .job import make_job
from .relief import make_relief, save_relief
from .slicer import slice_relief, summary
from .surface import load_surface


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    prepare = commands.add_parser("prepare", help="picture → relief → curved strokes → ROS job")
    prepare.add_argument("picture", type=Path)
    prepare.add_argument("--out", type=Path, default=Path("runs/print"))
    prepare.add_argument("--size-mm", type=float, default=45)
    prepare.add_argument("--fill", action="store_true", help="include filled areas; default is raised lines")
    args = parser.parse_args(argv)
    picture, output = args.picture.resolve(), args.out.resolve()
    root = next((p for p in [Path.cwd(), *Path.cwd().parents] if (p / "configs/process.yaml").is_file()), None)
    if root is None:
        parser.error("run inside the repository containing configs/process.yaml")
    os.chdir(root)
    try:
        if not picture.is_file() or not math.isfinite(args.size_mm) or args.size_mm <= 0:
            raise ValueError("supply an existing picture and a positive size")
        process = yaml.safe_load(Path("configs/process.yaml").read_text())
        cell = yaml.safe_load(Path("configs/cell.yaml").read_text())
        spec = yaml.safe_load(Path("configs/surface.yaml").read_text())
        made = make_relief(picture, args.size_mm, process, fill=args.fill)
        data = save_relief(made, output, picture)
        toolpath = slice_relief(data, load_surface(spec), process)
        job = make_job(toolpath, cell)
        write_json(output / "toolpath.json", toolpath, compact=True)
        write_json(output / "job.json", job, compact=True)
        (output / "moveit_report.json").unlink(missing_ok=True)
    except (ValueError, OSError, KeyError) as error:
        parser.error(str(error))
    print(summary(toolpath))
    print(f"Prepared (unplanned): {output / 'job.json'}")
    print("Next on Ubuntu with ROS/Flexiv installed: ./rviz.sh PATH/TO/job.json")


if __name__ == "__main__":
    main()
