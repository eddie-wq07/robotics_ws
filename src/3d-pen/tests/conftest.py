from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope="session")
def process():
    return yaml.safe_load((ROOT / "configs/process.yaml").read_text())


@pytest.fixture(scope="session")
def house(process):
    from tactile.relief import make_relief, relief_to_json

    return relief_to_json(make_relief(ROOT / "examples/house.png", 64, process))
