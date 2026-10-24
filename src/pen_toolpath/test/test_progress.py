import json
from unittest.mock import patch

from pen_toolpath.progress import ProgressWriter


def test_progress_is_throttled_atomic_and_estimates_remaining_time(tmp_path):
    with patch('pen_toolpath.progress.time.monotonic', return_value=100) as clock:
        progress = ProgressWriter(tmp_path)
        progress('trajectory', 0, 100)
        clock.return_value = 104
        progress('trajectory', 20, 100)
        path = tmp_path / 'progress.json'
        data = json.loads(path.read_text())
        assert data['fraction'] == .2
        assert data['eta_seconds'] == 16
        clock.return_value = 104.1
        progress('trajectory', 21, 100)
        assert json.loads(path.read_text()) == data
        progress('return')
        assert json.loads(path.read_text())['eta_seconds'] is None
        assert not path.with_suffix('.tmp').exists()
