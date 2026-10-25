import numpy as np
import pytest
from pen_toolpath.continuous_stroke import LocalStrokeTiming, StrokeTiming


def test_long_stroke_reaches_speed_briefly_then_cruises():
    timing=StrokeTiming(.1,.009,.09)
    assert timing.ramp==pytest.approx(.1)
    assert timing.distance(0)==0
    assert timing.distance(timing.duration)==pytest.approx(.1)
    t=np.linspace(.2,timing.duration-.2,100)
    velocity=np.diff([timing.distance(v) for v in t])/np.diff(t)
    assert velocity==pytest.approx(np.full_like(velocity,.009))
    assert timing.distance(.2)>.001


def test_short_stroke_brakes_without_overshooting():
    timing=StrokeTiming(.0001,.009,.09)
    assert timing.speed<.009
    t=np.linspace(0,timing.duration,1001)
    position=np.array([timing.distance(v) for v in t])
    assert np.diff(position).min()>=0
    assert position[-1]==pytest.approx(.0001)
    velocity=np.diff(position)/np.diff(t)
    assert velocity.max()<=timing.speed*1.001
    assert np.max(np.abs(np.diff(velocity)/np.diff(t[:-1])))<=.09001


@pytest.mark.parametrize('length,speed,acceleration',[(0,.01,.1),(.1,0,.1),(.1,.01,-1),(float('nan'),.01,.1)])
def test_invalid_timing_refused(length,speed,acceleration):
    with pytest.raises(ValueError):StrokeTiming(length,speed,acceleration)


def test_local_caps_allow_fast_sections_but_brake_for_tight_bend():
    positions=np.linspace(0,.1,1001)
    caps=np.full_like(positions,.04)
    caps[450:551]=.002
    timing=LocalStrokeTiming(positions,caps,.03)
    assert timing.speeds[0]==timing.speeds[-1]==0
    assert np.all(timing.speeds<=caps+1e-12)
    assert np.abs(timing.accelerations).max()<=.03+1e-12
    assert timing.speeds[:450].max()>.03
    assert timing.duration<StrokeTiming(.1,.002,.03).duration*.4
    times=np.linspace(0,timing.duration,10001)
    path=np.array([timing.distance(t) for t in times])
    assert path[0]==0 and path[-1]==pytest.approx(.1)
    assert np.diff(path).min()>=0
    assert timing.distance(-1)==0 and timing.distance(timing.duration+1)==pytest.approx(.1)


def test_local_timing_short_section_starts_and_stops():
    timing=LocalStrokeTiming([0,.00001],[.04,.04],.03)
    assert timing.duration>0
    assert timing.distance(timing.duration)==pytest.approx(.00001)
    assert timing.speeds[0]==timing.speeds[-1]==0
