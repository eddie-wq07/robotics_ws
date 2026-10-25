"""Tangent corner blends and bounded, short-ramp stroke timing."""
import math
import numpy as np


class StrokeTiming:
    """Accelerate briefly, cruise, then brake; never ease over the whole stroke."""
    def __init__(self, length, speed, acceleration):
        if not all(math.isfinite(v) and v>0 for v in (length,speed,acceleration)):
            raise ValueError('Stroke length, speed and acceleration must be finite and positive')
        self.length=length
        self.acceleration=acceleration
        self.speed=min(speed,math.sqrt(length*acceleration))
        self.ramp=self.speed/acceleration
        self.duration=length/self.speed+self.ramp

    def distance(self, time_s):
        t=min(self.duration,max(0.,time_s))
        if t<self.ramp:
            return .5*self.acceleration*t*t
        if t<=self.duration-self.ramp:
            return .5*self.speed*self.ramp+self.speed*(t-self.ramp)
        return self.length-.5*self.acceleration*(self.duration-t)**2


class LocalStrokeTiming:
    """Follow local speed caps with bounded acceleration and stopped endpoints."""
    def __init__(self, positions, speed_limits, acceleration):
        positions = np.asarray(positions, float)
        speed_limits = np.asarray(speed_limits, float)
        if (positions.ndim != 1 or len(positions) < 2 or positions.shape != speed_limits.shape
                or not np.isfinite(positions).all() or not np.isfinite(speed_limits).all()
                or positions[0] != 0 or np.any(np.diff(positions) <= 0)
                or np.any(speed_limits <= 0) or not math.isfinite(acceleration) or acceleration <= 0):
            raise ValueError('Local timing requires increasing distances and positive speed limits/acceleration')
        if len(positions) == 2:
            speed_limits = np.interp([0, positions[-1]/2, positions[-1]], positions, speed_limits)
            positions = np.array([0., positions[-1]/2, positions[-1]])
        squared = speed_limits**2
        squared[0] = squared[-1] = 0.
        for i in range(1, len(positions)):
            squared[i] = min(squared[i], squared[i-1] + 2*acceleration*(positions[i]-positions[i-1]))
        for i in range(len(positions)-2, -1, -1):
            squared[i] = min(squared[i], squared[i+1] + 2*acceleration*(positions[i+1]-positions[i]))
        self.positions = positions
        self.speeds = np.sqrt(squared)
        dt = 2*np.diff(positions)/(self.speeds[:-1]+self.speeds[1:])
        self.times = np.r_[0., np.cumsum(dt)]
        self.accelerations = np.diff(self.speeds)/dt
        self.duration = float(self.times[-1])

    def distance(self, time_s):
        t = min(self.duration, max(0., time_s))
        if t >= self.duration:
            return float(self.positions[-1])
        i = int(np.searchsorted(self.times, t, side='right')-1)
        dt = t-self.times[i]
        return float(self.positions[i]+self.speeds[i]*dt+.5*self.accelerations[i]*dt*dt)


def rounded_stroke(points, spacing=.00005, trim=.00025):
    points=np.asarray(points,float)
    result=[points[0]]
    def line(end):
        start=result[-1]
        count=max(1,int(np.ceil(np.linalg.norm(end-start)/spacing)))
        result.extend(start+(end-start)*(i/count) for i in range(1,count+1))
    for i in range(1,len(points)-1):
        a,b,c=points[i-1:i+2]
        left,right=np.linalg.norm(b-a),np.linalg.norm(c-b)
        cut=min(trim,.25*left,.25*right)
        entry=b+(a-b)*(cut/left);exit=b+(c-b)*(cut/right)
        line(entry)
        count=max(8,int(np.ceil(2*cut/spacing)))
        for j in range(1,count+1):
            u=j/count;result.append((1-u)**2*entry+2*u*(1-u)*b+u*u*exit)
    line(points[-1])
    result=np.array(result)
    result=result[np.r_[True,np.linalg.norm(np.diff(result,axis=0),axis=1)>1e-12]]
    distance=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(result,axis=0),axis=1))]
    return distance,result
