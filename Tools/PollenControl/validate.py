#!/usr/bin/env python3
"""Native ABI validation and optional target-for-target comparison against robotd."""
import argparse,ctypes as C,json,math,pathlib,time,hashlib
HOME=[0,-.0873,-.4579,-.0049,.453,.3491,.3491,0,0,0,0,.0873,.4579,.0049,-.453]
class Sensors(C.Structure):_fields_=[('positions',C.c_double*15),('velocities',C.c_double*15),('currents_ma',C.c_double*15),('gyro',C.c_double*3),('gravity',C.c_double*3),('quat',C.c_double*4)]
class Command(C.Structure):_fields_=[('forward',C.c_double),('left',C.c_double),('yaw_rate',C.c_double)]
class Output(C.Structure):_fields_=[('targets',C.c_double*15),('gain',C.c_uint32),('torque',C.c_uint32),('state',C.c_uint32),('fallen',C.c_uint32)]
def sensor(tick,pose=HOME):
    t=tick*.02
    gravity=[.5,0,-.8660254] if 3.5<=t<3.62 else [1,0,0] if 3.62<=t<4.8 else [0,0,-1]
    gyro=[0,3,0] if 3.5<=t<3.62 else [0,0,0]
    return Sensors((C.c_double*15)(*pose),(C.c_double*15)(),(C.c_double*15)(),(C.c_double*3)(*gyro),(C.c_double*3)(*gravity),(C.c_double*4)(1,0,0,0))
class Native:
    def __init__(self,library,models,ort):
        self.lib=C.CDLL(str(library));self.lib.md_last_error.restype=C.c_char_p
        self.lib.md_create.argtypes=[C.c_char_p,C.c_char_p,C.POINTER(C.c_void_p)]
        self.lib.md_step.argtypes=[C.c_void_p,C.POINTER(Sensors),C.POINTER(Command),C.c_uint64,C.POINTER(Output)]
        self.lib.md_reset.argtypes=[C.c_void_p];self.lib.md_destroy.argtypes=[C.c_void_p]
        assert self.lib.md_abi_version()==1
        assert C.sizeof(Sensors)==440 and C.sizeof(Command)==24 and C.sizeof(Output)==136
        self.handle=C.c_void_p();self.check(self.lib.md_create(str(models).encode(),str(ort).encode(),C.byref(self.handle)))
    def check(self,status):
        if status:raise AssertionError(self.lib.md_last_error().decode())
    def step(self,tick,command,pose=HOME):
        s=sensor(tick,pose);out=Output();self.check(self.lib.md_step(self.handle,C.byref(s),C.byref(command),tick*20000,C.byref(out)))
        return dict(tick=tick,targets=list(out.targets),kp=out.gain,torque=bool(out.torque),state=out.state,fallen=bool(out.fallen))
    def reset(self):self.check(self.lib.md_reset(self.handle))
    def close(self):self.lib.md_destroy(self.handle);self.handle=None

def trace(native,pause=False):
    out=[]
    for tick in range(300):
        if pause and tick==180:time.sleep(.1)
        # Same simulation-time deadman as the daemon's clock_trial: last move at tick139,
        # read on next tick's sim clock; expiry occurs at tick164.
        command=Command(.15 if tick<164 else 0,0,0)
        out.append(native.step(tick,command))
    return out

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--library',type=pathlib.Path,required=True);p.add_argument('--models',type=pathlib.Path,required=True);p.add_argument('--ort',type=pathlib.Path,required=True)
    p.add_argument('--daemon-trace',type=pathlib.Path);p.add_argument('--output',type=pathlib.Path,required=True);a=p.parse_args()
    n=Native(a.library,a.models,a.ort)
    try:
        one=trace(n);n.reset();other=trace(n,True);assert one==other,'Explicit reset or wall-clock pause changed native control'
        transitions=[]
        for row in one:
            key=(row['state'],row['kp'],row['fallen'])
            if not transitions or transitions[-1]['state_gain_fallen']!=list(key):transitions.append({'tick':row['tick'],'state_gain_fallen':list(key)})
        assert any(x['state']==3 for x in one),'No limp-fall transition';assert any(x['state']==4 for x in one),'No pose transition';assert one[-1]['state']==2,'Did not return to policy'
        s=sensor(300);command=Command();out=Output()
        assert n.lib.md_step(n.handle,C.byref(s),C.byref(command),299*20000,C.byref(out))!=0,'Duplicate timestamp accepted'
        assert b'timestamp' in n.lib.md_last_error()
        s.positions[0]=math.nan
        assert n.lib.md_step(n.handle,C.byref(s),C.byref(command),300*20000,C.byref(out))!=0,'NaN accepted'
        assert b'non-finite' in n.lib.md_last_error()
        s=sensor(300);n.check(n.lib.md_step(n.handle,C.byref(s),C.byref(command),300*20000,C.byref(out)))
        n.reset();seat=HOME.copy();seat[1]=seat[4]=seat[11]=seat[14]=0;seat[2]=-.4079;seat[3]=1.35;seat[12]=.4079;seat[13]=-1.35
        seated=[n.step(tick,Command(),seat) for tick in range(300)]
        assert all(math.isfinite(v) for row in seated for v in row['targets'])
        assert seated[100]['state']==2 and seated[99]['targets']==seat,'Seated startup did not preserve the seated homing target'
        result={'abi':1,'ticks':300,'pause_and_reset_identical':True,'invalid_input_rejected':True,'seated_startup':True,'transitions':transitions,'library_sha256':hashlib.sha256(a.library.read_bytes()).hexdigest()}
        if a.daemon_trace:
            saved=json.loads(a.daemon_trace.read_text());reference=saved['one']['outputs']
            assert len(reference)==len(one)
            max_error=0.
            for actual,expected in zip(one,reference):
                assert (actual['tick'],actual['kp'],actual['torque'])==(expected['tick'],expected['kp'],expected['torque']),(actual['tick'],actual['kp'],expected['kp'])
                error=max(abs(x-y) for x,y in zip(actual['targets'],expected['targets']));max_error=max(max_error,error)
                assert error<1e-10,(actual['tick'],error,actual['targets'],expected['targets'])
            result['daemon_comparison']={'trace_sha256':hashlib.sha256(a.daemon_trace.read_bytes()).hexdigest(),'max_target_error':max_error,'all_gains_and_torque_equal':True}
        a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps({'summary':result,'outputs':one},indent=2)+'\n');print(json.dumps(result,indent=2))
    finally:n.close()
if __name__=='__main__':main()
