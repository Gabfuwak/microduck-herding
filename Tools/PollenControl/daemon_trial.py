#!/usr/bin/env python3
"""Optional integration oracle: exercise the existing desktop robotd against scripted sensors.
Adapted from Tools/Robotd/clock_trial.py in the robotd-navigation project.
This script is development-only; it is never part of the Unity runtime.
"""
import argparse, json, pathlib, queue, socket, socketserver, subprocess, tempfile, threading, time, os
RUNTIME=None  # Set by main from --runtime; points at the existing desktop adapter installation.
HOME=[0,-.0873,-.4579,-.0049,.453,.3491,.3491,0,0,0,0,.0873,.4579,.0049,-.453]

def trial(speed, pause=False):
    ready=queue.Queue(); committed=queue.Queue(); outputs=[]; errors=[]; rows=[]
    index=0; kp=200; torque=False; connections=[]; pause_health=None
    def sensors():
        t=index*.02
        gravity=[.5,0,-.8660254] if 3.5<=t<3.62 else [1,0,0] if 3.62<=t<4.8 else [0,0,-1]
        gyro=[0,3,0] if 3.5<=t<3.62 else [0,0,0]
        return dict(positions=HOME, velocities=[0]*15, imu=dict(gyro=gyro,gravity=gravity,quat=[1,0,0,0]))
    class Body(socketserver.StreamRequestHandler):
        def handle(self):
            nonlocal index,kp,torque
            connections.append(self.connection)
            self.connection.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
            try:
                for line in self.rfile:
                    r=json.loads(line); op=r['op']; a={}
                    if op=='hello': assert r['protocol']==2 and r['joints']==15; a={'protocol':2}
                    elif op=='read': a=sensors()
                    elif op=='gain': kp=r['kp']
                    elif op=='torque': torque=r['on']
                    elif op=='write': outputs.append({'tick':index,'targets':r['targets'],'kp':kp,'torque':torque})
                    elif op=='slow': a={'volts':7.4,'temps_c':[32]*15}
                    elif op=='tick':
                        assert r['period_us']==20000
                        try: index=ready.get(timeout=.04); advance=True
                        except queue.Empty: advance=False
                        a={'time_us':index*20000,'advance':advance}
                    elif op=='commit': committed.put(index)
                    else: raise AssertionError(op)
                    self.wfile.write((json.dumps(a)+'\n').encode());self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError): pass
            except Exception as e: errors.append(repr(e))
    server=socketserver.ThreadingTCPServer(('127.0.0.1',0),Body);server.daemon_threads=True
    threading.Thread(target=server.serve_forever,daemon=True).start()
    with tempfile.TemporaryDirectory(prefix='md-clock-') as temp:
        temp=pathlib.Path(temp); sock=temp/'r.sock'; config=temp/'params.toml'
        config.write_text('[policy]\nwalk = '+json.dumps(str(RUNTIME/'policies/velstand.onnx'))+'\nstand="none"\nsitstand="none"\nground_pick="none"\nkick_left="none"\nkick_right="none"\nroulade="none"\n[pickup]\nenabled=false\n[audio]\nenabled=false\n[idle]\nenabled=false\n')
        env=os.environ|{'ORT_DYLIB_PATH':str(RUNTIME/'onnxruntime-linux-x64-1.23.2/lib/libonnxruntime.so.1.23.2'),'RUST_LOG':'warn','DUCK_RUNTIME_DIR':str(temp)}
        with (temp/'log').open('w') as log:
            process=subprocess.Popen([str(RUNTIME/'bin/robotd'),'--sim',f'127.0.0.1:{server.server_address[1]}','--sim-lockstep','--socket',str(sock),'--params',str(config)],env=env,stdout=log,stderr=log)
            try:
                deadline=time.monotonic()+10
                while not sock.exists():
                    if process.poll() is not None: raise RuntimeError((temp/'log').read_text())
                    assert time.monotonic()<deadline; time.sleep(.01)
                command=socket.socket(socket.AF_UNIX);command.settimeout(3);command.connect(str(sock));cf=command.makefile('rw')
                rid=0
                def rpc(method,params=None):
                    nonlocal rid
                    rid+=1;cf.write(json.dumps(dict(jsonrpc='2.0',id=rid,method=method,params=params or {}))+'\n');cf.flush()
                    a=json.loads(cf.readline());assert 'error' not in a,a;return a['result']
                assert rpc('robot.enable',{'on':True})['accepted']
                watch=socket.socket(socket.AF_UNIX);watch.connect(str(sock));wf=watch.makefile('rw')
                wf.write('{"jsonrpc":"2.0","id":1,"method":"robot.subscribe","params":{"hz":1000}}\n');wf.flush()
                def observe():
                    try:
                        for line in wf:
                            row=json.loads(line)
                            if row.get('method')=='robot.state': rows.append(row['params'])
                    except (OSError,ValueError): pass
                watcher=threading.Thread(target=observe,daemon=True);watcher.start()
                # Wait until startup's sensor read and model loading are complete, without advancing sim time.
                time.sleep(.2)
                start=time.monotonic()
                for tick in range(300):
                    if tick<140: rpc('robot.move',{'vx':.15,'vy':0,'vyaw':0})
                    if pause and tick==180:
                        time.sleep(.65)
                        pause_health=rpc('robot.health'); assert pause_health['healthy'],pause_health
                        assert len(outputs)==180, "Pause advanced the controller"
                    ready.put(tick)
                    assert committed.get(timeout=3)==tick
                    time.sleep(.02/speed)
                elapsed=time.monotonic()-start
                time.sleep(.05)
                assert len(outputs)==300,(len(outputs),errors,(temp/'log').read_text())
                assert not errors,errors
                # A live waiting reply is healthy; a dead transport must not be.
                server.shutdown(); server.server_close()
                for connection in connections:
                    try: connection.shutdown(socket.SHUT_RDWR); connection.close()
                    except OSError: pass
                time.sleep(.35)
                disconnected_health=rpc('robot.health')
                assert not disconnected_health['healthy'], disconnected_health
                result={'pause_health':pause_health,'disconnected_health':disconnected_health,'speed':speed,'wall_seconds':elapsed,'outputs':outputs,'states':rows,'log':(temp/'log').read_text()}
                return result
            finally:
                process.terminate();process.wait(timeout=5)
                server.shutdown();server.server_close()

def main():
    global RUNTIME
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--runtime',type=pathlib.Path,required=True)
    p.add_argument('--output',type=pathlib.Path,required=True)
    a=p.parse_args();RUNTIME=a.runtime.resolve()
    one=trial(1);four=trial(4,True)
    assert one['outputs']==four['outputs'], 'Daemon changed with wall clock speed'
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps({'one':one,'four':four},indent=2)+'\n')
    print(json.dumps({'ticks':len(one['outputs']),'outputs_identical':True,'output':str(a.output)}))
if __name__=='__main__':main()
