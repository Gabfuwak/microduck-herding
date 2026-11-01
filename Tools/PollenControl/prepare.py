#!/usr/bin/env python3
"""Generate the in-process native core from an exact, unmodified Pollen commit."""
import argparse, hashlib, io, json, pathlib, shutil, subprocess, tarfile
PIN = 'b2b0d213a402caeac492dd662ea891faa17f67c0'
REPO = 'https://github.com/pollen-robotics/microduck.git'
HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parents[1]

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--upstream',type=pathlib.Path,required=True,help='Local robotd git clone; working-tree edits are ignored')
    p.add_argument('--output',type=pathlib.Path,default=ROOT/'Builds/PollenControl/source')
    a=p.parse_args(); out=a.output.resolve(); out.mkdir(parents=True,exist_ok=True)
    paths=['Cargo.toml','Cargo.lock','LICENSE','AGENTS.md','CONTRIBUTING.md','duck-control','duck-ipc-proto','kinematics','robotd-params','robotd/src/main.rs','robotd/src/control.rs','robotd/src/posture.rs','deploy/robotd.toml']
    archive=subprocess.check_output(['git','-C',str(a.upstream),'archive',PIN,*paths])
    # Existing target artifacts are retained; tracked upstream files are restored from the pin.
    with tarfile.open(fileobj=io.BytesIO(archive)) as tf: tf.extractall(out,filter='data')
    original=(out/'robotd/src/main.rs').read_text(); pieces={}
    def take(name,start,end):
        assert original.count(start)==1,(name,'start',original.count(start))
        s=original.index(start); e=original.index(end,s)
        part=original[s:e]; pieces[name]={'file':'robotd/src/main.rs','start_line':original[:s].count('\n')+1,'end_line':original[:e].count('\n')+1,'sha256':hashlib.sha256(part.encode()).hexdigest()}
        return part
    helpers=take('fall_helpers','#[derive(Debug, Clone, Copy, PartialEq)]\nenum LimpFall {','#[derive(Parser, Debug)]')
    helpers+=take('bringup_helpers','#[derive(Debug, Clone, Copy, PartialEq)]\n// One of these exists','/// The shutdown\'s ramp')
    helpers+=take('slew','fn slew(','async fn adopt_startup_pose')
    helpers+=take('controller_constructor','fn try_controller(','/// How many times the torque cut')
    helpers+=take('coast_and_resume','struct Coast {','/// Whether a voice bank')
    helpers+=take('seat_and_ramp','const SEAT_POSITION:','/// What each bindable pad button')
    fall=take('fall_transitions','        if params.safety.limp_fall {','        // Pick-up detection')
    smooth=take('command_smoothing','        let twist_target = if in_limp_fall','        // Still: the policy')
    bringup=take('bringup_start','        if !powered_off\n            && let (Bringup::Limp','        // The ramp finishing')
    rampfinish=take('bringup_finish','            if let Bringup::Homing { to, .. } = bringup {','        // A loaded policy change')
    # The omitted prefix waits for model hot-swaps; this ABI loads once at create.
    rampfinish='        if let Bringup::Homing { .. } = bringup\n            && bringup.homing_target(tick_start).is_none()\n        {\n'+rampfinish
    driving=take('posture_and_driving','        if !snapshot.enabled {\n            posture_checked = false;','        // Voltage adaptation:')
    scale=take('voltage_and_fall_targets','        let scale_mult = if policy_cfg.voltage_adapt','            // The end of a shutdown, from the seat')
    inference=take('policy_and_homing_targets','            (true, Some(sensors)) => {\n                let controller = controller.as_mut().expect("driving implies a controller");','        state.moving.store(moving, Ordering::Relaxed);')
    # Expose upstream inference failure through the ABI as well as preserving its hold target.
    inference=inference.replace('tracing::warn!(error = %e, "inference failed; holding");','tracing::warn!(error = %e, "inference failed; holding");\n                        inference_error = Some(e.to_string());')
    core=out/'microduck-control'; (core/'src').mkdir(parents=True,exist_ok=True)
    for name in ['control','posture']:
        shutil.copyfile(out/f'robotd/src/{name}.rs',core/f'src/{name}.rs')
    (core/'src/upstream_helpers.rs').write_text('// Generated from pinned robotd/src/main.rs. See provenance.json.\n'+helpers)
    template=(HERE/'lib.rs').read_text()
    for name,body in [('FALL',fall),('SMOOTH',smooth),('BRINGUP',bringup+rampfinish),('DRIVING',driving),('TARGETS',scale+inference)]:
        template=template.replace('/* UPSTREAM_'+name+' */',body)
    (core/'src/lib.rs').write_text(template)
    shutil.copyfile(HERE/'Cargo.toml',core/'Cargo.toml')
    # Only these crates are used: no daemon, async runtime, serial bus or TCP backend.
    workspace=(out/'Cargo.toml').read_text()
    import re
    members='["microduck-control", "duck-control", "duck-ipc-proto", "kinematics", "robotd-params"]'
    workspace=re.sub(r'^members = .*$', 'members = '+members, workspace,flags=re.M)
    workspace=re.sub(r'^default-members = .*$', 'default-members = ["microduck-control"]',workspace,flags=re.M)
    (out/'Cargo.toml').write_text(workspace)
    f=out/'duck-control/src/lib.rs'; t=f.read_text().replace('pub mod bus;','#[cfg(feature = "hardware")]\npub mod bus;').replace('pub mod sim;','#[cfg(feature = "transport")]\npub mod sim;');f.write_text(t)
    f=out/'duck-control/Cargo.toml';t=f.read_text().replace('rustypot = "1.8.0"','rustypot = { version = "1.8.0", optional = true }').replace('serialport = { version = "4.8", default-features = false }','serialport = { version = "4.8", default-features = false, optional = true }');t+='\n[features]\ndefault = ["hardware", "transport"]\nhardware = ["dep:rustypot", "dep:serialport"]\ntransport = []\n';f.write_text(t)
    # Explicit dynamic-library initialization avoids changing process environment while Unity threads run.
    f=out/'duck-control/src/policy.rs';t=f.read_text();needle='fn dylib_name() -> String {\n';assert needle in t
    adapter='''static EXPLICIT_RUNTIME: OnceLock<String> = OnceLock::new();
/// In-process host adapter: select the packaged runtime without mutating the environment.
pub fn configure_runtime(path: &str) -> Result<(), String> {
    if let Some(previous) = EXPLICIT_RUNTIME.get() {
        return if previous == path { Ok(()) } else { Err("ONNX Runtime path cannot change inside a process".into()) };
    }
    ort::init_from(path).map_err(|e| e.to_string())?.commit();
    EXPLICIT_RUNTIME.set(path.to_owned()).map_err(|_| "concurrent runtime configuration".to_string())
}
'''
    t=t.replace(needle,adapter+needle+'    if let Some(path) = EXPLICIT_RUNTIME.get() { return path.clone(); }\n');f.write_text(t)
    provenance={'upstream':REPO,'commit':PIN,'archive_sha256':hashlib.sha256(archive).hexdigest(),'archive_paths':paths,'abi':1,'verbatim_modules':{},'extracted_fragments':pieces,'adaptations':['Feature-gate serial hardware and TCP backend; build only control dependency crates.','Supply simulation clock and memory RobotIo; fixed enabled navigation configuration, fresh sensors/commands at 50Hz.','Explicit ORT path initialization; no environment mutation.','Exclude disabled audio, pickup, idle, instruments, shutdown, IPC and model hot-swap paths.','Report inference errors across ABI while retaining original hold behavior.']}
    for file in ['robotd/src/control.rs','robotd/src/posture.rs','duck-control/src/obs.rs','duck-control/src/fall.rs','duck-control/src/safety.rs','duck-control/src/model.rs']:
        provenance['verbatim_modules'][file]=hashlib.sha256((out/file).read_bytes()).hexdigest()
    provenance['host_sources']={name:hashlib.sha256((HERE/name).read_bytes()).hexdigest() for name in ['lib.rs','prepare.py','Cargo.toml','microduck_control.h']}
    (out.parent/'provenance.json').write_text(json.dumps(provenance,indent=2)+'\n')
    print(out)
if __name__=='__main__':main()
