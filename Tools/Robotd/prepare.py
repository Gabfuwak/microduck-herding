#!/usr/bin/env python3
"""Build the pinned Linux robotd runtime without installing system services."""
import hashlib, json, os, pathlib, platform, shutil, subprocess, tarfile, urllib.request
ROOT=pathlib.Path(__file__).resolve().parents[2]
RUNTIME=ROOT/'Builds/Robotd'
REV='b2b0d213a402caeac492dd662ea891faa17f67c0'
POLICY_REV='665ae4eb20bfe13bca1ef4ea6d4bb6b4a096e448'
ORT_VERSION='1.23.2'
ORT_SHA='1fa4dcaef22f6f7d5cd81b28c2800414350c10116f5fdd46a2160082551c5f9b'
PATCH=ROOT/'Tools/Robotd/simulation-clock.patch'

def run(*args,cwd=None): subprocess.run(args,cwd=cwd,check=True)
def sha(path): return hashlib.sha256(path.read_bytes()).hexdigest()
def download(url,dest):
    dest.parent.mkdir(parents=True,exist_ok=True)
    if not dest.exists():
        print('Downloading',dest.name,flush=True)
        part=dest.with_suffix(dest.suffix+'.partial')
        urllib.request.urlretrieve(url,part);part.replace(dest)

def main():
    if platform.system()!='Linux' or platform.machine()!='x86_64':
        raise SystemExit('This runtime targets Linux x86_64, as does the desktop scene.')
    source=RUNTIME/'source'; binaries=RUNTIME/'bin'; binaries.mkdir(parents=True,exist_ok=True)
    if not (source/'.git').exists():
        source.mkdir(parents=True,exist_ok=True)
        run('git','init',str(source));run('git','remote','add','origin','https://github.com/pollen-robotics/microduck.git',cwd=source)
        run('git','fetch','--depth','1','origin',REV,cwd=source);run('git','checkout','--detach','FETCH_HEAD',cwd=source)
    actual=subprocess.check_output(['git','rev-parse','HEAD'],cwd=source,text=True).strip()
    if actual!=REV: raise SystemExit(f'Unexpected source revision {actual}; expected {REV}')
    status=subprocess.check_output(['git','diff','--binary'],cwd=source)
    already=bool(status)
    if already and status!=PATCH.read_bytes():
        raise SystemExit('Build source contains changes other than our recorded clock patch; preserve and inspect them.')
    if not already:
        run('cargo','build','--locked','--release','-p','robotd','-p','robotctl',cwd=source)
        shutil.copy2(source/'target/release/robotd',binaries/'robotd-upstream')
        shutil.copy2(source/'target/release/robotctl',binaries/'robotctl')
        run('git','apply','--check',str(PATCH),cwd=source);run('git','apply',str(PATCH),cwd=source)
    run('cargo','build','--locked','--release','-p','robotd',cwd=source)
    # Atomic replacement also allows a running Unity session to finish using its old executable.
    shutil.copy2(source/'target/release/robotd',binaries/'robotd.next');(binaries/'robotd.next').replace(binaries/'robotd')
    (RUNTIME/'models').mkdir(exist_ok=True)
    shutil.copy2(source/'duck-control/models/pickup_detector.onnx',RUNTIME/'models/pickup_detector.onnx')
    archive=RUNTIME/'onnxruntime.tgz'
    download(f'https://github.com/microsoft/onnxruntime/releases/download/v{ORT_VERSION}/onnxruntime-linux-x64-{ORT_VERSION}.tgz',archive)
    if sha(archive)!=ORT_SHA: raise SystemExit('ONNX Runtime archive checksum mismatch')
    ort=RUNTIME/f'onnxruntime-linux-x64-{ORT_VERSION}'
    if not ort.exists():
        with tarfile.open(archive) as tf: tf.extractall(RUNTIME,filter='data')
    policies=RUNTIME/'policies';base=f'https://huggingface.co/pollen-robotics/microduck-policies/resolve/{POLICY_REV}/'
    download(base+'manifest.json',policies/'manifest.json')
    manifest=json.loads((policies/'manifest.json').read_text())
    names=[p['file'] for p in manifest['policies']]
    for name in names:
        if pathlib.Path(name).name!=name: raise SystemExit('Invalid policy filename')
        download(base+name,policies/name)
    expected='2fc7f8ee6acd7d0d3f66a99927b5d5a56589aeed02ec714de329b252d5ccf492'
    if sha(policies/'velstand.onnx')!=expected: raise SystemExit('velstand checksum mismatch')
    provenance={'robotd_revision':REV,'clock_patch_sha256':sha(PATCH),'onnxruntime_version':ORT_VERSION,
        'onnxruntime_archive_sha256':ORT_SHA,'policy_revision':POLICY_REV,
        'policy_sha256':{name:sha(policies/name) for name in names},
        'robotd_sha256':sha(binaries/'robotd'),'pickup_detector_sha256':sha(RUNTIME/'models/pickup_detector.onnx')}
    (RUNTIME/'provenance.json').write_text(json.dumps(provenance,indent=2)+'\n')
    print('Ready:',binaries/'robotd')
if __name__=='__main__':main()
