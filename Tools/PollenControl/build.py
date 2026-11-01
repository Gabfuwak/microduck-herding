#!/usr/bin/env python3
"""Build the prepared native control library; consumers use the committed plugin binaries."""
import argparse,hashlib,json,os,pathlib,shutil,subprocess
HERE=pathlib.Path(__file__).resolve().parent
ROOT=HERE.parents[1]
def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source',type=pathlib.Path,default=ROOT/'Builds/PollenControl/source')
    p.add_argument('--platform',choices=['linux','android','all'],default='all')
    p.add_argument('--ndk',type=pathlib.Path,default=os.environ.get('ANDROID_NDK_ROOT'))
    p.add_argument('--offline',action='store_true')
    a=p.parse_args();source=a.source.resolve();base=source.parent
    if not (source/'microduck-control/Cargo.toml').exists():p.error('Run prepare.py first')
    provenance_path=base/'provenance.json';provenance=json.loads(provenance_path.read_text())
    artifacts=provenance.setdefault('artifacts',{})
    for platform in (['linux','android'] if a.platform=='all' else [a.platform]):
        env=os.environ.copy();command=['cargo','build','--release','--manifest-path',str(source/'Cargo.toml'),'-p','microduck-control']
        if a.offline:command.append('--offline')
        if platform=='android':
            if not a.ndk:p.error('--ndk or ANDROID_NDK_ROOT is required for Android')
            toolchain=a.ndk.resolve()/'toolchains/llvm/prebuilt/linux-x86_64/bin'
            linker=toolchain/'aarch64-linux-android29-clang'
            if not linker.exists():p.error(f'Missing NDK linker {linker}')
            env['CARGO_TARGET_AARCH64_LINUX_ANDROID_LINKER']=str(linker)
            env['CARGO_TARGET_AARCH64_LINUX_ANDROID_RUSTFLAGS']='-C link-arg=-Wl,-z,max-page-size=16384 -C link-arg=-Wl,-z,common-page-size=16384'
            command+=['--target','aarch64-linux-android'];binary=source/'target/aarch64-linux-android/release/libmicroduck_control.so';name='android-arm64'
        else:binary=source/'target/release/libmicroduck_control.so';name='linux-x86_64'
        subprocess.run(command,env=env,check=True)
        dest=base/'artifacts'/name/binary.name;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(binary,dest)
        artifacts[name]={'path':str(dest.relative_to(base)),'sha256':hashlib.sha256(dest.read_bytes()).hexdigest(),'bytes':dest.stat().st_size,'rustc':subprocess.check_output(['rustc','--version'],text=True).strip(),'cargo_lock_sha256':hashlib.sha256((source/'Cargo.lock').read_bytes()).hexdigest()}
        if platform=='android':artifacts[name]['ndk']=(a.ndk/'source.properties').read_text();artifacts[name]['android_api']=29;artifacts[name]['max_page_size']=16384
        print(dest)
    provenance_path.write_text(json.dumps(provenance,indent=2)+'\n')
if __name__=='__main__':main()
