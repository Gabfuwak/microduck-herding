//! C ABI host for the original Pollen controller and extracted navigation control loop.
//! Generated upstream fragments are marked by prepare.py and recorded in provenance.json.
#![allow(dead_code)]
use duck_control::{
    DEFAULT_POSITION, ImuData, IoError, JointTargets, NUM_JOINTS, RobotIo, Sensors, SlowSensors,
};
use duck_control::{
    fall::{FallPredictor, FallPredictorConfig},
    obs::{BodyPose, Command as PolicyCommand},
    policy::{DEFAULT_STANDING_THRESHOLD, Policy, PolicyError, PolicyPaths},
    safety::{Safety, SafetyConfig},
};
use robotd_params::{self as params, Mode};
use std::{
    cell::RefCell,
    ffi::{CStr, CString, c_char, c_void},
    path::Path,
    rc::Rc,
    sync::Mutex,
    time::{Duration, Instant},
};
mod control;
mod posture;
use control::{Controller, SkillTuning, Tuning};
const HOME_RAMP: Duration = Duration::from_secs(2);
const COAST_TICKS: u32 = 3;
const RESET_AFTER_PAUSE: Duration = Duration::from_millis(200);
const PERIOD: Duration = Duration::from_millis(20);
include!("upstream_helpers.rs");

#[repr(C)]
#[derive(Clone, Copy)]
pub struct MdSensors {
    pub positions: [f64; 15],
    pub velocities: [f64; 15],
    pub currents_ma: [f64; 15],
    pub gyro: [f64; 3],
    pub gravity: [f64; 3],
    pub quat: [f64; 4],
}
#[repr(C)]
#[derive(Clone, Copy)]
pub struct MdCommand {
    pub forward: f64,
    pub left: f64,
    pub yaw_rate: f64,
}
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct MdOutput {
    pub targets: [f64; 15],
    pub gain: u32,
    pub torque: u32,
    pub state: u32,
    pub fallen: u32,
}
struct MemoryState {
    sensors: Sensors,
    output: MdOutput,
}
struct MemoryIo(Rc<RefCell<MemoryState>>);
impl RobotIo for MemoryIo {
    fn read(&mut self) -> Result<Sensors, IoError> {
        Ok(self.0.borrow().sensors)
    }
    fn write(&mut self, target: &JointTargets) -> Result<(), IoError> {
        self.0.borrow_mut().output.targets = target.positions;
        Ok(())
    }
    fn set_gain(&mut self, gain: u16) -> Result<(), IoError> {
        self.0.borrow_mut().output.gain = gain.into();
        Ok(())
    }
    fn set_torque(&mut self, on: bool) -> Result<(), IoError> {
        self.0.borrow_mut().output.torque = u32::from(on);
        Ok(())
    }
    fn reboot(&mut self, _: u8) -> Result<(), IoError> {
        Ok(())
    }
    fn slow_sensors(&mut self) -> Result<SlowSensors, IoError> {
        Ok(SlowSensors {
            volts: 7.4,
            temps_c: [25.; 15],
        })
    }
}
struct LoopState {
    bringup: Bringup,
    limp_fall: LimpFall,
    falling: FallPredictor,
    coast: Coast,
    hold: [f64; 15],
    twist_ema: [f64; 3],
    head_ema: [f64; 4],
    body_ema: [f64; 3],
    was_driving: bool,
    posture_checked: bool,
    stopped_driving_at: Option<Instant>,
}
impl Default for LoopState {
    fn default() -> Self {
        Self {
            bringup: Bringup::Limp,
            limp_fall: LimpFall::Idle,
            falling: FallPredictor::new(FallPredictorConfig::default()),
            coast: Coast::new(),
            hold: DEFAULT_POSITION,
            twist_ema: [0.; 3],
            head_ema: [0.; 4],
            body_ema: [0.; 3],
            was_driving: false,
            posture_checked: false,
            stopped_driving_at: None,
        }
    }
}
// These fixed values are the selected simulator configuration, not substitutes for a daemon.
// The ABI supports autonomous navigation and upstream automatic fall/seated recovery.
// Hardware IPC, process shutdown, pickup, idle animation and explicit skill requests are absent.
struct PoseSnapshot {
    active: bool,
    body: [f64; 3],
}
struct Snapshot {
    enabled: bool,
    pose: PoseSnapshot,
}
struct Core {
    controller: Option<Controller>,
    params: params::Params,
    policy_cfg: params::ResolvedPolicy,
    state: LoopState,
    io: Rc<RefCell<MemoryState>>,
    safety: Safety<MemoryIo>,
    epoch: Instant,
    last_time: Option<u64>,
    faulted: bool,
}
impl Core {
    fn new(models: &Path, ort_path: &str) -> Result<Self, String> {
        duck_control::policy::configure_runtime(ort_path)?;
        let manifest_path = models.join("manifest.json");
        let manifest: params::SetManifest = serde_json::from_slice(
            &std::fs::read(&manifest_path)
                .map_err(|e| format!("{}: {e}", manifest_path.display()))?,
        )
        .map_err(|e| format!("{}: {e}", manifest_path.display()))?;
        let mut params = params::Params::default();
        params.policy.walk = Some(models.join("velstand.onnx"));
        params.policy.stand = Some("none".into());
        params.policy.sitstand = Some(models.join("alpha_sitstand.onnx"));
        params.policy.ground_pick = Some(models.join("alpha_ground_pick.onnx"));
        params.policy.kick_left = Some(models.join("ball_kick_left.onnx"));
        params.policy.kick_right = Some(models.join("ball_kick_right.onnx"));
        params.policy.roulade = Some(models.join("roulade.onnx"));
        params.pickup.enabled = false;
        params.audio.enabled = false;
        params.idle.enabled = false;
        let policy_cfg = params.policy.resolved_with(Some(&manifest));
        let controller =
            try_controller(&policy_cfg, params.safety.limp_fall).map_err(|e| e.to_string())?;
        let io = Rc::new(RefCell::new(MemoryState {
            sensors: Sensors::default(),
            output: MdOutput::default(),
        }));
        let safety = Safety::new(MemoryIo(io.clone()), SafetyConfig::default());
        Ok(Self {
            controller,
            params,
            policy_cfg,
            state: LoopState::default(),
            io,
            safety,
            epoch: Instant::now(),
            last_time: None,
            faulted: false,
        })
    }
    fn reset(&mut self) {
        if let Some(c) = self.controller.as_mut() {
            c.reset();
            c.stop_moves();
            c.leave_seat();
        }
        self.state = LoopState::default();
        self.epoch = Instant::now();
        self.last_time = None;
        self.faulted = false;
        self.io.borrow_mut().output = MdOutput::default();
        self.safety = Safety::new(MemoryIo(self.io.clone()), SafetyConfig::default());
    }
    fn step(
        &mut self,
        input: MdSensors,
        request: MdCommand,
        time_us: u64,
    ) -> Result<MdOutput, String> {
        if self.faulted {
            return Err("controller faulted; explicit reset or recreation required".into());
        }
        if let Some(last) = self.last_time {
            if time_us.checked_sub(last) != Some(20_000) {
                return Err(format!(
                    "expected simulation timestamp {}us, received {time_us}us",
                    last + 20_000
                ));
            }
        }
        let all = input
            .positions
            .iter()
            .chain(input.velocities.iter())
            .chain(input.currents_ma.iter())
            .chain(input.gyro.iter())
            .chain(input.gravity.iter())
            .chain(input.quat.iter());
        if !all.into_iter().all(|v| v.is_finite())
            || ![request.forward, request.left, request.yaw_rate]
                .iter()
                .all(|v| v.is_finite())
        {
            return Err("non-finite sensor or command".into());
        }
        let tick_start = self
            .epoch
            .checked_add(Duration::from_micros(time_us))
            .ok_or("simulation timestamp overflow")?;
        let sample = Sensors {
            positions: input.positions,
            velocities: input.velocities,
            currents_ma: input.currents_ma,
            imu: ImuData {
                gyro: input.gyro,
                gravity: input.gravity,
                quat: input.quat,
            },
        };
        self.io.borrow_mut().sensors = sample;
        let LoopState {
            mut bringup,
            mut limp_fall,
            mut falling,
            mut coast,
            mut hold,
            mut twist_ema,
            mut head_ema,
            mut body_ema,
            mut was_driving,
            mut posture_checked,
            mut stopped_driving_at,
        } = std::mem::take(&mut self.state);
        if self.last_time.is_none() {
            hold = sample.positions;
        }
        let safety = &mut self.safety;
        let controller = &mut self.controller;
        let params = &self.params;
        let policy_cfg = &self.policy_cfg;
        let period = PERIOD;
        let dt = period.as_secs_f64();
        let cmd_alpha = params.control.cmd_alpha.clamp(0., 1.);
        let head_alpha = params.control.head_alpha.clamp(0., 1.);
        let limp_fall_still = Duration::from_millis(params.safety.limp_fall_still_ms);
        let limp_fall_max = Duration::from_millis(params.safety.limp_fall_max_ms);
        let limp_fall_pose = Duration::from_millis(params.safety.limp_fall_pose_ms);
        let snapshot = Snapshot {
            enabled: true,
            pose: PoseSnapshot {
                active: false,
                body: [0.; 3],
            },
        };
        let shutdown_sit: Option<Instant> = None;
        let shutdown_rest: Option<Instant> = None;
        let powered_off = false;
        let pickup_paused = false;
        let fresh = Some(sample);
        let sensors = coast.sample(fresh);
        safety.observe(&sample, period);
        let (gated, _) = safety.gate(
            PolicyCommand {
                twist: [request.forward, request.left, request.yaw_rate],
                head: [0.; 4],
                body: BodyPose::default(),
            },
            Duration::ZERO,
        );
        /* UPSTREAM_FALL */
        /* UPSTREAM_SMOOTH */
        // Idle, breath and chorale disabled, so upstream command composition reduces to these EMAs.
        let command = PolicyCommand {
            twist: twist_ema,
            head: head_ema,
            body: BodyPose {
                z: body_ema[0],
                roll: body_ema[1],
                pitch: body_ema[2],
            },
        };
        /* UPSTREAM_BRINGUP */
        let imu_warm = safety.imu_ready();
        /* UPSTREAM_DRIVING */
        let battery_v: f64 = 7.4;
        let mut inference_error = None;
        /* UPSTREAM_TARGETS */
        if driving {
            targets[duck_control::model::MOUTH_INDEX] = duck_control::model::mouth_target(0.);
        }
        let applied = safety.apply(targets, hold, gain).map_err(|e| e.to_string());
        let state = match limp_fall {
            LimpFall::Limp { .. } => 3,
            LimpFall::Posing { .. } => 4,
            LimpFall::Idle => match bringup {
                Bringup::Limp => 0,
                Bringup::Homing { .. } => 1,
                Bringup::Ready => {
                    if driving {
                        2
                    } else {
                        0
                    }
                }
            },
        };
        let mut output = self.io.borrow().output;
        output.state = state;
        output.fallen = u32::from(safety.fallen());
        self.state = LoopState {
            bringup,
            limp_fall,
            falling,
            coast,
            hold,
            twist_ema,
            head_ema,
            body_ema,
            was_driving,
            posture_checked,
            stopped_driving_at,
        };
        self.last_time = Some(time_us);
        let _ = (moving, policy_label); // Process telemetry is deliberately outside the controller ABI.
        applied?;
        if let Some(error) = inference_error {
            self.faulted = true;
            return Err(error);
        }
        Ok(output)
    }
}
thread_local! {static LAST_ERROR:RefCell<CString>=RefCell::new(CString::new("").unwrap());}
static CREATION: Mutex<()> = Mutex::new(());
fn set_error(message: impl AsRef<str>) {
    let clean = message.as_ref().replace('\0', " ");
    LAST_ERROR.with(|e| *e.borrow_mut() = CString::new(clean).unwrap());
}
fn ffi(f: impl FnOnce() -> Result<(), String>) -> i32 {
    set_error("");
    match std::panic::catch_unwind(std::panic::AssertUnwindSafe(f)) {
        Ok(Ok(())) => 0,
        Ok(Err(e)) => {
            set_error(e);
            -1
        }
        Err(e) => {
            let message = e
                .downcast_ref::<String>()
                .map(String::as_str)
                .or_else(|| e.downcast_ref::<&str>().copied())
                .unwrap_or("native panic");
            set_error(message);
            -1
        }
    }
}
unsafe fn text_arg<'a>(ptr: *const c_char, name: &str) -> Result<&'a str, String> {
    if ptr.is_null() {
        return Err(format!("{name} is null"));
    }
    unsafe { CStr::from_ptr(ptr) }
        .to_str()
        .map_err(|_| format!("{name} is not UTF-8"))
}
#[unsafe(no_mangle)]
pub extern "C" fn md_abi_version() -> u32 {
    1
}
#[unsafe(no_mangle)]
pub extern "C" fn md_last_error() -> *const c_char {
    LAST_ERROR.with(|e| e.borrow().as_ptr())
}
#[unsafe(no_mangle)]
pub unsafe extern "C" fn md_create(
    models_dir: *const c_char,
    ort_path: *const c_char,
    out: *mut *mut c_void,
) -> i32 {
    ffi(|| {
        if out.is_null() {
            return Err("out_handle is null".into());
        }
        unsafe {
            *out = std::ptr::null_mut();
        }
        let models = unsafe { text_arg(models_dir, "models_dir")? };
        let ort = unsafe { text_arg(ort_path, "ort_path")? };
        let _guard = CREATION
            .lock()
            .map_err(|_| "runtime creation lock poisoned")?;
        let core = Core::new(Path::new(models), ort)?;
        unsafe {
            *out = Box::into_raw(Box::new(core)).cast();
        }
        Ok(())
    })
}
#[unsafe(no_mangle)]
pub unsafe extern "C" fn md_step(
    handle: *mut c_void,
    sensors: *const MdSensors,
    command: *const MdCommand,
    time_us: u64,
    out: *mut MdOutput,
) -> i32 {
    ffi(|| {
        if handle.is_null() || sensors.is_null() || command.is_null() || out.is_null() {
            return Err("null step argument".into());
        }
        let core = unsafe { &mut *handle.cast::<Core>() };
        let result = core.step(unsafe { *sensors }, unsafe { *command }, time_us)?;
        unsafe {
            *out = result;
        }
        Ok(())
    })
}
#[unsafe(no_mangle)]
pub unsafe extern "C" fn md_reset(handle: *mut c_void) -> i32 {
    ffi(|| {
        if handle.is_null() {
            return Err("null handle".into());
        }
        unsafe {
            (&mut *handle.cast::<Core>()).reset();
        }
        Ok(())
    })
}
#[unsafe(no_mangle)]
pub unsafe extern "C" fn md_destroy(handle: *mut c_void) {
    let _ = ffi(|| {
        if !handle.is_null() {
            drop(unsafe { Box::from_raw(handle.cast::<Core>()) });
        }
        Ok(())
    });
}
