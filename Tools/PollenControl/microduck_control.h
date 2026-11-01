#ifndef MICRODUCK_CONTROL_H
#define MICRODUCK_CONTROL_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
/* ABI 1. Per-handle calls must be serialized; different handles may run independently.
 * Positions/velocities: radians and rad/s in Pollen JOINT_NAMES order (15 joints).
 * Gyro and unit gravity: trunk coordinates, upright gravity=(0,0,-1).
 * Quaternion: trunk-to-world, scalar first (w,x,y,z). No Unity axis conversion here.
 * Command: forward m/s, left m/s, counterclockwise yaw rad/s in robot coordinates. */
typedef struct { double positions[15], velocities[15], currents_ma[15], gyro[3], gravity[3], quat[4]; } MdSensors;
typedef struct { double forward, left, yaw_rate; } MdCommand;
typedef struct { double targets[15]; uint32_t gain, torque, state, fallen; } MdOutput;
/* state: 0 initializing/held, 1 homing, 2 policy, 3 limp_fall, 4 limp_pose.
 * fallen is telemetry, independent of state. Gain is XL330 position P (200 nominal).
 * All int functions return 0 on success, -1 on failure; failed output is not valid.
 * Paths are UTF-8. ort_path must name the same runtime throughout this process. */
uint32_t md_abi_version(void);
int md_create(const char *models_dir, const char *ort_path, void **out_handle);
/* A fresh complete snapshot and command every 20,000 simulation microseconds.
 * First timestamp is arbitrary; subsequent timestamps must advance exactly 20,000.
 * Do not step while physics is paused; caller expires commands using simulation time. */
int md_step(void *handle, const MdSensors *sensors, const MdCommand *command, uint64_t sim_time_us, MdOutput *output);
/* Explicit initial placement only: discard the policy episode and restart homing.
 * Does not modify any physical pose. Never call because of a fall or surface rebuild. */
int md_reset(void *handle);
void md_destroy(void *handle);
/* Thread-local diagnostic valid until the next API call on this thread. */
const char *md_last_error(void);
#ifdef __cplusplus
}
#endif
#endif
