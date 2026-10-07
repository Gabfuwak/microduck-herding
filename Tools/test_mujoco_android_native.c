// Native loader/model stepping probe; fixed targets, no ONNX walking policy.
#include <mujoco/mujoco.h>
#include <math.h>
#include <stdio.h>
#include <time.h>
int main(int argc, char** argv) {
  if (argc != 2) return 2;
  char error[2048] = {0};
  mjModel* m = mj_loadXML(argv[1], NULL, error, sizeof(error));
  if (!m) { fprintf(stderr, "load error: %s\n", error); return 3; }
  mjData* d = mj_makeData(m);
  if (!d || m->nq != 21 || m->nu != 14) return 4;
  const char* names[] = {"left_hip_yaw","left_hip_roll","left_hip_pitch","left_knee","left_ankle","neck_pitch","head_pitch","head_yaw","head_roll","right_hip_yaw","right_hip_roll","right_hip_pitch","right_knee","right_ankle"};
  double pose[] = {0,-.0872664626,-.457924,-.004940,.452984,.3490658504,.3490658504,0,0,0,.0872664626,.457924,.004940,-.452984};
  d->qpos[2] = .12;
  for (int i=0;i<14;i++) {
    int j=mj_name2id(m,mjOBJ_JOINT,names[i]);
    int a=mj_name2id(m,mjOBJ_ACTUATOR,names[i]);
    if (j<0 || a<0) return 5;
    d->qpos[m->jnt_qposadr[j]]=pose[i]; d->ctrl[a]=pose[i];
  }
  mj_forward(m,d);
  struct timespec start,end; clock_gettime(CLOCK_MONOTONIC,&start);
  for(int i=0;i<2000;i++) {
    mj_step(m,d);
    for(int j=0;j<m->nq;j++) if(!isfinite(d->qpos[j])) return 6;
  }
  clock_gettime(CLOCK_MONOTONIC,&end);
  double elapsed=end.tv_sec-start.tv_sec+(end.tv_nsec-start.tv_nsec)*1e-9;
  printf("PASS native Android MuJoCo %s: nq=%ld nu=%ld physics=%.3fs elapsed=%.3fs rootHeight=%.3fm\n",mj_versionString(),(long)m->nq,(long)m->nu,d->time,elapsed,d->qpos[2]);
  mj_deleteData(d); mj_deleteModel(m); return 0;
}
