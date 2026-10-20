// Native Android smoke test for the pinned MuJoCo public C API.
#include <dlfcn.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <mujoco/mujoco.h>

typedef struct {
  __typeof__(&mj_version) version;
  __typeof__(&mj_versionString) version_string;
  __typeof__(&mj_defaultVFS) default_vfs;
  __typeof__(&mj_addBufferVFS) add_buffer_vfs;
  __typeof__(&mj_loadXML) load_xml;
  __typeof__(&mj_deleteVFS) delete_vfs;
  __typeof__(&mj_makeData) make_data;
  __typeof__(&mj_step) step;
  __typeof__(&mj_deleteData) delete_data;
  __typeof__(&mj_deleteModel) delete_model;
  __typeof__(&mj_stateSize) state_size;
  __typeof__(&mj_getState) get_state;
} MujocoApi;

#define LOAD_API(handle, api, member, symbol_name)                                  \
  do {                                                                               \
    void* symbol = dlsym((handle), (symbol_name));                                   \
    if (!symbol) {                                                                   \
      fprintf(stderr, "Missing MuJoCo symbol %s: %s\n", (symbol_name), dlerror()); \
      goto cleanup;                                                                  \
    }                                                                                \
    memcpy(&(api).member, &symbol, sizeof((api).member));                            \
  } while (0)

static int finite_values(const mjtNum* values, int count) {
  for (int i = 0; i < count; ++i) {
    if (!isfinite((double)values[i])) return 0;
  }
  return 1;
}

int main(int argc, char** argv) {
  int exit_code = 1;
  void* library = NULL;
  mjVFS vfs;
  int vfs_initialized = 0;
  mjModel* model = NULL;
  mjData* data = NULL;
  MujocoApi api = {0};
  const char* version_text = NULL;
  int version = 0;
  mjtNum state[16] = {0};

  if (argc != 2) {
    fprintf(stderr, "Usage: %s /absolute/path/to/libmujoco.so\n", argv[0]);
    return 2;
  }
  library = dlopen(argv[1], RTLD_NOW | RTLD_LOCAL);
  if (!library) {
    fprintf(stderr, "Cannot load %s: %s\n", argv[1], dlerror());
    goto cleanup;
  }

  LOAD_API(library, api, version, "mj_version");
  LOAD_API(library, api, version_string, "mj_versionString");
  LOAD_API(library, api, default_vfs, "mj_defaultVFS");
  LOAD_API(library, api, add_buffer_vfs, "mj_addBufferVFS");
  LOAD_API(library, api, load_xml, "mj_loadXML");
  LOAD_API(library, api, delete_vfs, "mj_deleteVFS");
  LOAD_API(library, api, make_data, "mj_makeData");
  LOAD_API(library, api, step, "mj_step");
  LOAD_API(library, api, delete_data, "mj_deleteData");
  LOAD_API(library, api, delete_model, "mj_deleteModel");
  LOAD_API(library, api, state_size, "mj_stateSize");
  LOAD_API(library, api, get_state, "mj_getState");

  version = api.version();
  version_text = api.version_string();
  if (version != mjVERSION_HEADER || !version_text || strcmp(version_text, "3.15.0") != 0) {
    fprintf(stderr, "Expected MuJoCo 3.15.0, got version=%d string=%s\n", version,
            version_text ? version_text : "(null)");
    goto cleanup;
  }

  {
    static const char xml[] =
        "<mujoco model=\"android-native-smoke\">"
        "<option timestep=\"0.005\" gravity=\"0 0 -9.81\"/>"
        "<worldbody>"
        "<geom name=\"floor\" type=\"plane\" size=\"2 2 0.1\"/>"
        "<body name=\"drop\" pos=\"0 0 0.5\">"
        "<freejoint/>"
        "<geom name=\"ball\" type=\"sphere\" size=\"0.1\" mass=\"1\"/>"
        "</body>"
        "</worldbody>"
        "</mujoco>";
    char error[1024] = {0};
    api.default_vfs(&vfs);
    vfs_initialized = 1;
    if (api.add_buffer_vfs(&vfs, "smoke.xml", xml, (int)(sizeof(xml) - 1)) != 0) {
      fprintf(stderr, "Could not add MJCF to VFS\n");
      goto cleanup;
    }
    model = api.load_xml("smoke.xml", &vfs, error, (int)sizeof(error));
    if (!model) {
      fprintf(stderr, "Could not compile MJCF: %s\n", error[0] ? error : "unknown error");
      goto cleanup;
    }
  }

  if (model->nq != 7 || model->nv != 6) {
    fprintf(stderr, "Unexpected drop model dimensions: nq=%d nv=%d\n", (int)model->nq,
            (int)model->nv);
    goto cleanup;
  }
  data = api.make_data(model);
  if (!data) {
    fprintf(stderr, "Could not allocate MuJoCo data\n");
    goto cleanup;
  }
  for (int i = 0; i < 1000; ++i) api.step(model, data);

  {
    const int signature = mjSTATE_TIME | mjSTATE_QPOS;
    const int count = api.state_size(model, signature);
    const double initial_height = 0.5;
    const double height = (double)data->qpos[2];
    const double quaternion_norm = sqrt((double)data->qpos[3] * (double)data->qpos[3] +
                                        (double)data->qpos[4] * (double)data->qpos[4] +
                                        (double)data->qpos[5] * (double)data->qpos[5] +
                                        (double)data->qpos[6] * (double)data->qpos[6]);
    if (count != 8 || count > (int)(sizeof(state) / sizeof(state[0]))) {
      fprintf(stderr, "Unexpected state size for time+qpos: %d\n", count);
      goto cleanup;
    }
    api.get_state(model, data, state, signature);
    if (!finite_values(state, count) || !isfinite((double)data->time) ||
        fabs((double)data->time - 5.0) > 1e-8) {
      fprintf(stderr, "Non-finite state or unexpected simulation time %.17g\n", (double)data->time);
      goto cleanup;
    }
    if (!finite_values(data->qpos, 7) || !isfinite(quaternion_norm) ||
        fabs(quaternion_norm - 1.0) > 1e-6) {
      fprintf(stderr, "Invalid final position or quaternion norm %.17g\n", quaternion_norm);
      goto cleanup;
    }
    if (!(height < initial_height - 0.1) || fabs(height - 0.1) > 0.02 ||
        fabs((double)data->qvel[2]) > 0.05) {
      fprintf(stderr, "Drop did not settle on floor: initial_z=%.9g final_z=%.9g vz=%.9g\n",
              initial_height, height, (double)data->qvel[2]);
      goto cleanup;
    }
    printf("{\"version\":\"%s\",\"version_number\":%d,\"steps\":1000,"
           "\"timestep\":0.005,\"time\":%.17g,\"initial_height\":%.17g,"
           "\"height\":%.17g,\"qpos\":[%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g],"
           "\"quaternion_norm\":%.17g,\"state\":[",
           version_text, version, (double)data->time, initial_height, height,
           (double)data->qpos[0], (double)data->qpos[1], (double)data->qpos[2],
           (double)data->qpos[3], (double)data->qpos[4], (double)data->qpos[5],
           (double)data->qpos[6], quaternion_norm);
    for (int i = 0; i < count; ++i) printf("%s%.17g", i ? "," : "", (double)state[i]);
    printf("]}\n");
    exit_code = 0;
  }

cleanup:
  if (data) api.delete_data(data);
  if (model) api.delete_model(model);
  if (vfs_initialized) api.delete_vfs(&vfs);
  if (library) dlclose(library);
  return exit_code;
}
