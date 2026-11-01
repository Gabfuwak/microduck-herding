// Small native loader check shared by Linux and Android validation.
#include <dlfcn.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

typedef struct {
  const void *(*get_api)(uint32_t version);
  const char *(*get_version)(void);
} OrtApiBase;

static void *load(const char *path) {
  void *library = dlopen(path, RTLD_NOW | RTLD_LOCAL);
  if (!library) fprintf(stderr, "Cannot load %s: %s\n", path, dlerror());
  return library;
}

int main(int argc, char **argv) {
  if (argc != 4) {
    fprintf(stderr, "Usage: runtime_loader LIBORT LIBMUJOCO LIBCONTROL\n");
    return 2;
  }
  void *ort = load(argv[1]);
  void *mujoco = load(argv[2]);
  void *control = load(argv[3]);
  if (!ort || !mujoco || !control) return 3;
  const OrtApiBase *(*ort_base)(void) = dlsym(ort, "OrtGetApiBase");
  int (*mj_version)(void) = dlsym(mujoco, "mj_version");
  uint32_t (*md_abi_version)(void) = dlsym(control, "md_abi_version");
  if (!ort_base || !mj_version || !md_abi_version) {
    fprintf(stderr, "One or more public runtime ABI exports are missing.\n");
    return 4;
  }
  const OrtApiBase *base = ort_base();
  const char *version = base->get_version();
  int ort_api_23 = base->get_api(23) != NULL;
  int mj = mj_version();
  uint32_t abi = md_abi_version();
  printf("{\"onnxruntime\":\"%s\",\"ort_api_23\":%s,\"mujoco\":%d,\"control_abi\":%u}\n",
         version, ort_api_23 ? "true" : "false", mj, abi);
  int good = strcmp(version, "1.23.2") == 0 && ort_api_23 && mj == 3015000 && abi == 1;
  dlclose(control);
  dlclose(mujoco);
  dlclose(ort);
  return good ? 0 : 5;
}
