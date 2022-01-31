#define RTAPI 1
#include <hal.h>      // HAL public API decls
#include <rtapi_app.h>  // EXPORT_SYMBOL(rtapi_app_main);
#include <stdlib.h>   // getenv(), setenv()
#include <string.h>   // strerror()


int rtapi_app_main(void)
{
  int comp_id = hal_init("hal_env_hack");
  char *ld_library_path = getenv("_LD_LIBRARY_PATH");
  if (ld_library_path == NULL) {
    hal_print_msg(RTAPI_MSG_ERR, "_LD_LIBRARY_PATH unset");
    hal_exit(comp_id);
    return -1;
  }
  int res = setenv("LD_LIBRARY_PATH", ld_library_path, 1);
  if (res) {
    hal_print_msg(RTAPI_MSG_ERR, "Failed to set LD_LIBRARY_PATH:  %s",
                  strerror(errno));
    hal_exit(comp_id);
    return -errno;
  }
  hal_print_msg(RTAPI_MSG_INFO, "LD_LIBRARY_PATH:  %s", ld_library_path);
  hal_ready(comp_id);
  return 0;
}

void rtapi_app_exit(void)
{
}
