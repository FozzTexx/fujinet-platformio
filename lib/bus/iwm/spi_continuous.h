#include <driver/spi_master.h>

#ifdef __cplusplus
extern "C" {
#endif

  void cspi_enable_continuous(spi_device_handle_t handle, spi_transaction_t *trans);
  void *cspi_lldesc(spi_device_handle_t handle);
  int cspi_end_continuous(spi_device_handle_t handle);
  uint8_t *cspi_get_lock(spi_device_handle_t handle, size_t *size);
  uint32_t cspi_get_bg_status(spi_device_handle_t handle);
  int cspi_get_count(spi_device_handle_t handle);
  int cspi_get_is_done(spi_device_handle_t handle);
  void cspi_set_is_done(spi_device_handle_t handle, int val);
  uint32_t cspi_running_cmd(spi_device_handle_t handle);
  
#if 1
  size_t cspi_current_pos(spi_device_handle_t handle);
#endif

#ifdef __cplusplus
}
#endif
