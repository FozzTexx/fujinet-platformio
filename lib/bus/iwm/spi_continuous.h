#include <driver/spi_master.h>

#ifdef __cplusplus
extern "C" {
#endif

  void cspi_enable_continuous(spi_device_handle_t handle, spi_transaction_t *trans);
  int cspi_end_continuous(spi_device_handle_t handle);
  uint32_t cspi_get_lock(spi_device_handle_t handle, size_t *size);
  int cspi_get_count(spi_device_handle_t handle);
  int cspi_get_is_done(spi_device_handle_t handle);
  
#if 0
  size_t cspi_current_pos(spi_device_handle_t handle);
#endif

#ifdef __cplusplus
}
#endif
