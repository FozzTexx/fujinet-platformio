#include "spi_continuous.h"
#include <soc/lldesc.h>
#include <esp_log.h>
#include <esp_private/spi_common_internal.h>
#include <hal/spi_hal.h>
#include <freertos/queue.h>
#include <stdatomic.h>
#include <freertos/semphr.h>

// spi_device_t and spi_host_t reference each other
typedef struct spi_device_t spi_device_t;
//spi_device_handle_t is a pointer to spi_device_t

typedef struct {
  spi_transaction_t *trans;
  const uint32_t *buffer_to_send;
  uint32_t *buffer_to_rcv;
} spi_trans_priv_t;

typedef struct {
  int id;
  spi_device_t *device[DEV_NUM_MAX];
  intr_handle_t intr;
  spi_hal_context_t hal;
  spi_trans_priv_t cur_trans_buf;
  int cur_cs;
  const spi_bus_attr_t *bus_attr;
  spi_device_t *device_acquiring_lock;

  //debug information
  bool polling;   //in process of a polling, avoid of queue new transactions into ISR
} spi_host_t;

struct spi_device_t {
  int id;
  QueueHandle_t trans_queue;
  QueueHandle_t ret_queue;
  spi_device_interface_config_t cfg;
  spi_hal_dev_config_t hal_dev;
  spi_host_t *host;
  spi_bus_lock_dev_handle_t dev_lock;
};

struct spi_bus_lock_dev_t;
typedef struct spi_bus_lock_dev_t spi_bus_lock_dev_t;
typedef struct spi_bus_lock_t spi_bus_lock_t;

struct spi_bus_lock_t {
    /**
     * The core of the lock. These bits are status of the lock, which should be always available.
     * No intermediate status is allowed. This is realized by atomic operations, mainly
     * `atomic_fetch_and`, `atomic_fetch_or`, which atomically read the status, and bitwise write
     * status value ORed / ANDed with given masks.
     *
     * The request bits together pending bits represent the actual bg request state of one device.
     * Either one of them being active indicates the device has pending bg requests.
     *
     * Whenever a bit is written to the status, it means the a device on a task is trying to
     * acquire the lock. But this will succeed only when no LOCK or BG bits active.
     *
     * The acquiring processor is responsible to call the scheduler to pass its role to other tasks
     * or the BG, unless it clear the last bit in the status register.
     */
    //// Critical resources, they are only writable by acquiring processor, and stable only when read by the acquiring processor.
    atomic_uint_fast32_t    status;
    spi_bus_lock_dev_t* volatile acquiring_dev;   ///< The acquiring device
    bool                volatile acq_dev_bg_active;    ///< BG is the acquiring processor serving the acquiring device, used for the wait_bg to skip waiting quickly.
    bool                volatile in_isr;         ///< ISR is touching HW
    //// End of critical resources

    atomic_intptr_t     dev[DEV_NUM_MAX];     ///< Child locks.
    bg_ctrl_func_t      bg_enable;      ///< Function to enable background operations.
    bg_ctrl_func_t      bg_disable;     ///< Function to disable background operations
    void*               bg_arg;            ///< Argument for `bg_enable` and `bg_disable` functions.

    spi_bus_lock_dev_t* last_dev;       ///< Last used device, to decide whether to refresh all registers.
    int                 periph_cs_num;  ///< Number of the CS pins the HW has.

    //debug information
    int                 host_id;        ///< Host ID, for debug information printing
    uint32_t            new_req;        ///< Last int_req when `spi_bus_lock_bg_start` is called. Debug use.
};

struct spi_bus_lock_dev_t {
    SemaphoreHandle_t   semphr;     ///< Binray semaphore to notify the device it claimed the bus
    spi_bus_lock_t*     parent;     ///< Pointer to parent spi_bus_lock_t
    uint32_t            mask;       ///< Bitwise OR-ed mask of the REQ, PEND, LOCK bits of this device
};

void cspi_enable_continuous(spi_device_handle_t handle, spi_transaction_t *trans)
{
  spi_host_t *host = handle->host;
  spi_dev_t *hw = host->hal.hw;
  lldesc_t *first, *last;


  // Walk linked list and find the last one
  first = host->hal.dmadesc_rx;
  for (last = first; last->qe.stqe_next; last = last->qe.stqe_next)
    ;
  last->eof = 0;
  last->qe.stqe_next = first;

  // Circular/ring buffer mode
  hw->dma_conf.dma_continue = 1;
  return;
}

#define REQ_SHIFT       0
#define MAX_DEV_NUM     10
#define BIT1_MASK(high, low)   ((UINT32_MAX << (high)) ^ (UINT32_MAX << (low)))
#define BG_MASK             BIT1_MASK(REQ_SHIFT+MAX_DEV_NUM*2, REQ_SHIFT)

int cspi_end_continuous(spi_device_handle_t handle)
{
  spi_host_t *host = handle->host;
  spi_dev_t *hw = host->hal.hw;
  int ret;


  hw->dma_in_link.stop = 1;
  hw->dma_conf.dma_continue = 0;
  hw->cmd.usr = 0;

  spi_ll_set_int_stat(hw);

  {
    spi_bus_lock_handle_t lock = host->bus_attr->lock;
    spi_bus_lock_dev_handle_t desired_dev;
    BaseType_t do_yield;
    size_t size;
    int status;
    void* buffer;


    if (lock->acquiring_dev) {
      status = cspi_get_lock(handle, &size);
      if (status & (lock->acquiring_dev->mask & BG_MASK))
	return -status;
    }

    do {
      spi_bus_lock_bg_check_dev_acq(lock, &desired_dev);
      if (!desired_dev)
	break;
      // Move REQ bits to PEND bits
      spi_bus_lock_bg_check_dev_req(desired_dev);
      // Clear PEND bits
      spi_bus_lock_bg_clear_req(desired_dev);
      // Release core
      ret = spi_bus_lock_bg_exit(lock, 0, &do_yield);
    } while (!ret);

    // Flush out anything stuck in the queue
    while (xQueueReceive(handle->trans_queue, &buffer, 0) == pdTRUE)
      ;
  }

#if 0
  {
    spi_bus_lock_dev_t *lock = handle->dev_lock;


    if (lock->parent->acquiring_dev == lock)
      spi_device_release_bus(handle);
  }
#endif

  return hw->slave.trans_done;
  return ret;
}

uint32_t cspi_get_lock(spi_device_handle_t handle, size_t *size)
{
  spi_host_t *host = handle->host;
  spi_bus_lock_handle_t lock = host->bus_attr->lock;
  uint32_t status = atomic_load(&lock->status);


#if 0
  *size = sizeof(spi_bus_lock_t);
  return (uint8_t *) lock;
#endif

  *size = sizeof(status);
  return status & BG_MASK;
}

int cspi_get_count(spi_device_handle_t handle)
{
  spi_bus_lock_dev_t *dev_lock = handle->dev_lock;


  return uxSemaphoreGetCount(dev_lock->semphr);
}

int cspi_get_is_done(spi_device_handle_t handle)
{
  spi_host_t *host = handle->host;


  //return spi_hal_usr_is_done(&host->hal);
  return host->hal.hw->slave.trans_done;
}

#if 0
static const char *SPI_TAG = "cspi";

void cspi_continuous_link(lldesc_t *dmadesc, const void *data, int len,
			 int max_desc_size, bool isrx)
{
  int n = 0;
  while (len) {
    int dmachunklen = len;
    if (dmachunklen > max_desc_size) {
      dmachunklen = max_desc_size;
    }
    if (isrx) {
      //Receive needs DMA length rounded to next 32-bit boundary
      dmadesc[n].size = (dmachunklen + 3) & (~3);
      dmadesc[n].length = (dmachunklen + 3) & (~3);
    }
    else {
      dmadesc[n].size = dmachunklen;
      dmadesc[n].length = dmachunklen;
    }
    dmadesc[n].buf = (uint8_t *)data;
    dmadesc[n].eof = 0;
    dmadesc[n].sosf = 0;
    dmadesc[n].owner = 1;
    dmadesc[n].qe.stqe_next = &dmadesc[n + 1];
    len -= dmachunklen;
    data += dmachunklen;
    n++;
  }

  // This is the one thing that is different from lldesc.c/lldesc_setup_link_constrained()
  dmadesc[n - 1].qe.stqe_next = dmadesc; // Link back to first
}

void cspi_hal_prepare_data(spi_hal_context_t *hal, const spi_hal_dev_config_t *dev,
			  const spi_hal_trans_config_t *trans)
{
  spi_dev_t *hw = hal->hw;

  //Fill DMA descriptors
  if (trans->rcv_buffer) {
    if (!hal->dma_enabled) {
      //No need to setup anything; we'll copy the result out of the
      //work registers directly later.
    }
    else {
      cspi_continuous_link(hal->dmadesc_rx, trans->rcv_buffer, ((trans->rx_bitlen + 7) / 8),
			   LLDESC_MAX_NUM_PER_DESC, true);

      spi_dma_ll_rx_reset(hal->dma_in, hal->rx_dma_chan);
      spi_ll_dma_rx_fifo_reset(hal->hw);
      spi_ll_infifo_full_clr(hal->hw);
      spi_ll_dma_rx_enable(hal->hw, 1);
      spi_dma_ll_rx_start(hal->dma_in, hal->rx_dma_chan, hal->dmadesc_rx);
    }
  }
#if CONFIG_IDF_TARGET_ESP32
  else {
    //DMA temporary workaround: let RX DMA work somehow to avoid the
    //issue in ESP32 v0/v1 silicon
    if (hal->dma_enabled && !dev->half_duplex) {
      spi_ll_dma_rx_enable(hal->hw, 1);
      spi_dma_ll_rx_start(hal->dma_in, hal->rx_dma_chan, 0);
    }
  }
#endif

  if (trans->send_buffer) {
    if (!hal->dma_enabled) {
      //Need to copy data to registers manually
      spi_ll_write_buffer(hw, trans->send_buffer, trans->tx_bitlen);
    }
    else {
      lldesc_setup_link(hal->dmadesc_tx, trans->send_buffer,
			(trans->tx_bitlen + 7) / 8, false);

      spi_dma_ll_tx_reset(hal->dma_out, hal->tx_dma_chan);
      spi_ll_dma_tx_fifo_reset(hal->hw);
      spi_ll_outfifo_empty_clr(hal->hw);
      spi_ll_dma_tx_enable(hal->hw, 1);
      spi_dma_ll_tx_start(hal->dma_out, hal->tx_dma_chan, hal->dmadesc_tx);
    }
  }

  //in ESP32 these registers should be configured after the DMA is set
  if ((!dev->half_duplex && trans->rcv_buffer) || trans->send_buffer) {
    spi_ll_enable_mosi(hw, 1);
  }
  else {
    spi_ll_enable_mosi(hw, 0);
  }
  spi_ll_enable_miso(hw, (trans->rcv_buffer) ? 1 : 0);
}

static SPI_MASTER_ISR_ATTR void spi_setup_device(spi_device_t *dev)
{
  spi_bus_lock_dev_handle_t dev_lock = dev->dev_lock;
  spi_hal_context_t *hal = &dev->host->hal;
  spi_hal_dev_config_t *hal_dev = &(dev->hal_dev);

  if (spi_bus_lock_touch(dev_lock)) {
    /* Configuration has not been applied yet. */
    spi_hal_setup_device(hal, hal_dev);
  }
}

void SPI_MASTER_ISR_ATTR cspi_new_trans(spi_device_handle_t dev, spi_trans_priv_t *trans_buf)
{
  spi_transaction_t *trans = trans_buf->trans;
  spi_host_t *host = dev->host;
  spi_hal_context_t *hal = &(host->hal);
  spi_hal_dev_config_t *hal_dev = &(dev->hal_dev);

  host->cur_cs = dev->id;

  //Reconfigure according to device settings, the function only has
  //effect when the dev_id is changed.
  spi_setup_device(dev);

  //set the transaction specific configuration each time before a transaction setup
  spi_hal_trans_config_t hal_trans = {};
  hal_trans.tx_bitlen = trans->length;
  hal_trans.rx_bitlen = trans->rxlength;
  hal_trans.rcv_buffer = (uint8_t*)host->cur_trans_buf.buffer_to_rcv;
  hal_trans.send_buffer = (uint8_t*)host->cur_trans_buf.buffer_to_send;
  hal_trans.cmd = trans->cmd;
  hal_trans.addr = trans->addr;
  hal_trans.cs_keep_active = (trans->flags & SPI_TRANS_CS_KEEP_ACTIVE) ? 1 : 0;

  //Set up OIO/QIO/DIO if needed
  hal_trans.line_mode.data_lines = (trans->flags & SPI_TRANS_MODE_DIO) ? 2 :
    (trans->flags & SPI_TRANS_MODE_QIO) ? 4 : 1;
#if SOC_SPI_SUPPORT_OCT
  if (trans->flags & SPI_TRANS_MODE_OCT) {
    hal_trans.line_mode.data_lines = 8;
  }
#endif
  hal_trans.line_mode.addr_lines = (trans->flags & SPI_TRANS_MULTILINE_ADDR)
    ? hal_trans.line_mode.data_lines : 1;
  hal_trans.line_mode.cmd_lines = (trans->flags & SPI_TRANS_MULTILINE_CMD)
    ? hal_trans.line_mode.data_lines : 1;

  if (trans->flags & SPI_TRANS_VARIABLE_CMD) {
    hal_trans.cmd_bits = ((spi_transaction_ext_t *)trans)->command_bits;
  }
  else {
    hal_trans.cmd_bits = dev->cfg.command_bits;
  }
  if (trans->flags & SPI_TRANS_VARIABLE_ADDR) {
    hal_trans.addr_bits = ((spi_transaction_ext_t *)trans)->address_bits;
  }
  else {
    hal_trans.addr_bits = dev->cfg.address_bits;
  }
  if (trans->flags & SPI_TRANS_VARIABLE_DUMMY) {
    hal_trans.dummy_bits = ((spi_transaction_ext_t *)trans)->dummy_bits;
  }
  else {
    hal_trans.dummy_bits = dev->cfg.dummy_bits;
  }

  spi_hal_setup_trans(hal, hal_dev, &hal_trans);
  spi_hal_prepare_data(hal, hal_dev, &hal_trans);

  //Call pre-transmission callback, if any
  if (dev->cfg.pre_cb) dev->cfg.pre_cb(trans);
  //Kick off transfer
  hal->hw->dma_conf.dma_continue = 1;
  spi_hal_user_start(hal);
}

static SPI_MASTER_ISR_ATTR void uninstall_priv_desc(spi_trans_priv_t* trans_buf)
{
  spi_transaction_t *trans_desc = trans_buf->trans;
  if ((void *)trans_buf->buffer_to_send != &trans_desc->tx_data[0] &&
      trans_buf->buffer_to_send != trans_desc->tx_buffer) {
    free((void *)trans_buf->buffer_to_send); //force free, ignore const
  }
  // copy data from temporary DMA-capable buffer back to IRAM buffer and free the temporary one.
  if (trans_buf->buffer_to_rcv &&
      (void *)trans_buf->buffer_to_rcv != &trans_desc->rx_data[0] &&
      trans_buf->buffer_to_rcv != trans_desc->rx_buffer) { // NOLINT(clang-analyzer-unix.Malloc)
    if (trans_desc->flags & SPI_TRANS_USE_RXDATA) {
      memcpy((uint8_t *) & trans_desc->rx_data[0], trans_buf->buffer_to_rcv, (trans_desc->rxlength + 7) / 8);
    }
    else {
      memcpy(trans_desc->rx_buffer, trans_buf->buffer_to_rcv, (trans_desc->rxlength + 7) / 8);
    }
    free(trans_buf->buffer_to_rcv);
  }
}

static SPI_MASTER_ISR_ATTR esp_err_t setup_priv_desc(spi_transaction_t *trans_desc,
						     spi_trans_priv_t* new_desc, bool isdma)
{
  *new_desc = (spi_trans_priv_t) {
    .trans = trans_desc,
  };

  // rx memory assign
  uint32_t* rcv_ptr;
  if ( trans_desc->flags & SPI_TRANS_USE_RXDATA ) {
    rcv_ptr = (uint32_t *)&trans_desc->rx_data[0];
  }
  else {
    //if not use RXDATA neither rx_buffer, buffer_to_rcv assigned to NULL
    rcv_ptr = trans_desc->rx_buffer;
  }
  if (rcv_ptr && isdma && (!esp_ptr_dma_capable(rcv_ptr) || ((int)rcv_ptr % 4 != 0))) {
    //if rxbuf in the desc not DMA-capable, malloc a new one. The rx
    //buffer need to be length of multiples of 32 bits to avoid heap
    //corruption.
    ESP_LOGD(SPI_TAG, "Allocate RX buffer for DMA" );
    rcv_ptr = heap_caps_malloc((trans_desc->rxlength + 31) / 8, MALLOC_CAP_DMA);
    if (rcv_ptr == NULL)
      goto clean_up;
  }
  new_desc->buffer_to_rcv = rcv_ptr;

  // tx memory assign
  const uint32_t *send_ptr;
  if ( trans_desc->flags & SPI_TRANS_USE_TXDATA ) {
    send_ptr = (uint32_t *)&trans_desc->tx_data[0];
  }
  else {
    //if not use TXDATA neither tx_buffer, tx data assigned to NULL
    send_ptr = trans_desc->tx_buffer ;
  }
  if (send_ptr && isdma && !esp_ptr_dma_capable( send_ptr )) {
    //if txbuf in the desc not DMA-capable, malloc a new one
    ESP_LOGD(SPI_TAG, "Allocate TX buffer for DMA" );
    uint32_t *temp = heap_caps_malloc((trans_desc->length + 7) / 8, MALLOC_CAP_DMA);
    if (temp == NULL)
      goto clean_up;

    memcpy( temp, send_ptr, (trans_desc->length + 7) / 8 );
    send_ptr = temp;
  }
  new_desc->buffer_to_send = send_ptr;

  return ESP_OK;

 clean_up:
  uninstall_priv_desc(new_desc);
  return ESP_ERR_NO_MEM;
}

esp_err_t cspi_begin_trans(spi_device_handle_t handle, spi_transaction_t *trans_desc)
{
  esp_err_t ret;
  spi_host_t *host = handle->host;


  if (!host->bus_attr->dma_enabled)
    return ESP_ERR_INVALID_ARG;
  ret = spi_device_acquire_bus(handle, portMAX_DELAY);
  if (ret)
    return ret;
  ret = setup_priv_desc(trans_desc, &host->cur_trans_buf, 1);
  if (ret)
    return ret;
  cspi_new_trans(handle, &host->cur_trans_buf);
  return ESP_OK;
}

void cspi_end_trans(spi_device_handle_t handle)
{
  spi_dev_t *hw = handle->host->hal.hw;


  if (hw->dma_conf.dma_continue) {
    hw->dma_conf.dma_continue = 0;
    hw->cmd.usr = 0;
    spi_device_release_bus(handle);
  }

  return;
}

#if 0
// Get current SPI position
size_t cspi_current_pos(spi_device_handle_t handle)
{
  spi_host_t *host = handle->host;
  //spi_dev_t *hw = host->hal.hw;
  spi_dev_t *hw = &SPI3;
  uint8_t *ptr1, *ptr2, *ptr3;


  // Access the current descriptor being used by DMA
  ptr1 = (typeof(ptr1)) hw->dma_inlink_dscr_bf1;
  lldesc_t *current_desc = (lldesc_t *) hw->dma_inlink_dscr;
  ptr2 = (typeof(ptr2)) current_desc->buf;
  ptr3 = (typeof(ptr3)) host->cur_trans_buf.buffer_to_rcv;

  return ptr2 - ptr3;
}
#endif
#endif
