

#include "macros_common.h"
#include <data_fifo.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(microphone);

extern struct data_fifo fifo_rx;

void init_fifo() {
#ifdef CONFIG_NRF5340_AUDIO
  int ret;
  if (!fifo_rx.initialized) {
    ret = data_fifo_init(&fifo_rx);
    ERR_CHK_MSG(ret, "Failed to set up rx FIFO");
  }
#endif
}

void empty_fifo() {
#ifdef CONFIG_NRF5340_AUDIO
  int ret;
  if (fifo_rx.initialized) {
    ret = data_fifo_empty(&fifo_rx);
    ERR_CHK_MSG(ret, "Failed to empty rx FIFO");
  }
#endif
}