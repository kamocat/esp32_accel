#include <stddef.h>
#include <stdint.h>

size_t accel_get_latest_idx(void);
size_t accel_copy_new_binary(size_t last_idx, uint8_t *buf, size_t buf_size);
void accel_reader_task(void *pvParameters);
