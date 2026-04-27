#include <stddef.h>
#include <stdint.h>

size_t accel_get_latest_idx(void);
size_t accel_copy_new_binary(size_t last_idx, uint8_t *buf, size_t buf_size);
void accel_reader_task(void *pvParameters);

/* Runtime configuration.
 * dlpf_cfg: 0–6  (MPU6050 DLPF_CFG; 0 = filter bypassed, 260 Hz accel BW)
 * afs_sel:  0–3  (MPU6050 AFS_SEL; 0=±2G, 1=±4G, 2=±8G, 3=±16G)
 * Sample rate is derived from dlpf_cfg via a fixed lookup table. */
void     accel_set_config(uint8_t dlpf_cfg, uint8_t afs_sel);
uint32_t accel_get_sample_rate(void);   /* Hz */
float    accel_get_scale_factor(void);  /* mm/s² per LSB */
uint8_t  accel_get_dlpf_cfg(void);
uint8_t  accel_get_afs_sel(void);
