#ifndef XVF3800_EQ_H
#define XVF3800_EQ_H

#include <stdint.h>
#include <stdbool.h>

/*
 * XVF3800 PP Equalizer control via I2C
 *
 * I2C slave address : 0x2C
 * Protocol          : device_control (RESID | CMD | LEN | payload)
 * PP Resource ID    : 0x11
 *
 * Usage:
 *   xvf3800_eq_enable();              // VOICE 커브 적용
 *   xvf3800_eq_disable();             // flat / bypass (전 밴드 1.0)
 *   xvf3800_eq_toggle();              // 현재 상태 반전, true=ON 반환
 *   xvf3800_eq_is_enabled();          // 현재 상태 확인
 */

#define XVF3800_I2C_ADDR    0x2C

/* Return codes */
#define XVF3800_EQ_OK        0
#define XVF3800_EQ_ERR_I2C  -1
#define XVF3800_EQ_ERR_NAK  -2

#define XVF3800_EQ_NUM_BANDS 40

/*
 * Preset indices for xvf3800_eq_set_preset()
 */
typedef enum {
    XVF3800_PRESET_FLAT     = 0,   /* bypass - all 1.0 */
    XVF3800_PRESET_VOICE    = 1,   /* low -5dB / 2k+5 / 4k+10 / 8k+15 */
    XVF3800_PRESET_COUNT
} xvf3800_eq_preset_t;

/* Write 40 linear-amplitude float coefficients to the XVF3800 PP EQ. */
int xvf3800_eq_write(const float coeff[XVF3800_EQ_NUM_BANDS]);

/* Apply the built-in VOICE preset EQ curve */
int xvf3800_eq_enable(void);

/* Set all bands to 1.0 (flat) */
int xvf3800_eq_disable(void);

/* Apply one of the named presets */
int xvf3800_eq_set_preset(xvf3800_eq_preset_t preset);

/* Toggle between VOICE and FLAT. Returns true if EQ is now ON. */
bool xvf3800_eq_toggle(void);

/* Returns current EQ state: true = VOICE curve active, false = flat */
bool xvf3800_eq_is_enabled(void);

#endif /* XVF3800_EQ_H */
