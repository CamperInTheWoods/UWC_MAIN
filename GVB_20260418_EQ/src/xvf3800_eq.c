/*
 * xvf3800_eq.c  -  XVF3800 PP Equalizer control from EFM32HG310 via I2C
 *
 * Protocol: device_control over I2C
 *   Write packet: [RESID][CMD][PAYLOAD_LEN][payload...]
 *   After each write, XVF3800 returns 1 status byte (0x00 = OK)
 *
 * EQ update sequence (40 bands, 15 per chunk):
 *   1. Set offset=0    [11 62 04 00 00 00 00]
 *   2. Start trigger   [11 61 04 01 00 00 00]
 *   3. bands 0-14      [11 63 3C <60 bytes>]
 *   4. Set offset=15   [11 62 04 0F 00 00 00]
 *   5. bands 15-29     [11 63 3C <60 bytes>]
 *   6. Set offset=30   [11 62 04 1E 00 00 00]
 *   7. bands 30-39     [11 63 28 <40 bytes>]
 */

#include "xvf3800_eq.h"
#include <string.h>
#include <stdbool.h>

/* ------------------------------------------------------------------ */
/* HAL - main.c의 I2C_WriteReadBytes 직접 사용 (static 제거됨)        */
/* ------------------------------------------------------------------ */
extern bool I2C_WriteReadBytes(uint8_t addr7,
                                const uint8_t *wdata, uint8_t wlen,
                                uint8_t       *rdata, uint8_t rlen);

static int xvf3800_i2c_write_read(uint8_t addr,
                                   const uint8_t *tx_buf, uint16_t tx_len,
                                   uint8_t       *rx_buf, uint16_t rx_len)
{
    return I2C_WriteReadBytes(addr, tx_buf, (uint8_t)tx_len,
                              rx_buf, (uint8_t)rx_len) ? 0 : -1;
}

/* ------------------------------------------------------------------ */
/* device_control 상수                                                 */
/* ------------------------------------------------------------------ */
#define PP_RESID                            0x11u
#define CMD_EQUALIZATION_START              0x61u
#define CMD_EQUALIZATION_COEFF_START_OFFSET 0x62u
#define CMD_PP_EQUALIZATION                 0x63u

#define CHUNKS_OF   15u
#define FLOAT_BYTES  4u

/* ------------------------------------------------------------------ */
/* EQ 계수 테이블                                                      */
/* ------------------------------------------------------------------ */

/* Flat / bypass */
static const float eq_flat[XVF3800_EQ_NUM_BANDS] = {
    1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f,
    1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f,
    1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f,
    1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f,
    1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f
};

/*
 * VOICE 프리셋 - pchip 보간
 * 제어점: 0Hz 0dB, 100Hz -5dB, 500Hz -5dB, 1kHz -2dB,
 *         2kHz +5dB, 4kHz +10dB, 8kHz +15dB
 */
static const float eq_voice[XVF3800_EQ_NUM_BANDS] = {
    1.000000f,  /* band  0 ~    0 Hz   0.00 dB */
    0.562341f,  /* band  1 ~  192 Hz  -5.00 dB */
    0.562341f,  /* band  2 ~  385 Hz  -5.00 dB */
    0.570625f,  /* band  3 ~  577 Hz  -4.87 dB */
    0.648938f,  /* band  4 ~  769 Hz  -3.76 dB */
    0.770860f,  /* band  5 ~  962 Hz  -2.26 dB */
    0.898865f,  /* band  6 ~ 1154 Hz  -0.93 dB */
    1.068936f,  /* band  7 ~ 1346 Hz  +0.58 dB */
    1.276073f,  /* band  8 ~ 1538 Hz  +2.12 dB */
    1.501631f,  /* band  9 ~ 1731 Hz  +3.53 dB */
    1.710451f,  /* band 10 ~ 1923 Hz  +4.66 dB */
    1.869732f,  /* band 11 ~ 2115 Hz  +5.44 dB */
    2.019765f,  /* band 12 ~ 2308 Hz  +6.11 dB */
    2.165893f,  /* band 13 ~ 2500 Hz  +6.71 dB */
    2.307377f,  /* band 14 ~ 2692 Hz  +7.26 dB */
    2.443859f,  /* band 15 ~ 2885 Hz  +7.76 dB */
    2.575372f,  /* band 16 ~ 3077 Hz  +8.22 dB */
    2.702345f,  /* band 17 ~ 3269 Hz  +8.63 dB */
    2.825588f,  /* band 18 ~ 3462 Hz  +9.02 dB */
    2.946287f,  /* band 19 ~ 3654 Hz  +9.39 dB */
    3.065983f,  /* band 20 ~ 3846 Hz  +9.73 dB */
    3.186588f,  /* band 21 ~ 4038 Hz +10.07 dB */
    3.310332f,  /* band 22 ~ 4231 Hz +10.40 dB */
    3.437496f,  /* band 23 ~ 4423 Hz +10.72 dB */
    3.567720f,  /* band 24 ~ 4615 Hz +11.05 dB */
    3.700586f,  /* band 25 ~ 4808 Hz +11.37 dB */
    3.835607f,  /* band 26 ~ 5000 Hz +11.68 dB */
    3.972234f,  /* band 27 ~ 5192 Hz +11.98 dB */
    4.109845f,  /* band 28 ~ 5385 Hz +12.28 dB */
    4.247751f,  /* band 29 ~ 5577 Hz +12.56 dB */
    4.385192f,  /* band 30 ~ 5769 Hz +12.84 dB */
    4.521340f,  /* band 31 ~ 5962 Hz +13.11 dB */
    4.655299f,  /* band 32 ~ 6154 Hz +13.36 dB */
    4.786112f,  /* band 33 ~ 6346 Hz +13.60 dB */
    4.912766f,  /* band 34 ~ 6538 Hz +13.83 dB */
    5.034197f,  /* band 35 ~ 6731 Hz +14.04 dB */
    5.149300f,  /* band 36 ~ 6923 Hz +14.23 dB */
    5.256940f,  /* band 37 ~ 7115 Hz +14.41 dB */
    5.355964f,  /* band 38 ~ 7308 Hz +14.58 dB */
    5.445216f   /* band 39 ~ 7500 Hz +14.72 dB */
};

static const float * const eq_presets[XVF3800_PRESET_COUNT] = {
    [XVF3800_PRESET_FLAT]  = eq_flat,
    [XVF3800_PRESET_VOICE] = eq_voice,
};

/* ------------------------------------------------------------------ */
/* 내부 헬퍼                                                           */
/* ------------------------------------------------------------------ */
static int dc_write(uint8_t cmd, const uint8_t *payload, uint8_t plen)
{
    uint8_t tx[3 + 64];
    uint8_t status = 0xFF;
    int     rc;

    tx[0] = PP_RESID;
    tx[1] = cmd;
    tx[2] = plen;
    memcpy(&tx[3], payload, plen);

    rc = xvf3800_i2c_write_read(XVF3800_I2C_ADDR,
                                 tx, (uint16_t)(3u + plen),
                                 &status, 1u);
    if (rc != 0)        return XVF3800_EQ_ERR_I2C;
    if (status != 0x00) return XVF3800_EQ_ERR_NAK;
    return XVF3800_EQ_OK;
}

static int dc_write_u32(uint8_t cmd, uint32_t value)
{
    uint8_t buf[4];
    buf[0] = (uint8_t)(value);
    buf[1] = (uint8_t)(value >>  8);
    buf[2] = (uint8_t)(value >> 16);
    buf[3] = (uint8_t)(value >> 24);
    return dc_write(cmd, buf, 4u);
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */
int xvf3800_eq_write(const float coeff[XVF3800_EQ_NUM_BANDS])
{
    int      rc;
    uint32_t offset;
    uint32_t band;
    uint32_t chunk_size;
    uint8_t  payload[CHUNKS_OF * FLOAT_BYTES];

    rc = dc_write_u32(CMD_EQUALIZATION_COEFF_START_OFFSET, 0u);
    if (rc != XVF3800_EQ_OK) return rc;

    rc = dc_write_u32(CMD_EQUALIZATION_START, 1u);
    if (rc != XVF3800_EQ_OK) return rc;

    for (offset = 0; offset < XVF3800_EQ_NUM_BANDS; offset += CHUNKS_OF)
    {
        chunk_size = XVF3800_EQ_NUM_BANDS - offset;
        if (chunk_size > CHUNKS_OF) chunk_size = CHUNKS_OF;

        if (offset > 0u)
        {
            rc = dc_write_u32(CMD_EQUALIZATION_COEFF_START_OFFSET, offset);
            if (rc != XVF3800_EQ_OK) return rc;
        }

        for (band = 0; band < chunk_size; band++)
        {
            float f = coeff[offset + band];
            memcpy(&payload[band * FLOAT_BYTES], &f, FLOAT_BYTES);
        }

        rc = dc_write(CMD_PP_EQUALIZATION,
                      payload,
                      (uint8_t)(chunk_size * FLOAT_BYTES));
        if (rc != XVF3800_EQ_OK) return rc;
    }

    return XVF3800_EQ_OK;
}

int xvf3800_eq_enable(void)
{
    return xvf3800_eq_write(eq_voice);
}

int xvf3800_eq_disable(void)
{
    return xvf3800_eq_write(eq_flat);
}

int xvf3800_eq_set_preset(xvf3800_eq_preset_t preset)
{
    if ((unsigned)preset >= XVF3800_PRESET_COUNT) return XVF3800_EQ_ERR_NAK;
    return xvf3800_eq_write(eq_presets[preset]);
}

/* ------------------------------------------------------------------ */
/* Toggle / state                                                      */
/* ------------------------------------------------------------------ */
static bool s_eq_enabled = false;

bool xvf3800_eq_toggle(void)
{
    int rc;
    if (s_eq_enabled) {
        rc = xvf3800_eq_disable();
        if (rc == XVF3800_EQ_OK) s_eq_enabled = false;
    } else {
        rc = xvf3800_eq_enable();
        if (rc == XVF3800_EQ_OK) s_eq_enabled = true;
    }
    return s_eq_enabled;
}

bool xvf3800_eq_is_enabled(void)
{
    return s_eq_enabled;
}
