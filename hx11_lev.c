/* hx11_lev.c - HX11 four-magnet levitation pod firmware, STM32H7 Nucleo-144. */

#include "main.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <math.h>

/* ############################ CONFIGURATION ############################
 * FILL = unknown, value shown is only a starting guess.
 * TUNE = working value, adjust on hardware.
 * Per-magnet arrays are ordered FL, FR, RL, RR.
 * Body frame: x forward, y right, z down, origin at the CoM.
 * ##################################################################### */

#define NUM_COILS     4
#define NUM_SUPPLIES  3   /* PSU A, B, C */

/* Which PSU feeds each coil (0 = A, 1 = B): A = FL+RR, B = FR+RL. */
static const uint8_t COIL_SUPPLY[NUM_COILS] = { 0, 1, 1, 0 };

/* ---- Loop rates / serial ---- */
#define PWM_HZ        20000u   /* TUNE: G2 PWM frequency [Hz], max 100 kHz */
#define FAST_DIV      1u       /* PWM periods per current-loop tick */
#define OUTER_DIV     20u      /* current-loop ticks per outer tick (20 -> 1 kHz) */
#define TELEM_DIV     5u       /* outer ticks per telemetry line (5 -> 200 Hz) */
#define UART_BAUD     921600u  /* drop to 460800 if the ST-LINK garbles output */

/* ---- Pins (FILL: check against your Nucleo-144 manual and H7 datasheet) ----
 * Peripherals are fixed: TIM8 CH1-4, ADC1, SPI1, USART3.
 * PA2 and PC1 go to Ethernet on the Nucleo: open those solder bridges or move them. */
typedef struct { GPIO_TypeDef *port; uint16_t pin; uint32_t af; } pin_t;
typedef struct { GPIO_TypeDef *port; uint16_t pin; uint32_t ch; } adc_in_t;

static const pin_t PWM_PIN[NUM_COILS] = {
    { GPIOC, GPIO_PIN_6, GPIO_AF3_TIM8 },
    { GPIOC, GPIO_PIN_7, GPIO_AF3_TIM8 },
    { GPIOC, GPIO_PIN_8, GPIO_AF3_TIM8 },
    { GPIOC, GPIO_PIN_9, GPIO_AF3_TIM8 },
};
static const uint32_t PWM_CH[NUM_COILS] = { TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4 };
/* Coils on the same PSU must be opposite (180 deg) so their pulses don't overlap. */
static const bool PWM_PHASE_180[NUM_COILS] = { false, false, true, true };

static const pin_t DIR_PIN[NUM_COILS] = {
    { GPIOG, GPIO_PIN_2, 0 },
    { GPIOG, GPIO_PIN_3, 0 },
    { GPIOD, GPIO_PIN_0, 0 },
    { GPIOD, GPIO_PIN_1, 0 },
};
/* FILL: DIR level (1 = high) that builds lift current. Verify per coil with BENCH. */
static const bool DIR_FWD[NUM_COILS] = { 1, 1, 1, 1 };

/* ADC1 inputs in this exact order: 4 currents, 4 gaps, 3 bus dividers.
 * FILL: verify each ADC1_INPx number in your H7 datasheet. */
static const adc_in_t ADC_IN[11] = {
    { GPIOA, GPIO_PIN_3, ADC_CHANNEL_15 },  /* INA240 FL */
    { GPIOC, GPIO_PIN_0, ADC_CHANNEL_10 },  /* INA240 FR */
    { GPIOC, GPIO_PIN_3, ADC_CHANNEL_13 },  /* INA240 RL */
    { GPIOA, GPIO_PIN_4, ADC_CHANNEL_18 },  /* INA240 RR */
    { GPIOA, GPIO_PIN_5, ADC_CHANNEL_19 },  /* gap FL */
    { GPIOA, GPIO_PIN_6, ADC_CHANNEL_3  },  /* gap FR */
    { GPIOB, GPIO_PIN_1, ADC_CHANNEL_5  },  /* gap RL */
    { GPIOC, GPIO_PIN_2, ADC_CHANNEL_12 },  /* gap RR */
    { GPIOA, GPIO_PIN_0, ADC_CHANNEL_16 },  /* bus A */
    { GPIOC, GPIO_PIN_1, ADC_CHANNEL_11 },  /* bus B */
    { GPIOA, GPIO_PIN_2, ADC_CHANNEL_14 },  /* bus C */
};

static const pin_t SPI_SCK  = { GPIOB, GPIO_PIN_3, GPIO_AF5_SPI1 };
static const pin_t SPI_MISO = { GPIOB, GPIO_PIN_4, GPIO_AF5_SPI1 };
static const pin_t SPI_MOSI = { GPIOB, GPIO_PIN_5, GPIO_AF5_SPI1 };
static const pin_t SPI_CS   = { GPIOD, GPIO_PIN_14, 0 };
#define SPI_MAX_HZ    8000000u   /* ISM330DHCX max 10 MHz */
static const pin_t UART_TX  = { GPIOD, GPIO_PIN_8, GPIO_AF7_USART3 };  /* ST-LINK VCP */
static const pin_t UART_RX  = { GPIOD, GPIO_PIN_9, GPIO_AF7_USART3 };

/* If your linker script puts .bss in DTCM, place the ADC DMA buffer elsewhere, e.g.
 * __attribute__((section(".RAM_D2"))) */
#define DMA_BUF_SECTION

/* ---- Mass properties and geometry ---- */
#define POD_MASS_KG    40.0f    /* FILL: weigh the assembled pod [kg] */
#define GRAVITY_MPS2   9.81f
#define J_ROLL_KGM2    0.50f    /* FILL: inertia about body x through CoM [kg m^2], CAD */
#define J_PITCH_KGM2   1.50f    /* FILL: inertia about body y through CoM [kg m^2], CAD */
#define J_RP_KGM2      0.00f    /* FILL: x-y tensor entry = -integral(xy dm). Negate if CAD gives +integral. */

/* FILL: magnet force centres relative to CoM [m] */
static const float MAG_X_M[NUM_COILS]  = { +0.20f, +0.20f, -0.20f, -0.20f };
static const float MAG_Y_M[NUM_COILS]  = { -0.06f, +0.06f, -0.06f, +0.06f };
/* FILL: gap-sensor measuring spots relative to CoM [m] */
static const float SENS_X_M[NUM_COILS] = { +0.27f, +0.27f, -0.27f, -0.27f };
static const float SENS_Y_M[NUM_COILS] = { -0.06f, +0.06f, -0.06f, +0.06f };

/* ---- Magnet model: F = K0(z) * (Ieff/z)^2,  Ieff = I / (1 + I/I_SAT) ---- */
#define K_TAB_N 8u
static const float K_TAB_Z_MM[K_TAB_N] = { 3.96f, 4.92f, 5.93f, 6.95f, 7.99f, 8.98f, 9.97f, 11.0f };
static const float K_TAB_K[K_TAB_N]    = { 0.000488f, 0.000465f, 0.000442f, 0.000454f,
                                           0.000424f, 0.000433f, 0.000437f, 0.000429f };
#define K_TAB_CAL_CURRENT_A  2.0f   /* FILL: coil current used when K_TAB was measured [A] */
#define I_SAT_A              3.6f   /* FILL/TUNE: saturation current [A]. Refit from a force-vs-current
                                       sweep or hover currents. 1e6 disables saturation. */
static const float K_SCALE[NUM_COILS] = { 1.0f, 1.0f, 1.0f, 1.0f };  /* TUNE: per-magnet K trim from hover */

static const float R_COIL_OHM[NUM_COILS] = { 2.857f, 2.857f, 2.857f, 2.857f };  /* FILL: measure hot, incl. wiring [ohm] */
#define COIL_L_H             0.050f /* FILL: from current step response, tau = L/R [H] */

/* ---- Sensor scaling ---- */
#define ADC_VREF_V           3.3f
#define INA240_GAIN          20.0f  /* INA240A1 */
#define R_SHUNT_OHM          0.010f /* FILL: installed shunt [ohm] */
static const float CUR_SIGN[NUM_COILS] = { +1.0f, +1.0f, +1.0f, +1.0f };  /* FILL: so lift current reads positive */

#define GAP_BURDEN_OHM       150.0f /* FILL: burden resistor [ohm] */
#define GAP_MM_AT_4MA        0.0f   /* FILL: sensor's configured distance at 4 mA [mm] */
#define GAP_MM_AT_20MA       16.0f  /* FILL: distance at 20 mA [mm] */
/* FILL: per-sensor fit to the magnet-face gap, gap = A*g^2 + B*g + C (g = raw reading, mm).
 * Fit against gauge blocks; C absorbs the sensor-to-magnet-face offset. */
static const float GAP_CAL_A[NUM_COILS] = { 0.0f, 0.0f, 0.0f, 0.0f };
static const float GAP_CAL_B[NUM_COILS] = { 1.0f, 1.0f, 1.0f, 1.0f };
static const float GAP_CAL_C[NUM_COILS] = { 0.0f, 0.0f, 0.0f, 0.0f };
#define GAP_LPF_HZ           300.0f /* TUNE: gap low-pass [Hz], 0 = off */
#define HEAVE_RATE_LPF_HZ    160.0f /* TUNE: low-pass on dh/dt [Hz] */

static const float BUS_DIV_RATIO[NUM_SUPPLIES] = { 11.0f, 11.0f, 11.0f };  /* FILL: (Rtop+Rbot)/Rbot */
#define BUS_LPF_HZ           2000.0f

/* ISM330DHCX gyro */
#define GYRO_CTRL2_FS_BITS   (0x1u << 2)  /* +/-500 dps */
#define GYRO_DPS_PER_LSB     0.0175f      /* matches +/-500 dps */
#define GYRO_LPF1_FTYPE      0x0u         /* TUNE: LPF1 bandwidth code (datasheet) */
#define GYRO_ROLL_AXIS       0            /* FILL: IMU axis (0=X,1=Y,2=Z) along body x */
#define GYRO_ROLL_SIGN       +1.0f        /* FILL: right side down reads positive */
#define GYRO_PITCH_AXIS      1            /* FILL: IMU axis along body y */
#define GYRO_PITCH_SIGN      +1.0f        /* FILL: nose up reads positive */

#define CAL_SETTLE_TICKS     200u   /* outer ticks before averaging offsets / gyro bias */
#define CAL_AVG_TICKS        1000u  /* outer ticks averaged */

/* ---- Gains ---- */
/* Current PI: Kp = 2*pi*BW*L, Ki = 2*pi*BW*R, plus R*Iref feedforward. */
#define CUR_BW_HZ            300.0f /* TUNE */
#define CUR_INT_LIMIT_V      8.0f

/* Outer loops output accelerations, so gains don't depend on mass/inertia. */
#define HEAVE_KP             600.0f   /* TUNE [1/s^2] */
#define HEAVE_KD             60.0f    /* TUNE [1/s]   */
#define HEAVE_KI             1500.0f  /* TUNE [1/s^3] */
#define HEAVE_I_LIMIT        2.0f     /* [m/s^2] */
#define ROLL_KP              600.0f   /* TUNE */
#define ROLL_KD              50.0f    /* TUNE */
#define ROLL_KI              1000.0f  /* TUNE */
#define ROLL_I_LIMIT         20.0f    /* [rad/s^2] */
#define PITCH_KP             600.0f   /* TUNE */
#define PITCH_KD             50.0f    /* TUNE */
#define PITCH_KI             1000.0f  /* TUNE */
#define PITCH_I_LIMIT        20.0f    /* [rad/s^2] */

/* ---- Limits ---- */
#define I_MAX_A              6.0f   /* TUNE: coil current command clamp [A] */
#define DUTY_MAX             0.98f
#define BRAKE_FLOOR_A        0.2f   /* no reverse drive below this current [A] */

/* ---- Setpoints and ramps ---- */
#define HOVER_GAP_MM         7.0f   /* TUNE: default hover gap */
#define SP_MIN_MM            4.0f
#define SP_MAX_MM            11.0f
#define SP_RATE_MM_S         2.0f
#define LIFT_FORCE_RAMP_S    0.5f
#define LIFT_SP_RATE_MM_S    2.0f
#define LIFT_DONE_TOL_MM     0.3f
#define LAND_SP_RATE_MM_S    2.0f
#define LAND_GAP_MM          14.0f  /* FILL: at or beyond the resting gap on the catch stops [mm] */
#define LAND_FORCE_RAMP_S    0.5f

/* ########################## END CONFIGURATION ########################## */


#define MM_TO_M          1.0e-3f
#define TWO_PI_F         6.28318531f
#define DEG_TO_RAD       0.0174532925f
#define FAST_HZ          ((float)PWM_HZ / (float)FAST_DIV)
#define DT_FAST          (1.0f / FAST_HZ)
#define DT_OUTER         ((float)OUTER_DIV / FAST_HZ)
#define ADC_V_PER_COUNT  (ADC_VREF_V / 4095.0f)
#define AMPS_PER_SENSE_V (1.0f / (INA240_GAIN * R_SHUNT_OHM))
#define GAP_MODEL_MIN_M  1.0e-3f
#define VBUS_MIN_DIV     5.0f

#define ADC_CUR0   0
#define ADC_GAP0   4
#define ADC_BUS0   8
#define ADC_NUM_CH 11
#define ADC_BUF_LEN 16u   /* padded to one 32-byte cache line */

#define RX_RING_SIZE  256u
#define CMD_LINE_MAX  64u
#define TX_BUF_SIZE   320u

#define PRIO_ADC    1u
#define PRIO_SPI    2u
#define PRIO_OUTER  3u
#define PRIO_UART   4u

#define ISM_SPI_READ        0x80u
#define ISM_WHO_AM_I        0x0Fu
#define ISM_WHO_AM_I_VAL    0x6Bu
#define ISM_CTRL2_G         0x11u
#define ISM_CTRL3_C         0x12u
#define ISM_CTRL4_C         0x13u
#define ISM_CTRL6_C         0x15u
#define ISM_OUTX_L_G        0x22u
#define ISM_CTRL3_SW_RESET  0x01u
#define ISM_CTRL3_BDU       0x40u
#define ISM_CTRL3_IF_INC    0x04u
#define ISM_CTRL4_I2C_DIS   0x04u
#define ISM_CTRL4_LPF1_G    0x02u
#define ISM_CTRL2_ODR_6K66  0xA0u
#define GYRO_FRAME_BYTES    7u

/* ============================== Types ============================== */
typedef enum {
    ST_CALIBRATE, ST_DISARMED, ST_ARMED, ST_BENCH, ST_LIFTOFF, ST_LEVITATE, ST_LANDING
} state_t;
static const char *const STATE_NAME[] = {
    "CAL", "DISARMED", "ARMED", "BENCH", "LIFTOFF", "LEVITATE", "LANDING"
};

typedef enum {
    CMD_NONE, CMD_ARM, CMD_DISARM, CMD_LIFT, CMD_LAND, CMD_KILL, CMD_ZERO, CMD_SP, CMD_BENCH
} cmd_id_t;
typedef struct { cmd_id_t id; float a; float b; } cmd_t;

typedef struct {
    uint32_t cur[NUM_COILS];
    uint32_t gap[NUM_COILS];
    uint32_t n;
} acc_t;

typedef struct {
    uint32_t ms;
    uint8_t  state;
    float    h, sp, roll, pitch, hdot, p, q;
    float    gap[NUM_COILS], iref[NUM_COILS], imeas[NUM_COILS];
    float    vbus[NUM_SUPPLIES];
} telem_t;

/* ========================= Peripheral state ========================= */
static TIM_HandleTypeDef  s_htim;
static ADC_HandleTypeDef  s_hadc;
static DMA_HandleTypeDef  s_hdma_adc;
static SPI_HandleTypeDef  s_hspi;
static UART_HandleTypeDef s_huart;

static volatile uint16_t s_adc_buf[ADC_BUF_LEN] __attribute__((aligned(32))) DMA_BUF_SECTION;
static uint32_t       s_arr;
static uint8_t        s_spi_tx[8], s_spi_rx[8];
static volatile bool  s_spi_busy;
static uint8_t        s_uart_rx_byte;

/* ========================== Shared state =========================== */
static volatile float    g_iref[NUM_COILS];
static volatile bool     g_drive_on;
static volatile float    g_vbus[NUM_SUPPLIES];
static acc_t             s_acc[2];
static volatile uint8_t  s_acc_w;
static volatile cmd_t    g_cmd;
static volatile uint32_t g_cmd_seq;
static const char *volatile g_reply;
static volatile uint32_t g_reply_seq;
static telem_t           s_telem;
static volatile bool     g_telem_ready;
static volatile int16_t  g_gyro_raw[3];

/* fast loop */
static float    s_cur_offset_v[NUM_COILS];
static float    s_cur_int[NUM_COILS];
static float    s_cur_kp;
static float    s_cur_ki[NUM_COILS];
static float    s_bus_alpha;
static uint32_t s_fast_div_cnt;

/* outer loop */
static state_t  s_state = ST_CALIBRATE;
static uint32_t s_state_ticks;
static uint32_t s_cmd_seq_done;
static float    s_fit[3][NUM_COILS];
static float    s_alloc[NUM_COILS][3];
static float    s_k0_factor, s_ieff_max;
static float    s_gap_alpha, s_rate_alpha;
static bool     s_have_gaps;
static float    s_gap_m[NUM_COILS];
static float    s_imeas[NUM_COILS];
static float    s_h, s_h_prev, s_hdot, s_roll, s_pitch, s_p, s_q;
static float    s_gyro_bias[2];
static float    s_sp_m, s_sp_target_m;
static float    s_int_h, s_int_r, s_int_p;
static bool     s_outer_sat;
static float    s_force_scale;
static uint8_t  s_land_phase;
static uint8_t  s_bench_coil;
static float    s_bench_amps;
static float    s_iref_out[NUM_COILS];
static float    s_cal_cur_v[NUM_COILS], s_cal_gyro[2];
static uint32_t s_cal_n;
static uint32_t s_telem_div;

/* ============================= Helpers ============================= */
static inline float clampf(float x, float lo, float hi)
{
    return (x < lo) ? lo : ((x > hi) ? hi : x);
}

static inline float lpf_alpha(float f_hz, float dt)
{
    return (f_hz <= 0.0f) ? 1.0f : (1.0f - expf(-TWO_PI_F * f_hz * dt));
}

static inline bool ramp_toward(float *x, float target, float step)
{
    if (*x < target - step) { *x += step; return false; }
    if (*x > target + step) { *x -= step; return false; }
    *x = target;
    return true;
}

static inline uint32_t irq_off(void)
{
    uint32_t pm = __get_PRIMASK();
    __disable_irq();
    return pm;
}

static inline void irq_restore(uint32_t pm)
{
    __set_PRIMASK(pm);
}

static void inv3(const float m[3][3], float r[3][3])
{
    float c00 = m[1][1] * m[2][2] - m[1][2] * m[2][1];
    float c01 = m[1][2] * m[2][0] - m[1][0] * m[2][2];
    float c02 = m[1][0] * m[2][1] - m[1][1] * m[2][0];
    float id  = 1.0f / (m[0][0] * c00 + m[0][1] * c01 + m[0][2] * c02);
    r[0][0] = c00 * id;
    r[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * id;
    r[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * id;
    r[1][0] = c01 * id;
    r[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * id;
    r[1][2] = (m[0][2] * m[1][0] - m[0][0] * m[1][2]) * id;
    r[2][0] = c02 * id;
    r[2][1] = (m[0][1] * m[2][0] - m[0][0] * m[2][1]) * id;
    r[2][2] = (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * id;
}

/* ======================== Peripheral setup ========================= */
static void pin_init(GPIO_TypeDef *port, uint16_t pin, uint32_t mode, uint32_t pull, uint32_t af)
{
    GPIO_InitTypeDef g = { 0 };
    g.Pin       = pin;
    g.Mode      = mode;
    g.Pull      = pull;
    g.Speed     = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = af;
    HAL_GPIO_Init(port, &g);
}

static uint32_t tim8_clock_hz(void)
{
    uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();
#if defined(RCC_D2CFGR_D2PPRE2)
    bool div1 = (RCC->D2CFGR & RCC_D2CFGR_D2PPRE2) == 0u;
#else
    bool div1 = (RCC->CDCFGR2 & RCC_CDCFGR2_CDPPRE2) == 0u;
#endif
    return div1 ? pclk2 : 2u * pclk2;
}

static void pwm_init(void)
{
    __HAL_RCC_TIM8_CLK_ENABLE();
    s_arr = tim8_clock_hz() / (2u * PWM_HZ);   /* centre-aligned: period = 2*ARR */

    s_htim.Instance               = TIM8;
    s_htim.Init.Prescaler         = 0u;
    s_htim.Init.CounterMode       = TIM_COUNTERMODE_CENTERALIGNED1;
    s_htim.Init.Period            = s_arr;
    s_htim.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    s_htim.Init.RepetitionCounter = 2u * FAST_DIV - 1u;   /* one ADC trigger per FAST_DIV periods */
    s_htim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&s_htim);

    TIM_MasterConfigTypeDef m = { 0 };
    m.MasterOutputTrigger = TIM_TRGO_UPDATE;
    m.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&s_htim, &m);

    for (uint32_t k = 0; k < NUM_COILS; k++) {
        TIM_OC_InitTypeDef oc = { 0 };
        oc.OCMode       = PWM_PHASE_180[k] ? TIM_OCMODE_PWM2 : TIM_OCMODE_PWM1;
        oc.Pulse        = PWM_PHASE_180[k] ? s_arr + 1u : 0u;
        oc.OCPolarity   = TIM_OCPOLARITY_HIGH;
        oc.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
        oc.OCFastMode   = TIM_OCFAST_DISABLE;
        oc.OCIdleState  = TIM_OCIDLESTATE_RESET;
        oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;
        HAL_TIM_PWM_ConfigChannel(&s_htim, &oc, PWM_CH[k]);

        pin_init(PWM_PIN[k].port, PWM_PIN[k].pin, GPIO_MODE_AF_PP, GPIO_PULLDOWN, PWM_PIN[k].af);
        pin_init(DIR_PIN[k].port, DIR_PIN[k].pin, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, 0u);
        HAL_GPIO_WritePin(DIR_PIN[k].port, DIR_PIN[k].pin, DIR_FWD[k] ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

static void adc_init(void)
{
    static const uint32_t RANK[ADC_NUM_CH] = {
        ADC_REGULAR_RANK_1, ADC_REGULAR_RANK_2, ADC_REGULAR_RANK_3, ADC_REGULAR_RANK_4,
        ADC_REGULAR_RANK_5, ADC_REGULAR_RANK_6, ADC_REGULAR_RANK_7, ADC_REGULAR_RANK_8,
        ADC_REGULAR_RANK_9, ADC_REGULAR_RANK_10, ADC_REGULAR_RANK_11 };

    __HAL_RCC_ADC12_CLK_ENABLE();
    __HAL_RCC_ADC_CONFIG(RCC_ADCCLKSOURCE_CLKP);   /* per_ck (HSI by default) */
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_SYSCFG_CLK_ENABLE();

    /* Connect PA0 / PC2 / PC3 to their regular ADC pads. */
    HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PA0, SYSCFG_SWITCH_PA0_CLOSE);
    HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PC2, SYSCFG_SWITCH_PC2_CLOSE);
    HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PC3, SYSCFG_SWITCH_PC3_CLOSE);

    s_hdma_adc.Instance                 = DMA1_Stream0;
    s_hdma_adc.Init.Request             = DMA_REQUEST_ADC1;
    s_hdma_adc.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    s_hdma_adc.Init.PeriphInc           = DMA_PINC_DISABLE;
    s_hdma_adc.Init.MemInc              = DMA_MINC_ENABLE;
    s_hdma_adc.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    s_hdma_adc.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    s_hdma_adc.Init.Mode                = DMA_CIRCULAR;
    s_hdma_adc.Init.Priority            = DMA_PRIORITY_VERY_HIGH;
    s_hdma_adc.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&s_hdma_adc);
    __HAL_LINKDMA(&s_hadc, DMA_Handle, s_hdma_adc);

    s_hadc.Instance                      = ADC1;
    s_hadc.Init.ClockPrescaler           = ADC_CLOCK_ASYNC_DIV4;
    s_hadc.Init.Resolution               = ADC_RESOLUTION_12B;
    s_hadc.Init.ScanConvMode             = ADC_SCAN_ENABLE;
    s_hadc.Init.EOCSelection             = ADC_EOC_SEQ_CONV;
    s_hadc.Init.LowPowerAutoWait         = DISABLE;
    s_hadc.Init.ContinuousConvMode       = DISABLE;
    s_hadc.Init.NbrOfConversion          = ADC_NUM_CH;
    s_hadc.Init.DiscontinuousConvMode    = DISABLE;
    s_hadc.Init.ExternalTrigConv         = ADC_EXTERNALTRIG_T8_TRGO;
    s_hadc.Init.ExternalTrigConvEdge     = ADC_EXTERNALTRIGCONVEDGE_RISING;
    s_hadc.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
    s_hadc.Init.Overrun                  = ADC_OVR_DATA_OVERWRITTEN;
    s_hadc.Init.LeftBitShift             = ADC_LEFTBITSHIFT_NONE;
    s_hadc.Init.OversamplingMode         = DISABLE;
    HAL_ADC_Init(&s_hadc);
    HAL_ADCEx_Calibration_Start(&s_hadc, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);

    for (uint32_t i = 0; i < ADC_NUM_CH; i++) {
        pin_init(ADC_IN[i].port, ADC_IN[i].pin, GPIO_MODE_ANALOG, GPIO_NOPULL, 0u);
        ADC_ChannelConfTypeDef c = { 0 };
        c.Channel      = ADC_IN[i].ch;
        c.Rank         = RANK[i];
        c.SamplingTime = ADC_SAMPLETIME_8CYCLES_5;
        c.SingleDiff   = ADC_SINGLE_ENDED;
        c.OffsetNumber = ADC_OFFSET_NONE;
        HAL_ADC_ConfigChannel(&s_hadc, &c);
    }
    HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, PRIO_ADC, 0u);
    HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
}

static void spi_init(void)
{
    static const uint32_t PRESC[] = {
        SPI_BAUDRATEPRESCALER_2,  SPI_BAUDRATEPRESCALER_4,  SPI_BAUDRATEPRESCALER_8,
        SPI_BAUDRATEPRESCALER_16, SPI_BAUDRATEPRESCALER_32, SPI_BAUDRATEPRESCALER_64,
        SPI_BAUDRATEPRESCALER_128, SPI_BAUDRATEPRESCALER_256 };
    uint32_t clk = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI123), idx = 0, div = 2u;
    while (idx < 7u && clk / div > SPI_MAX_HZ) { idx++; div <<= 1; }

    __HAL_RCC_SPI1_CLK_ENABLE();
    pin_init(SPI_SCK.port,  SPI_SCK.pin,  GPIO_MODE_AF_PP, GPIO_NOPULL, SPI_SCK.af);
    pin_init(SPI_MISO.port, SPI_MISO.pin, GPIO_MODE_AF_PP, GPIO_NOPULL, SPI_MISO.af);
    pin_init(SPI_MOSI.port, SPI_MOSI.pin, GPIO_MODE_AF_PP, GPIO_NOPULL, SPI_MOSI.af);
    pin_init(SPI_CS.port,   SPI_CS.pin,   GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, 0u);
    HAL_GPIO_WritePin(SPI_CS.port, SPI_CS.pin, GPIO_PIN_SET);

    s_hspi.Instance               = SPI1;
    s_hspi.Init.Mode              = SPI_MODE_MASTER;
    s_hspi.Init.Direction         = SPI_DIRECTION_2LINES;
    s_hspi.Init.DataSize          = SPI_DATASIZE_8BIT;
    s_hspi.Init.CLKPolarity       = SPI_POLARITY_HIGH;   /* SPI mode 3 */
    s_hspi.Init.CLKPhase          = SPI_PHASE_2EDGE;
    s_hspi.Init.NSS               = SPI_NSS_SOFT;
    s_hspi.Init.BaudRatePrescaler = PRESC[idx];
    s_hspi.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    s_hspi.Init.TIMode            = SPI_TIMODE_DISABLE;
    s_hspi.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    s_hspi.Init.CRCPolynomial     = 7u;
    s_hspi.Init.CRCLength         = SPI_CRC_LENGTH_DATASIZE;
    s_hspi.Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;
    s_hspi.Init.NSSPolarity       = SPI_NSS_POLARITY_LOW;
    s_hspi.Init.FifoThreshold     = SPI_FIFO_THRESHOLD_01DATA;
    s_hspi.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
    HAL_SPI_Init(&s_hspi);
    HAL_NVIC_SetPriority(SPI1_IRQn, PRIO_SPI, 0u);
    HAL_NVIC_EnableIRQ(SPI1_IRQn);
}

static void uart_init(void)
{
    __HAL_RCC_USART3_CLK_ENABLE();
    pin_init(UART_TX.port, UART_TX.pin, GPIO_MODE_AF_PP, GPIO_PULLUP, UART_TX.af);
    pin_init(UART_RX.port, UART_RX.pin, GPIO_MODE_AF_PP, GPIO_PULLUP, UART_RX.af);
    s_huart.Instance                    = USART3;
    s_huart.Init.BaudRate               = UART_BAUD;
    s_huart.Init.WordLength             = UART_WORDLENGTH_8B;
    s_huart.Init.StopBits               = UART_STOPBITS_1;
    s_huart.Init.Parity                 = UART_PARITY_NONE;
    s_huart.Init.Mode                   = UART_MODE_TX_RX;
    s_huart.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
    s_huart.Init.OverSampling           = UART_OVERSAMPLING_16;
    s_huart.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
    s_huart.Init.ClockPrescaler         = UART_PRESCALER_DIV1;
    s_huart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    HAL_UART_Init(&s_huart);
    HAL_NVIC_SetPriority(USART3_IRQn, PRIO_UART, 0u);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
    HAL_UART_Receive_IT(&s_huart, &s_uart_rx_byte, 1u);
}

/* Signed duty per coil: sign -> DIR pin, magnitude -> PWM. */
static void set_duty(const float duty[NUM_COILS])
{
    for (uint32_t k = 0; k < NUM_COILS; k++) {
        bool  fwd = (duty[k] >= 0.0f);
        float mag = fminf(fwd ? duty[k] : -duty[k], 1.0f);
        uint32_t counts = (uint32_t)(mag * (float)s_arr + 0.5f);
        if (counts > 0u) {
            HAL_GPIO_WritePin(DIR_PIN[k].port, DIR_PIN[k].pin,
                              (fwd == DIR_FWD[k]) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        }
        uint32_t ccr = PWM_PHASE_180[k] ? ((counts == 0u) ? s_arr + 1u : s_arr - counts) : counts;
        __HAL_TIM_SET_COMPARE(&s_htim, PWM_CH[k], ccr);
    }
}

static void spi_xfer_blocking(const uint8_t *tx, uint8_t *rx, uint16_t n)
{
    HAL_GPIO_WritePin(SPI_CS.port, SPI_CS.pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&s_hspi, (uint8_t *)tx, rx, n, 10u);
    HAL_GPIO_WritePin(SPI_CS.port, SPI_CS.pin, GPIO_PIN_SET);
}

static void spi_xfer_async(const uint8_t *tx, uint16_t n)
{
    if (s_spi_busy) return;
    memcpy(s_spi_tx, tx, n);
    s_spi_busy = true;
    HAL_GPIO_WritePin(SPI_CS.port, SPI_CS.pin, GPIO_PIN_RESET);
    if (HAL_SPI_TransmitReceive_IT(&s_hspi, s_spi_tx, s_spi_rx, n) != HAL_OK) {
        HAL_GPIO_WritePin(SPI_CS.port, SPI_CS.pin, GPIO_PIN_SET);
        s_spi_busy = false;
    }
}

static bool uart_busy(void)
{
    return s_huart.gState != HAL_UART_STATE_READY;
}

/* ========================== Magnet model =========================== */
static float k_tab_at(float z_mm)
{
    if (z_mm <= K_TAB_Z_MM[0])           return K_TAB_K[0];
    if (z_mm >= K_TAB_Z_MM[K_TAB_N - 1]) return K_TAB_K[K_TAB_N - 1];
    for (uint32_t i = 1; i < K_TAB_N; i++) {
        if (z_mm <= K_TAB_Z_MM[i]) {
            float t = (z_mm - K_TAB_Z_MM[i - 1]) / (K_TAB_Z_MM[i] - K_TAB_Z_MM[i - 1]);
            return K_TAB_K[i - 1] + t * (K_TAB_K[i] - K_TAB_K[i - 1]);
        }
    }
    return K_TAB_K[K_TAB_N - 1];
}

/* K0 = unsaturated coefficient (K_TAB was measured at K_TAB_CAL_CURRENT_A). */
static inline float k0_at(int k, float z_m)
{
    return k_tab_at(z_m / MM_TO_M) * K_SCALE[k] * s_k0_factor;
}

static float force_max(int k, float z_m)
{
    float z = fmaxf(z_m, GAP_MODEL_MIN_M);
    float r = s_ieff_max / z;
    return k0_at(k, z) * r * r;
}

static float force_to_current(int k, float F, float z_m, bool *sat)
{
    if (F <= 0.0f) return 0.0f;
    float z    = fmaxf(z_m, GAP_MODEL_MIN_M);
    float ieff = z * sqrtf(F / k0_at(k, z));
    if (ieff >= s_ieff_max) { *sat = true; return I_MAX_A; }
    return ieff / (1.0f - ieff / I_SAT_A);
}

/* ==================== Fast loop: current PI (20 kHz) =================== */
static float current_pi(int k, float iref, float imeas, float vbus)
{
    float d_lo = (imeas > BRAKE_FLOOR_A) ? -DUTY_MAX : 0.0f;

    if (iref <= 0.0f) {
        s_cur_int[k] = 0.0f;
        return d_lo;
    }

    float e = iref - imeas;
    float v = R_COIL_OHM[k] * iref + s_cur_kp * e + s_cur_int[k];
    float d = v / fmaxf(vbus, VBUS_MIN_DIV);

    bool sat_hi = false, sat_lo = false;
    if (d > DUTY_MAX) { d = DUTY_MAX; sat_hi = true; }
    if (d < d_lo)     { d = d_lo;     sat_lo = true; }

    if (!(sat_hi && e > 0.0f) && !(sat_lo && e < 0.0f)) {
        s_cur_int[k] = clampf(s_cur_int[k] + s_cur_ki[k] * e * DT_FAST,
                              -CUR_INT_LIMIT_V, CUR_INT_LIMIT_V);
    }
    return d;
}

static void fast_loop(const volatile uint16_t *adc)
{
    acc_t *acc = &s_acc[s_acc_w];
    float duty[NUM_COILS];

    for (uint32_t s = 0; s < NUM_SUPPLIES; s++) {
        float v = (float)adc[ADC_BUS0 + s] * ADC_V_PER_COUNT * BUS_DIV_RATIO[s];
        g_vbus[s] += s_bus_alpha * (v - g_vbus[s]);
    }

    for (uint32_t k = 0; k < NUM_COILS; k++) {
        uint16_t raw = adc[ADC_CUR0 + k];
        acc->cur[k] += raw;
        acc->gap[k] += adc[ADC_GAP0 + k];
        float imeas = CUR_SIGN[k] * ((float)raw * ADC_V_PER_COUNT - s_cur_offset_v[k]) * AMPS_PER_SENSE_V;
        if (g_drive_on) {
            duty[k] = current_pi((int)k, g_iref[k], imeas, g_vbus[COIL_SUPPLY[k]]);
        } else {
            s_cur_int[k] = 0.0f;
            duty[k] = 0.0f;
        }
    }
    acc->n++;
    set_duty(duty);

    if (++s_fast_div_cnt >= OUTER_DIV) {
        s_fast_div_cnt = 0;
        uint8_t next = (uint8_t)(s_acc_w ^ 1u);
        memset(&s_acc[next], 0, sizeof(acc_t));
        s_acc_w = next;
        NVIC_SetPendingIRQ(TIM7_IRQn);   /* run outer loop */
    }
}

/* =============================== IMU =============================== */
static const uint8_t GYRO_READ_TX[GYRO_FRAME_BYTES] = { ISM_OUTX_L_G | ISM_SPI_READ, 0, 0, 0, 0, 0, 0 };

static void imu_write(uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { reg, val }, rx[2];
    spi_xfer_blocking(tx, rx, 2u);
}

static bool imu_init(void)
{
    uint8_t tx[2] = { ISM_WHO_AM_I | ISM_SPI_READ, 0 }, rx[2] = { 0, 0 };
    spi_xfer_blocking(tx, rx, 2u);
    imu_write(ISM_CTRL3_C, ISM_CTRL3_SW_RESET);
    HAL_Delay(10u);
    imu_write(ISM_CTRL3_C, ISM_CTRL3_BDU | ISM_CTRL3_IF_INC);
    imu_write(ISM_CTRL4_C, ISM_CTRL4_I2C_DIS | ISM_CTRL4_LPF1_G);
    imu_write(ISM_CTRL6_C, (uint8_t)(GYRO_LPF1_FTYPE & 0x07u));
    imu_write(ISM_CTRL2_G, (uint8_t)(ISM_CTRL2_ODR_6K66 | GYRO_CTRL2_FS_BITS));
    return rx[1] == ISM_WHO_AM_I_VAL;
}

static void gyro_rates(float *p, float *q)
{
    int16_t raw[3];
    uint32_t st = irq_off();
    raw[0] = g_gyro_raw[0]; raw[1] = g_gyro_raw[1]; raw[2] = g_gyro_raw[2];
    irq_restore(st);
    *p = GYRO_ROLL_SIGN  * (float)raw[GYRO_ROLL_AXIS]  * GYRO_DPS_PER_LSB * DEG_TO_RAD;
    *q = GYRO_PITCH_SIGN * (float)raw[GYRO_PITCH_AXIS] * GYRO_DPS_PER_LSB * DEG_TO_RAD;
}

/* ============================ Estimation ============================ */
static void estimate(const acc_t *acc)
{
    float n = (acc->n > 0u) ? (float)acc->n : 1.0f;

    for (uint32_t k = 0; k < NUM_COILS; k++) {
        float ma = ((float)acc->gap[k] / n) * ADC_V_PER_COUNT / GAP_BURDEN_OHM * 1000.0f;
        float g  = GAP_MM_AT_4MA + (ma - 4.0f) * ((GAP_MM_AT_20MA - GAP_MM_AT_4MA) / 16.0f);
        float z  = (GAP_CAL_A[k] * g * g + GAP_CAL_B[k] * g + GAP_CAL_C[k]) * MM_TO_M;
        s_gap_m[k] = s_have_gaps ? s_gap_m[k] + s_gap_alpha * (z - s_gap_m[k]) : z;

        float v = (float)acc->cur[k] / n * ADC_V_PER_COUNT;
        s_imeas[k] = CUR_SIGN[k] * (v - s_cur_offset_v[k]) * AMPS_PER_SENSE_V;
    }

    /* Rigid-plane fit: z_i = h + y_i*roll - x_i*pitch */
    float x[3] = { 0.0f, 0.0f, 0.0f };
    for (uint32_t r = 0; r < 3; r++)
        for (uint32_t k = 0; k < NUM_COILS; k++)
            x[r] += s_fit[r][k] * s_gap_m[k];
    s_h = x[0]; s_roll = x[1]; s_pitch = x[2];

    if (!s_have_gaps) { s_h_prev = s_h; s_hdot = 0.0f; }
    s_hdot += s_rate_alpha * ((s_h - s_h_prev) / DT_OUTER - s_hdot);
    s_h_prev = s_h;
    s_have_gaps = true;

    float p, q;
    gyro_rates(&p, &q);
    s_p = p - s_gyro_bias[0];
    s_q = q - s_gyro_bias[1];
}

/* ============================ Controller ============================ */
static void reset_outer_integrators(void)
{
    s_int_h = s_int_r = s_int_p = 0.0f;
    s_outer_sat = false;
}

static void run_controller(bool integrate, float scale, float iref[NUM_COILS])
{
    float e_h = s_h - s_sp_m;
    float e_r = s_roll;
    float e_p = s_pitch;

    if (integrate && !s_outer_sat) {
        s_int_h = clampf(s_int_h + HEAVE_KI * e_h * DT_OUTER, -HEAVE_I_LIMIT, HEAVE_I_LIMIT);
        s_int_r = clampf(s_int_r + ROLL_KI  * e_r * DT_OUTER, -ROLL_I_LIMIT,  ROLL_I_LIMIT);
        s_int_p = clampf(s_int_p + PITCH_KI * e_p * DT_OUTER, -PITCH_I_LIMIT, PITCH_I_LIMIT);
    }

    float a_up = HEAVE_KP * e_h + HEAVE_KD * s_hdot + s_int_h;
    float Fz   = fmaxf(POD_MASS_KG * (GRAVITY_MPS2 + a_up), 0.0f);

    float al_r  = -(ROLL_KP  * e_r + ROLL_KD  * s_p + s_int_r);
    float al_p  = -(PITCH_KP * e_p + PITCH_KD * s_q + s_int_p);
    float tau_r = J_ROLL_KGM2 * al_r + J_RP_KGM2    * al_p;
    float tau_p = J_RP_KGM2   * al_r + J_PITCH_KGM2 * al_p;

    /* Allocation: lift has priority, attitude share scaled down on saturation. */
    float fh[NUM_COILS], fa[NUM_COILS], s = 1.0f;
    for (uint32_t i = 0; i < NUM_COILS; i++) {
        fh[i] = s_alloc[i][0] * Fz;
        fa[i] = s_alloc[i][1] * tau_r + s_alloc[i][2] * tau_p;
        float hi = force_max((int)i, s_gap_m[i]) - fh[i];
        float lo = -fh[i];
        if (fa[i] > hi) s = fminf(s, (hi > 0.0f) ? hi / fa[i] : 0.0f);
        if (fa[i] < lo) s = fminf(s, (lo < 0.0f) ? lo / fa[i] : 0.0f);
    }
    bool sat = (s < 1.0f);

    for (uint32_t i = 0; i < NUM_COILS; i++) {
        float F = scale * (fh[i] + s * fa[i]);
        iref[i] = clampf(force_to_current((int)i, F, s_gap_m[i], &sat), 0.0f, I_MAX_A);
    }
    s_outer_sat = sat;
}

/* =========================== State machine =========================== */
static void set_state(state_t st)
{
    s_state = st;
    s_state_ticks = 0;
}

static void reply(const char *msg)
{
    g_reply = msg;
    g_reply_seq++;
}

static void handle_command(void)
{
    cmd_t c;
    uint32_t st = irq_off();
    bool fresh = (g_cmd_seq != s_cmd_seq_done);
    c.id = g_cmd.id; c.a = g_cmd.a; c.b = g_cmd.b;
    s_cmd_seq_done = g_cmd_seq;
    irq_restore(st);
    if (!fresh) return;

    switch (c.id) {
    case CMD_KILL:
        reset_outer_integrators();
        set_state(ST_DISARMED);
        reply("OK KILL");
        return;
    case CMD_SP:
        s_sp_target_m = clampf(c.a, SP_MIN_MM, SP_MAX_MM) * MM_TO_M;
        reply("OK SP");
        return;
    case CMD_ARM:
        if (s_state != ST_DISARMED) { reply("ERR ARM: not DISARMED"); return; }
        set_state(ST_ARMED);
        reply("OK ARM");
        return;
    case CMD_DISARM:
        if (s_state != ST_ARMED && s_state != ST_BENCH) { reply("ERR DISARM: not ARMED (use LAND or KILL)"); return; }
        set_state(ST_DISARMED);
        reply("OK DISARM");
        return;
    case CMD_LIFT:
        if (s_state != ST_ARMED) { reply("ERR LIFT: not ARMED"); return; }
        reset_outer_integrators();
        s_sp_m = s_h;
        s_force_scale = 0.0f;
        set_state(ST_LIFTOFF);
        reply("OK LIFT");
        return;
    case CMD_LAND:
        if (s_state != ST_LIFTOFF && s_state != ST_LEVITATE) { reply("ERR LAND: not flying"); return; }
        s_land_phase = 0;
        set_state(ST_LANDING);
        reply("OK LAND");
        return;
    case CMD_ZERO:
        if (s_state != ST_DISARMED) { reply("ERR ZERO: not DISARMED"); return; }
        s_cal_n = 0;
        memset(s_cal_cur_v, 0, sizeof s_cal_cur_v);
        memset(s_cal_gyro, 0, sizeof s_cal_gyro);
        set_state(ST_CALIBRATE);
        reply("OK ZERO");
        return;
    case CMD_BENCH: {
        int coil = (int)c.a;
        if (s_state != ST_ARMED && s_state != ST_BENCH) { reply("ERR BENCH: not ARMED"); return; }
        if (coil < 0 || coil >= NUM_COILS)              { reply("ERR BENCH: coil 0..3"); return; }
        s_bench_coil = (uint8_t)coil;
        s_bench_amps = clampf(c.b, 0.0f, I_MAX_A);
        set_state(s_bench_amps > 0.0f ? ST_BENCH : ST_ARMED);
        reply("OK BENCH");
        return;
    }
    default:
        return;
    }
}

/* Outer loop (1 kHz), software-pended from the fast loop. */
static void outer_loop(void)
{
    const acc_t *acc = &s_acc[s_acc_w ^ 1u];
    float iref[NUM_COILS] = { 0.0f, 0.0f, 0.0f, 0.0f };
    bool drive = false;

    s_state_ticks++;
    estimate(acc);
    handle_command();

    switch (s_state) {
    case ST_CALIBRATE:
        if (s_state_ticks > CAL_SETTLE_TICKS) {
            float n = (acc->n > 0u) ? (float)acc->n : 1.0f;
            float p, q;
            gyro_rates(&p, &q);
            for (uint32_t k = 0; k < NUM_COILS; k++)
                s_cal_cur_v[k] += (float)acc->cur[k] / n * ADC_V_PER_COUNT;
            s_cal_gyro[0] += p;
            s_cal_gyro[1] += q;
            if (++s_cal_n >= CAL_AVG_TICKS) {
                for (uint32_t k = 0; k < NUM_COILS; k++)
                    s_cur_offset_v[k] = s_cal_cur_v[k] / (float)s_cal_n;
                s_gyro_bias[0] = s_cal_gyro[0] / (float)s_cal_n;
                s_gyro_bias[1] = s_cal_gyro[1] / (float)s_cal_n;
                set_state(ST_DISARMED);
                reply("OK CAL done");
            }
        }
        break;

    case ST_DISARMED:
        reset_outer_integrators();
        break;

    case ST_ARMED:
        drive = true;
        break;

    case ST_BENCH:
        drive = true;
        iref[s_bench_coil] = s_bench_amps;
        break;

    case ST_LIFTOFF:
        drive = true;
        if (s_force_scale < 1.0f) {
            s_force_scale = fminf(s_force_scale + DT_OUTER / LIFT_FORCE_RAMP_S, 1.0f);
        } else {
            bool at_sp = ramp_toward(&s_sp_m, s_sp_target_m, LIFT_SP_RATE_MM_S * MM_TO_M * DT_OUTER);
            if (at_sp && fabsf(s_h - s_sp_target_m) < LIFT_DONE_TOL_MM * MM_TO_M)
                set_state(ST_LEVITATE);
        }
        run_controller(s_force_scale >= 1.0f, s_force_scale, iref);
        break;

    case ST_LEVITATE:
        drive = true;
        (void)ramp_toward(&s_sp_m, s_sp_target_m, SP_RATE_MM_S * MM_TO_M * DT_OUTER);
        run_controller(true, 1.0f, iref);
        break;

    case ST_LANDING:
        drive = true;
        if (s_land_phase == 0u) {
            if (ramp_toward(&s_sp_m, LAND_GAP_MM * MM_TO_M, LAND_SP_RATE_MM_S * MM_TO_M * DT_OUTER))
                s_land_phase = 1u;
        } else {
            s_force_scale = fmaxf(s_force_scale - DT_OUTER / LAND_FORCE_RAMP_S, 0.0f);
            if (s_force_scale <= 0.0f) set_state(ST_ARMED);
        }
        if (s_state == ST_LANDING) run_controller(false, s_force_scale, iref);
        break;
    }

    for (uint32_t k = 0; k < NUM_COILS; k++) {
        float i = clampf(iref[k], 0.0f, I_MAX_A);
        s_iref_out[k] = i;
        g_iref[k] = i;
    }
    g_drive_on = drive;

    if (++s_telem_div >= TELEM_DIV && !g_telem_ready) {
        s_telem_div = 0;
        s_telem.ms = HAL_GetTick();
        s_telem.state = (uint8_t)s_state;
        s_telem.h = s_h; s_telem.sp = s_sp_m; s_telem.roll = s_roll; s_telem.pitch = s_pitch;
        s_telem.hdot = s_hdot; s_telem.p = s_p; s_telem.q = s_q;
        for (uint32_t k = 0; k < NUM_COILS; k++) {
            s_telem.gap[k]   = s_gap_m[k];
            s_telem.iref[k]  = s_iref_out[k];
            s_telem.imeas[k] = s_imeas[k];
        }
        for (uint32_t s = 0; s < NUM_SUPPLIES; s++) s_telem.vbus[s] = g_vbus[s];
        g_telem_ready = true;
    }

    spi_xfer_async(GYRO_READ_TX, GYRO_FRAME_BYTES);   /* gyro sample for next tick */
}

/* ========================= Precomputation ========================== */
static void precompute(void)
{
    /* Gap fit: rows [1, y, -x], fit = (A'A)^-1 A' */
    float ata[3][3] = { { 0 } }, iata[3][3];
    for (uint32_t k = 0; k < NUM_COILS; k++) {
        float a[3] = { 1.0f, SENS_Y_M[k], -SENS_X_M[k] };
        for (uint32_t r = 0; r < 3; r++)
            for (uint32_t c = 0; c < 3; c++)
                ata[r][c] += a[r] * a[c];
    }
    inv3(ata, iata);
    for (uint32_t k = 0; k < NUM_COILS; k++) {
        float a[3] = { 1.0f, SENS_Y_M[k], -SENS_X_M[k] };
        for (uint32_t r = 0; r < 3; r++)
            s_fit[r][k] = iata[r][0] * a[0] + iata[r][1] * a[1] + iata[r][2] * a[2];
    }

    /* Allocation: columns [1, -y, x], alloc = B' (B B')^-1 (minimum-norm) */
    float bbt[3][3] = { { 0 } }, ibbt[3][3];
    for (uint32_t i = 0; i < NUM_COILS; i++) {
        float b[3] = { 1.0f, -MAG_Y_M[i], MAG_X_M[i] };
        for (uint32_t r = 0; r < 3; r++)
            for (uint32_t c = 0; c < 3; c++)
                bbt[r][c] += b[r] * b[c];
    }
    inv3(bbt, ibbt);
    for (uint32_t i = 0; i < NUM_COILS; i++) {
        float b[3] = { 1.0f, -MAG_Y_M[i], MAG_X_M[i] };
        for (uint32_t c = 0; c < 3; c++)
            s_alloc[i][c] = b[0] * ibbt[0][c] + b[1] * ibbt[1][c] + b[2] * ibbt[2][c];
    }

    float f = 1.0f + K_TAB_CAL_CURRENT_A / I_SAT_A;
    s_k0_factor = f * f;
    s_ieff_max  = I_MAX_A / (1.0f + I_MAX_A / I_SAT_A);

    s_gap_alpha  = lpf_alpha(GAP_LPF_HZ, DT_OUTER);
    s_rate_alpha = lpf_alpha(HEAVE_RATE_LPF_HZ, DT_OUTER);
    s_bus_alpha  = lpf_alpha(BUS_LPF_HZ, DT_FAST);
    s_cur_kp     = TWO_PI_F * CUR_BW_HZ * COIL_L_H;
    for (uint32_t k = 0; k < NUM_COILS; k++) {
        s_cur_ki[k] = TWO_PI_F * CUR_BW_HZ * R_COIL_OHM[k];
        s_cur_offset_v[k] = 0.5f * ADC_VREF_V;   /* replaced by boot calibration */
    }
    s_sp_target_m = HOVER_GAP_MM * MM_TO_M;
}

/* ============================ Serial link ============================ */
static volatile uint8_t  s_rx_ring[RX_RING_SIZE];
static volatile uint16_t s_rx_head, s_rx_tail;
static char              s_line[CMD_LINE_MAX];
static uint16_t          s_line_len;
static char              s_tx[TX_BUF_SIZE];
static char              s_pending[TX_BUF_SIZE];
static bool              s_pending_valid;

static void uart_rx_byte(uint8_t byte)
{
    uint16_t next = (uint16_t)((s_rx_head + 1u) % RX_RING_SIZE);
    if (next != s_rx_tail) {
        s_rx_ring[s_rx_head] = byte;
        s_rx_head = next;
    }
}

static void queue_text(const char *a, const char *b)
{
    if (s_pending_valid) return;
    snprintf(s_pending, sizeof s_pending, "%s%s\n", a, b);
    s_pending_valid = true;
}

static void post_command(cmd_id_t id, float a, float b)
{
    uint32_t st = irq_off();
    g_cmd.id = id; g_cmd.a = a; g_cmd.b = b;
    g_cmd_seq++;
    irq_restore(st);
}

static void parse_line(char *line)
{
    for (char *p = line; *p; p++) *p = (char)toupper((unsigned char)*p);
    char *tok = strtok(line, " \t");
    if (tok == NULL) return;
    char *a1 = strtok(NULL, " \t");
    char *a2 = strtok(NULL, " \t");

    if      (!strcmp(tok, "ARM"))    post_command(CMD_ARM, 0, 0);
    else if (!strcmp(tok, "DISARM")) post_command(CMD_DISARM, 0, 0);
    else if (!strcmp(tok, "LIFT"))   post_command(CMD_LIFT, 0, 0);
    else if (!strcmp(tok, "LAND"))   post_command(CMD_LAND, 0, 0);
    else if (!strcmp(tok, "KILL"))   post_command(CMD_KILL, 0, 0);
    else if (!strcmp(tok, "ZERO"))   post_command(CMD_ZERO, 0, 0);
    else if (!strcmp(tok, "SP") && a1)          post_command(CMD_SP, strtof(a1, NULL), 0);
    else if (!strcmp(tok, "BENCH") && a1 && a2) post_command(CMD_BENCH, strtof(a1, NULL), strtof(a2, NULL));
    else queue_text("ERR unknown: ", tok);
}

static void poll_rx(void)
{
    while (s_rx_tail != s_rx_head) {
        char ch = (char)s_rx_ring[s_rx_tail];
        s_rx_tail = (uint16_t)((s_rx_tail + 1u) % RX_RING_SIZE);
        if (ch == '\r' || ch == '\n') {
            if (s_line_len > 0u) {
                s_line[s_line_len] = '\0';
                parse_line(s_line);
                s_line_len = 0;
            }
        } else if (s_line_len < CMD_LINE_MAX - 1u) {
            s_line[s_line_len++] = ch;
        }
    }
}

static long scaled(float x, float k)
{
    float v = x * k;
    return (long)(v >= 0.0f ? v + 0.5f : v - 0.5f);
}

static int format_telem(const telem_t *t, char *buf, int cap)
{
    int n = snprintf(buf, (size_t)cap, "T,%lu,%s,%ld,%ld,%ld,%ld,%ld,%ld,%ld",
                     (unsigned long)t->ms, STATE_NAME[t->state],
                     scaled(t->h, 1e6f), scaled(t->sp, 1e6f),
                     scaled(t->roll, 1e6f), scaled(t->pitch, 1e6f),
                     scaled(t->hdot, 1e6f), scaled(t->p, 1e3f), scaled(t->q, 1e3f));
    for (uint32_t k = 0; k < NUM_COILS && n > 0 && n < cap; k++)
        n += snprintf(buf + n, (size_t)(cap - n), ",%ld", scaled(t->gap[k], 1e6f));
    for (uint32_t k = 0; k < NUM_COILS && n > 0 && n < cap; k++)
        n += snprintf(buf + n, (size_t)(cap - n), ",%ld", scaled(t->iref[k], 1e3f));
    for (uint32_t k = 0; k < NUM_COILS && n > 0 && n < cap; k++)
        n += snprintf(buf + n, (size_t)(cap - n), ",%ld", scaled(t->imeas[k], 1e3f));
    for (uint32_t s = 0; s < NUM_SUPPLIES && n > 0 && n < cap; s++)
        n += snprintf(buf + n, (size_t)(cap - n), ",%ld", scaled(t->vbus[s], 1e3f));
    if (n > 0 && n < cap - 1) { buf[n++] = '\n'; buf[n] = '\0'; }
    return (n > 0 && n < cap) ? n : 0;
}

static void service_tx(void)
{
    static uint32_t reply_seen;

    if (g_reply_seq != reply_seen && !s_pending_valid) {
        reply_seen = g_reply_seq;
        queue_text(g_reply, "");
    }
    if (uart_busy()) return;

    int n = 0;
    if (s_pending_valid) {
        n = snprintf(s_tx, sizeof s_tx, "%s", s_pending);
        s_pending_valid = false;
    } else if (g_telem_ready) {
        telem_t t;
        uint32_t st = irq_off();
        t = s_telem;
        g_telem_ready = false;
        irq_restore(st);
        n = format_telem(&t, s_tx, (int)sizeof s_tx);
    }
    if (n > 0) HAL_UART_Transmit_IT(&s_huart, (uint8_t *)s_tx, (uint16_t)n);
}

static void send_blocking_text(const char *s)
{
    while (uart_busy()) { }
    snprintf(s_tx, sizeof s_tx, "%s", s);
    HAL_UART_Transmit_IT(&s_huart, (uint8_t *)s_tx, (uint16_t)strlen(s_tx));
    while (uart_busy()) { }
}

/* ======================== IRQs and HAL callbacks ======================== */
void DMA1_Stream0_IRQHandler(void) { HAL_DMA_IRQHandler(&s_hdma_adc); }
void TIM7_IRQHandler(void)         { outer_loop(); }
void SPI1_IRQHandler(void)         { HAL_SPI_IRQHandler(&s_hspi); }
void USART3_IRQHandler(void)       { HAL_UART_IRQHandler(&s_huart); }

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    (void)hadc;
    if (SCB->CCR & SCB_CCR_DC_Msk)
        SCB_InvalidateDCache_by_Addr((void *)s_adc_buf, (int32_t)sizeof s_adc_buf);
    fast_loop(s_adc_buf);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    (void)hspi;
    HAL_GPIO_WritePin(SPI_CS.port, SPI_CS.pin, GPIO_PIN_SET);
    s_spi_busy = false;
    for (uint32_t a = 0; a < 3; a++)
        g_gyro_raw[a] = (int16_t)((uint16_t)s_spi_rx[1 + 2 * a] | ((uint16_t)s_spi_rx[2 + 2 * a] << 8));
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    (void)hspi;
    HAL_GPIO_WritePin(SPI_CS.port, SPI_CS.pin, GPIO_PIN_SET);
    s_spi_busy = false;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uart_rx_byte(s_uart_rx_byte);
    HAL_UART_Receive_IT(huart, &s_uart_rx_byte, 1u);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    HAL_UART_Receive_IT(huart, &s_uart_rx_byte, 1u);
}

/* =============================== Entry =============================== */
/* Call from main.c (USER CODE BEGIN 2) after HAL_Init() and SystemClock_Config(). Never returns. */
void hx11_main(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();

    pwm_init();
    adc_init();
    spi_init();
    uart_init();
    HAL_NVIC_SetPriority(TIM7_IRQn, PRIO_OUTER, 0u);
    HAL_NVIC_EnableIRQ(TIM7_IRQn);

    send_blocking_text("# HX11 lev firmware\n");
    send_blocking_text("# T,ms,state,h_um,sp_um,roll_urad,pitch_urad,hdot_um_s,p_mrad_s,q_mrad_s,"
                       "gFL_um,gFR_um,gRL_um,gRR_um,irFL_mA,irFR_mA,irRL_mA,irRR_mA,"
                       "imFL_mA,imFR_mA,imRL_mA,imRR_mA,vA_mV,vB_mV,vC_mV\n");
    send_blocking_text("# cmds: ARM DISARM LIFT LAND KILL ZERO | SP <mm> | BENCH <coil0-3> <A>\n");

    precompute();
    if (!imu_init()) send_blocking_text("# IMU not responding\n");
    set_state(ST_CALIBRATE);

    HAL_ADC_Start_DMA(&s_hadc, (uint32_t *)s_adc_buf, ADC_NUM_CH);
    __HAL_DMA_DISABLE_IT(&s_hdma_adc, DMA_IT_HT);
    for (uint32_t k = 0; k < NUM_COILS; k++)
        HAL_TIM_PWM_Start(&s_htim, PWM_CH[k]);

    for (;;) {
        poll_rx();
        service_tx();
    }
}
