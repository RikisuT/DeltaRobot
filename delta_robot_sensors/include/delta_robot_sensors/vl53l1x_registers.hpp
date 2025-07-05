#ifndef _VL53L1X_REGISTERS_HPP_
#define _VL53L1X_REGISTERS_HPP_

//#define defaultAddress_VL53L1X 0x29  7 bit address reported by the PI
#define AddressDefault 0x29 //0x52

#define SOFT_RESET                          0x0000
#define I2C_SLAVE__DEVICE_ADDRESS           0x0001
#define VL53L1_IDENTIFICATION__MODEL_ID     0x010F
#define FIRMWARE__SYSTEM_STATUS             0x00E5
#define OSC_MEASURED__FAST_OSC__FREQUENCY   0x0006
#define RESULT__OSC_CALIBRATE_VAL           0x00DE
#define VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND        0x0008
#define VHV_CONFIG__INIT                             0x000B
#define RESULT__RANGE_STATUS                         0x0089
#define PHASECAL_CONFIG__OVERRIDE                    0x004D
#define CAL_CONFIG__VCSEL_START                      0x0047
#define PHASECAL_RESULT__VCSEL_START                 0x00D8

// Static config registers
#define DSS_CONFIG__TARGET_TOTAL_RATE_MCPS           0x0024
#define GPIO__TIO_HV_STATUS                          0x0031
#define SIGMA_ESTIMATOR__EFFECTIVE_PULSE_WIDTH_NS    0x0036
#define SIGMA_ESTIMATOR__EFFECTIVE_AMBIENT_WIDTH_NS  0x0037
#define ALGO__CROSSTALK_COMPENSATION_VALID_HEIGHT_MM 0x0039
#define ALGO__RANGE_IGNORE_VALID_HEIGHT_MM           0x003E
#define ALGO__RANGE_MIN_CLIP                         0x003F
#define ALGO__CONSISTENCY_CHECK__TOLERANCE           0x0040

// General config registers
#define SYSTEM__THRESH_RATE_HIGH                     0x0050
#define SYSTEM__THRESH_RATE_LOW                      0x0052
#define DSS_CONFIG__APERTURE_ATTENUATION             0x0057

// Timing config registers
#define RANGE_CONFIG__SIGMA_THRESH                   0x0064
#define RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS  0x0066

// Dynamic Config regiters
#define SYSTEM__GROUPED_PARAMETER_HOLD_0             0x0071
#define SYSTEM__GROUPED_PARAMETER_HOLD_1             0x007C
#define SD_CONFIG__QUANTIFIER                        0x007E

// timed_ranging registers
#define SYSTEM__GROUPED_PARAMETER_HOLD               0x0082
#define SYSTEM__SEED_CONFIG                          0x0077
#define SYSTEM__SEQUENCE_CONFIG                      0x0081
#define DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT    0x0054
#define DSS_CONFIG__ROI_MODE_CONTROL                 0x004F

// Distance mode parameters
#define RANGE_CONFIG__VCSEL_PERIOD_A                 0x0060
#define RANGE_CONFIG__VCSEL_PERIOD_B                 0x0063
#define RANGE_CONFIG__VALID_PHASE_HIGH               0x0069
#define SD_CONFIG__WOI_SD0                           0x0078
#define SD_CONFIG__WOI_SD1                           0x0079
#define SD_CONFIG__INITIAL_PHASE_SD0                 0x007A
#define SD_CONFIG__INITIAL_PHASE_SD1                 0x007B

//Measurement timing budget parameters
#define PHASECAL_CONFIG__TIMEOUT_MACROP              0x004B
#define MM_CONFIG__TIMEOUT_MACROP_A                  0x005A
#define MM_CONFIG__TIMEOUT_MACROP_B                  0x005C
#define RANGE_CONFIG__TIMEOUT_MACROP_A               0x005E
#define RANGE_CONFIG__TIMEOUT_MACROP_B               0x0061

// Other registers
#define ALGO__PART_TO_PART_RANGE_OFFSET_MM           0x001E
#define MM_CONFIG__OUTER_OFFSET_MM                   0x0022

// Reading Distance
#define SYSTEM__INTERMEASUREMENT_PERIOD              0x006C
#define SYSTEM__INTERRUPT_CLEAR                      0x0086
#define SYSTEM__MODE_START                           0x0087
#define VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND        0x0008
#define PHASECAL_CONFIG__OVERRIDE                    0x004D

// Update DSS registers
#define DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT    0x0054

#endif // _VL53L1X_REGISTERS_HPP_