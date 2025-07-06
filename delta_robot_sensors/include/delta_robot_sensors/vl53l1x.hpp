/**
 * @file vl53l1x.hpp
 * @author Sharwin Patil (sharwinpatil@u.northwestern.edu)
 * @brief Header file containing class definition for VL53L1X that interfaces with the sensor
 * over I2C.
 * @date 2025-03-18
 *
 */

#ifndef VL53L1X_HPP
#define VL53L1X_HPP

#include <stdio.h>
#include <linux/i2c-dev.h> // for the ioctl() function
#include <unistd.h> // for the read() and write() function
#include <fcntl.h> // for the open() function include <stdio.h>
#include <string.h> // for the strlen() function
#include <stdlib.h> // for exit
#include <sys/ioctl.h>
#include <errno.h>
#include <chrono>
#include <thread>

#include "vl53l1x_registers.hpp"

 /**
  * @class VL53L1X
  * @brief Class for interfacing with the VL53L1X time-of-flight sensor over I2C.
  *
  */
class VL53L1X {
public:
  /**
   * @brief Constructor for the VL53L1X class.
   */
  VL53L1X();

  /**
   * @brief Destructor for the VL53L1X class.
   */
  ~VL53L1X() = default;

  /**
   * @brief Enum for the distance mode of the sensor.
   * @note The options are Short, Medium, Long, and Unknown.
   * Each mode has different timing budgets and accuracy.
   *
   */
  enum DistanceMode { Short, Medium, Long, Unknown };

  /**
   * @brief Enum for the status of the sensor reading.
   *
   */
  enum RangeStatus : uint8_t {
    RangeValid = 0,

    // "sigma estimator check is above the internal defined threshold"
    // (sigma = standard deviation of measurement)
    SigmaFail = 1,

    // "signal value is below the internal defined threshold"
    SignalFail = 2,

    // "Target is below minimum detection threshold."
    RangeValidMinRangeClipped = 3,

    // "phase is out of bounds"
    // (nothing detected in range; try a longer distance mode if applicable)
    OutOfBoundsFail = 4,

    // "HW or VCSEL failure"
    HardwareFail = 5,

    // "The Range is valid but the wraparound check has not been done."
    RangeValidNoWrapCheckFail = 6,

    // "Wrapped target, not matching phases"
    // "no matching phase in other VCSEL period timing."
    WrapTargetFail = 7,

    // "Internal algo underflow or overflow in lite ranging."
 // ProcessingFail            =   8: not used in API

    // "Specific to lite ranging."
    // should never occur with this lib (which uses low power auto ranging,
    // as the API does)
    XtalkSignalFail = 9,

    // "1st interrupt when starting ranging in back to back mode. Ignore
    // data."
    // should never occur with this lib
    SynchronizationInt = 10, // (the API spells this "syncronisation")

    // "All Range ok but object is result of multiple pulses merging together.
    // Used by RQL for merged pulse detection"
 // RangeValid MergedPulse    =  11: not used in API

    // "Used by RQL as different to phase fail."
 // TargetPresentLackOfSignal =  12:

    // "Target is below minimum detection threshold."
    MinRangeFail = 13,

    // "The reported range is invalid"
 // RangeInvalid              =  14: can't actually be returned by API (range can never become negative, even after correction)

    // "No Update."
    None = 255,
  };

  /**
   * @brief Struct for packaging the range data obtained over an I2C read.
   *
   */
  struct RangingData {
    uint16_t range_mm; /** @brief range reading in millimeters */
    RangeStatus range_status; /** @brief  The status of the range reading */
    float peak_signal_count_rate_MCPS; /** @brief peak signal count rate in MCPS */
    float ambient_count_rate_MCPS; /** @brief ambient count rate in MCPS */
  };

  /**
   * @brief The most recent ranging data obtained from the sensor.
   *
   */
  RangingData ranging_data;

  /**
   * @brief Attempts to initialize the sensor and establishes communication over I2C.
   *
   * @return true if the initialization was successful
   * @return false if the initialization failed for any reason
   */
  bool init();

  /**
   * @brief Sets the I2C address of the sensor.
   *
   * @param new_addr the new I2C address to set
   */
  void setAddress(uint8_t new_addr);

  /**
   * @brief Set the distance mode of the sensor.
   *
   * @param mode the enum value for the distance mode
   * @return true if the distance mode was set successfully
   * @return false if an invalid distance mode was provided
   */
  bool setDistanceMode(DistanceMode mode);

  /**
   * @brief Gets the current distance mode of the sensor.
   *
   * @return DistanceMode the current distance mode of the sensor as an enum
   */
  DistanceMode getDistanceMode() { return distance_mode; }

  /**
   * @brief Sets the timing budget of the sensor, which is the amount of time
   * the sensor is budgeted to make a reading. A higher timing budget allows for
   * more accurate readings but induces more latency with each reading.
   *
   * @note Each distance mode has different ranges of timing budgets:
   * Short: 20-33 ms, Medium: 33-50 ms, Long: 50-100 ms, Unknown: 33-100 ms
   *
   * @param budget_us the amount of time to budget the sensor to make a reading [us]
   * @return true if the given timing budget was set successfully
   * @return false if the given timing budget was invalid
   */
  bool setMeasurementTimingBudget(uint32_t budget_us);

  /**
   * @brief Gets the current timing budget of the sensor.
   *
   * @return uint32_t the timing budget in microseconds
   */
  uint32_t getMeasurementTimingBudget();

  /**
   * @brief Start continuous range measurements with the given inter-measurement period.
   *
   * @param period_ms The amount of time [ms] between each range measurement
   */
  void startContinuous(uint32_t period_ms);

  /**
   * @brief Stop continuous range measurements.
   *
   */
  void stopContinuous();

  /**
   * @brief Read the sensor and obtain the reading.
   *
   * @param blocking whether to block until a reading is available. Defaults to true.
   * @return uint16_t the range reading [mm]
   */
  uint16_t read_range(bool blocking = true);

  /**
   * @brief Perform a single range reading without continuous measurements.
   *
   * @param blocking whether to block until a reading is available.
   * @return uint16_t the range reading [mm]
   */
  uint16_t readSingle(bool blocking);

  /**
   * @brief Alias of readSingle()
   *
   */
  uint16_t readRangeSingleMillimeters(bool blocking = true) { return readSingle(blocking); } // alias of readSingle()

  /**
   * @brief Check if the sensor has a new reading available.
   *
   * @note assumes interrupt is active low (GPIO_HV_MUX__CTRL bit 4 is 1)
   *
   * @return true if a new reading is available
   */
  bool dataReady() { return (readReg(GPIO__TIO_HV_STATUS) & 0x01) == 0; }

  /**
   * @brief Set the sensor timeout length
   *
   * @param timeout the amount of time to wait for a reading before timing out [ms]
   */
  void setTimeout(uint16_t timeout) { io_timeout = timeout; }

  /**
   * @brief Check if a timeout has occurred since the last call to this function.
   *
   * @return true if a timeout has occurred when this function was called
   */
  bool timeoutOccurred();
private:

  // VariablesAddressDefault
  int fd;  //File Descriptor

  uint8_t address;  //I2C address 

  // for storing values read from RESULT__RANGE_STATUS (0x0089)
  // through RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0_LOW
  // (0x0099)
  struct ResultBuffer {
    uint8_t range_status;
    // uint8_t report_status: not used
    uint8_t stream_count;
    uint16_t dss_actual_effective_spads_sd0;
    // uint16_t peak_signal_count_rate_mcps_sd0: not used
    uint16_t ambient_count_rate_mcps_sd0;
    // uint16_t sigma_sd0: not used
    // uint16_t phase_sd0: not used
    uint16_t final_crosstalk_corrected_range_mm_sd0;
    uint16_t peak_signal_count_rate_crosstalk_corrected_mcps_sd0;
  };

  // making this static would save RAM for multiple instances as long as there
  // aren't multiple sensors being read at the same time (e.g. on separate
  // I2C buses)
  ResultBuffer results;

  // value used in measurement timing budget calculations
  // assumes PresetMode is LOWPOWER_AUTONOMOUS
  //
  // vhv = LOWPOWER_AUTO_VHV_LOOP_DURATION_US + LOWPOWERAUTO_VHV_LOOP_BOUND
  //       (tuning parm default) * LOWPOWER_AUTO_VHV_LOOP_DURATION_US
  //     = 245 + 3 * 245 = 980
  // TimingGuard = LOWPOWER_AUTO_OVERHEAD_BEFORE_A_RANGING +
  //               LOWPOWER_AUTO_OVERHEAD_BETWEEN_A_B_RANGING + vhv
  //             = 1448 + 2100 + 980 = 4528
  static const uint32_t TimingGuard = 4528;

  // value in DSS_CONFIG__TARGET_TOTAL_RATE_MCPS register, used in DSS
  // calculations
  static const uint16_t TargetRate = 0x0A00;

  uint8_t last_status;
  std::chrono::time_point<std::chrono::system_clock> timeout_start_ms;
  bool did_timeout;
  uint16_t io_timeout;

  uint16_t fast_osc_frequency;
  uint16_t osc_calibrate_val;

  bool calibrated;
  uint8_t saved_vhv_init;
  uint8_t saved_vhv_timeout;

  DistanceMode distance_mode;

  // Functions
  void writeReg(uint16_t reg, uint8_t value);
  void writeReg16Bit(uint16_t reg, uint16_t value);
  void writeReg32Bit(uint16_t reg, uint32_t value);

  uint8_t readReg(uint16_t reg);
  uint16_t readReg16Bit(uint16_t reg);

  void setupManualCalibration();

  void startTimeout();
  bool checkTimeoutExpired();
  uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_us, uint32_t macro_period_us);
  uint32_t timeoutMclksToMicroseconds(uint32_t timeout_mclks, uint32_t macro_period_us);
  uint16_t encodeTimeout(uint32_t timeout_mclks);
  uint32_t decodeTimeout(uint16_t reg_val);

  uint32_t calcMacroPeriod(uint8_t vcsel_period);

  void readResults();
  void updateDSS();
  void getRangingData();

  // Convert count rate from fixed point 9.7 format to float
  float countRateFixedToFloat(uint16_t count_rate_fixed) { return (float)count_rate_fixed / (1 << 7); }
};

#endif