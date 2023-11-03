// Copyright 2023 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <libhal/i2c.hpp>
#include <libhal/timeout.hpp>
#include <libhal/units.hpp>

namespace hal::icm {

class icm20948
{

public:
  enum class icm20948_cycle : hal::byte
  {
    icm20948_no_cycle = 0x00,
    icm20948_gyro_cycle = 0x10,
    icm20948_acc_cycle = 0x20,
    icm20948_acc_gyro_cycle = 0x30,
    icm20948_acc_gyro_i2c_mst_cycle = 0x70
  };

  enum class icm20948_int_pin_pol : hal::byte
  {
    icm20948_act_high,
    icm20948_act_low
  };

  enum class icm20948_int_type : hal::byte
  {
    icm20948_fsync_int = 0x01,
    icm20948_wom_int = 0x02,
    icm20948_dmp_int = 0x04,
    icm20948_data_ready_int = 0x08,
    icm20948_fifo_ovf_int = 0x10,
    icm20948_fifo_wm_int = 0x20
  };

  enum class icm20948_fifo_type : hal::byte
  {
    icm20948_fifo_acc = 0x10,
    icm20948_fifo_gyr = 0x0E,
    icm20948_fifo_acc_gyr = 0x1E
  };

  enum icm20948_fifo_mode_choice : hal::byte
  {
    icm20948_continuous,
    icm20948_stop_when_full
  };

  enum icm20948_gyro_range : hal::byte
  {
    icm20948_gyro_range_250,
    icm20948_gyro_range_500,
    icm20948_gyro_range_1000,
    icm20948_gyro_range_2000
  };

  enum icm20948_dlpf : hal::byte
  {
    icm20948_dlpf_0,
    icm20948_dlpf_1,
    icm20948_dlpf_2,
    icm20948_dlpf_3,
    icm20948_dlpf_4,
    icm20948_dlpf_5,
    icm20948_dlpf_6,
    icm20948_dlpf_7,
    icm20948_dlpf_off
  };

  enum icm20948_gyro_avg_low_pwr : hal::byte
  {
    icm20948_gyro_avg_1,
    icm20948_gyro_avg_2,
    icm20948_gyro_avg_4,
    icm20948_gyro_avg_8,
    icm20948_gyro_avg_16,
    icm20948_gyro_avg_32,
    icm20948_gyro_avg_64,
    icm20948_gyro_avg_128
  };

  enum icm20948_acc_range : hal::byte
  {
    icm20948_acc_range_2g,
    icm20948_acc_range_4g,
    icm20948_acc_range_8g,
    icm20948_acc_range_16g
  };

  enum icm20948_acc_avg_low_pwr : hal::byte
  {
    icm20948_acc_avg_4,
    icm20948_acc_avg_8,
    icm20948_acc_avg_16,
    icm20948_acc_avg_32
  };

  enum icm20948_wom_comp : hal::byte
  {
    icm20948_wom_comp_disable,
    icm20948_wom_comp_enable
  };

  enum ak09916_op_mode : hal::byte
  {
    ak09916_pwr_down = 0x00,
    ak09916_trigger_mode = 0x01,
    ak09916_cont_mode_10hz = 0x02,
    ak09916_cont_mode_20hz = 0x04,
    ak09916_cont_mode_50hz = 0x06,
    ak09916_cont_mode_100hz = 0x08
  };

  enum icm20948_orientation : hal::byte
  {
    icm20948_flat,
    icm20948_flat_1,
    icm20948_xy,
    icm20948_xy_1,
    icm20948_yx,
    icm20948_yx_1
  };

  struct accel_read_t
  {
    float x;
    float y;
    float z;
  };

  struct gyro_read_t
  {
    float x;
    float y;
    float z;
  };

  struct mag_read_t
  {
    float x;
    float y;
    float z;
  };

  struct temp_read_t
  {
    float temp;
  };

  /**
   * @brief Read acceleration data from out_x_msb_r, out_x_lsb_r,
   *        out_y_msb_r, out_y_lsb_r, out_z_msb_r, out_z_lsb_r
   *        and perform acceleration conversion to g.
   */
  [[nodiscard]] hal::result<accel_read_t> read_acceleration();

  /**
   * @brief Read gyroscope data from out_x_msb_r, out_x_lsb_r,
   *        out_y_msb_r, out_y_lsb_r, out_z_msb_r, out_z_lsb_r
   *        and perform gyroscope conversion to rad/s.
   */
  [[nodiscard]] hal::result<gyro_read_t> read_gyroscope();

  /**
   * @brief Read magnetometer data from out_x_msb_r, out_x_lsb_r,
   *        out_y_msb_r, out_y_lsb_r, out_z_msb_r, out_z_lsb_r
   *        and perform magnetometer conversion to uT.
   */
  [[nodiscard]] hal::result<mag_read_t> read_magnetometer();

  /**
   * @brief Read pressure data from out_t_msb_r and out_t_lsb_r
   *        and perform temperature conversion to celsius.
   */
  [[nodiscard]] hal::result<temp_read_t> read_temperature();

  [[nodiscard]] static result<icm20948> create(hal::i2c& p_i2c);

  hal::status init();

  hal::status auto_offsets();

  /**
   * @brief Set default acceleration offsets
   *    All offset parameters are in 'g's
   */
  hal::status set_acc_offsets(float p_xmin,
                              float p_xmax,
                              float p_ymin,
                              float p_ymax,
                              float p_zmin,
                              float p_zmax);

  hal::status set_gyro_offsets(float p_x_offset,
                               float p_y_offset,
                               float p_z_offset);

  hal::result<hal::byte> whoami();

  hal::status enable_acc(bool p_en_acc);
  hal::status set_acc_range(icm20948_acc_range p_acc_range);
  hal::status set_acc_dlpf(icm20948_dlpf p_dlpf);
  hal::status set_acc_sample_rate_div(uint16_t p_acc_spl_rate_div);
  hal::status enable_gyr(bool p_enGyr);
  hal::status set_gyro_range(icm20948_gyro_range gyroRange);
  hal::status set_gyro_dlpf(icm20948_dlpf p_dlpf);
  hal::status set_gyro_sample_rate_div(hal::byte p_gyro_spl_rate_div);
  hal::status set_temp_dlpf(icm20948_dlpf p_dlpf);

  /* Power, Sleep, Standby */
  hal::status enable_cycle(icm20948_cycle p_cycle);
  hal::status enable_low_power(bool p_enable_low_power);
  hal::status set_gyro_averg_cycle_mode(icm20948_gyro_avg_low_pwr p_avg);
  hal::status set_acc_averg_cycle_mode(icm20948_acc_avg_low_pwr p_avg);
  hal::status sleep(bool p_sleep);

  /* Magnetometer */
  hal::status init_mag();
  hal::status enable_bypass_mode();
  hal::result<hal::byte> mag_status1();
  hal::result<hal::byte> mag_status2();
  hal::status reset_mag();
  hal::result<hal::byte> check_mag_mode();
  hal::result<hal::byte> whoami_ak09916_wia1_direct();
  hal::result<hal::byte> whoami_ak09916_wia2_direct();
  hal::status set_mag_op_mode(ak09916_op_mode p_op_mode);
  hal::status write_ak09916_register8(hal::byte p_reg, hal::byte p_val);

private:
  hal::i2c* m_i2c;
  hal::byte m_current_bank;
  accel_read_t m_acc_offset_val;
  accel_read_t m_acc_corr_factor;
  gyro_read_t m_gyro_offset_val;
  hal::byte m_acc_range_factor;
  hal::byte m_gyro_range_factor;
  hal::byte m_reg_val;  // intermediate storage of register values

  explicit icm20948(hal::i2c& p_i2c)
    : m_i2c(&p_i2c)
  {
  }

  hal::status set_clock_auto_select();
  hal::status switch_bank(hal::byte p_newBank);
  hal::status write_register8(hal::byte p_bank,
                              hal::byte p_reg,
                              hal::byte p_val);
  hal::status write_register16(hal::byte p_bank, hal::byte reg, int16_t p_val);

  [[nodiscard]] hal::result<hal::byte> read_register8(hal::byte p_bank,
                                                      hal::byte p_reg);
  [[nodiscard]] hal::result<hal::byte> read_register16(hal::byte p_bank,
                                                       hal::byte p_reg);
  hal::status enable_mag_data_read(hal::byte p_reg, hal::byte p_bytes);

  hal::status reset_icm20948();
};

}  // namespace hal::icm