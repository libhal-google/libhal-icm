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

#include "icm20948_reg.hpp"
#include <array>
#include <cmath>
#include <libhal-icm/icm20948.hpp>
#include <libhal-util/i2c.hpp>

namespace hal::icm {
using namespace std::literals;

result<icm20948> icm20948::create(hal::i2c& p_i2c,
                                  hal::byte p_device_address = icm20948_address)
{
  icm20948 icm(p_i2c, p_device_address);

  HAL_CHECK(icm.init());

  return icm;
}

hal::status icm20948::init()
{
  m_currentBank = 0;
  reset_icm20948();
  if (HAL_CHECK(whoami()) != icm20948_who_am_i_content) {
    return hal::new_error();
  }

  m_acc_offset_val.x = 0.0;
  m_acc_offset_val.y = 0.0;
  m_acc_offset_val.z = 0.0;
  m_acc_corr_factor.x = 1.0;
  m_acc_corr_factor.y = 1.0;
  m_acc_corr_factor.z = 1.0;
  m_acc_range_factor = 1.0;
  m_gyr_offset_val.x = 0.0;
  m_gyr_offset_val.y = 0.0;
  m_gyr_offset_val.z = 0.0;
  m_gyr_range_factor = 1.0;

  sleep(false);
  enable_acc(true);
  enable_gyr(true);

  write_register8(2, icm20948_odr_align_en, 1);  // aligns ODR
  return hal::success();
}

hal::status icm20948::auto_offsets()
{

  set_gyr_dlpf(icm20948_dlpf_6);           // lowest noise
  set_gyr_range(icm20948_gyro_range_250);  // highest resolution
  set_acc_range(icm20948_acc_range_2g);
  set_acc_dlpf(icm20948_dlpf_6);
  set_temp_dlpf(icm20948_dlpf_6);

  set_mag_op_mode(ak09916_cont_mode_20hz);  // For Mag

  return hal::success();
}

hal::status icm20948::set_acc_offsets(float p_xmin,
                                      float p_xmax,
                                      float p_ymin,
                                      float p_ymax,
                                      float p_zmin,
                                      float p_zmax)
{
  m_acc_offset_val.x = (p_xmax + p_xmin) * 0.5;
  m_acc_offset_val.y = (p_ymax + p_ymin) * 0.5;
  m_acc_offset_val.z = (p_zmax + p_zmin) * 0.5;
  m_acc_corr_factor.x = (p_xmax + abs(p_xmin)) / 32768.0;
  m_acc_corr_factor.y = (p_ymax + abs(p_ymin)) / 32768.0;
  m_acc_corr_factor.z = (p_zmax + abs(p_zmin)) / 32768.0;

  return hal::success();
}

hal::status icm20948::set_gyr_offsets(float p_xOffset,
                                      float p_yOffset,
                                      float p_zOffset)
{
  m_gyr_offset_val.x = p_xOffset;
  m_gyr_offset_val.y = p_yOffset;
  m_gyr_offset_val.z = p_zOffset;

  return hal::success();
}

hal::result<hal::byte> icm20948::whoami()
{
  return HAL_CHECK(read_register8(0, icm20948_who_am_i));
}

hal::status icm20948::enable_acc(bool p_en_acc)
{
  m_reg_val = HAL_CHECK(read_register8(0, icm20948_pwr_mgmt_2));

  if (p_en_acc) {
    m_reg_val &= ~icm20948_acc_en;
  } else {
    m_reg_val |= icm20948_acc_en;
  }

  HAL_CHECK(write_register8(0, icm20948_pwr_mgmt_2, m_reg_val));

  return hal::success();
}

hal::status icm20948::set_acc_range(icm20948_acc_range p_accRange)
{
  m_reg_val = HAL_CHECK(read_register8(2, icm20948_accel_config));
  m_reg_val &= ~(0x06);
  m_reg_val |= (p_accRange << 1);
  HAL_CHECK(write_register8(2, icm20948_accel_config, m_reg_val));

  return hal::success();
}

hal::status icm20948::set_acc_dlpf(icm20948_dlpf p_dlpf)
{
  m_reg_val = HAL_CHECK(read_register8(2, icm20948_accel_config));

  if (p_dlpf == icm20948_dlpf_off) {
    m_reg_val &= 0xFE;
    HAL_CHECK(write_register8(2, icm20948_accel_config, m_reg_val));
    return hal::success();
  } else {
    m_reg_val |= 0x01;
    m_reg_val &= 0xC7;
    m_reg_val |= (p_dlpf << 3);
  }
  HAL_CHECK(write_register8(2, icm20948_accel_config, m_reg_val));

  return hal::success();
}

hal::status icm20948::set_acc_sample_rate_div(uint16_t p_acc_spl_rate_div)
{
  HAL_CHECK(
    write_register16(2, icm20948_accel_smplrt_div_1, p_acc_spl_rate_div));
  return hal::success();
}

hal::status icm20948::enable_gyr(bool p_enGyr)
{
  m_reg_val = HAL_CHECK(read_register8(0, icm20948_pwr_mgmt_2));
  if (p_enGyr) {
    m_reg_val &= ~icm20948_gyr_en;
  } else {
    m_reg_val |= icm20948_gyr_en;
  }
  HAL_CHECK(write_register8(0, icm20948_pwr_mgmt_2, m_reg_val));

  return hal::success();
}

hal::status icm20948::set_gyr_range(icm20948_gyro_range p_gyro_range)
{
  m_reg_val = HAL_CHECK(read_register8(2, icm20948_gyro_config_1));
  m_reg_val &= ~(0x06);
  m_reg_val |= (p_gyro_range << 1);
  HAL_CHECK(write_register8(2, icm20948_gyro_config_1, m_reg_val));

  return hal::success();
}

hal::status icm20948::set_gyr_dlpf(icm20948_dlpf p_dlpf)
{
  m_reg_val = HAL_CHECK(read_register8(2, icm20948_gyro_config_1));

  if (p_dlpf == icm20948_dlpf_off) {
    m_reg_val &= 0xFE;
    HAL_CHECK(write_register8(2, icm20948_gyro_config_1, m_reg_val));
    return hal::success();
  } else {
    m_reg_val |= 0x01;
    m_reg_val &= 0xC7;
    m_reg_val |= (p_dlpf << 3);
  }
  HAL_CHECK(write_register8(2, icm20948_gyro_config_1, m_reg_val));

  return hal::success();
}

hal::status icm20948::set_gyr_sample_rate_div(hal::byte p_gyr_spl_rate_div)
{
  HAL_CHECK(write_register8(2, icm20948_gyro_smplrt_div, p_gyr_spl_rate_div));
  return hal::success();
}

hal::status icm20948::set_temp_dlpf(icm20948_dlpf p_dlpf)
{
  HAL_CHECK(write_register8(2, icm20948_temp_config, p_dlpf));
  return hal::success();
}

hal::result<icm20948::accel_read_t> icm20948::read_acceleration()
{
  std::array<hal::byte, 6> data{};
  accel_read_t accel_read, accel_read_raw;
  switch_bank(0);
  data = HAL_CHECK(
    hal::write_then_read<6>(*m_i2c,
                            m_address,
                            std::array<hal::byte, 1>{ icm20948_accel_out },
                            hal::never_timeout()));

  accel_read_raw.x = static_cast<int16_t>(((data[0]) << 8) | data[1]) * 1.0;
  accel_read_raw.y = static_cast<int16_t>(((data[2]) << 8) | data[3]) * 1.0;
  accel_read_raw.z = static_cast<int16_t>(((data[4]) << 8) | data[5]) * 1.0;

  accel_read.x = (accel_read.x - (m_acc_offset_val.x / m_acc_range_factor)) /
                 m_acc_corr_factor.x;
  accel_read.y = (accel_read.y - (m_acc_offset_val.y / m_acc_range_factor)) /
                 m_acc_corr_factor.y;
  accel_read.z = (accel_read.z - (m_acc_offset_val.z / m_acc_range_factor)) /
                 m_acc_corr_factor.z;

  accel_read.x = accel_read_raw.x * m_acc_range_factor / 16384.0;
  accel_read.y = accel_read_raw.y * m_acc_range_factor / 16384.0;
  accel_read.z = accel_read_raw.z * m_acc_range_factor / 16384.0;

  return accel_read;
}

hal::result<icm20948::gyro_read_t> icm20948::read_gyroscope()
{
  std::array<hal::byte, 6> data{};
  gyro_read_t gyro_read, gyro_read_raw;

  switch_bank(0);
  data = HAL_CHECK(
    hal::write_then_read<6>(*m_i2c,
                            m_address,
                            std::array<hal::byte, 1>{ icm20948_gyro_out },
                            hal::never_timeout()));

  gyro_read_raw.x = static_cast<int16_t>(((data[0]) << 8) | data[1]) * 1.0;
  gyro_read_raw.y = static_cast<int16_t>(((data[2]) << 8) | data[3]) * 1.0;
  gyro_read_raw.z = static_cast<int16_t>(((data[4]) << 8) | data[5]) * 1.0;

  gyro_read.x -= (m_gyr_offset_val.x / m_gyr_range_factor);
  gyro_read.y -= (m_gyr_offset_val.y / m_gyr_range_factor);
  gyro_read.z -= (m_gyr_offset_val.z / m_gyr_range_factor);

  gyro_read.x = gyro_read_raw.x * m_gyr_range_factor * 250.0 / 32768.0;
  gyro_read.y = gyro_read_raw.y * m_gyr_range_factor * 250.0 / 32768.0;
  gyro_read.z = gyro_read_raw.z * m_gyr_range_factor * 250.0 / 32768.0;

  return gyro_read;
}

hal::result<icm20948::mag_read_t> icm20948::read_magnetometer()
{
  // read_ak09916_register16(icm20948_ext_slv_sens_data_00);
  std::array<hal::byte, 6> data{};
  int16_t x, y, z;
  mag_read_t mag_read;

  switch_bank(0);
  data = HAL_CHECK(hal::write_then_read<6>(
    *m_i2c,
    m_address,
    std::array<hal::byte, 1>{ icm20948_ext_slv_sens_data_00 },
    hal::never_timeout()));

  x = static_cast<int16_t>((data[0]) << 8) | data[1];
  y = static_cast<int16_t>((data[2]) << 8) | data[3];
  z = static_cast<int16_t>((data[4]) << 8) | data[5];

  mag_read.x = x * ak09916_mag_lsb;
  mag_read.y = y * ak09916_mag_lsb;
  mag_read.z = z * ak09916_mag_lsb;

  return mag_read;
}

hal::result<icm20948::temp_read_t> icm20948::read_temperature()
{
  std::array<hal::byte, 2> data{};
  temp_read_t temp_read;

  switch_bank(0);
  data = HAL_CHECK(
    hal::write_then_read<2>(*m_i2c,
                            m_address,
                            std::array<hal::byte, 1>{ icm20948_temp_out },
                            hal::never_timeout()));
  int16_t rawTemp = static_cast<int16_t>(((data[0]) << 8) | data[1]);
  temp_read.temp =
    (rawTemp * 1.0 - icm20948_room_temp_offset) / icm20948_t_sensitivity + 21.0;
  return temp_read;
}

/********* Power, Sleep, Standby *********/

hal::status icm20948::enable_cycle(icm20948_cycle p_cycle)
{
  m_reg_val = HAL_CHECK(read_register8(0, icm20948_lp_config));
  m_reg_val &= 0x0F;
  m_reg_val |= p_cycle;

  HAL_CHECK(write_register8(0, icm20948_lp_config, m_reg_val));
  return hal::success();
}

hal::status icm20948::enable_low_power(bool p_enLP)
{
  m_reg_val = HAL_CHECK(read_register8(0, icm20948_pwr_mgmt_1));
  if (p_enLP) {
    m_reg_val |= icm20948_lp_en;
  } else {
    m_reg_val &= ~icm20948_lp_en;
  }
  HAL_CHECK(write_register8(0, icm20948_pwr_mgmt_1, m_reg_val));
  return hal::success();
}

hal::status icm20948::set_gyr_averg_cycle_mode(icm20948_gyro_avg_low_pwr p_avg)
{
  HAL_CHECK(write_register8(2, icm20948_gyro_config_2, p_avg));
  return hal::success();
}

hal::status icm20948::set_acc_averg_cycle_mode(icm20948_acc_avg_low_pwr p_avg)
{
  HAL_CHECK(write_register8(2, icm20948_accel_config_2, p_avg));
  return hal::success();
}

hal::status icm20948::sleep(bool p_sleep)
{

  if (p_sleep) {
    m_reg_val |= icm20948_sleep;
  } else {
    m_reg_val &= ~icm20948_sleep;
  }
  HAL_CHECK(write_register8(0, icm20948_pwr_mgmt_1, m_reg_val));
  return hal::success();
}

/************** Magnetometer **************/

hal::status icm20948::init_mag()
{
  enable_i2c_host();
  reset_mag();
  // reset_icm20948();
  // sleep(false);
  // write_register8(2, ICM20948_ODR_ALIGN_EN, 1);  // aligns ODR
  enable_i2c_host();

  // auto whoAmI = HAL_CHECK(whoami_mag());
  // if (!((whoAmI == AK09916_WHO_AM_I_1) || (whoAmI == AK09916_WHO_AM_I_2))) {
  //   return hal::new_error();
  // }

  return hal::success();
}

hal::result<hal::byte> icm20948::whoami_mag()
{
  return HAL_CHECK(read_ak09916_register16(ak09916_wia_2));
}

void icm20948::set_mag_op_mode(ak09916_op_mode p_op_mode)
{
  write_ak09916_register8(ak09916_cntl_2, p_op_mode);
  if (p_op_mode != ak09916_pwr_down) {
    enable_mag_data_read(ak09916_hxl, 0x08);
  }
}

void icm20948::reset_mag()
{
  write_ak09916_register8(ak09916_cntl_3, 0x01);
}

/************************************************
     Private Functions
*************************************************/

hal::status icm20948::set_clock_auto_select()
{
  m_reg_val = HAL_CHECK(read_register8(0, icm20948_pwr_mgmt_1));
  m_reg_val |= 0x01;
  HAL_CHECK(write_register8(0, icm20948_pwr_mgmt_1, m_reg_val));
  return hal::success();
}

hal::status icm20948::switch_bank(hal::byte p_newBank)
{
  if (p_newBank != m_currentBank) {
    m_currentBank = p_newBank;
    m_currentBank = m_currentBank << 4;
  }
  auto reg_buffer = HAL_CHECK(
    hal::write_then_read<1>(*m_i2c,
                            m_address,
                            std::array<hal::byte, 1>{ icm20948_reg_bank_sel },
                            hal::never_timeout()));

  hal::byte reg_val = reg_buffer[0];
  HAL_CHECK(hal::write(*m_i2c,
                       m_address,
                       std::array<hal::byte, 2>{ reg_val, m_currentBank },
                       hal::never_timeout()));

  return hal::success();
}

hal::status icm20948::write_register8(hal::byte p_bank,
                                      hal::byte p_reg_addr,
                                      hal::byte p_val)
{
  switch_bank(p_bank);
  HAL_CHECK(hal::write(*m_i2c,
                       m_address,
                       std::array<hal::byte, 2>{ p_reg_addr, p_val },
                       hal::never_timeout()));
  return hal::success();
}

hal::status icm20948::write_register16(hal::byte p_bank,
                                       hal::byte p_reg,
                                       int16_t p_val)
{
  switch_bank(p_bank);
  int8_t MSByte = static_cast<int8_t>((p_val >> 8) & 0xFF);
  hal::byte LSByte = p_val & 0xFF;

  HAL_CHECK(hal::write(*m_i2c,
                       m_address,
                       std::array<hal::byte, 3>{ p_reg, MSByte, LSByte },
                       hal::never_timeout()));

  return hal::success();
}

hal::result<hal::byte> icm20948::read_register8(hal::byte p_bank,
                                                hal::byte p_read_reg)
{
  switch_bank(p_bank);
  auto ctrl_buffer =
    HAL_CHECK(hal::write_then_read<1>(*m_i2c,
                                      m_address,
                                      std::array<hal::byte, 1>{ p_read_reg },
                                      hal::never_timeout()));
  return ctrl_buffer[0];
}

hal::result<hal::byte> icm20948::read_register16(hal::byte p_bank,
                                                 hal::byte p_reg)
{

  switch_bank(p_bank);
  // hal::byte MSByte = 0, LSByte = 0;
  hal::byte reg16Val = 0;

  auto MSByte =
    HAL_CHECK(hal::write_then_read<1>(*m_i2c,
                                      m_address,
                                      std::array<hal::byte, 1>{ p_reg },
                                      hal::never_timeout()));
  auto LSByte =
    HAL_CHECK(hal::write_then_read<1>(*m_i2c,
                                      m_address,
                                      std::array<hal::byte, 1>{ p_reg },
                                      hal::never_timeout()));

  reg16Val = (MSByte[0] << 8) + LSByte[1];
  return reg16Val;
}

hal::status icm20948::write_ak09916_register8(hal::byte p_reg, hal::byte p_val)
{

  write_register8(3, icm20948_i2c_slv0_addr, ak09916_address);  // write AK09916
  write_register8(3,
                  icm20948_i2c_slv0_reg,
                  p_reg);  // define AK09916 register to be written to
  write_register8(3, icm20948_i2c_slv0_do, p_val);

  return hal::success();
}

hal::result<hal::byte> icm20948::read_ak09916_register8(hal::byte p_reg)
{
  enable_mag_data_read(p_reg, 0x01);
  enable_mag_data_read(ak09916_hxl, 0x08);
  m_reg_val = HAL_CHECK(read_register8(0, icm20948_ext_slv_sens_data_00));
  return m_reg_val;
}

hal::result<int16_t> icm20948::read_ak09916_register16(hal::byte p_reg)
{
  int16_t m_reg_value = 0;
  enable_mag_data_read(p_reg, 0x02);
  m_reg_value = HAL_CHECK(read_register16(0, icm20948_ext_slv_sens_data_00));
  enable_mag_data_read(ak09916_hxl, 0x08);
  return m_reg_value;
}

hal::status icm20948::reset_icm20948()
{
  HAL_CHECK(write_register8(0, icm20948_pwr_mgmt_1, icm20948_reset));
  return hal::success();
}

hal::status icm20948::enable_i2c_host()
{
  HAL_CHECK(write_register8(
    0, icm20948_user_ctrl, icm20948_i2c_mst_en));  // enable I2C master
  HAL_CHECK(write_register8(
    3, icm20948_i2c_mst_ctrl, 0x07));  // set I2C clock to 345.60 kHz

  return hal::success();
}

hal::status icm20948::enable_mag_data_read(hal::byte p_reg, hal::byte p_bytes)
{
  HAL_CHECK(write_register8(3,
                            icm20948_i2c_slv0_addr,
                            ak09916_address | ak09916_read));  // read AK09916
  HAL_CHECK(write_register8(
    3, icm20948_i2c_slv0_reg, p_reg));  // define AK09916 register to be read
  HAL_CHECK(write_register8(3,
                            icm20948_i2c_slv0_ctrl,
                            0x80 | p_bytes));  // enable read | number of byte

  return hal::success();
}

}  // namespace hal::icm