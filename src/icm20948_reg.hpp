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

#include <libhal-util/bit.hpp>
#include <libhal/units.hpp>

namespace hal::icm {

static constexpr hal::byte icm20948_address = 0x69;
static constexpr hal::byte ak09916_address = 0x0C;

/* Registers ICM20948 USER BANK 0*/
static constexpr hal::byte icm20948_who_am_i = 0x00;
static constexpr hal::byte icm20948_user_ctrl = 0x03;
static constexpr hal::byte icm20948_lp_config = 0x05;
static constexpr hal::byte icm20948_pwr_mgmt_1 = 0x06;
static constexpr hal::byte icm20948_pwr_mgmt_2 = 0x07;
static constexpr hal::byte icm20948_int_pin_cfg = 0x0F;
static constexpr hal::byte icm20948_int_enable = 0x10;
static constexpr hal::byte icm20948_int_enable_1 = 0x11;
static constexpr hal::byte icm20948_int_enable_2 = 0x12;
static constexpr hal::byte icm20948_int_enable_3 = 0x13;
static constexpr hal::byte icm20948_i2c_mst_status = 0x17;
static constexpr hal::byte icm20948_int_status = 0x19;
static constexpr hal::byte icm20948_int_status_1 = 0x1A;
static constexpr hal::byte icm20948_int_status_2 = 0x1B;
static constexpr hal::byte icm20948_int_status_3 = 0x1C;
static constexpr hal::byte icm20948_delay_time_h = 0x28;
static constexpr hal::byte icm20948_delay_time_l = 0x29;
static constexpr hal::byte icm20948_accel_out =
  0x2D;  // accel data registers begin
static constexpr hal::byte icm20948_gyro_out =
  0x33;  // gyro data registers begin
static constexpr hal::byte icm20948_temp_out = 0x39;
static constexpr hal::byte icm20948_ext_slv_sens_data_00 = 0x3B;
static constexpr hal::byte icm20948_ext_slv_sens_data_01 = 0x3C;
static constexpr hal::byte icm20948_fifo_en_1 = 0x66;
static constexpr hal::byte icm20948_fifo_en_2 = 0x67;
static constexpr hal::byte icm20948_fifo_rst = 0x68;
static constexpr hal::byte icm20948_fifo_mode = 0x69;
static constexpr hal::byte icm20948_fifo_count = 0x70;
static constexpr hal::byte icm20948_fifo_r_w = 0x72;
static constexpr hal::byte icm20948_data_rdy_status = 0x74;
static constexpr hal::byte icm20948_fifo_cfg = 0x76;

/* Registers ICM20948 USER BANK 1*/
static constexpr hal::byte icm20948_self_test_x_gyro = 0x02;
static constexpr hal::byte icm20948_self_test_y_gyro = 0x03;
static constexpr hal::byte icm20948_self_test_z_gyro = 0x04;
static constexpr hal::byte icm20948_self_test_x_accel = 0x0E;
static constexpr hal::byte icm20948_self_test_y_accel = 0x0F;
static constexpr hal::byte icm20948_self_test_z_accel = 0x10;
static constexpr hal::byte icm20948_xa_offs_h = 0x14;
static constexpr hal::byte icm20948_xa_offs_l = 0x15;
static constexpr hal::byte icm20948_ya_offs_h = 0x17;
static constexpr hal::byte icm20948_ya_offs_l = 0x18;
static constexpr hal::byte icm20948_za_offs_h = 0x1A;
static constexpr hal::byte icm20948_za_offs_l = 0x1B;
static constexpr hal::byte icm20948_timebase_corr_pll = 0x28;

/* Registers ICM20948 USER BANK 2*/
static constexpr hal::byte icm20948_gyro_smplrt_div = 0x00;
static constexpr hal::byte icm20948_gyro_config_1 = 0x01;
static constexpr hal::byte icm20948_gyro_config_2 = 0x02;
static constexpr hal::byte icm20948_xg_offs_usrh = 0x03;
static constexpr hal::byte icm20948_xg_offs_usrl = 0x04;
static constexpr hal::byte icm20948_yg_offs_usrh = 0x05;
static constexpr hal::byte icm20948_yg_offs_usrl = 0x06;
static constexpr hal::byte icm20948_zg_offs_usrh = 0x07;
static constexpr hal::byte icm20948_zg_offs_usrl = 0x08;
static constexpr hal::byte icm20948_odr_align_en = 0x09;
static constexpr hal::byte icm20948_accel_smplrt_div_1 = 0x10;
static constexpr hal::byte icm20948_accel_smplrt_div_2 = 0x11;
static constexpr hal::byte icm20948_accel_intel_ctrl = 0x12;
static constexpr hal::byte icm20948_accel_wom_thr = 0x13;
static constexpr hal::byte icm20948_accel_config = 0x14;
static constexpr hal::byte icm20948_accel_config_2 = 0x15;
static constexpr hal::byte icm20948_fsync_config = 0x52;
static constexpr hal::byte icm20948_temp_config = 0x53;
static constexpr hal::byte icm20948_mod_ctrl_usr = 0x54;

/* Registers ICM20948 USER BANK 3*/
static constexpr hal::byte icm20948_i2c_mst_odr_cfg = 0x00;
static constexpr hal::byte icm20948_i2c_mst_ctrl = 0x01;
static constexpr hal::byte icm20948_i2c_mst_delay_ctrl = 0x02;
static constexpr hal::byte icm20948_i2c_slv0_addr = 0x03;
static constexpr hal::byte icm20948_i2c_slv0_reg = 0x04;
static constexpr hal::byte icm20948_i2c_slv0_ctrl = 0x05;
static constexpr hal::byte icm20948_i2c_slv0_do = 0x06;

/* Registers ICM20948 ALL BANKS */
static constexpr hal::byte icm20948_reg_bank_sel = 0x7F;

/* Registers AK09916 */
static constexpr hal::byte ak09916_wia_1 = 0x00;  // Who I am, Company ID
static constexpr hal::byte ak09916_wia_2 = 0x01;  // Who I am, Device ID
static constexpr hal::byte ak09916_status_1 = 0x10;
static constexpr hal::byte ak09916_hxl = 0x11;
static constexpr hal::byte ak09916_hxh = 0x12;
static constexpr hal::byte ak09916_hyl = 0x13;
static constexpr hal::byte ak09916_hyh = 0x14;
static constexpr hal::byte ak09916_hzl = 0x15;
static constexpr hal::byte ak09916_hzh = 0x16;
static constexpr hal::byte ak09916_status_2 = 0x18;
static constexpr hal::byte ak09916_cntl_2 = 0x31;
static constexpr hal::byte ak09916_cntl_3 = 0x32;

/* Register Bits */
static constexpr hal::byte icm20948_reset = 0x41;
static constexpr hal::byte icm20948_i2c_mst_en = 0x20;
static constexpr hal::byte icm20948_sleep = 0x40;
static constexpr hal::byte icm20948_lp_en = 0x20;
static constexpr hal::byte icm20948_bypass_en = 0x02;
static constexpr hal::byte icm20948_gyro_en = 0x07;
static constexpr hal::byte icm20948_acc_en = 0x38;
static constexpr hal::byte icm20948_fifo_en = 0x40;
static constexpr hal::byte icm20948_int1_actl = 0x80;
static constexpr hal::byte icm20948_int_1_latch_en = 0x20;
static constexpr hal::byte icm20948_actl_fsync = 0x08;
static constexpr hal::byte icm20948_int_anyrd_2clear = 0x10;
static constexpr hal::byte icm20948_fsync_int_mode_en = 0x06;
static constexpr hal::byte ak09916_16_bit = 0x10;
static constexpr hal::byte ak09916_ovf = 0x08;
static constexpr hal::byte ak09916_read = 0x80;

static constexpr uint16_t ak09916_who_am_i_1 = 0x48;
static constexpr uint16_t ak09916_who_am_i_2 = 0x09;

static constexpr hal::byte icm20948_who_am_i_content = 0xEA;
static constexpr float icm20948_room_temp_offset = 0.0;
static constexpr float icm20948_t_sensitivity = 333.87;
static constexpr float ak09916_mag_lsb = 0.1495;

}  // namespace hal::icm