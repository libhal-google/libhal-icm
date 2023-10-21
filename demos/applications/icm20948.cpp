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

#include "../hardware_map.hpp"
#include <libhal-icm/icm20948.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>

hal::status application(hardware_map& p_map)
{
  using namespace std::chrono_literals;
  using namespace hal::literals;

  auto& clock = *p_map.clock;
  auto& console = *p_map.console;
  auto& i2c = *p_map.i2c;

  hal::print(console, "icm Application Starting...\n\n");
  (void)hal::delay(clock, 200ms);
  auto icm_device = HAL_CHECK(hal::icm::icm20948::create(i2c, 0x69));
  hal::print(console, "icm Device Created\n\n");
  (void)hal::delay(clock, 200ms);
  icm_device.init_mag();
  (void)hal::delay(clock, 100ms);
  icm_device.reset_mag();
  (void)hal::delay(clock, 100ms);
  hal::print(console, "passed mag init\n\n");
  (void)hal::delay(clock, 100ms);
  icm_device.auto_offsets();
  hal::print(console, "passed offset\n\n");



  auto who_am_i_wia1 = HAL_CHECK(icm_device.whoami_ak09916_wia1_direct());
  auto who_am_i_wia2 = HAL_CHECK(icm_device.whoami_ak09916_wia2_direct());

  hal::print<128>(console, "Who am I Mag1: %x\n", who_am_i_wia1);
  hal::print<128>(console, "Who am I Mag2: %x\n", who_am_i_wia2);

  if (who_am_i_wia1 != 0x48 || who_am_i_wia2 != 0x09) {
    hal::print(console, "Who am I Mag Failed\n");
  }else{
    hal::print(console, "Who am I Mag Passed\n");
  }

  auto mag_status = HAL_CHECK(icm_device.mag_status());
  auto mag_mode = HAL_CHECK(icm_device.check_mag_mode());
  hal::print<128>(console, "Mag Status: %x\n", mag_status);
  hal::print<128>(console, "Mag Mode: %x\n", mag_mode);



  while (true) {
    hal::print(console, "\n\n================Reading IMU================\n");
    
    (void)hal::delay(clock, 500ms);
    hal::print<128>(console, "Mag Status: %x\n", mag_status);
    hal::print<128>(console, "Mag Mode: %x\n", mag_mode);
    (void)hal::delay(clock, 100ms);
    auto mag = HAL_CHECK(icm_device.read_magnetometer());
    (void)hal::delay(clock, 200ms);

    

    hal::print<128>(console,
                    "\n\nMagnetometer Values: x = %f, y = %f, z = %f",
                    mag.x,
                    mag.y,
                    mag.z);

    hal::print(console, "\n\n===========================================\n");
  }
  return hal::success();
}