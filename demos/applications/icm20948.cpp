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
#include <cmath>

#define M_PI 3.14159265358979323846

float computeHeading(float x, float y, float offset = 0.0) { 
    float angle = atan2(y, x) * (180.0 / M_PI);  // Convert from radians to degrees
    angle += offset;  // Apply offset
    if (angle < 0) {
        angle += 360;
    } else if (angle >= 360) {
        angle -= 360;
    }
    return angle;
}



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
  (void)hal::delay(clock, 200ms);
  icm_device.init_mag();
  (void)hal::delay(clock, 100ms);
  icm_device.auto_offsets();

  while (true) {


    auto mag_status1 = HAL_CHECK(icm_device.mag_status1());
    auto mag_status2 = HAL_CHECK(icm_device.mag_status2());
    // (void)hal::delay(clock, 500ms);
    auto accel = HAL_CHECK(icm_device.read_acceleration());
    (void)hal::delay(clock, 10ms);
    auto gyro = HAL_CHECK(icm_device.read_gyroscope());
    (void)hal::delay(clock, 10ms);
    auto temp = HAL_CHECK(icm_device.read_temperature());
    (void)hal::delay(clock, 10ms);
    auto mag = HAL_CHECK(icm_device.read_magnetometer());
    (void)hal::delay(clock, 10ms);
    hal::print(console, "\n\n================Reading IMU================\n");

    hal::print<128>(console,"\n\nMag Status 1:      %x", mag_status1);

    hal::print<128>(console, "\n\nMag Status 2:      %x", mag_status2);

    hal::print<128>(console,
                    "\n\nG-Accel Values:    x = %fg, y = %fg, z = %fg",
                    accel.x,
                    accel.y,
                    accel.z);


    hal::print<128>(console,
                    "\n\nGyro Values:       x = %f,  y = %f,  z = %f",
                    gyro.x,
                    gyro.y,
                    gyro.z);

    hal::print<128>(console, "\n\nCurrent Temperature: %f°C", temp.temp);


    hal::print<128>(console,
                    "\n\nMagnetometer Values: x = %f, y = %f, z = %f",
                    mag.x,
                    mag.y,
                    mag.z);

    float heading = computeHeading(-mag.x, mag.y, 0.0);
    hal::print<128>(console, "\n\nHeading: %f°", heading);


    hal::print(console, "\n\n===========================================\n");
  }
  return hal::success();
}