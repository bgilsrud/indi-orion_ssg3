/**
 * Orion StarShoot G3 driver
 *
 * Copyright (c) 2020 Ben Gilsrud (bgilsrud@gmail.com)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>. 
 */

#ifndef ORION_SSG3_H
#define ORION_SSG3_H
#include <stdint.h>

struct orion_ssg3 {
    libusb_device_handle *devh;
    uint8_t gain;
    uint8_t offset;
    uint16_t binning;
};

enum {
    SSG3_BINNING_1x1 = 0x0101,
    SSG3_BINNING_2x2 = 0x0202,
};

int orion_ssg3_open(struct orion_ssg3 &ssg3);
int orion_ssg3_close(struct orion_ssg3 *ssg3);

int orion_ssg3_set_cooling(struct orion_ssg3 *ssg3, int on);
int orion_ssg3_set_gain(struct orion_ssg3 *ssg3, uint8_t gain);
int orion_ssg3_set_offset(struct orion_ssg3 *ssg3, uint8_t offset);
int orion_ssg3_set_binning(struct orion_ssg3 *ssg3, uint16_t bin);
int orion_ssg3_start_exposure(struct orion_ssg3 *ssg3, uint32_t msec);
#endif /* ORION_SSG3_H */
