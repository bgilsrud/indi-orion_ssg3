/**
 * Orion StarShoot G3 driver
 * This code was reverse engineered by inspecting USB traffic.
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

#include <errno.h>
#include <libusb.h>
#include <stdint.h>
#include "orion_ssg3.h"

#define ORION_SSG3_VID 0x07ee
#define ORION_SSG3_PID 0x0502
#define ORION_SSG3_INTERFACE_NUM 0

/* These are the defaults that Orion Camera Studio sets */
#define ORION_SSG3_DEFAULT_OFFSET 127
#define ORION_SSG3_DEFAULT_GAIN 185
#define ORION_SSG3_DEFAULT_BINNING 1

#define ICX419_EFFECTIVE_X_START 3
#define ICX419_EFFECTIVE_X_COUNT 752
#define ICX419_EFFECTIVE_Y_START 12
#define ICX419_EFFECTIVE_Y_COUNT 582

enum {
    SSG3_CMD_BINNING = 13,
    SSG3_CMD_START_EXPOSURE = 16,
    SSG3_CMD_COOLER = 22,
    SSG3_CMD_GAIN_OFFSET = 25,
    SSG3_CMD_X_READOUT_START = 31,
    SSG3_CMD_X_READOUT_END = 32,
    SSG3_CMD_Y_READOUT_START = 33,
    SSG3_CMD_Y_READOUT_END = 34,
};

static int orion_ssg3_set_gain_offset(struct orion_ssg3 *ssg3, uint8_t gain,
        uint8_t offset);

/**
 * Convert libusb error codes to standard errno
 * @param lu_err: The libusb error code to be translated
 * @return: The corresponding errno
 */
static int libusb_to_errno(int lu_err)
{
    int rc = ENOTTY;

    switch (lu_err) {
    case LIBUSB_ERROR_IO:
        rc = EIO;
        break;
    case LIBUSB_ERROR_INVALID_PARAM:
        rc = EINVAL;
        break;
    case LIBUSB_ERROR_ACCESS:
        rc = EPERM;
        break;
    case LIBUSB_ERROR_NO_DEVICE:
        rc = ENODEV;
        break;
    case LIBUSB_ERROR_BUSY:
        rc = EBUSY;
        break;
    case LIBUSB_ERROR_TIMEOUT:
        rc = ETIMEDOUT;
        break;
    case LIBUSB_ERROR_OVERFLOW:
        rc = EOVERFLOW;
        break;
    case LIBUSB_ERROR_PIPE:
        rc = EPIPE;
        break;
    case LIBUSB_ERROR_INTERRUPTED:
        rc = EINTR;
        break;
    case LIBUSB_ERROR_NO_MEM:
        rc = ENOMEM;
        break;
    case LIBUSB_ERROR_NOT_SUPPORTED:
        rc = ENOMEM;
        break;
    }

    return rc;
}

/**
 * Open a connection to an Orion StarShoot G3 device.
 * @param ssg3: A orion_ssg3 struct that will be used to handle the connection
 * @return: 0 on success, -errno on failure
 */
int orion_ssg3_open(struct orion_ssg3 *ssg3)
{
    int rc;
    int cnt;
    int i;
    libusb_device_list **list;
    libusb_device *dev;
    libusb_device_descriptor desc;
    int found = 0;

    libusb_init(NULL);

    rc = libusb_get_device_list(NULL, &list);
    if (rc < 0) {
        return rc;
    }
    cnt = rc;

    for (i = 0; i < cnt; i++) {
        dev = list[i];
        rc = libusb_get_device_descriptor(dev, &desc);
        if (rc) {
            continue;
        }
        if ((desc.idVendor == ORION_SSG3_VID) &&
            (desc.idProduct == ORION_SSG3_PID)) {
            found = 1;
            break;
        }
    }

    if (!found) {
        return -ENODEV;
    }

	rc = libusb_open(dev, &ssg3->devh);
	if (rc) {
		return -libusb_to_errno(rc);
	}
    rc = libusb_claim_interface(ssg3->devh, ORION_SSG3_INTERFACE_NUM);
	if (rc) {
		libusb_close(ssg3->devh);
		return -libusb_to_errno(rc);
	}

    /* Set defaults since we don't know how to read them from the camera */
    ssg3->offset = ORION_SSG3_DEFAULT_OFFSET;
    ssg3->gain = ORION_SSG3_DEFAULT_GAIN;
    ssg3->bin_x = ORION_SSG3_DEFAULT_BINNING;
    ssg3->bin_y = ORION_SSG3_DEFAULT_BINNING;
    ssg3->subframe_x1 = ICX419_EFFECTIVE_X_START;
    ssg3->subframe_x2 = ICX419_EFFECTIVE_X_START + ICX419_EFFECTIVE_X_COUNT - 1;
    ssg3->subframe_y1 = ICX419_EFFECTIVE_Y_START;
    ssg3->subframe_y2 = ICX419_EFFECTIVE_Y_START + ICX419_EFFECTIVE_Y_COUNT - 1;

    rc = orion_ssg3_set_gain_offset(ssg3, ssg3->gain, ssg3->offset);
    if (rc) {
        return rc;
    }

    rc = orion_ssg3_set_binning(ssg3, ssg3->x, ssg3->y);
    if (rc) {
        return rc;
    }
    return 0;
}

/**
 * Close a connection to an Orion StarShoot G3 device.
 * @param ssg3: A orion_ssg3 struct that was used to open a connection
 * @return: 0 on success, -errno on failure
 */
int orion_ssg3_close(struct orion_ssg3 *ssg3)
{
	int rc;

    libusb_release_interface(ssg3->devh, ORION_SSG3_INTERFACE_NUM);
	rc = libusb_close(ssg3->devh);
	if (rc) {
		return -libusb_to_errno(rc);
	}

	return 0;
}

/**
 * Helper function for sending commands to Orion SSG3.
 * All of the command/control of the SSG3 is done using control transfers.
 */
static int orion_ssg3_control_set(struct orion_ssg3 *ssg3, uint16_t cmd, uint16_t wValue, uint16_t wIndex)
{
    int rc;
    uint8_t bmRequestType;

    bmRequestType = LIBUSB_RECIPIENT_DEVICE | LIBUSB_REQUEST_TYPE_RESERVED
                    | LIBUSB_ENDPOINT_OUT;
    rc = libusb_control_transfer(bmRequestType, cmd, wValue, wIndex, NULL, 0, 10);

    if (rc) {
        return -libusb_to_errno(rc);
    }

    return 0;
}

/**
 * Helper function for sending commands to Orion SSG3.
 * All of the command/control of the SSG3 is done using control transfers.
 */
static int orion_ssg3_control_get(struct orion_ssg3 *ssg3, uint16_t cmd,
        uint16_t wValue, uint16_t wIndex, uint8_t *data, uint16_t wLength)
{
    int rc;
    uint8_t bmRequestType;

    bmRequestType = LIBUSB_RECIPIENT_DEVICE | LIBUSB_REQUEST_TYPE_RESERVED
                    | LIBUSB_ENDPOINT_IN;
    rc = libusb_control_transfer(bmrequesttype, cmd, wValue, wIndex, data, 0, 10);

    if (rc) {
        return -libusb_to_errno(rc);
    }

    return 0;
}

/* This was observed via usb traffic dumps, though I don't know what it means */
#define SSG3_COOLER_WVALUE 0x003b

int orion_ssg3_set_cooling(struct orion_ssg3 *ssg3, int on)
{
    return orion_ssg3_control_set(ssg3, SSG3_CMD_COOLER, SSG3_COOLER_WVALUE,
            on ? 1 : 0);
}

/**
 * Set the gain and offset.
 * I would like to be able to set gain or offset independently, but none of the
 * usb dumps show a read transaction for these values. So, instead, we manually
 * track the gain and offset in the ssg3 structure.
 * @param ssg3: The ssg3 structure that handles the connection to the device
 * @param gain: The gain to be set. This is 0-255
 * @param offset: The gain to be set. This is 0-255
 * @return: 0 on success, -errno on failure
 */
static int orion_ssg3_set_gain_offset(struct orion_ssg3 *ssg3, uint8_t gain,
        uint8_t offset)
{
    int rc;

    rc = orion_ssg3_control_set(ssg3, SSG3_CMD_GAIN_OFFSET, (gain << 8) | offset, 2);
    if (!rc) {
        ssg3->offset = offset;
        ssg3->gain = gain;
    }

    return rc;
}

int orion_ssg3_set_gain(struct orion_ssg3 *ssg3, uint8_t gain)
{
    return orion_ssg3_set_gain_offset(ssg3, gain, ssg3->offset);
}

int orion_ssg3_set_offset(struct orion_ssg3 *ssg3, uint8_t offset)
{
    return orion_ssg3_set_gain_offset(ssg3, ssg3->gain, offset);
}

/**
 * Set the CCD binning
 * OCS allows for any combination of 1/2 binning (1x1, 1x2, 2x1, 2x2). I don't
 * know how the x/y bin is set, but I'm guessing that x is sent in the MSB. It
 * would be easy to confirm with an additional packet capture.
 * @param ssg3: The ssg3 structure used to control the camera
 * @param x: The number of pixels to bin x
 * @param y: The number of pixels to bin y
 * @return: 0 on success, -errno otherwise
 */
int orion_ssg3_set_binning(struct orion_ssg3 *ssg3, uint8_t x, uint8_t y);
{
    int rc;

    if (x > 2 || y > 2) {
        return -EINVAL;
    }
    rc = orion_ssg3_control_set(ssg3, SSG3_CMD_BINNING, (x << 8) | y, 0);
    if (!rc) {
        ssg3->bin_x = x;
        ssg3->bin_y = y;
    }

    return rc;
}

/**
 * Start an exposure on the camera.
 * This sequence mimics what Orion Camera Studio does. For each exposure, OCS
 * sends a sequence of commands that specify the offsets at which the data that
 * is clocked out of the CCD should be sent over the USB. This would allow the
 * same firmware to be used for CCDs that have different geometries without any
 * changes.
 */
int orion_ssg3_start_exposure(struct orion_ssg3 *ssg3, uint32_t msec)
{
    return orion_ssg3_control_set(ssg3, SSG3_CMD_START_EXPOSURE, msec, msec >> 16);
}
