/*
Copyright (c) 2013, Michael Ossmann <mike@ossmann.com>
Copyright (c) 2012, Jared Boone <jared@sharebrained.com>
Copyright (c) 2014, Youssef Touil <youssef@airspy.com>
Copyright (c) 2014, Benjamin Vernoux <bvernoux@airspy.com>
Copyright (c) 2015, Ian Gilmour <ian@sdrsharp.com>
Copyright (c) 2021, Malcolm Robb <support@attavionics.com>

All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

		Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
		Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the
		documentation and/or other materials provided with the distribution.
		Neither the name of AirSpy nor the names of its contributors may be used to endorse or promote products derived from this software
		without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "airspy1090.h"

#if (_WIN32) && (_MSC_VER < 1400)
uint64_t strtoull( char * p, char** pEnd, int iLen) {
    return (0);
}
#define SERIAL_NUMBER_UNUSED (0UI64)

#else

#define SERIAL_NUMBER_UNUSED ((uint64_t) (0))

#endif

#define USB_PRODUCT_ID      (2)
#define STR_DESCRIPTOR_SIZE (250)
#define AIRSPY_SAMPLERATE_20MSPS (20000)

#define STR_PREFIX_SERIAL_AIRSPY_SIZE (10)
//static const char str_prefix_serial_airspy[STR_PREFIX_SERIAL_AIRSPY_SIZE] =
//{ 'A', 'I', 'R', 'S', 'P', 'Y', ' ', 'S', 'N', ':' };

#define SERIAL_AIRSPY_EXPECTED_SIZE (26)

#define GAIN_COUNT (22)

uint8_t airspy_linearity_vga_gains[GAIN_COUNT] = { 13, 12, 11, 11, 11, 11, 11, 10, 10, 10, 10, 10, 10, 10, 10, 10, 9, 8, 7, 6, 5, 4 };
uint8_t airspy_linearity_mixer_gains[GAIN_COUNT] = { 12, 12, 11, 9, 8, 7, 6, 6, 5, 0, 0, 1, 0, 0, 2, 2, 1, 1, 1, 1, 0, 0 };
uint8_t airspy_linearity_lna_gains[GAIN_COUNT] = { 14, 14, 14, 13, 12, 10, 9, 9, 8, 9, 8, 6, 5, 3, 1, 0, 0, 0, 0, 0, 0, 0 };
//
//=========================================================================
//
static int cancel_transfers(airspy_device_t* device) {

	uint32_t transfer_index;

	if (device->transfers != NULL) {

		for (transfer_index = 0; transfer_index < MODES_ASYNC_BUF_NUMBER; transfer_index++) {

			if (device->transfers[transfer_index] != NULL)
				{libusb_cancel_transfer(device->transfers[transfer_index]);}
		}
		return (AIRSPY_SUCCESS);
	}

	return (AIRSPY_ERROR_OTHER);
}
//
//=========================================================================
//
static int free_transfers(airspy_device_t* device) {
	uint32_t transfer_index;

	if (device->transfers != NULL) {

		// libusb_close() should free all transfers referenced from this array.
		for (transfer_index = 0; transfer_index < MODES_ASYNC_BUF_NUMBER; transfer_index++) {
			if (device->transfers[transfer_index] != NULL) {
				free(device->transfers[transfer_index]->buffer);
				libusb_free_transfer(device->transfers[transfer_index]);
				device->transfers[transfer_index] = NULL;
			}
		}
		free(device->transfers);
		device->transfers = NULL;
	}

	return (AIRSPY_SUCCESS);
}
//
//=========================================================================
//
static int allocate_transfers(airspy_device_t* const device) {
	uint32_t transfer_index;

	if (device->transfers == NULL) {
		device->transfers = (struct libusb_transfer**) calloc(MODES_ASYNC_BUF_NUMBER, sizeof(struct libusb_transfer));
		if (device->transfers == NULL)
		    {return (AIRSPY_ERROR_NO_MEM);}

		for (transfer_index = 0; transfer_index < MODES_ASYNC_BUF_NUMBER; transfer_index++) {

			device->transfers[transfer_index] = libusb_alloc_transfer(0);
			if (device->transfers[transfer_index] == NULL)
			    {return (AIRSPY_ERROR_LIBUSB);}

			libusb_fill_bulk_transfer(
				device->transfers[transfer_index],
				device->usb_device,
				0,
				(unsigned char*)malloc(MODES_ASYNC_BUF_SIZE),
				MODES_ASYNC_BUF_SIZE,
				NULL,
				device,
				0
				);

			if (device->transfers[transfer_index]->buffer == NULL)
			    {return (AIRSPY_ERROR_NO_MEM);}
		}
		return (AIRSPY_SUCCESS);

	}
	return (AIRSPY_ERROR_BUSY);
}
//
//=========================================================================
//
static int prepare_transfers(airspy_device_t* device, const uint_fast8_t endpoint_address, libusb_transfer_cb_fn callback) {

	int error;
	uint32_t transfer_index;

	if (device->transfers != NULL) {
		for (transfer_index = 0; transfer_index < MODES_ASYNC_BUF_NUMBER; transfer_index++) {
			device->transfers[transfer_index]->endpoint = endpoint_address;
			device->transfers[transfer_index]->callback = callback;

			error = libusb_submit_transfer(device->transfers[transfer_index]);
			if (error != 0)
				{return (AIRSPY_ERROR_LIBUSB);}
		}
		return (AIRSPY_SUCCESS);
	}

	// This shouldn't happen.
	return (AIRSPY_ERROR_OTHER);
}
//
//=========================================================================
//
// This is the callback from libusb giving us a buffer full of new data.
// Add the buffer to the buffer queue to be processed, and signal the
// IF_thread to deal with it.
//
static void airspy_libusb_transfer_callback(struct libusb_transfer* usb_transfer) {

	uint32_t  uIF_In;
	uint16_t* pTemp;
	airspy_device_t* device = (airspy_device_t*)usb_transfer->user_data;

	if ( (!device->streaming) || (Modes.exit) ) {
		return;
	}

	if ( (usb_transfer->status        == LIBUSB_TRANSFER_COMPLETED) 
      && (usb_transfer->actual_length == usb_transfer->length) ) {

		// Acquire the IF mutex
		pthread_mutex_lock(&Modes.IF_mutex);

		// If there is some space in the IF_Fifo
		if (((uIF_In = Modes.uIF_In) - Modes.uIF_Out) < MODES_ASYNC_BUF_NUMBER) {
			uIF_In &= (MODES_ASYNC_BUF_NUMBER - 1);
			pTemp   = Modes.IF_Fifo[uIF_In].pFifo;

			Modes.IF_Fifo[uIF_In].pFifo = (uint16_t *)usb_transfer->buffer;
			Modes.IF_Fifo[uIF_In].uLost = 0;
			Modes.IF_Fifo[uIF_In].lTime = Modes.timestampBlk;
            Modes.uIF_In++;

			usb_transfer->buffer = (uint8_t*)pTemp;

			// Signal the IF thread there is something to do. It won't
			// execute yet because we still own the IF_mutex
			pthread_cond_signal(&Modes.IF_cond);

		} else { 
			// The IF Fifo is full. The 'newest' FIFO entry is (Modes.uIF_In-1),
			// So increase the lost count for that entry
			uIF_In = (uIF_In - 1) & (MODES_ASYNC_BUF_NUMBER - 1);
			Modes.IF_Fifo[uIF_In].uLost++;
		}
        Modes.timestampBlk += (MODES_ASYNC_BUF_SAMPLES);

		// Release the IF_mutex. This allows the IF thread to run.
		pthread_mutex_unlock(&Modes.IF_mutex);

		if (libusb_submit_transfer(usb_transfer) != 0) {
            device->streaming = FALSE;
        }

	} else {
		device->streaming = FALSE;
	}
}
//
//=========================================================================
//
static void* transfer_threadproc(void* arg) {
	airspy_device_t* device = (airspy_device_t*)arg;
	int error;
	struct timeval timeout = { 0, 500000 };

#ifdef _WIN32
	SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_ABOVE_NORMAL);
#endif

	while (device->streaming && !Modes.exit) {
		error = libusb_handle_events_timeout_completed(device->usb_context, &timeout, NULL);
		if (error < 0) {
			if (error != LIBUSB_ERROR_INTERRUPTED)
				device->streaming = FALSE;
		}
	}

#ifdef _WIN32
	return (NULL);
#else
	pthread_exit(NULL);
#endif
}
//
//=========================================================================
//
static int kill_io_threads(airspy_device_t* device) {
	if (device->streaming) {
		Modes.exit = TRUE;
		cancel_transfers(device);

		pthread_join(Modes.reader_thread, NULL);

		device->streaming = FALSE;
	}
	return (AIRSPY_SUCCESS);
}
//
//=========================================================================
//
static int create_io_threads(airspy_device_t* device) {
	int result;
	pthread_attr_t attr;

	if ( (!device->streaming) && (!Modes.exit) ) {
		device->streaming = TRUE;

		if ((result = prepare_transfers(device, LIBUSB_ENDPOINT_IN | 1, (libusb_transfer_cb_fn)airspy_libusb_transfer_callback)))
		    {return (result);}

		pthread_attr_init(&attr);
		pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);

		if (pthread_create(&Modes.reader_thread, &attr, transfer_threadproc, device))
		    {return (AIRSPY_ERROR_THREAD);}

		pthread_attr_destroy(&attr);

	} else {
		return (AIRSPY_ERROR_BUSY);
	}

	return (AIRSPY_SUCCESS);
}
//
//=========================================================================
//
static void airspy_open_exit(airspy_device_t* device) {
	if (device->usb_device != NULL) {
		libusb_release_interface(device->usb_device, 0);
		libusb_close(device->usb_device);
		device->usb_device = NULL;
	}
	libusb_exit(device->usb_context);
	device->usb_context = NULL;
}
//
//=========================================================================
//
static void airspy_open_device(airspy_device_t* device, int* ret, uint16_t vid, uint16_t pid, uint64_t serial_number_val) {
	int i;
	int result;
	libusb_device_handle** libusb_dev_handle;
	int serial_number_len;
	libusb_device_handle* dev_handle;
	libusb_device *dev;
	libusb_device** devices = NULL;

	ssize_t cnt;
	int serial_descriptor_index;
	struct libusb_device_descriptor device_descriptor;
	unsigned char serial_number[SERIAL_AIRSPY_EXPECTED_SIZE + 1];

	libusb_dev_handle = &device->usb_device;
	*libusb_dev_handle = NULL;

	cnt = libusb_get_device_list(device->usb_context, &devices);
	if (cnt < 0) {
		*ret = AIRSPY_ERROR_NOT_FOUND;
		return;
	}

	i = 0;
	while ((dev = devices[i++]) != NULL) {

		libusb_get_device_descriptor(dev, &device_descriptor);

		if ((device_descriptor.idVendor == vid) &&
			(device_descriptor.idProduct == pid)) {
			if (serial_number_val != SERIAL_NUMBER_UNUSED) {
				serial_descriptor_index = device_descriptor.iSerialNumber;
				if (serial_descriptor_index > 0) {
					if (libusb_open(dev, libusb_dev_handle) != 0) {
						*libusb_dev_handle = NULL;
						continue;
					}
					dev_handle = *libusb_dev_handle;
					serial_number_len = libusb_get_string_descriptor_ascii(dev_handle,
						(uint8_t) serial_descriptor_index,
						serial_number,
						sizeof(serial_number));

					if (serial_number_len == SERIAL_AIRSPY_EXPECTED_SIZE) {
						uint64_t serial = 0;
						// use same code to determine device's serial number as in airspy_list_devices()
						{
							char *start, *end;

							serial_number[SERIAL_AIRSPY_EXPECTED_SIZE] = 0;
							start = (char*)(serial_number + STR_PREFIX_SERIAL_AIRSPY_SIZE);
							end = NULL;
							serial = strtoull(start, &end, 16);
						}

						if (serial == serial_number_val) {
#ifdef __linux__
							/* Check whether a kernel driver is attached to interface #0. If so, we'll
							* need to detach it.
							*/
							if (libusb_kernel_driver_active(dev_handle, 0))
							    {libusb_detach_kernel_driver(dev_handle, 0);}
#endif
							result = libusb_set_configuration(dev_handle, 1);
							if (result != 0) {
								libusb_close(dev_handle);
								*libusb_dev_handle = NULL;
								continue;
							}
							result = libusb_claim_interface(dev_handle, 0);
							if (result != 0) {
								libusb_close(dev_handle);
								*libusb_dev_handle = NULL;
								continue;
							}
							break;
						} else {
							libusb_close(dev_handle);
							*libusb_dev_handle = NULL;
							continue;
						}
					} else {
						libusb_close(dev_handle);
						*libusb_dev_handle = NULL;
						continue;
					}
				}
			} else {
				if (libusb_open(dev, libusb_dev_handle) == 0) {
					dev_handle = *libusb_dev_handle;
#ifdef __linux__
					/* Check whether a kernel driver is attached to interface #0. If so, we'll
					* need to detach it.
					*/
					if (libusb_kernel_driver_active(dev_handle, 0)) {
						libusb_detach_kernel_driver(dev_handle, 0);
					}
#endif
					result = libusb_set_configuration(dev_handle, 1);
					if (result != 0) {
						libusb_close(dev_handle);
						*libusb_dev_handle = NULL;
						continue;
					}
					result = libusb_claim_interface(dev_handle, 0);
					if (result != 0) {
						libusb_close(dev_handle);
						*libusb_dev_handle = NULL;
						continue;
					}
					break;
				}
			}
		}
	}
	libusb_free_device_list(devices, 1);

	dev_handle = device->usb_device;
	if (dev_handle == NULL) {
		*ret = AIRSPY_ERROR_NOT_FOUND;
		return;
	}

	*ret = AIRSPY_SUCCESS;
	return;
}
//
//=========================================================================
//
int airspy_open_sn(airspy_device_t** device, uint64_t serial_number) {

	airspy_device_t* lib_device;
	int libusb_error;
	int result;

	*device = NULL;

	lib_device = (airspy_device_t*)calloc(1, sizeof(airspy_device_t));
	if (lib_device == NULL) 
        {return (AIRSPY_ERROR_NO_MEM);}

	libusb_error = libusb_init(&lib_device->usb_context);
	if (libusb_error != 0) {
		free(lib_device);
		return (AIRSPY_ERROR_LIBUSB);
	}

	airspy_open_device(lib_device, &result, airspy_usb_vid, airspy_usb_pid, serial_number);

	if (lib_device->usb_device == NULL) {
		libusb_exit(lib_device->usb_context);
		free(lib_device);
		return (result);
	}

	lib_device->transfers       = NULL;
	lib_device->streaming       = FALSE;

	result = allocate_transfers(lib_device);
	if (result != 0) {
		airspy_open_exit(lib_device);
		free(lib_device);
		return (AIRSPY_ERROR_NO_MEM);
	}

	airspy_set_packing   (lib_device, 0);
	airspy_set_samplerate(lib_device, AIRSPY_SAMPLERATE_20MSPS);

	*device = lib_device;

	return (AIRSPY_SUCCESS);
}
//
//=========================================================================
//
int airspy_close(airspy_device_t* device) {
	int result = AIRSPY_SUCCESS;

	if (device != NULL) {
		result = airspy_stop_rx(device);

		airspy_open_exit(device);
		free_transfers(device);
		free(device);
	}
	return (result);
}
//
//=========================================================================
//
int airspy_set_samplerate(airspy_device_t* device, uint32_t samplerate) {
	int result;
	uint8_t retval;
	uint8_t length;

	libusb_clear_halt(device->usb_device, LIBUSB_ENDPOINT_IN | 1);

	length = 1;

	result = libusb_control_transfer(
			device->usb_device,
			LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
			AIRSPY_SET_SAMPLERATE,
			0,
			(uint16_t) samplerate,
			&retval,
			length,
			0
			);

	if (result < length)
    	{return (AIRSPY_ERROR_LIBUSB);}

    return (AIRSPY_SUCCESS);
}
//
//=========================================================================
//
int airspy_set_receiver_mode(airspy_device_t* device, receiver_mode_t value) {

	int result;
	result = libusb_control_transfer(
			device->usb_device,
			LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
			AIRSPY_RECEIVER_MODE,
			(uint16_t) value,
			0,
			NULL,
			0,
			0
			);

	if (result != 0)
    	{return (AIRSPY_ERROR_LIBUSB);}

    return (AIRSPY_SUCCESS);
}
//
//=========================================================================
//
int airspy_start_rx(airspy_device_t* device) {
	int result;

	result = airspy_set_receiver_mode(device, RECEIVER_MODE_OFF);
	if (result != AIRSPY_SUCCESS)
		{return (result);}

	libusb_clear_halt(device->usb_device, LIBUSB_ENDPOINT_IN | 1);

	result = airspy_set_receiver_mode(device, RECEIVER_MODE_RX);
	if (result == AIRSPY_SUCCESS) {
		result = create_io_threads(device);
	}

	return (result);
}
//
//=========================================================================
//
int airspy_stop_rx(airspy_device_t* device) {
	int result1, result2;
	result1 = kill_io_threads(device);

	result2 = airspy_set_receiver_mode(device, RECEIVER_MODE_OFF);
	if (result2 != AIRSPY_SUCCESS)
		{return (result2);}

	return (result1);
}
//
//=========================================================================
//
int airspy_gpio_write(airspy_device_t* device, airspy_gpio_port_t port, airspy_gpio_pin_t pin, uint8_t value) {
	int result;
	uint8_t port_pin;

	port_pin = ((uint8_t)port) << 5;
	port_pin = port_pin | pin;

	result = libusb_control_transfer(
			device->usb_device,
			LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
			AIRSPY_GPIO_WRITE,
			value,
			port_pin,
			NULL,
			0,
			0);

	if (result != 0)
    	{return (AIRSPY_ERROR_LIBUSB);}

    return (AIRSPY_SUCCESS);
	}
//
//=========================================================================
//
int airspy_board_partid_serialno_read(airspy_device_t* device, airspy_read_partid_serialno_t* read_partid_serialno) {
	uint8_t length;
	int result;

	length = sizeof(airspy_read_partid_serialno_t);
	result = libusb_control_transfer(
			device->usb_device,
			LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
			AIRSPY_BOARD_PARTID_SERIALNO_READ,
			0,
			0,
			(unsigned char*)read_partid_serialno,
			length,
			0);

	if (result < length)
    	{return (AIRSPY_ERROR_LIBUSB);}

	return (AIRSPY_SUCCESS);
}
//
//=========================================================================
//
int airspy_set_freq(airspy_device_t* device, const uint32_t freq_hz) {
	uint8_t length;
	int     result;

	length = sizeof(uint32_t);

	result = libusb_control_transfer(
			device->usb_device,
			LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
			AIRSPY_SET_FREQ,
			0,
			0,
			(unsigned char*)&freq_hz,
			length,
			0
			);

	if (result < length)
    	{return (AIRSPY_ERROR_LIBUSB);}

    return (AIRSPY_SUCCESS);
}
//
//=========================================================================
//
int airspy_set_lna_gain(airspy_device_t* device, uint8_t value) {
	int     result;
	uint8_t retval;
	uint8_t length;

	if (value > 15)
		value = 15;

	length = 1;

	result = libusb_control_transfer(
			device->usb_device,
			LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
			AIRSPY_SET_LNA_GAIN,
			0,
			value,
			&retval,
			length,
			0
			);

	if (result < length)
    	{return (AIRSPY_ERROR_LIBUSB);}

    return (AIRSPY_SUCCESS);
}
//
//=========================================================================
//
int airspy_set_mixer_gain(airspy_device_t* device, uint8_t value) {
	int result;
	uint8_t retval;
	uint8_t length;

	if (value > 15)
		value = 15;

	length = 1;

	result = libusb_control_transfer(
			device->usb_device,
			LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
			AIRSPY_SET_MIXER_GAIN,
			0,
			value,
			&retval,
			length,
			0
			);

	if (result < length)
    	{return (AIRSPY_ERROR_LIBUSB);}

    return (AIRSPY_SUCCESS);
}
//
//=========================================================================
//
int airspy_set_vga_gain(airspy_device_t* device, uint8_t value) {
	int result;
	uint8_t retval;
	uint8_t length;

	if (value > 15)
		value = 15;

	length = 1;

	result = libusb_control_transfer(
			device->usb_device,
			LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
			AIRSPY_SET_VGA_GAIN,
			0,
			value,
			&retval,
			length,
			0
			);

	if (result < length)
    	{return (AIRSPY_ERROR_LIBUSB);}

    return (AIRSPY_SUCCESS);
}
//
//=========================================================================
//
int airspy_set_lna_agc(airspy_device_t* device, uint8_t value) {
	int result;
	uint8_t retval;
	uint8_t length;

	length = 1;

	result = libusb_control_transfer(
			device->usb_device,
			LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
			AIRSPY_SET_LNA_AGC,
			0,
			value,
			&retval,
			length,
			0
			);

	if (result < length)
    	{return (AIRSPY_ERROR_LIBUSB);}

    return (AIRSPY_SUCCESS);
}
//
//=========================================================================
//
int airspy_set_mixer_agc(airspy_device_t* device, uint8_t value) {
	int result;
	uint8_t retval;
	uint8_t length;

	length = 1;

	result = libusb_control_transfer(
			device->usb_device,
			LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
			AIRSPY_SET_MIXER_AGC,
			0,
			value,
			&retval,
			length,
			0
			);

	if (result < length)
    	{return (AIRSPY_ERROR_LIBUSB);}

    return (AIRSPY_SUCCESS);
}
//
//=========================================================================
//
int airspy_set_linearity_gain(struct airspy_device* device, uint8_t value) {
	int rc;

	if (value >= GAIN_COUNT)
		{value = GAIN_COUNT - 1;}

	value = GAIN_COUNT - 1 - value;

	rc = airspy_set_mixer_agc(device, 0);
	if (rc < 0)
		{return (rc);}

	rc = airspy_set_lna_agc(device, 0);
	if (rc < 0)
		{return (rc);}

	rc = airspy_set_vga_gain(device, airspy_linearity_vga_gains[value]);
	if (rc < 0)
		{return (rc);}

	rc = airspy_set_mixer_gain(device, airspy_linearity_mixer_gains[value]);
	if (rc < 0)
		{return (rc);}

	rc = airspy_set_lna_gain(device, airspy_linearity_lna_gains[value]);
	if (rc < 0)
		{return (rc);}

	return (AIRSPY_SUCCESS);
}
//
//=========================================================================
//
int airspy_set_If_bias(airspy_device_t* device, uint8_t value) {
	return airspy_gpio_write(device, GPIO_PORT1, GPIO_PIN13, value);
}
//
//=========================================================================
//
int airspy_set_packing(airspy_device_t* device, uint8_t value) {
	int     result;
	uint8_t retval;

	if (device->streaming)
    	{return (AIRSPY_ERROR_BUSY);}

	result = libusb_control_transfer(
			device->usb_device,
			LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
			AIRSPY_SET_PACKING,
			0,
			value,
			&retval,
			1,
			0);

	if (result < 1)
		{return  (AIRSPY_ERROR_LIBUSB);}

	return (AIRSPY_SUCCESS);
}
//
//=========================================================================
//
const char* airspy_error_name(enum airspy_error errcode) {
	switch (errcode) {

		case AIRSPY_SUCCESS:
			return "AIRSPY_SUCCESS";

		case AIRSPY_TRUE:
			return "AIRSPY_TRUE";

		case AIRSPY_ERROR_INVALID_PARAM:
			return "AIRSPY_ERROR_INVALID_PARAM";

		case AIRSPY_ERROR_NOT_FOUND:
			return "AIRSPY_ERROR_NOT_FOUND";

		case AIRSPY_ERROR_BUSY:
			return "AIRSPY_ERROR_BUSY";

		case AIRSPY_ERROR_NO_MEM:
			return "AIRSPY_ERROR_NO_MEM";

		case AIRSPY_ERROR_LIBUSB:
			return "AIRSPY_ERROR_LIBUSB";

		case AIRSPY_ERROR_THREAD:
			return "AIRSPY_ERROR_THREAD";

		case AIRSPY_ERROR_STREAMING_THREAD_ERR:
			return "AIRSPY_ERROR_STREAMING_THREAD_ERR";

		case AIRSPY_ERROR_STREAMING_STOPPED:
			return "AIRSPY_ERROR_STREAMING_STOPPED";

		case AIRSPY_ERROR_OTHER:
			return "AIRSPY_ERROR_OTHER";

		default:
			return "airspy unknown error";
	}
}
//
// =============================== Airspy Device handling ==========================
//

void airspy_InitDevice(void) {

	int iResult;
	airspy_read_partid_serialno_t serial;
	Modes.dev = NULL;

	if (AIRSPY_SUCCESS != (iResult = airspy_open_sn(&Modes.dev, Modes.dev_index))) {
		fprintf(stderr, "airspy_open_init() failed: %s (%d)\n", airspy_error_name(iResult), iResult);
		exit(1);
	}

	if (airspy_board_partid_serialno_read(Modes.dev, &serial) == AIRSPY_SUCCESS) {
		fprintf(stderr, "Acquired Airspy device with serial %08X%08X\n", serial.serial_no[2], serial.serial_no[3]);
	}

	if (AIRSPY_SUCCESS != (iResult = airspy_set_freq(Modes.dev, Modes.freq))) {
		fprintf(stderr, "airspy_set_freq() failed: %s (%d)\n", airspy_error_name(iResult), iResult);
		airspy_close(Modes.dev);
		exit(1);
	}

	if (AIRSPY_SUCCESS != (iResult = airspy_set_If_bias(Modes.dev, (uint8_t)Modes.bias_t))) {
		fprintf(stderr, "airspy_set_If_bias() failed: %s (%d)\n", airspy_error_name(iResult), iResult);
		airspy_close(Modes.dev);
		exit(1);
	}

	if (AIRSPY_SUCCESS != (iResult = airspy_set_linearity_gain(Modes.dev, (int8_t)Modes.gain))) {
		fprintf(stderr, "airspy_set_linearity_gain() failed: %s (%d)\n", airspy_error_name(iResult), iResult);
		airspy_close(Modes.dev);
		exit(1);
	}

	if ((Modes.lnagain >= 0) && (Modes.lnagain < 16)
		&& (AIRSPY_SUCCESS != (iResult = airspy_set_lna_gain(Modes.dev, (int8_t)Modes.lnagain)))) {
		fprintf(stderr, "airspy_set_lna_gain() failed: %s (%d)\n", airspy_error_name(iResult), iResult);
		airspy_close(Modes.dev);
		exit(1);
	}

	if ( (Modes.vgagain >= 0) && (Modes.vgagain < 16)
	  && (AIRSPY_SUCCESS != (iResult = airspy_set_vga_gain(Modes.dev, (int8_t)Modes.vgagain))) ) {
		fprintf(stderr, "airspy_set_vga_gain() failed: %s (%d)\n", airspy_error_name(iResult), iResult);
		airspy_close(Modes.dev);
		exit(1);
	}

	if ((Modes.mixergain >= 0) && (Modes.mixergain < 16)
		&& (AIRSPY_SUCCESS != (iResult = airspy_set_mixer_gain(Modes.dev, (int8_t)Modes.mixergain)))) {
		fprintf(stderr, "airspy_set_mixer_gain() failed: %s (%d)\n", airspy_error_name(iResult), iResult);
		airspy_close(Modes.dev);
		exit(1);
	}
}
//
//=========================================================================
//
// This is used when --ifile is specified in order to read data from file
// instead of using an Airspy device
//
void* Replay_threadproc (void* arg) {

	uint32_t uIF_In;
    ssize_t nread, toread;
    unsigned char* p;
    UNUSED (arg);

	while (!Modes.exit) {
		// We own the IF_mutex here. If the IF Fifo is empty, read some data
		// We can't queue input data because we're reading it all into Modes.pFileData
		// So wait till the previous buffer has been done before reading the next one.
		if ( (Modes.uIF_In == Modes.uIF_Out)
          && (Modes.uAF_In == Modes.uAF_Out)
          && ( (Modes.uDecode_ModeS_In == Modes.uDecode_ModeS_Out)
            || (Modes.uDecode_ModeA_In == Modes.uDecode_ModeA_Out) )
           ) {
			// The IF Fifo is empty, so read the next chunk of data
			toread = MODES_ASYNC_BUF_SIZE;
			p = (unsigned char*) Modes.pFileData;
			while (toread) {
				nread = read(Modes.fd, p, toread);
				if (nread <= 0) {

            		pthread_mutex_unlock(&Modes.IF_mutex);
        		    while ( (Modes.uIF_In - Modes.uIF_Out) 
						||  (Modes.uIF_In - Modes.uAF_Out)
						||  (Modes.uAF_In - Modes.uAF_Out) ) {
                    		usleep(1000);
                        }
            		pthread_mutex_lock(&Modes.IF_mutex);
					Modes.exit = 1; // Signal the other threads to exit.
					break;
				}
				p += nread;
				toread -= nread;
			}

			if (!Modes.exit) {
				if (toread) {
					// Not enough data on file to fill the buffer? Pad with no signal.
					memset(p, 127, toread);
				}

				uIF_In = Modes.uIF_In & (MODES_ASYNC_BUF_NUMBER - 1); // Just incase!!!

				// Queue the new data
				Modes.IF_Fifo[uIF_In].pFifo = Modes.pFileData;
				Modes.IF_Fifo[uIF_In].uLost = 0;
				Modes.IF_Fifo[uIF_In].lTime = Modes.timestampBlk;
				Modes.uIF_In++;
                Modes.timestampBlk += (MODES_ASYNC_BUF_SAMPLES);
				// Signal to the IF thread that new data is ready. It won't run yet because we hold the IF mutex
				pthread_cond_signal(&Modes.IF_cond);
			}

		} else { 
			// We own the mutex, and there is some data in the IF Fifo
			// Allow the IF thread to run for a while to use that data
			pthread_mutex_unlock(&Modes.IF_mutex);
			usleep(3200);
			pthread_mutex_lock(&Modes.IF_mutex);
		}
	}

	// Release the IF mutex so the thread can terminate
	pthread_mutex_unlock(&Modes.IF_mutex);

#ifdef _WIN32
	return (NULL);
#else
	pthread_exit(NULL);
#endif
}
//
//=========================================================================
//
void airspy_start_replay(void) {
	pthread_attr_t attr;

	pthread_attr_init(&attr);

	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);

    pthread_create (&Modes.Replay_thread, &attr, Replay_threadproc, NULL);

	pthread_attr_destroy(&attr);
}
//
//=========================================================================
//
