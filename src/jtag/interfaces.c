// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007,2008 Øyvind Harboe                                 *
 *   oyvind.harboe@zylin.com                                               *
 *                                                                         *
 *   Copyright (C) 2009 SoftPLC Corporation                                *
 *       http://softplc.com                                                *
 *   dick@softplc.com                                                      *
 *                                                                         *
 *   Copyright (C) 2009 Zachary T Welch                                    *
 *   zw@superlucidity.net                                                  *
 *                                                                         *
 *   Copyright (C) 2020, Ampere Computing LLC                              *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "interfaces.h"

/** @file
 * This file collects all the built-in JTAG interfaces in the adapter_drivers
 * array.
 *
 * Dynamic loading can be implemented be searching for shared libraries
 * that contain an adapter_driver structure that can added to this list.
 */

/**
 * The list of built-in JTAG interfaces, containing entries for those
 * drivers that were enabled by the @c configure script.
 */
struct adapter_driver *adapter_drivers[] = {
#if BUILD_PARPORT == 1
		&parport_adapter_driver,
#endif
#if BUILD_DUMMY == 1
		&dummy_adapter_driver,
#endif
#if BUILD_FTDI == 1
		&ftdi_adapter_driver,
#endif
#if BUILD_USB_BLASTER || BUILD_USB_BLASTER_2 == 1
		&usb_blaster_adapter_driver,
#endif
#if BUILD_ESP_USB_JTAG == 1
		&esp_usb_adapter_driver,
#endif
#if BUILD_JTAG_VPI == 1
		&jtag_vpi_adapter_driver,
#endif
#if BUILD_VDEBUG == 1
		&vdebug_adapter_driver,
#endif
#if BUILD_JTAG_DPI == 1
		&jtag_dpi_adapter_driver,
#endif
#if BUILD_FT232R == 1
		&ft232r_adapter_driver,
#endif
#if BUILD_AMTJTAGACCEL == 1
		&amt_jtagaccel_adapter_driver,
#endif
#if BUILD_EP93XX == 1
		&ep93xx_adapter_driver,
#endif
#if BUILD_AT91RM9200 == 1
		&at91rm9200_adapter_driver,
#endif
#if BUILD_GW16012 == 1
		&gw16012_adapter_driver,
#endif
#if BUILD_PRESTO
		&presto_adapter_driver,
#endif
#if BUILD_USBPROG == 1
		&usbprog_adapter_driver,
#endif
#if BUILD_OPENJTAG == 1
		&openjtag_adapter_driver,
#endif
#if BUILD_JLINK == 1
		&jlink_adapter_driver,
#endif
#if BUILD_VSLLINK == 1
		&vsllink_adapter_driver,
#endif
#if BUILD_RLINK == 1
		&rlink_adapter_driver,
#endif
#if BUILD_ULINK == 1
		&ulink_adapter_driver,
#endif
#if BUILD_ANGIE == 1
		&angie_adapter_driver,
#endif
#if BUILD_ARMJTAGEW == 1
		&armjtagew_adapter_driver,
#endif
#if BUILD_BUSPIRATE == 1
		&buspirate_adapter_driver,
#endif
#if BUILD_REMOTE_BITBANG == 1
		&remote_bitbang_adapter_driver,
#endif
#if BUILD_HLADAPTER == 1
		&hl_adapter_driver,
#endif
#if BUILD_OSBDM == 1
		&osbdm_adapter_driver,
#endif
#if BUILD_OPENDOUS == 1
		&opendous_adapter_driver,
#endif
#if BUILD_SYSFSGPIO == 1
		&sysfsgpio_adapter_driver,
#endif
#if BUILD_LINUXGPIOD == 1
		&linuxgpiod_adapter_driver,
#endif
#if BUILD_XLNX_PCIE_XVC == 1
		&xlnx_pcie_xvc_adapter_driver,
#endif
#if BUILD_BCM2835GPIO == 1
		&bcm2835gpio_adapter_driver,
#endif
#if BUILD_PICOPROBE == 1
		&picoprobe_adapter_driver,
#endif
#if BUILD_CMSIS_DAP_USB == 1 || BUILD_CMSIS_DAP_HID == 1
		&cmsis_dap_adapter_driver,
#endif
#if BUILD_KITPROG == 1
		&kitprog_adapter_driver,
#endif
#if BUILD_IMX_GPIO == 1
		&imx_gpio_adapter_driver,
#endif
#if BUILD_XDS110 == 1
		&xds110_adapter_driver,
#endif
#if BUILD_HLADAPTER_STLINK == 1
		&stlink_dap_adapter_driver,
#endif
#if BUILD_RSHIM == 1
		&rshim_dap_adapter_driver,
#endif
#if BUILD_DMEM == 1
		&dmem_dap_adapter_driver,
#endif
#if BUILD_AM335XGPIO == 1
		&am335xgpio_adapter_driver,
#endif
		NULL,
	};
