// Copyright 2021 1Conan (@1Conan)
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once

/*
 * GPIO and SPI IRQs needs to be set to 0 to avoid erratic behavior
 */

/**
 * PAL driver settings
 */
#define PAL_USE_CALLBACKS               TRUE
#define SN32_GPIOA_IRQ_PRIORITY         0

/**
 * SPI driver settings
 */
#define HAL_USE_SPI                     TRUE
#define SPI_USE_MUTUAL_EXCLUSION        FALSE
#define SPI_USE_WAIT                    FALSE
#define SPI_USE_ASSERT_ON_ERROR         FALSE
#define SPI_SELECT_MODE                 SPI_SELECT_MODE_NONE

#define SN32_SPI_SPI0_IRQ_PRIORITY      0

#include_next <halconf.h>
