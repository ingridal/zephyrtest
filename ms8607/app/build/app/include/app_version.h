/**
 * @file app_version.h
 *
 * Application version information.
 *
 * Copyright (c) 2021 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef APP_VERSION_H_
#define APP_VERSION_H_

/** Application major version. */
#define APP_VERSION_MAJOR 1
/** Application minor version. */
#define APP_VERSION_MINOR 0
/** Application patch version. */
#define APP_VERSION_PATCH 0

/** Application version. */
#define APP_VERSION \
	((APP_VERSION_MAJOR << 16) + \
	 (APP_VERSION_MINOR << 8) + \
	  APP_VERSION_PATCH)

/** Application version (string). */
#define APP_VERSION_STR "1.0.0"

#endif /* APP_VERSION_H_ */
