/**
 * \file Version.h
 * \brief Version and Build NumberHelper Class
 *
 * This helper macros exposes the static methods to get the firmware version and the build number.
 * Use the build() and version() metho5ds anywhere in the program including this file
*/
#ifndef __VERSION_H__
#define __VERSION_H__

//! Incremental build number
#define build() "1.2.4"
//! Firmware version
#define version() "1.4"
//! Project name
#define project() "Circuit Control"

#endif //__VERSION_H__
