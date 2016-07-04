// %pacpus:license{
// This file is part of the PACPUS framework distributed under the
// CECILL-C License, Version 1.0.
// %pacpus:license}
/// @file
/// @version $Id$
/// @copyright Copyright (c) UTC/CNRS Heudiasyc 2006 - 2013. All rights reserved.
/// @brief Brief description.
///
/// Detailed description.


#ifndef DEF_PACPUS_ROAD_TIME_H
#define DEF_PACPUS_ROAD_TIME_H

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

#include <stddef.h>  // defines: NULL

#include "cstdint.h"

/// Export macro for ROAD_TIME DLL for Windows only
#ifdef WIN32
#   ifdef ROAD_TIME_EXPORTS
#       define ROAD_TIME_API __declspec(dllexport)
#   else
#       define ROAD_TIME_API __declspec(dllimport)
#   endif
#else
#   define ROAD_TIME_API /* nothing */
#endif

/// Timestamp type
typedef uint64_t road_time_t;
/// Timerange type
typedef int32_t road_timerange_t;

/// Timestamp difference type
typedef int64_t road_time_diff_t;
/// Timerange difference type
typedef int32_t road_timerange_diff_t;

#ifdef WIN32

/// Structure exchanged to give the Initialization parameters. All these fields are filled when the
/// first process attached to the DLL. It gives the Real Time and the value of the Performance Counter
/// at the same time.
/// Description of each field is provided inside the structure
struct ROAD_TIME_API Initialisation_Time
{
    /// Time (extended to micro seconds precision)
    /// that was given by the Real Time Clock of
    /// windows (using ftime()) at the first time the DLL was called.
    road_time_t    Real_Time;

    /// Correpsonding number of cycles of the CPU Performance Counter,
    /// if it is available. Normally it is the Multimedia Counter.
    /// If the Performance Counter is not availaible, normally it
    /// should return the Real Time Counter Value (accuracy of 10 ms)..
    uint64_t	Multimedia_Counter;

    /// Frequency of the Performance Counter, if it is available
    /// This Frequency is given in Hertz.
    uint64_t	Multimedia_Counter_Frequency;

    /// Delta_t is used to have less work when asking time.
    /// Delta_t is equal to Real_Time - (Multimedia_Counter/Multimedia_Counter_Frequancy)/1000000
    road_time_t    delta_t;
};

/// This method just return the actual time using a method based on delta_t (1 less operation).
/// Return a road_time_t: ellapsed time in microseconds since the 1/01/1970
road_time_t ROAD_TIME_API road_time(void);

/// This method just return the actual time using all the fields of the Initiaiztion_Time Structure.
/// Return a road_time_t: ellapsed time in microseconds since the 1/01/1970
road_time_t ROAD_TIME_API road_time2(void);

/// This method just return the Initialization Parameter
/// Return an Initialization_Time structure (see below)
struct Initialisation_Time ROAD_TIME_API road_time_init(void);

#else // WIN32

// UNIX

#include <sys/time.h>

static road_time_t road_time(void)
{
    struct timeval t;
    gettimeofday(&t, NULL);
    return(road_time_t)((road_time_t)(t.tv_sec)*1000000 + (road_time_t)(t.tv_usec));
}

#endif // WIN32

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // DEF_PACPUS_ROAD_TIME_H
