// rpdsp.h — Arduino library discovery shim
//
// This file exists solely so arduino-cli's library resolver can detect the
// rpdsp library. It is NOT intended to be included by user code.
// Include the namespaced headers directly, e.g.:
//   #include <rpdsp/oscillator.h>
//   #include <rpdsp/filter.h>
//
// The resolver only scans the src/ root for headers; placing a file here
// causes the library to be detected and src/ to be added to the include path,
// which then makes the nested <rpdsp/...> headers resolvable.

#pragma once
#include "rpdsp/DSPFunctions.h"
#include "rpdsp/algorithm.h"
#include "rpdsp/analog_adsr.h"
#include "rpdsp/analysis.h"
#include "rpdsp/bbd_delay.h"
#include "rpdsp/clock_tracker.h"
#include "rpdsp/config.h"
#include "rpdsp/control_surface.h"
#include "rpdsp/delay_line.h"
#include "rpdsp/dynamics.h"
#include "rpdsp/effects.h"
#include "rpdsp/envelope.h"
#include "rpdsp/filter.h"
#include "rpdsp/frequency_shifter.h"
#include "rpdsp/gate_pattern.h"
#include "rpdsp/hardware_interpolator.h"
#include "rpdsp/hypersaw.h"
#include "rpdsp/joystick_recorder.h"
#include "rpdsp/knob_bank.h"
#include "rpdsp/ladder.h"
#include "rpdsp/oscillator.h"
#include "rpdsp/parameter_smoother.h"
#include "rpdsp/parameter_track.h"
#include "rpdsp/pickup_knob.h"
#include "rpdsp/realtime.h"
#include "rpdsp/rhythm_sequencer.h"
#include "rpdsp/scale_table.h"
#include "rpdsp/tape_delay.h"
#include "rpdsp/voice.h"
#include "rpdsp/wavefolder.h"
#include "rpdsp/waveguide.h"