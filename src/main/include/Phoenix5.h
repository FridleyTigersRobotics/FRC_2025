#pragma once

#include "wpi/deprecated.h"

// Used to prevent deprecation warnings
// Seems to be an issue on CTRE's end.
// https://www.chiefdelphi.com/t/ctre-phoenix-5-deprecation-warnings/449650
WPI_IGNORE_DEPRECATED
#include <ctre/Phoenix.h>
WPI_UNIGNORE_DEPRECATED
