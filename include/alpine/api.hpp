#pragma once

// alpine/api.hpp
//
// The single header a user includes. Mirrors the lemlib/api.hpp pattern: users
// write
//
//     #include "alpine/api.hpp"
//
// and never have to know the internal file layout. That also means you can
// rearrange everything under include/alpine/ later without breaking anyone's
// project, as long as this file keeps exporting the same names.

#include "alpine/pose.hpp"
#include "alpine/util.hpp"
#include "alpine/localization/field.hpp"
#include "alpine/localization/particle.hpp"