// Copyright 2006 The Android Open Source Project
// Copyright 2020 Evgeniy Reizner
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

use crate::fdot6::FDot6;
use crate::floating_point::SaturateCast;
use crate::math::{left_shift64, bound};

/// A 16.16 fixed point.
///
/// 32 bit signed integer used to represent fractions values with 16 bits
/// to the right of the decimal point.
pub type Fixed = i32;

pub const HALF: Fixed = (1 << 16) / 2;
pub const ONE: Fixed = 1 << 16;

// `from_f32` seems to lack a rounding step. For all fixed-point
// values, this version is as accurate as possible for (fixed -> float -> fixed). Rounding reduces
// accuracy if the intermediate floats are in the range that only holds integers (adding 0.5 to an
// odd integer then snaps to nearest even). Using double for the rounding math gives maximum
// accuracy for (float -> fixed -> float), but that's usually overkill.
pub fn from_f32(x: f32) -> Fixed {
    i32::saturate_from(x * ONE as f32)
}

pub fn round_to_i32(x: Fixed) -> i32 {
    (x + HALF) >> 16
}

// The divide may exceed 32 bits. Clamp to a signed 32 bit result.
pub fn mul(a: Fixed, b: Fixed) -> Fixed {
    ((i64::from(a) * i64::from(b)) >> 16) as Fixed
}

// The divide may exceed 32 bits. Clamp to a signed 32 bit result.
pub fn div(numer: FDot6, denom: FDot6) -> Fixed {
    let v = left_shift64(numer as i64, 16) / denom as i64;
    let n = bound(i32::MIN as i64, v, i32::MAX as i64);
    n as i32
}
