// Copyright 2011 The Android Open Source Project
// Copyright 2020 Evgeniy Reizner
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

pub mod path_aa;
pub mod path;
pub mod hairline_aa;


use crate::{Rect, ScreenIntRect, IntRect};

use crate::blitter::Blitter;


#[inline]
pub fn fill_rect(
    rect: &Rect,
    clip: &ScreenIntRect,
    blitter: &mut dyn Blitter,
) -> Option<()> {
    fill_int_rect(&rect.round(), clip, blitter)
}

#[inline]
fn fill_int_rect(
    rect: &IntRect,
    clip: &ScreenIntRect,
    blitter: &mut dyn Blitter,
) -> Option<()> {
    let rect = rect.intersect(&clip.to_int_rect())?.to_screen_int_rect()?;
    blitter.blit_rect(&rect);
    Some(())
}

#[inline]
pub fn fill_rect_aa(
    rect: &Rect,
    clip: &ScreenIntRect,
    blitter: &mut dyn Blitter,
) -> Option<()> {
    hairline_aa::fill_rect(rect, clip, blitter)
}

