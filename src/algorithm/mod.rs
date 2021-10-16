use compat_no_std::{self as std, ptr, result};
use core::intrinsics::copy_nonoverlapping;
use std::prelude::v1::*;

use core::cmp::min;
use core::convert::TryInto;
use core::{cmp, mem};
use cortex_m_semihosting::hprint;
use num_traits::cast::NumCast;
use num_traits::Float;

pub fn heart_rate_and_oxygen_saturation(
    ir_slices: (&[u8], &[u8]),
    red_slices: (&[u8], &[u8]),
) -> () {
    let (mut ir_buffer, mut red_buffer) = populate_buffers(ir_slices, red_slices);
}

fn linear_regression_beta<T>(x: &[T], xmean: T, sum_x2: T) -> T
where
    T: Float,
{
    let mut t = -xmean;
    let step = T::from(1).unwrap();
    let mut i = 0_usize;
    let mut beta = T::zero();

    while t < xmean {
        beta = beta + t * x[i];
        i += 1;
        t = t + step;
    }

    beta / sum_x2
}

fn autocorrelation<T>(x: &[T], lag: usize) -> T
where
    T: Float,
{
    let lag = cmp::min(lag, x.len() - 1);
    let n = x.len() - lag;
    let mut sum: T = T::zero();

    for i in 0..n {
        sum = sum + x[i] * x[i + lag];
    }

    sum / T::from(n).unwrap()
}

fn rms<T>(x: &[T]) -> (T, T)
where
    T: Float,
{
    let len = NumCast::from(x.len()).unwrap();
    let mut sumsq = x
        .iter()
        .fold(T::zero(), |sumsq: T, elem| sumsq + *elem * *elem);
    sumsq = sumsq / len;
    (sumsq.sqrt(), sumsq)
}

fn p_correlation<T>(set: &[(T, T)]) -> T
where
    T: Float,
{
    let len = NumCast::from(set.len()).unwrap();
    set.iter()
        .fold(T::zero(), |acc: T, elem| acc + elem.0 * elem.1)
        / len
}

fn populate_buffers(
    ir_slices: (&[u8], &[u8]),
    red_slices: (&[u8], &[u8]),
) -> ([u32; 150], [u32; 150]) {
    let mut ir_buffer = [0u8; 600];
    let mut red_buffer = [0u8; 600];

    unsafe {
        // Populate ir_buffer
        let count_bytes_to_copy = min(ir_slices.0.len(), 600);

        copy_nonoverlapping(
            ir_slices.0.as_ptr(),
            ir_buffer.as_mut_ptr(),
            count_bytes_to_copy,
        );

        copy_nonoverlapping(
            ir_slices.1.as_ptr(),
            ir_buffer
                .as_mut_ptr()
                .offset(count_bytes_to_copy.try_into().unwrap()),
            min(ir_slices.1.len(), 600 - count_bytes_to_copy),
        );

        // Populate red_buffer
        let count_bytes_to_copy = min(red_slices.0.len(), 600);

        copy_nonoverlapping(
            red_slices.0.as_ptr(),
            red_buffer.as_mut_ptr(),
            count_bytes_to_copy,
        );

        copy_nonoverlapping(
            red_slices.1.as_ptr(),
            red_buffer
                .as_mut_ptr()
                .offset(count_bytes_to_copy.try_into().unwrap()),
            min(red_slices.1.len(), 600 - count_bytes_to_copy),
        );

        // Convert buffers type and return
        (
            std::mem::transmute::<[u8; 600], [u32; 150]>(ir_buffer),
            std::mem::transmute::<[u8; 600], [u32; 150]>(red_buffer),
        )
    }
}
