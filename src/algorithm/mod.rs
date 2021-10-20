use compat_no_std as std;
use core::fmt::Result;
use core::intrinsics::copy_nonoverlapping;
use core::ops::Add;
use cortex_m_semihosting::hprint;
use std::prelude::v1::*;

use core::cmp;
use core::cmp::min;
use core::convert::TryInto;
// use cortex_m_semihosting::hprint;
use num_traits::cast::NumCast;
use num_traits::Float;

pub fn heart_rate_and_oxygen_saturation(
    ir_slices: (&[u8], &[u8]),
    red_slices: (&[u8], &[u8]),
) -> () {
    // Sum of squares of ST*FS numbers from -mean_X (see below) to +mean_X incremented be one. For example, given ST=4 and FS=25,
    // the sum consists of 100 terms: (-49.5)2 + (-48.5)^2 + (-47.5)^2 + ... + (47.5)^2 + (48.5)^2 + (49.5)^2
    // The sum is symmetrc, so you can evaluate it by multiplying its positive half by 2. It is precalcuated here for enhanced
    // performance.
    //const SUM_X2 = 83325; // 2 sec
    const SUM_X2: f32 = 281237.5;
    //const SUM_X2 = 1302062.5; // 5 sec

    const MEAN_X: f32 = 149.0 / 2.0;

    // let (mut ir_buffer, mut red_buffer) = populate_buffers(ir_slices, red_slices);
    let mut ir_buffer = [0_f32; 150];
    let mut red_buffer = [0_f32; 150];

    populate_buffer(ir_slices, &mut ir_buffer).unwrap();
    populate_buffer(red_slices, &mut red_buffer).unwrap();

    // calculates DC mean and subtracts DC from ir and red
    let ir_mean = ir_buffer.iter().sum::<f32>() / ir_buffer.len() as f32;
    let red_mean = red_buffer.iter().sum::<f32>() / red_buffer.len() as f32;

    // remove DC
    for i in 0..ir_buffer.len() {
        ir_buffer[i] = ir_buffer[i] - ir_mean;
    }

    for i in 0..red_buffer.len() {
        red_buffer[i] = red_buffer[i] - red_mean;
    }

    // Remove linear trend (baseline leveling)
    let beta_ir = linear_regression_beta(&ir_buffer, MEAN_X, SUM_X2);
    let beta_red = linear_regression_beta(&red_buffer, MEAN_X, SUM_X2);

    let mut x = -MEAN_X;
    for i in 0..ir_buffer.len() {
        ir_buffer[i] = ir_buffer[i] - beta_ir * x;
        x = x + 1.0;
    }
    x = -MEAN_X;
    for i in 0..red_buffer.len() {
        red_buffer[i] = red_buffer[i] - beta_red * x;
        x = x + 1.0;
    }

    // For SpO2 calculate RMS of both AC signals. In addition, pulse detector needs raw sum of squares for IR
    let (ir_ac, ir_sumsq) = rms(&ir_buffer);
    let (red_ac, red_sumsq) = rms(&red_buffer);

    // Calculate Pearson correlation between red and IR
    let correl = p_correlation(&ir_buffer, &red_buffer);
    ()
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

fn p_correlation<T>(x: &[T], y: &[T]) -> T
where
    T: Float,
{
    let len = min(x.len(), y.len());
    let mut r: T = T::zero();

    for i in 0..len {
        r = r + x[i] * y[i];
    }
    r / NumCast::from(len).unwrap()
}

fn populate_buffer(slices: (&[u8], &[u8]), buffer: &mut [f32]) -> Result {
    let samples_to_read = min(slices.0.len() / 3, 150);
    let remaind_bytes = min(slices.0.len(), 150 * 3) - samples_to_read;
    let mut w = [0_u8; 4];

    for i in 0..samples_to_read {
        for k in 0..3 {
            w[k + 1] = slices.0[i * 3 + k];
        }
        buffer[i] = f32::from_le_bytes(w);
    }

    if slices.1.len() > 0 && remaind_bytes > 0 {
        let continue_from = if slices.0.len() - samples_to_read * 3 == 0 {
            samples_to_read
        } else {
            // convert splitted sample
            for i in 0..remaind_bytes {
                w[i + 1] = slices.0[samples_to_read * 3 + i];
            }

            for i in 0..(3 - remaind_bytes) {
                w[remaind_bytes + 1 + i] = slices.1[i];
            }

            buffer[samples_to_read] = f32::from_le_bytes(w);

            samples_to_read + 1
        };

        for i in continue_from..150 {
            for k in 0..3 {
                w[k + 1] = slices.1[(i - samples_to_read) * 3 + k];
            }
            buffer[i] = f32::from_le_bytes(w);
        }
    }

    Ok(())
}

// fn populate_buffers(
//     ir_slices: (&[u8], &[u8]),
//     red_slices: (&[u8], &[u8]),
// ) -> ([f32; 150], [f32; 150]) {
//     unsafe {
//         // Populate ir_buffer
//         let mut tmp_buffer = [0u8; 600];
//         let count_bytes_to_copy = min(ir_slices.0.len(), 600);

//         copy_nonoverlapping(
//             ir_slices.0.as_ptr(),
//             tmp_buffer.as_mut_ptr(),
//             count_bytes_to_copy,
//         );

//         copy_nonoverlapping(
//             ir_slices.1.as_ptr(),
//             tmp_buffer
//                 .as_mut_ptr()
//                 .offset(count_bytes_to_copy.try_into().unwrap()),
//             min(ir_slices.1.len(), 600 - count_bytes_to_copy),
//         );

//         let tmp_buffer = std::mem::transmute::<[u8; 600], [u32; 150]>(tmp_buffer);
//         let mut ir_buffer = [0_f32; 150];

//         for i in 0..150 {
//             ir_buffer[i] = tmp_buffer[i] as f32;
//         }

//         // Populate red_buffer
//         let mut tmp_buffer = [0u8; 600];
//         let count_bytes_to_copy = min(red_slices.0.len(), 600);

//         copy_nonoverlapping(
//             red_slices.0.as_ptr(),
//             tmp_buffer.as_mut_ptr(),
//             count_bytes_to_copy,
//         );

//         copy_nonoverlapping(
//             red_slices.1.as_ptr(),
//             tmp_buffer
//                 .as_mut_ptr()
//                 .offset(count_bytes_to_copy.try_into().unwrap()),
//             min(red_slices.1.len(), 600 - count_bytes_to_copy),
//         );

//         let tmp_buffer = std::mem::transmute::<[u8; 600], [u32; 150]>(tmp_buffer);
//         let mut red_buffer = [0_f32; 150];

//         for i in 0..150 {
//             red_buffer[i] = tmp_buffer[i] as f32;
//         }

//         (ir_buffer, red_buffer)
//     }
// }
