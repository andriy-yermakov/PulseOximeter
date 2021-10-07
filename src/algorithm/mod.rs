use core::cmp;
use num_traits::cast::NumCast;
use num_traits::Float;

fn heart_rate_and_oxygen_saturation<T>(read_data: &[(T, T)]) -> ()
where
    T: Float,
{
    let mut ir_mean = T::zero();
    let mut red_mean = T::zero();

    // Calculate DC mean and substract mean from ir and red
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
