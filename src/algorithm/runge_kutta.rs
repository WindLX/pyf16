use crate::utils::{Matrix, Vector};

pub fn runge_kutta(
    func: Box<dyn Fn(&Vector, &Vector) -> Vector>,
    y_0: Vector,
    x_span: (Vector, Vector),
    n: usize,
) -> Matrix {
    let x_eval = Matrix::linespace(&x_span.0, &x_span.1, n);
    let mut y_eval = Matrix::zero_like(&x_eval);
    y_eval[0] = y_0;
    let len = x_eval.shape().0;
    for i in 1..len {
        let h = x_eval[i].clone() - x_eval[i - 1].clone();
        let k_1 = func(&x_eval[i - 1], &y_eval[i - 1]);
        let k_2 = func(
            &(x_eval[i - 1].clone() + h.clone() * 0.5),
            &(y_eval[i - 1].clone() + h.clone() * k_1.clone() * 0.5),
        );
        let k_3 = func(
            &(x_eval[i - 1].clone() + h.clone() * 0.5),
            &(y_eval[i - 1].clone() + h.clone() * k_2.clone() * 0.5),
        );
        let k_4 = func(
            &(x_eval[i - 1].clone() + h.clone()),
            &(y_eval[i - 1].clone() + h.clone() * k_3.clone()),
        );
        y_eval[i] = y_eval[i - 1].clone() + (h / 6.0) * (k_1 + k_2 * 2.0 + k_3 * 2.0 + k_4);
    }
    return y_eval;
}

#[cfg(test)]
mod algorithm_tests {
    use super::*;

    #[test]
    fn test_rk() {
        let func = |x: &Vector, y: &Vector| {
            return y.clone() - x.clone() * x.clone() + 1.0;
        };

        let analytic_func = |x: &Vector| {
            let temp = x.map(|xx| xx.exp());
            return (x.clone() + Vector::from(vec![1.0])) * (x.clone() + Vector::from(vec![1.0]))
                - temp * 0.5;
        };

        let y_0 = Vector::from(vec![0.5]);
        let x_span = (Vector::from(vec![0.0]), Vector::from(vec![1.0]));
        let n = 11;

        let result = runge_kutta(Box::new(func), y_0, x_span.clone(), n);
        println!("result: {:#?}", result);

        let x_eval = Matrix::linespace(&x_span.0, &x_span.1, n);
        let mut analytic_result = Vec::new();
        let mut err = Vec::new();
        for i in 0..x_eval.dim() {
            let res = analytic_func(&x_eval[i]);
            err.push((res.clone() - result[i].clone()).norm());
            analytic_result.push(res);
        }

        println!("analytic result: {:#?}", analytic_result);
        println!("{:#?}", err.iter().sum::<f64>() / err.len() as f64);
    }
}
