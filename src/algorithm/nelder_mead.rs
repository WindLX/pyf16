use crate::utils::{error::FatalCoreError, Matrix, Vector};
use log::trace;
use serde::{Deserialize, Serialize};

/// 单纯形搜索法设置
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct NelderMeadOptions {
    /// 最大函数计算次数
    pub max_fun_evals: usize,
    /// 最大迭代次数
    pub max_iter: usize,
    /// 函数值的终止容差
    pub tol_fun: f64,
    /// 正标量 x 的终止容差
    pub tol_x: f64,
}

impl std::fmt::Display for NelderMeadOptions {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "max_fun_evals: {}, max_iter: {}, tol_fun: {}, tol_x: {}",
            self.max_fun_evals, self.max_iter, self.tol_fun, self.tol_x
        )
    }
}

impl Default for NelderMeadOptions {
    fn default() -> Self {
        Self {
            max_fun_evals: 50000,
            max_iter: 10000,
            tol_fun: 1e-10,
            tol_x: 1e-10,
        }
    }
}

/// 单纯形搜索法结果
#[derive(Debug, Clone)]
pub struct NelderMeadResult {
    /// 最小值所在点
    pub x: Vector,
    /// 最小值
    pub fval: f64,
    /// 迭代次数
    pub iter: usize,
    /// 函数计算次数
    pub fun_evals: usize,
}

impl std::fmt::Display for NelderMeadResult {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "x: {:?}, fval: {}, iter: {}, fun_evals: {}",
            self.x, self.fval, self.iter, self.fun_evals
        )
    }
}

/// nelder_mead 单纯形搜索法求解器
/// 此求解器用于搜索目标函数最小值
/// Args:
///     func: Box<dyn Fn(&Vector) -> Result<f64, FatalError>>: 目标函数
///     x_0: Vector: 搜索的初始值
///     options: Option<NelderMeadOptions>: 求解器设置
pub fn nelder_mead(
    func: Box<dyn Fn(&Vector) -> Result<f64, FatalCoreError>>,
    x_0: Vector,
    options: Option<NelderMeadOptions>,
) -> Result<NelderMeadResult, FatalCoreError> {
    trace!("nelder_mead start");

    let options = options.unwrap_or_default();
    // 设定反射 rho、扩展 gamma、收缩 psi、回退 sigma 系数
    let rho = 1.0;
    let gamma = 2.0;
    let alpha = 0.5;
    let sigma = 0.5;

    let n = x_0.dim();
    // 设置初始单纯形
    let mut sim = Matrix::new((n + 1, n));
    sim[0] = x_0.clone();
    for k in 0..n {
        let mut y = x_0.clone();
        if y[k] != 0.0 {
            y[k] = (1.0 + 0.05) * y[k];
        } else {
            y[k] = 0.00025;
        }
        sim[k + 1] = y;
    }

    let mut fval_sim = Vector::new(n + 1);

    let mut fun_evals = 1;
    let mut iter = 1;

    for k in 0..n + 1 {
        fval_sim[k] = func(&sim[k])?;
    }

    fun_evals += n;

    let mut sim = fval_sim.zip_sort(&sim);

    trace!(
        "iter: {}, func-count: {}, f(x): {:.4}, procedure: {}",
        0,
        fun_evals,
        fval_sim.min(),
        "init"
    );

    while iter < options.max_iter && fun_evals < options.max_fun_evals {
        // 当顶点距离最大值或者目标值最大值小于一定值时，结束迭代
        let tol_x = (Matrix::from(&sim[1..]) - sim[0].clone())
            .ravel()
            .abs()
            .max();
        let tol_fun = (Vector::from(&fval_sim[1..]) - fval_sim[0]).abs().max();
        if tol_fun <= options.tol_fun && tol_x <= options.tol_x {
            break;
        }

        // 重心
        let x_bar = Matrix::from(&sim[..n]).mean();

        // 反射值
        let x_r = x_bar.clone() * (1.0 + rho) - sim.last().unwrap() * rho;
        let fval_x_r = func(&x_r)?;
        fun_evals += 1;
        trace!(
            "iter: {}, func-count: {}, f(x): {:.4}, procedure: {}",
            iter,
            fun_evals,
            fval_sim.min(),
            "reflect"
        );

        // 控制是否回退
        let mut doshrink = false;

        // 如果反射点为最优点，计算扩展点
        if fval_x_r < fval_sim[0] {
            // 扩展点
            let x_e = x_bar.clone() * (1.0 + rho * gamma) - sim.last().unwrap() * rho * gamma;
            let fval_x_e = func(&x_e)?;
            fun_evals += 1;
            if fval_x_e < fval_x_r {
                // 如果扩展点优于反射点，将最差点替换为扩展点
                sim[n] = x_e;
                fval_sim[n] = fval_x_e;
            } else {
                // 如果扩展点不是最优点，将最差点替换为反射点
                sim[n] = x_r;
                fval_sim[n] = fval_x_r;
            }
            trace!(
                "iter: {}, func-count: {}, f(x): {:.4}, procedure: {}",
                iter,
                fun_evals,
                fval_sim.min(),
                "expand"
            );
        } else {
            // 反射点不是最优点
            if fval_x_r < fval_sim[n - 1] {
                // 如果反射点不是最优点，但是优于第二差点，替换最差点为反射点
                sim[n] = x_r;
                fval_sim[n] = fval_x_r;
            } else {
                // 反射点不是最优点，但是优于第二差点
                if fval_x_r < fval_sim[n] {
                    // 反射点优于最差点
                    // 外收缩点
                    let x_c =
                        x_bar.clone() * (1.0 + alpha * rho) - sim.last().unwrap() * alpha * rho;

                    let fval_x_c = func(&x_c)?;
                    fun_evals += 1;
                    if fval_x_c <= fval_x_r {
                        // 如果收缩点优于反射点
                        sim[n] = x_c;
                        fval_sim[n] = fval_x_c
                    } else {
                        doshrink = true;
                    }
                    trace!(
                        "iter: {}, func-count: {}, f(x): {:.4}, procedure: {}",
                        iter,
                        fun_evals,
                        fval_sim.min(),
                        "contract outside"
                    );
                } else {
                    // 反射点差于最差点
                    // 内收缩点
                    let x_c_c = x_bar.clone() * (1.0 - alpha) + sim.last().unwrap() * alpha;
                    let fval_x_c_c = func(&x_c_c)?;
                    fun_evals += 1;
                    if fval_x_c_c < fval_sim[n] {
                        // 如果内收缩点优于最差点，替代最差点
                        sim[n] = x_c_c;
                        fval_sim[n] = fval_x_c_c;
                    } else {
                        doshrink = true;
                    }
                    trace!(
                        "iter: {}, func-count: {}, f(x): {:.4}, procedure: {}",
                        iter,
                        fun_evals,
                        fval_sim.min(),
                        "contract inside"
                    );
                }
            }
            if doshrink {
                // 回退
                for j in 1..n + 1 {
                    sim[j] = sim[0].clone() + (sim[j].clone() - sim[0].clone()) * sigma;
                    fval_sim[j] = func(&sim[j])?
                }
                fun_evals += n;
                trace!(
                    "iter: {}, func-count: {}, f(x): {:.4}, procedure: {}",
                    iter,
                    fun_evals,
                    fval_sim.min(),
                    "backward"
                );
            }
        }

        sim = fval_sim.zip_sort(&sim);
        iter += 1
    }
    let x = sim[0].clone();
    let fval = fval_sim.min();

    Ok(NelderMeadResult {
        x,
        fval,
        iter,
        fun_evals,
    })
}

#[cfg(test)]
mod algorithm_tests {
    use super::*;
    use crate::utils::test_logger_init;

    #[test]
    fn test_nm() {
        test_logger_init();
        let func = |x: &Vector| Ok(100.0 * (x[1] - x[0].powi(2)).powi(2) + (1.0 - x[1]).powi(2));
        let x_0 = Vector::from(vec![-1.2, 1.0]);
        let options = NelderMeadOptions {
            max_fun_evals: 5000,
            max_iter: 1000,
            tol_fun: 1e-10,
            tol_x: 1e-10,
        };
        let result = nelder_mead(Box::new(func), x_0, Some(options)).unwrap();
        println!("{:#?} {:#?}", result.x, result.fval);
        println!("{:#?} {:#?}", result.iter, result.fun_evals);
    }
}
