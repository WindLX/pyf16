use super::matrix::Matrix;
use rayon::prelude::*;
use serde::{Deserialize, Serialize};
use std::{
    cmp::Ordering,
    ops::{
        Add, AddAssign, Div, DivAssign, Index, IndexMut, Mul, MulAssign, Range, RangeFrom,
        RangeFull, RangeTo, Sub, SubAssign,
    },
};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Vector {
    dim: usize,
    pub data: Vec<f64>,
}

impl Vector {
    pub fn new(dim: usize) -> Self {
        Self {
            dim,
            data: vec![0.0; dim],
        }
    }

    pub fn dim(&self) -> usize {
        self.dim
    }

    pub fn dim_eq(&self, other: &Self) -> bool {
        self.dim == other.dim
    }

    pub fn norm_sq(&self) -> f64 {
        self.data.par_iter().map(|x| x.powi(2)).sum::<f64>()
    }

    pub fn norm(&self) -> f64 {
        self.data.par_iter().map(|x| x.powi(2)).sum::<f64>().sqrt()
    }

    pub fn fill(&mut self, value: f64) {
        self.data.par_iter_mut().for_each(|x| *x = value);
    }

    pub fn normalize(&self) -> Self {
        let norm = self.norm();
        let mut vec = self.clone();
        vec.data.par_iter_mut().for_each(|x| *x /= norm);
        vec
    }

    pub fn dot(&self, other: &Self) -> f64 {
        assert!(self.dim_eq(other));
        self.data
            .par_iter()
            .zip(other.data.par_iter())
            .map(|(a, b)| a * b)
            .sum()
    }

    pub fn cross(&self, other: &Self) -> Self {
        assert!(self.dim_eq(other));
        assert!(self.dim == 3);
        let mut out = Self::zero(self.dim);
        out[0] = self[1] * other[2] - self[2] * other[1];
        out[1] = self[2] * other[0] - self[0] * other[2];
        out[2] = self[0] * other[1] - self[1] * other[0];
        out
    }

    pub fn zero(dim: usize) -> Self {
        Self::new(dim)
    }

    pub fn zero_like(&self) -> Self {
        Self::zero(self.dim)
    }

    pub fn ones(dim: usize) -> Self {
        let mut ones = Self::new(dim);
        ones.fill(1.0);
        ones
    }

    pub fn ones_like(&self) -> Self {
        Self::ones(self.dim)
    }

    pub fn sort(&mut self) {
        self.data.par_sort_by(|a, b| a.partial_cmp(b).unwrap());
    }

    pub fn zip_sort(&mut self, other: &Matrix) -> Matrix {
        let data = self.data.clone();
        let mut data = data
            .iter()
            .zip(other.data.iter())
            .map(|(a, b)| (*a, b.clone()))
            .collect::<Vec<_>>();
        data.par_sort_by(|a, b| {
            if a.0.is_nan() || b.0.is_nan() {
                return Ordering::Equal;
            };
            a.0.partial_cmp(&b.0).unwrap()
        });
        let mut vec_a = Vec::with_capacity(data.len());
        let mut vec_b = Vec::with_capacity(data.len());

        let len = data.len().clone();
        for i in 0..len {
            vec_a.push(data[i].0.clone())
        }
        for i in 0..len {
            vec_b.push(data[i].1.clone())
        }
        self.data = vec_a;
        Matrix::from(vec_b)
    }

    pub fn arg_sort(&self) -> Vec<usize> {
        let data = self.data.clone();
        let mut data = data
            .par_iter()
            .enumerate()
            .map(|(i, a)| (a, i))
            .collect::<Vec<_>>();
        data.par_sort_by(|a, b| a.0.partial_cmp(b.0).unwrap());
        data.par_iter().map(|(_, i)| *i).collect()
    }

    pub fn abs(&self) -> Self {
        Vector::from(self.data.par_iter().map(|x| x.abs()).collect::<Vec<_>>())
    }

    pub fn max(&self) -> f64 {
        self.data
            .iter()
            .max_by(|a, b| {
                if a.is_nan() || b.is_nan() {
                    return Ordering::Equal;
                }
                a.partial_cmp(b).unwrap()
            })
            .unwrap()
            .clone()
    }

    pub fn min(&self) -> f64 {
        self.data
            .iter()
            .min_by(|a, b| {
                if a.is_nan() || b.is_nan() {
                    return Ordering::Equal;
                }
                a.partial_cmp(b).unwrap()
            })
            .unwrap()
            .clone()
    }

    pub fn map(&self, f: impl Fn(&f64) -> f64 + Send + Sync) -> Self {
        Vector::from(self.data.par_iter().map(f).collect::<Vec<_>>())
    }
}

impl From<Vec<f64>> for Vector {
    fn from(value: Vec<f64>) -> Self {
        Self {
            dim: value.len(),
            data: value,
        }
    }
}

impl From<&[f64]> for Vector {
    fn from(value: &[f64]) -> Self {
        Self {
            dim: value.len(),
            data: value.to_vec(),
        }
    }
}

impl PartialEq for Vector {
    fn eq(&self, other: &Self) -> bool {
        self.dim == other.dim && self.data == other.data
    }
}

impl Eq for Vector {}

impl Index<usize> for Vector {
    type Output = f64;
    fn index(&self, index: usize) -> &Self::Output {
        &self.data[index]
    }
}

impl IndexMut<usize> for Vector {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        &mut self.data[index]
    }
}

impl Index<RangeFrom<usize>> for Vector {
    type Output = [f64];

    fn index(&self, index: RangeFrom<usize>) -> &Self::Output {
        &self.data[index]
    }
}

impl Index<RangeTo<usize>> for Vector {
    type Output = [f64];

    fn index(&self, index: RangeTo<usize>) -> &Self::Output {
        &self.data[index]
    }
}

impl Index<Range<usize>> for Vector {
    type Output = [f64];

    fn index(&self, index: Range<usize>) -> &Self::Output {
        &self.data[index]
    }
}

impl Index<RangeFull> for Vector {
    type Output = [f64];

    fn index(&self, index: RangeFull) -> &Self::Output {
        &self.data[index]
    }
}

impl Add for Vector {
    type Output = Self;
    fn add(self, rhs: Self) -> Self::Output {
        assert!(self.dim_eq(&rhs));
        Self {
            dim: self.dim,
            data: self
                .data
                .iter()
                .zip(rhs.data.iter())
                .map(|(a, b)| a + b)
                .collect(),
        }
    }
}

impl AddAssign for Vector {
    fn add_assign(&mut self, rhs: Self) {
        assert!(self.dim_eq(&rhs));
        self.data
            .iter_mut()
            .zip(rhs.data.iter())
            .for_each(|(a, b)| *a += b);
    }
}

impl Sub for Vector {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self::Output {
        assert!(self.dim_eq(&rhs));
        Self {
            dim: self.dim,
            data: self
                .data
                .iter()
                .zip(rhs.data.iter())
                .map(|(a, b)| a - b)
                .collect(),
        }
    }
}

impl SubAssign for Vector {
    fn sub_assign(&mut self, rhs: Self) {
        assert!(self.dim_eq(&rhs));
        self.data
            .iter_mut()
            .zip(rhs.data.iter())
            .for_each(|(a, b)| *a -= b);
    }
}

impl Add<f64> for Vector {
    type Output = Self;
    fn add(self, rhs: f64) -> Self::Output {
        Self {
            dim: self.dim,
            data: self.data.par_iter().map(|a| a + rhs).collect(),
        }
    }
}

impl AddAssign<f64> for Vector {
    fn add_assign(&mut self, rhs: f64) {
        self.data.par_iter_mut().for_each(|a| *a += rhs);
    }
}

impl Sub<f64> for Vector {
    type Output = Self;
    fn sub(self, rhs: f64) -> Self::Output {
        Self {
            dim: self.dim,
            data: self.data.par_iter().map(|a| a - rhs).collect(),
        }
    }
}

impl SubAssign<f64> for Vector {
    fn sub_assign(&mut self, rhs: f64) {
        self.data.par_iter_mut().for_each(|a| *a -= rhs);
    }
}

impl Mul for Vector {
    type Output = Self;
    fn mul(self, rhs: Self) -> Self::Output {
        assert!(self.dim_eq(&rhs));
        Self {
            dim: self.dim,
            data: self
                .data
                .par_iter()
                .zip(rhs.data.par_iter())
                .map(|(a, b)| a * b)
                .collect(),
        }
    }
}

impl MulAssign for Vector {
    fn mul_assign(&mut self, rhs: Self) {
        assert!(self.dim_eq(&rhs));
        self.data
            .par_iter_mut()
            .zip(rhs.data.par_iter())
            .for_each(|(a, b)| *a *= b);
    }
}

impl Div for Vector {
    type Output = Self;
    fn div(self, rhs: Self) -> Self::Output {
        assert!(self.dim_eq(&rhs));
        Self {
            dim: self.dim,
            data: self
                .data
                .par_iter()
                .zip(rhs.data.par_iter())
                .map(|(a, b)| a / b)
                .collect(),
        }
    }
}

impl DivAssign for Vector {
    fn div_assign(&mut self, rhs: Self) {
        assert!(self.dim_eq(&rhs));
        self.data
            .par_iter_mut()
            .zip(rhs.data.par_iter())
            .for_each(|(a, b)| *a /= b);
    }
}

impl Mul<f64> for Vector {
    type Output = Self;
    fn mul(self, rhs: f64) -> Self::Output {
        Self {
            dim: self.dim,
            data: self.data.iter().map(|a| a * rhs).collect(),
        }
    }
}

impl MulAssign<f64> for Vector {
    fn mul_assign(&mut self, rhs: f64) {
        self.data.iter_mut().for_each(|a| *a *= rhs);
    }
}

impl Div<f64> for Vector {
    type Output = Self;
    fn div(self, rhs: f64) -> Self::Output {
        Self {
            dim: self.dim,
            data: self.data.iter().map(|a| a / rhs).collect(),
        }
    }
}

impl DivAssign<f64> for Vector {
    fn div_assign(&mut self, rhs: f64) {
        self.data.iter_mut().for_each(|a| *a /= rhs);
    }
}

#[cfg(test)]
mod utils_tests {
    use super::*;

    #[test]
    fn test_add() {
        let v_a = Vector::from(vec![1., 2., 3.]);
        let v_b = Vector::from(vec![4., 5., 6.]);
        let result = v_a + v_b;
        assert_eq!(result, Vector::from(vec![5., 7., 9.]));
    }

    #[test]
    fn test_zip_sort() {
        let mut v = Vector::from(vec![2., 1., 3.]);
        let m = Matrix::from(vec![vec![1., 2., 3.], vec![3., 4., 5.], vec![5., 6., 7.]]);
        let m = v.zip_sort(&m);
        assert_eq!(
            m,
            Matrix::from(vec![vec![3., 4., 5.], vec![1., 2., 3.], vec![5., 6., 7.]])
        )
    }
}
