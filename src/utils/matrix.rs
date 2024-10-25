use super::vector::Vector;
use rayon::prelude::*;
use std::{
    ops::{Add, AddAssign, Div, DivAssign, Index, IndexMut, Mul, MulAssign, Sub, SubAssign},
    ops::{RangeFrom, RangeTo},
    sync::{Arc, Mutex},
};

#[derive(Debug, Clone)]
pub struct Matrix {
    dim: usize,
    pub data: Vec<Vector>,
}

impl Matrix {
    pub fn new(shape: (usize, usize)) -> Self {
        Matrix {
            dim: shape.0,
            data: vec![Vector::new(shape.1); shape.0],
        }
    }

    pub fn dim(&self) -> usize {
        self.dim
    }

    pub fn shape(&self) -> (usize, usize) {
        (self.dim, self.data[0].dim())
    }

    pub fn dim_eq(&self, other: &Self) -> bool {
        self.dim == other.dim
    }

    pub fn shape_eq(&self, other: &Self) -> bool {
        self.shape() == other.shape()
    }

    pub fn fill(&self, value: f64) {
        let mut vec = self.clone();
        vec.data.par_iter_mut().for_each(|x| x.fill(value));
    }

    pub fn zero(dim: (usize, usize)) -> Self {
        Self::new(dim)
    }

    pub fn zero_like(&self) -> Self {
        Self::zero(self.shape())
    }

    pub fn ones(dim: (usize, usize)) -> Self {
        let matrix = Self::new(dim);
        matrix.fill(1.0);
        matrix
    }

    pub fn ones_like(&self) -> Self {
        Self::ones(self.shape())
    }

    pub fn ravel(&self) -> Vector {
        let mut data = self.data.clone();
        let mut new_data = Vec::new();
        data.iter_mut()
            .for_each(|x| new_data.extend(x.data.clone()));
        Vector::from(new_data)
    }

    pub fn sum(&self) -> Vector {
        let matrix = self.clone();
        let res = Vector::zero(self.shape().1);
        let res_lock = Arc::new(Mutex::new(res));
        matrix
            .data
            .par_iter()
            .for_each(|x| *(res_lock.lock().unwrap()) += x.clone());
        let new_data = Arc::try_unwrap(res_lock).unwrap().into_inner().unwrap();
        Vector::from(new_data)
    }

    pub fn mean(&self) -> Vector {
        self.sum() / self.shape().0 as f64
    }

    pub fn last(&self) -> Option<Vector> {
        match self.data.last() {
            Some(x) => Some(Vector::from(x.clone())),
            None => None,
        }
    }

    pub fn linespace(start: &Vector, end: &Vector, n: usize) -> Self {
        let mut data = Vec::with_capacity(n);
        let mut i = 0;
        while i < n {
            data.push(start.clone() + (end.clone() - start.clone()) * (i as f64) / (n as f64));
            i += 1;
        }
        Self::from(data)
    }
}

impl From<Vec<Vector>> for Matrix {
    fn from(value: Vec<Vector>) -> Self {
        Self {
            dim: value.len(),
            data: value,
        }
    }
}

impl From<&[Vector]> for Matrix {
    fn from(value: &[Vector]) -> Self {
        Self {
            dim: value.len(),
            data: value.to_vec(),
        }
    }
}

impl From<Vec<Vec<f64>>> for Matrix {
    fn from(value: Vec<Vec<f64>>) -> Self {
        Self {
            dim: value.len(),
            data: value.par_iter().map(|v| Vector::from(v.clone())).collect(),
        }
    }
}

impl PartialEq for Matrix {
    fn eq(&self, other: &Self) -> bool {
        self.dim == other.dim && self.data == other.data
    }
}

impl Eq for Matrix {}

impl Index<usize> for Matrix {
    type Output = Vector;
    fn index(&self, index: usize) -> &Self::Output {
        &self.data[index]
    }
}

impl IndexMut<usize> for Matrix {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        &mut self.data[index]
    }
}

impl Index<(usize, usize)> for Matrix {
    type Output = f64;
    fn index(&self, index: (usize, usize)) -> &Self::Output {
        &self.data[index.0][index.1]
    }
}

impl IndexMut<(usize, usize)> for Matrix {
    fn index_mut(&mut self, index: (usize, usize)) -> &mut Self::Output {
        &mut self.data[index.0][index.1]
    }
}

impl Index<RangeFrom<usize>> for Matrix {
    type Output = [Vector];

    fn index(&self, index: RangeFrom<usize>) -> &Self::Output {
        &self.data[index]
    }
}

impl Index<RangeTo<usize>> for Matrix {
    type Output = [Vector];

    fn index(&self, index: RangeTo<usize>) -> &Self::Output {
        &self.data[index]
    }
}

impl Add for Matrix {
    type Output = Self;
    fn add(self, rhs: Self) -> Self::Output {
        assert!(self.dim_eq(&rhs));
        Self {
            dim: self.dim,
            data: self
                .data
                .par_iter()
                .zip(rhs.data.par_iter())
                .map(|(a, b)| a.clone() + b.clone())
                .collect(),
        }
    }
}

impl AddAssign for Matrix {
    fn add_assign(&mut self, rhs: Self) {
        assert!(self.dim_eq(&rhs));
        self.data
            .par_iter_mut()
            .zip(rhs.data.par_iter())
            .for_each(|(a, b)| *a += b.clone());
    }
}

impl Add<Vector> for Matrix {
    type Output = Self;
    fn add(self, rhs: Vector) -> Self::Output {
        assert!(self.shape().1 == rhs.dim());
        Self {
            dim: self.dim,
            data: self
                .data
                .par_iter()
                .map(|a| a.clone() + rhs.clone())
                .collect(),
        }
    }
}

impl AddAssign<Vector> for Matrix {
    fn add_assign(&mut self, rhs: Vector) {
        assert!(self.shape().1 == rhs.dim());
        self.data.par_iter_mut().for_each(|a| *a += rhs.clone());
    }
}

impl Sub for Matrix {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self::Output {
        assert!(self.dim_eq(&rhs));
        Self {
            dim: self.dim,
            data: self
                .data
                .par_iter()
                .zip(rhs.data.par_iter())
                .map(|(a, b)| a.clone() - b.clone())
                .collect(),
        }
    }
}

impl SubAssign for Matrix {
    fn sub_assign(&mut self, rhs: Self) {
        assert!(self.dim_eq(&rhs));
        self.data
            .par_iter_mut()
            .zip(rhs.data.par_iter())
            .for_each(|(a, b)| *a -= b.clone());
    }
}

impl Sub<Vector> for Matrix {
    type Output = Self;
    fn sub(self, rhs: Vector) -> Self::Output {
        assert!(self.shape().1 == rhs.dim());
        Self {
            dim: self.dim,
            data: self
                .data
                .par_iter()
                .map(|a| a.clone() - rhs.clone())
                .collect(),
        }
    }
}

impl SubAssign<Vector> for Matrix {
    fn sub_assign(&mut self, rhs: Vector) {
        assert!(self.shape().1 == rhs.dim());
        self.data.par_iter_mut().for_each(|a| *a -= rhs.clone());
    }
}

impl Mul<f64> for Matrix {
    type Output = Self;
    fn mul(self, rhs: f64) -> Self::Output {
        Self {
            dim: self.dim,
            data: self.data.par_iter().map(|a| a.clone() * rhs).collect(),
        }
    }
}

impl MulAssign<f64> for Matrix {
    fn mul_assign(&mut self, rhs: f64) {
        self.data.par_iter_mut().for_each(|a| *a *= rhs);
    }
}

impl Div<f64> for Matrix {
    type Output = Self;
    fn div(self, rhs: f64) -> Self::Output {
        Self {
            dim: self.dim,
            data: self.data.par_iter().map(|a| a.clone() / rhs).collect(),
        }
    }
}

impl DivAssign<f64> for Matrix {
    fn div_assign(&mut self, rhs: f64) {
        self.data.par_iter_mut().for_each(|a| *a /= rhs);
    }
}
