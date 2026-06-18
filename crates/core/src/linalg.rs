//! Minimal linear-algebra types used throughout the converter: `Vec3f`, the `Mat3x4f`
//! affine transform (RVM's row layout, with `identity`/`get_scale`/multiply helpers), and
//! `BBox3f` with `transform_bbox`. Kept dependency-free so the core stays wasm-friendly.

/// 3-component float vector
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Vec3f {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vec3f {
    #[inline]
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }
    #[inline]
    pub fn splat(v: f32) -> Self {
        Self { x: v, y: v, z: v }
    }
    #[inline]
    pub fn from_slice(s: &[f32]) -> Self {
        Self {
            x: s[0],
            y: s[1],
            z: s[2],
        }
    }
}

impl std::ops::Add for Vec3f {
    type Output = Self;
    fn add(self, b: Self) -> Self {
        Self::new(self.x + b.x, self.y + b.y, self.z + b.z)
    }
}
impl std::ops::Sub for Vec3f {
    type Output = Self;
    fn sub(self, b: Self) -> Self {
        Self::new(self.x - b.x, self.y - b.y, self.z - b.z)
    }
}
impl std::ops::Mul<f32> for Vec3f {
    type Output = Self;
    fn mul(self, s: f32) -> Self {
        Self::new(self.x * s, self.y * s, self.z * s)
    }
}
impl std::ops::Mul<Vec3f> for f32 {
    type Output = Vec3f;
    fn mul(self, v: Vec3f) -> Vec3f {
        v * self
    }
}

#[inline]
pub fn dot(a: Vec3f, b: Vec3f) -> f32 {
    a.x * b.x + a.y * b.y + a.z * b.z
}

#[inline]
pub fn cross(a: Vec3f, b: Vec3f) -> Vec3f {
    Vec3f::new(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x,
    )
}

#[inline]
pub fn length(a: Vec3f) -> f32 {
    dot(a, a).sqrt()
}

#[inline]
pub fn length_sq(a: Vec3f) -> f32 {
    dot(a, a)
}

#[inline]
pub fn distance_sq(a: Vec3f, b: Vec3f) -> f32 {
    length_sq(a - b)
}

#[inline]
pub fn normalize(a: Vec3f) -> Vec3f {
    a * (1.0 / length(a))
}

#[inline]
pub fn vec_min(a: Vec3f, b: Vec3f) -> Vec3f {
    Vec3f::new(a.x.min(b.x), a.y.min(b.y), a.z.min(b.z))
}
#[inline]
pub fn vec_max(a: Vec3f, b: Vec3f) -> Vec3f {
    Vec3f::new(a.x.max(b.x), a.y.max(b.y), a.z.max(b.z))
}

// ─── double precision vector ───────────────────────────────────────────────
#[derive(Clone, Copy, Debug, Default)]
pub struct Vec3d {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vec3d {
    #[inline]
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }
    #[inline]
    pub fn from_f32_slice(s: &[f32]) -> Self {
        Self {
            x: s[0] as f64,
            y: s[1] as f64,
            z: s[2] as f64,
        }
    }
}

impl std::ops::Add for Vec3d {
    type Output = Self;
    fn add(self, b: Self) -> Self {
        Self::new(self.x + b.x, self.y + b.y, self.z + b.z)
    }
}
impl std::ops::Mul<f64> for Vec3d {
    type Output = Self;
    fn mul(self, s: f64) -> Self {
        Self::new(self.x * s, self.y * s, self.z * s)
    }
}

// ─── 3×4 transform matrices ────────────────────────────────────────────────

/// Column-major 3×3 matrix
#[derive(Clone, Copy, Debug, Default)]
pub struct Mat3f {
    /// Stored column-major: `cols[col][row]` (each col is a Vec3f)
    pub cols: [Vec3f; 3],
}

impl Mat3f {
    pub fn from_data(d: &[f32; 9]) -> Self {
        // C++ storage: m00 m10 m20 | m01 m11 m21 | m02 m12 m22
        // i.e. d[0..3] = col0, d[3..6] = col1, d[6..9] = col2
        Self {
            cols: [
                Vec3f::new(d[0], d[1], d[2]),
                Vec3f::new(d[3], d[4], d[5]),
                Vec3f::new(d[6], d[7], d[8]),
            ],
        }
    }

    pub fn get_scale(&self) -> f32 {
        let sx = length(self.cols[0]);
        let sy = length(self.cols[1]);
        let sz = length(self.cols[2]);
        sx.max(sy).max(sz)
    }
}

/// Column-major 3×4 matrix (rotation+scale 3×3 + translation column)
#[derive(Clone, Copy, Debug, Default)]
pub struct Mat3x4f {
    pub data: [f32; 12],
}

impl Mat3x4f {
    /// Identity affine transform (column-major: 3×3 identity, zero translation).
    pub fn identity() -> Self {
        Self {
            data: [
                1.0, 0.0, 0.0, // col0
                0.0, 1.0, 0.0, // col1
                0.0, 0.0, 1.0, // col2
                0.0, 0.0, 0.0, // translation
            ],
        }
    }

    /// Multiply matrix by Vec3f (affine transform with translation)
    pub fn mul_vec(&self, v: Vec3f) -> Vec3f {
        let d = &self.data;
        Vec3f::new(
            d[0] * v.x + d[3] * v.y + d[6] * v.z + d[9],
            d[1] * v.x + d[4] * v.y + d[7] * v.z + d[10],
            d[2] * v.x + d[5] * v.y + d[8] * v.z + d[11],
        )
    }

    pub fn rotation_mat3(&self) -> Mat3f {
        let d = &self.data;
        let mut arr = [0f32; 9];
        arr.copy_from_slice(&d[0..9]);
        Mat3f::from_data(&arr)
    }

    pub fn get_scale(&self) -> f32 {
        self.rotation_mat3().get_scale()
    }
}

/// Column-major 3×4 double-precision matrix
#[derive(Clone, Copy, Debug, Default)]
pub struct Mat3x4d {
    pub data: [f64; 12],
}

impl Mat3x4d {
    pub fn from_f32(src: &[f32; 12]) -> Self {
        let mut d = [0f64; 12];
        for (i, &v) in src.iter().enumerate() {
            d[i] = v as f64;
        }
        Self { data: d }
    }

    pub fn mul_vec(&self, v: Vec3d) -> Vec3d {
        let d = &self.data;
        Vec3d::new(
            d[0] * v.x + d[3] * v.y + d[6] * v.z + d[9],
            d[1] * v.x + d[4] * v.y + d[7] * v.z + d[10],
            d[2] * v.x + d[5] * v.y + d[8] * v.z + d[11],
        )
    }
}

// ─── axis-aligned bounding box ────────────────────────────────────────────

#[derive(Clone, Copy, Debug)]
pub struct BBox3f {
    pub min: Vec3f,
    pub max: Vec3f,
}

impl BBox3f {
    pub fn empty() -> Self {
        Self {
            min: Vec3f::splat(f32::MAX),
            max: Vec3f::splat(f32::MIN),
        }
    }
    pub fn is_empty(&self) -> bool {
        self.max.x < self.min.x
    }
    pub fn engulf_point(&mut self, p: Vec3f) {
        self.min = vec_min(self.min, p);
        self.max = vec_max(self.max, p);
    }
    pub fn engulf_box(&mut self, other: &BBox3f) {
        self.min = vec_min(self.min, other.min);
        self.max = vec_max(self.max, other.max);
    }
    pub fn diagonal(&self) -> f32 {
        length(self.max - self.min)
    }
}

pub fn transform_bbox(m: &Mat3x4f, bbox: &BBox3f) -> BBox3f {
    let corners = [
        Vec3f::new(bbox.min.x, bbox.min.y, bbox.min.z),
        Vec3f::new(bbox.min.x, bbox.min.y, bbox.max.z),
        Vec3f::new(bbox.min.x, bbox.max.y, bbox.min.z),
        Vec3f::new(bbox.min.x, bbox.max.y, bbox.max.z),
        Vec3f::new(bbox.max.x, bbox.min.y, bbox.min.z),
        Vec3f::new(bbox.max.x, bbox.min.y, bbox.max.z),
        Vec3f::new(bbox.max.x, bbox.max.y, bbox.min.z),
        Vec3f::new(bbox.max.x, bbox.max.y, bbox.max.z),
    ];
    let transformed: Vec<Vec3f> = corners.iter().map(|&p| m.mul_vec(p)).collect();
    let mut result = BBox3f::empty();
    for p in &transformed {
        result.engulf_point(*p);
    }
    result
}
