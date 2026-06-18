//! Geometry data model: the parametric primitive kinds (`GeometryKind`), their parameter
//! payloads (`GeometryShape` — Box, Cylinder, Snout, tori, dishes, Pyramid, Sphere, Line)
//! plus `FacetGroup`, the per-primitive `Geometry` (shape + accumulated transform), and
//! the `Triangulation` the tessellator produces. Available at parse time; consumed by the
//! tessellator and the instancing shape-key.

use crate::linalg::{BBox3f, Mat3x4f, Vec3f};

// ─── Geometry kinds ──────────────────────────────────────────────────────

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GeometryKind {
    Pyramid,
    Box,
    RectangularTorus,
    CircularTorus,
    EllipticalDish,
    SphericalDish,
    Snout,
    Cylinder,
    Sphere,
    Line,
    FacetGroup,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GeometryType {
    Primitive,
    Obstruction,
    Insulation,
}

// ─── Connection ──────────────────────────────────────────────────────────

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ConnectionFlags(pub u8);

impl ConnectionFlags {
    pub const NONE: Self = Self(0);
    pub const HAS_CIRCULAR_SIDE: Self = Self(1 << 0);
    pub const HAS_RECTANGULAR_SIDE: Self = Self(1 << 1);

    pub fn has(&self, f: ConnectionFlags) -> bool {
        self.0 & f.0 != 0
    }
    pub fn set(&mut self, f: ConnectionFlags) {
        self.0 |= f.0;
    }
}

/// Stored inline – we use indices into a flat array rather than pointers.
/// `geo_idx[n]` is an index into `RvmParser::geometries`, `INVALID` means null.
pub const INVALID_GEO: usize = usize::MAX;

#[derive(Clone, Debug)]
pub struct Connection {
    pub geo_idx: [usize; 2],
    pub offset: [u32; 2],
    pub p: Vec3f,
    pub d: Vec3f,
    pub flags: ConnectionFlags,
}

impl Connection {
    pub fn new() -> Self {
        Self {
            geo_idx: [INVALID_GEO; 2],
            offset: [0; 2],
            p: Vec3f::default(),
            d: Vec3f::default(),
            flags: ConnectionFlags::NONE,
        }
    }
}

// ─── FacetGroup data ─────────────────────────────────────────────────────

#[derive(Clone, Debug, Default)]
pub struct Contour {
    pub vertices: Vec<f32>, // flat: [x0,y0,z0, x1,y1,z1, ...]
}

#[derive(Clone, Debug, Default)]
pub struct Polygon {
    pub contours: Vec<Contour>,
}

// ─── Geometry primitive shapes ───────────────────────────────────────────

#[derive(Clone, Debug)]
pub enum GeometryShape {
    Pyramid {
        bottom: [f32; 2],
        top: [f32; 2],
        offset: [f32; 2],
        height: f32,
    },
    Box {
        lengths: [f32; 3],
    },
    RectangularTorus {
        inner_radius: f32,
        outer_radius: f32,
        height: f32,
        angle: f32,
    },
    CircularTorus {
        offset: f32,
        radius: f32,
        angle: f32,
    },
    EllipticalDish {
        base_radius: f32,
        height: f32,
    },
    SphericalDish {
        base_radius: f32,
        height: f32,
    },
    Snout {
        offset: [f32; 2],
        bshear: [f32; 2],
        tshear: [f32; 2],
        radius_b: f32,
        radius_t: f32,
        height: f32,
    },
    Cylinder {
        radius: f32,
        height: f32,
    },
    Sphere {
        diameter: f32,
    },
    Line {
        a: f32,
        b: f32,
    },
    FacetGroup {
        polygons: Vec<Polygon>,
    },
}

impl GeometryShape {
    /// The `GeometryKind` tag matching this shape variant (the two enums are
    /// parallel). Lets the instanced path re-tessellate a stored shape.
    pub fn kind(&self) -> GeometryKind {
        match self {
            GeometryShape::Pyramid { .. } => GeometryKind::Pyramid,
            GeometryShape::Box { .. } => GeometryKind::Box,
            GeometryShape::RectangularTorus { .. } => GeometryKind::RectangularTorus,
            GeometryShape::CircularTorus { .. } => GeometryKind::CircularTorus,
            GeometryShape::EllipticalDish { .. } => GeometryKind::EllipticalDish,
            GeometryShape::SphericalDish { .. } => GeometryKind::SphericalDish,
            GeometryShape::Snout { .. } => GeometryKind::Snout,
            GeometryShape::Cylinder { .. } => GeometryKind::Cylinder,
            GeometryShape::Sphere { .. } => GeometryKind::Sphere,
            GeometryShape::Line { .. } => GeometryKind::Line,
            GeometryShape::FacetGroup { .. } => GeometryKind::FacetGroup,
        }
    }
}

// ─── Geometry node ───────────────────────────────────────────────────────

/// `connections[i]` is an index into `RvmParser::connections` or INVALID_GEO.
#[derive(Clone, Debug)]
pub struct Geometry {
    pub kind: GeometryKind,
    pub geo_type: GeometryType,
    pub transparency: u32,
    pub m_3x4: Mat3x4f,
    pub bbox_local: BBox3f,
    pub bbox_world: BBox3f,
    pub sample_start_angle: f32,
    pub connections: [usize; 6], // indices into connections vec, or INVALID_GEO
    pub shape: GeometryShape,
}

impl Geometry {
    pub fn new(kind: GeometryKind, shape: GeometryShape) -> Self {
        Self {
            kind,
            geo_type: GeometryType::Primitive,
            transparency: 0,
            m_3x4: Mat3x4f::default(),
            bbox_local: BBox3f::empty(),
            bbox_world: BBox3f::empty(),
            sample_start_angle: 0.0,
            connections: [INVALID_GEO; 6],
            shape,
        }
    }
}

// ─── Triangulation result ────────────────────────────────────────────────

#[derive(Clone, Debug, Default)]
pub struct Triangulation {
    pub vertices: Vec<f32>, // flat [x,y,z, ...]
    pub normals: Vec<f32>,  // flat [nx,ny,nz, ...], same count as vertices
    pub indices: Vec<u32>,
    pub vertices_n: u32,
    pub triangles_n: u32,
    pub id: u32,
    pub color: u32,
    pub error: f32,
}
