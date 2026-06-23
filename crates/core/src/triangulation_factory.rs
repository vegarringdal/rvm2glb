//! Per-primitive tessellation: turns each `GeometryShape` into a `Triangulation`
//! (positions + indices). Circle segment counts come from a chord-height (sagitta)
//! rule scaled by `tolerance` (min 3 segments to match the C++ reference; optional
//! round-to-4 via `align_segments`). Also draws RVM Line primitives as a 2-plate "+"
//! cross (`line_cross`). The math is a close port of the C++/cdyk reference.

use crate::geometry::*;
use crate::linalg::*;
use std::f32::consts::PI;

const TWO_PI: f32 = 2.0 * PI;
const HALF_PI: f32 = PI / 2.0;

// ─── helpers ─────────────────────────────────────────────────────────────

fn quad_indices(out: &mut Vec<u32>, o: u32, v0: u32, v1: u32, v2: u32, v3: u32) {
    out.extend_from_slice(&[o + v0, o + v1, o + v2, o + v2, o + v3, o + v0]);
}

fn push_vertex(verts: &mut Vec<f32>, x: f32, y: f32, z: f32) {
    verts.push(x);
    verts.push(y);
    verts.push(z);
}

/// Fan-tessellate a ring of N vertices referenced by `src`.
/// Writes triangle indices into `out`.
fn tessellate_circle(out: &mut Vec<u32>, mut src: Vec<u32>) {
    let mut tmp: Vec<u32> = Vec::new();
    let mut n = src.len();
    while n >= 3 {
        tmp.clear();
        let mut i = 0;
        while i + 2 < n {
            out.push(src[i]);
            out.push(src[i + 1]);
            out.push(src[i + 2]);
            tmp.push(src[i]);
            i += 2;
        }
        while i < n {
            tmp.push(src[i]);
            i += 1;
        }
        n = tmp.len();
        std::mem::swap(&mut src, &mut tmp);
    }
}

// ─── Interface checking (for cap removal) ────────────────────────────────

#[derive(Debug)]
enum Interface {
    Undefined,
    Circular { radius: f32 },
    Square { p: [Vec3f; 4] },
}

fn get_interface(geo: &Geometry, connections: &[Connection], o: usize) -> Interface {
    let con_idx = geo.connections[o];
    if con_idx == INVALID_GEO {
        return Interface::Undefined;
    }
    let con = &connections[con_idx];
    let scale = geo.m_3x4.get_scale();

    match &geo.shape {
        GeometryShape::Pyramid {
            bottom,
            top,
            offset,
            height,
        } => {
            let (bx, by) = (0.5 * bottom[0], 0.5 * bottom[1]);
            let (tx, ty) = (0.5 * top[0], 0.5 * top[1]);
            let (ox, oy) = (0.5 * offset[0], 0.5 * offset[1]);
            let h2 = 0.5 * height;
            let quad = [
                [
                    Vec3f::new(-bx - ox, -by - oy, -h2),
                    Vec3f::new(bx - ox, -by - oy, -h2),
                    Vec3f::new(bx - ox, by - oy, -h2),
                    Vec3f::new(-bx - ox, by - oy, -h2),
                ],
                [
                    Vec3f::new(-tx + ox, -ty + oy, h2),
                    Vec3f::new(tx + ox, -ty + oy, h2),
                    Vec3f::new(tx + ox, ty + oy, h2),
                    Vec3f::new(-tx + ox, ty + oy, h2),
                ],
            ];
            let m = &geo.m_3x4;
            let mut p = [Vec3f::default(); 4];
            if o < 4 {
                let oo = (o + 1) & 3;
                p[0] = m.mul_vec(quad[0][o]);
                p[1] = m.mul_vec(quad[0][oo]);
                p[2] = m.mul_vec(quad[1][oo]);
                p[3] = m.mul_vec(quad[1][o]);
            } else {
                for k in 0..4 {
                    p[k] = m.mul_vec(quad[o - 4][k]);
                }
            }
            Interface::Square { p }
        }
        GeometryShape::Box { lengths } => {
            let (xp, xm) = (0.5 * lengths[0], -0.5 * lengths[0]);
            let (yp, ym) = (0.5 * lengths[1], -0.5 * lengths[1]);
            let (zp, zm) = (0.5 * lengths[2], -0.5 * lengths[2]);
            let v: [[Vec3f; 4]; 6] = [
                [
                    Vec3f::new(xm, ym, zp),
                    Vec3f::new(xm, yp, zp),
                    Vec3f::new(xm, yp, zm),
                    Vec3f::new(xm, ym, zm),
                ],
                [
                    Vec3f::new(xp, ym, zm),
                    Vec3f::new(xp, yp, zm),
                    Vec3f::new(xp, yp, zp),
                    Vec3f::new(xp, ym, zp),
                ],
                [
                    Vec3f::new(xp, ym, zm),
                    Vec3f::new(xp, ym, zp),
                    Vec3f::new(xm, ym, zp),
                    Vec3f::new(xm, ym, zm),
                ],
                [
                    Vec3f::new(xm, yp, zm),
                    Vec3f::new(xm, yp, zp),
                    Vec3f::new(xp, yp, zp),
                    Vec3f::new(xp, yp, zm),
                ],
                [
                    Vec3f::new(xm, yp, zm),
                    Vec3f::new(xp, yp, zm),
                    Vec3f::new(xp, ym, zm),
                    Vec3f::new(xm, ym, zm),
                ],
                [
                    Vec3f::new(xm, ym, zp),
                    Vec3f::new(xp, ym, zp),
                    Vec3f::new(xp, yp, zp),
                    Vec3f::new(xm, yp, zp),
                ],
            ];
            let m = &geo.m_3x4;
            let p = [
                m.mul_vec(v[o][0]),
                m.mul_vec(v[o][1]),
                m.mul_vec(v[o][2]),
                m.mul_vec(v[o][3]),
            ];
            Interface::Square { p }
        }
        GeometryShape::RectangularTorus {
            outer_radius,
            inner_radius,
            height,
            angle,
        } => {
            let h2 = 0.5 * height;
            let sq = [
                [*outer_radius, -h2],
                [*inner_radius, -h2],
                [*inner_radius, h2],
                [*outer_radius, h2],
            ];
            let m = &geo.m_3x4;
            let p = if o == 0 {
                [
                    m.mul_vec(Vec3f::new(sq[0][0], 0.0, sq[0][1])),
                    m.mul_vec(Vec3f::new(sq[1][0], 0.0, sq[1][1])),
                    m.mul_vec(Vec3f::new(sq[2][0], 0.0, sq[2][1])),
                    m.mul_vec(Vec3f::new(sq[3][0], 0.0, sq[3][1])),
                ]
            } else {
                [
                    m.mul_vec(Vec3f::new(
                        sq[0][0] * angle.cos(),
                        sq[0][0] * angle.sin(),
                        sq[0][1],
                    )),
                    m.mul_vec(Vec3f::new(
                        sq[1][0] * angle.cos(),
                        sq[1][0] * angle.sin(),
                        sq[1][1],
                    )),
                    m.mul_vec(Vec3f::new(
                        sq[2][0] * angle.cos(),
                        sq[2][0] * angle.sin(),
                        sq[2][1],
                    )),
                    m.mul_vec(Vec3f::new(
                        sq[3][0] * angle.cos(),
                        sq[3][0] * angle.sin(),
                        sq[3][1],
                    )),
                ]
            };
            Interface::Square { p }
        }
        GeometryShape::CircularTorus { radius, .. } => Interface::Circular {
            radius: scale * radius,
        },
        GeometryShape::EllipticalDish { base_radius, .. } => Interface::Circular {
            radius: scale * base_radius,
        },
        GeometryShape::SphericalDish {
            base_radius,
            height,
        } => {
            let r_circ = *base_radius;
            let h = *height;
            let r_sphere = (r_circ * r_circ + h * h) / (2.0 * h);
            Interface::Circular {
                radius: scale * r_sphere,
            }
        }
        GeometryShape::Snout {
            radius_b, radius_t, ..
        } => {
            let is_first = con.geo_idx[0] == INVALID_GEO; // approximation; see note
            let r = if con.offset[if is_first { 0 } else { 1 }] == 0 {
                *radius_b
            } else {
                *radius_t
            };
            Interface::Circular { radius: scale * r }
        }
        GeometryShape::Cylinder { radius, .. } => Interface::Circular {
            radius: scale * radius,
        },
        _ => Interface::Undefined,
    }
}

fn do_interfaces_match(
    geo: &Geometry,
    geo_all: &[Geometry],
    connections: &[Connection],
    con_idx: usize,
) -> bool {
    let con = &connections[con_idx];
    // Determine which side this geo is
    let is_first = geo as *const _
        == geo_all
            .get(con.geo_idx[0])
            .map(|g| g as *const _)
            .unwrap_or(std::ptr::null());
    let (this_idx, that_idx) = if is_first {
        (0usize, 1usize)
    } else {
        (1usize, 0usize)
    };

    if con.geo_idx[this_idx] == INVALID_GEO || con.geo_idx[that_idx] == INVALID_GEO {
        return false;
    }

    let this_geo = &geo_all[con.geo_idx[this_idx]];
    let that_geo = &geo_all[con.geo_idx[that_idx]];
    let this_o = con.offset[this_idx] as usize;
    let that_o = con.offset[that_idx] as usize;

    let this_iface = get_interface(this_geo, connections, this_o);
    let that_iface = get_interface(that_geo, connections, that_o);

    match (&this_iface, &that_iface) {
        (Interface::Circular { radius: r1 }, Interface::Circular { radius: r2 }) => {
            r1 <= &(1.05 * r2)
        }
        (Interface::Square { p: p1 }, Interface::Square { p: p2 }) => {
            for j in 0..4 {
                let found = (0..4).any(|i| distance_sq(p1[j], p2[i]) < 0.001 * 0.001);
                if !found {
                    return false;
                }
            }
            true
        }
        _ => false,
    }
}

// ─── TriangulationFactory ────────────────────────────────────────────────

pub struct TriangulationFactory {
    pub tolerance: f32,
    min_samples: u32,
    max_samples: u32,
    align_segments: bool,
}

impl TriangulationFactory {
    pub fn new(tolerance: f32, align_segments: bool) -> Self {
        Self {
            tolerance,
            // Match the C++ reference's minimum segment count (3) so default output is
            // size-identical. Combined with `--align-segments` off, this reproduces the
            // C++ sagitta formula exactly.
            min_samples: 3,
            max_samples: 100,
            align_segments,
        }
    }

    /// Compute segment count using the sagitta (chord-height) criterion.
    ///
    /// By default returns the raw chord-height count (≈ the C++ reference, smaller
    /// output). With `--align-segments`, full circles (arc ≈ 2π) are rounded up to the
    /// nearest multiple of 4 and partial arcs up to even, so 0/90/180/270 vertices are
    /// always present — a cylinder and an adjacent torus/snout of *slightly different*
    /// radius then share boundary-ring vertices, making the joint invisible under flat
    /// shading. Costs ~25% more geometry; same-radius neighbours already share counts
    /// without it, so it only changes radius-transition joints.
    fn sagitta_segments(&self, arc: f32, radius: f32, scale: f32) -> u32 {
        let clamped = (1.0_f32 - self.tolerance / (scale * radius)).clamp(-1.0, 1.0);
        let raw = arc / clamped.acos();
        let n = self
            .max_samples
            .min((self.min_samples as f32).max(raw.ceil()) as u32);

        if !self.align_segments {
            return n;
        }
        if arc >= TWO_PI - 0.01 {
            (n + 3) / 4 * 4
        } else {
            (n + 1) / 2 * 2
        }
    }

    // ─── Pyramid ─────────────────────────────────────────────────────────

    pub fn pyramid(
        &self,
        geo: &Geometry,
        geo_all: &[Geometry],
        connections: &[Connection],
    ) -> Triangulation {
        let (bottom, top, offset, height) = match &geo.shape {
            GeometryShape::Pyramid {
                bottom,
                top,
                offset,
                height,
            } => (*bottom, *top, *offset, *height),
            _ => unreachable!(),
        };

        let (bx, by) = (0.5 * bottom[0], 0.5 * bottom[1]);
        let (tx, ty) = (0.5 * top[0], 0.5 * top[1]);
        let (ox, oy) = (0.5 * offset[0], 0.5 * offset[1]);
        let h2 = 0.5 * height;

        let quad: [[Vec3f; 4]; 2] = [
            [
                Vec3f::new(-bx - ox, -by - oy, -h2),
                Vec3f::new(bx - ox, -by - oy, -h2),
                Vec3f::new(bx - ox, by - oy, -h2),
                Vec3f::new(-bx - ox, by - oy, -h2),
            ],
            [
                Vec3f::new(-tx + ox, -ty + oy, h2),
                Vec3f::new(tx + ox, -ty + oy, h2),
                Vec3f::new(tx + ox, ty + oy, h2),
                Vec3f::new(-tx + ox, ty + oy, h2),
            ],
        ];

        let mut cap = [true; 6];
        cap[4] = 1e-7 <= bottom[0].abs().min(bottom[1].abs());
        cap[5] = 1e-7 <= top[0].abs().min(top[1].abs());

        // Check connections for removable caps
        for i in 0..6 {
            if !cap[i] {
                continue;
            }
            let con_idx = geo.connections[i];
            if con_idx == INVALID_GEO {
                continue;
            }
            let con = &connections[con_idx];
            if con.flags != ConnectionFlags::HAS_RECTANGULAR_SIDE {
                continue;
            }
            if do_interfaces_match(geo, geo_all, connections, con_idx) {
                cap[i] = false;
            }
        }

        let caps_n: usize = cap.iter().filter(|&&c| c).count();
        let mut verts = Vec::with_capacity(4 * caps_n * 3);
        let mut indices = Vec::with_capacity(2 * caps_n * 3);

        // Side quads (sides 0..4)
        for i in 0..4usize {
            if !cap[i] {
                continue;
            }
            let ii = (i + 1) & 3;
            let base = (verts.len() / 3) as u32;
            push_vertex(&mut verts, quad[0][i].x, quad[0][i].y, quad[0][i].z);
            push_vertex(&mut verts, quad[0][ii].x, quad[0][ii].y, quad[0][ii].z);
            push_vertex(&mut verts, quad[1][ii].x, quad[1][ii].y, quad[1][ii].z);
            push_vertex(&mut verts, quad[1][i].x, quad[1][i].y, quad[1][i].z);
            quad_indices(&mut indices, base, 0, 1, 2, 3);
        }
        if cap[4] {
            let base = (verts.len() / 3) as u32;
            for k in 0..4 {
                push_vertex(&mut verts, quad[0][k].x, quad[0][k].y, quad[0][k].z);
            }
            quad_indices(&mut indices, base, 3, 2, 1, 0);
        }
        if cap[5] {
            let base = (verts.len() / 3) as u32;
            for k in 0..4 {
                push_vertex(&mut verts, quad[1][k].x, quad[1][k].y, quad[1][k].z);
            }
            quad_indices(&mut indices, base, 0, 1, 2, 3);
        }

        make_tri(verts, indices)
    }

    // ─── Box ─────────────────────────────────────────────────────────────

    pub fn box_shape(
        &self,
        geo: &Geometry,
        geo_all: &[Geometry],
        connections: &[Connection],
    ) -> Triangulation {
        let lengths = match &geo.shape {
            GeometryShape::Box { lengths } => *lengths,
            _ => unreachable!(),
        };
        let (xp, xm) = (0.5 * lengths[0], -0.5 * lengths[0]);
        let (yp, ym) = (0.5 * lengths[1], -0.5 * lengths[1]);
        let (zp, zm) = (0.5 * lengths[2], -0.5 * lengths[2]);

        let v: [[Vec3f; 4]; 6] = [
            [
                Vec3f::new(xm, ym, zp),
                Vec3f::new(xm, yp, zp),
                Vec3f::new(xm, yp, zm),
                Vec3f::new(xm, ym, zm),
            ],
            [
                Vec3f::new(xp, ym, zm),
                Vec3f::new(xp, yp, zm),
                Vec3f::new(xp, yp, zp),
                Vec3f::new(xp, ym, zp),
            ],
            [
                Vec3f::new(xp, ym, zm),
                Vec3f::new(xp, ym, zp),
                Vec3f::new(xm, ym, zp),
                Vec3f::new(xm, ym, zm),
            ],
            [
                Vec3f::new(xm, yp, zm),
                Vec3f::new(xm, yp, zp),
                Vec3f::new(xp, yp, zp),
                Vec3f::new(xp, yp, zm),
            ],
            [
                Vec3f::new(xm, yp, zm),
                Vec3f::new(xp, yp, zm),
                Vec3f::new(xp, ym, zm),
                Vec3f::new(xm, ym, zm),
            ],
            [
                Vec3f::new(xm, ym, zp),
                Vec3f::new(xp, ym, zp),
                Vec3f::new(xp, yp, zp),
                Vec3f::new(xm, yp, zp),
            ],
        ];

        let mut faces = [
            1e-5 <= lengths[0],
            1e-5 <= lengths[0],
            1e-5 <= lengths[1],
            1e-5 <= lengths[1],
            1e-5 <= lengths[2],
            1e-5 <= lengths[2],
        ];

        for i in 0..6 {
            if !faces[i] {
                continue;
            }
            let con_idx = geo.connections[i];
            if con_idx == INVALID_GEO {
                continue;
            }
            let con = &connections[con_idx];
            if con.flags != ConnectionFlags::HAS_RECTANGULAR_SIDE {
                continue;
            }
            if do_interfaces_match(geo, geo_all, connections, con_idx) {
                faces[i] = false;
            }
        }

        let faces_n: usize = faces.iter().filter(|&&f| f).count();
        let mut verts = Vec::with_capacity(4 * faces_n * 3);
        let mut indices = Vec::with_capacity(2 * faces_n * 3);

        for f in 0..6 {
            if !faces[f] {
                continue;
            }
            let base = (verts.len() / 3) as u32;
            for k in 0..4 {
                push_vertex(&mut verts, v[f][k].x, v[f][k].y, v[f][k].z);
            }
            quad_indices(&mut indices, base, 0, 1, 2, 3);
        }

        make_tri(verts, indices)
    }

    // ─── RectangularTorus ────────────────────────────────────────────────

    pub fn rectangular_torus(
        &self,
        geo: &Geometry,
        geo_all: &[Geometry],
        connections: &[Connection],
        scale: f32,
    ) -> Triangulation {
        let (inner_radius, outer_radius, height, angle) = match &geo.shape {
            GeometryShape::RectangularTorus {
                inner_radius,
                outer_radius,
                height,
                angle,
            } => (*inner_radius, *outer_radius, *height, *angle),
            _ => unreachable!(),
        };

        let segments = self.sagitta_segments(angle, outer_radius, scale);
        let samples = segments + 1; // open arc

        let mut cap = [true; 2];
        for i in 0..2 {
            let con_idx = geo.connections[i];
            if con_idx == INVALID_GEO {
                continue;
            }
            let con = &connections[con_idx];
            if con.flags == ConnectionFlags::HAS_RECTANGULAR_SIDE {
                if do_interfaces_match(geo, geo_all, connections, con_idx) {
                    cap[i] = false;
                }
            }
        }

        let h2 = 0.5 * height;
        let sq = [
            [outer_radius, -h2],
            [inner_radius, -h2],
            [inner_radius, h2],
            [outer_radius, h2],
        ];

        let t: Vec<[f32; 2]> = (0..samples as usize)
            .map(|i| {
                let a = (angle / segments as f32) * i as f32;
                [a.cos(), a.sin()]
            })
            .collect();

        // shell: 4 * 2 * samples vertices (pairs per step)
        let shell_verts = 4 * 2 * samples as usize;
        let cap0_verts = if cap[0] { 4 } else { 0 };
        let cap1_verts = if cap[1] { 4 } else { 0 };
        let total_verts = shell_verts + cap0_verts + cap1_verts;

        let mut verts = Vec::with_capacity(total_verts * 3);
        let mut indices = Vec::new();

        // shell vertices (pairs: current step, next step edge)
        for i in 0..samples as usize {
            for k in 0..4 {
                let kk = (k + 1) & 3;
                push_vertex(&mut verts, sq[k][0] * t[i][0], sq[k][0] * t[i][1], sq[k][1]);
                push_vertex(
                    &mut verts,
                    sq[kk][0] * t[i][0],
                    sq[kk][0] * t[i][1],
                    sq[kk][1],
                );
            }
        }
        if cap[0] {
            for k in 0..4 {
                push_vertex(&mut verts, sq[k][0] * t[0][0], sq[k][0] * t[0][1], sq[k][1]);
            }
        }
        if cap[1] {
            let last = samples as usize - 1;
            for k in 0..4 {
                push_vertex(
                    &mut verts,
                    sq[k][0] * t[last][0],
                    sq[k][0] * t[last][1],
                    sq[k][1],
                );
            }
        }

        // shell indices
        for i in 0..samples as usize - 1 {
            for k in 0..4u32 {
                let base = (4 * 2 * i as u32) + 2 * k;
                let next = (4 * 2 * (i + 1) as u32) + 2 * k;
                indices.push(base);
                indices.push(base + 1);
                indices.push(next);
                indices.push(next);
                indices.push(base + 1);
                indices.push(next + 1);
            }
        }

        let mut o = (4 * 2 * samples) as u32;
        if cap[0] {
            indices.push(o + 0);
            indices.push(o + 2);
            indices.push(o + 1);
            indices.push(o + 2);
            indices.push(o + 0);
            indices.push(o + 3);
            o += 4;
        }
        if cap[1] {
            indices.push(o + 0);
            indices.push(o + 1);
            indices.push(o + 2);
            indices.push(o + 2);
            indices.push(o + 3);
            indices.push(o + 0);
        }

        make_tri(verts, indices)
    }

    // ─── CircularTorus ───────────────────────────────────────────────────

    pub fn circular_torus(
        &self,
        geo: &Geometry,
        geo_all: &[Geometry],
        connections: &[Connection],
        scale: f32,
    ) -> Triangulation {
        let (offset, radius, angle) = match &geo.shape {
            GeometryShape::CircularTorus {
                offset,
                radius,
                angle,
            } => (*offset, *radius, *angle),
            _ => unreachable!(),
        };

        let segs_l = self.sagitta_segments(angle, offset + radius, scale);
        let segs_s = self.sagitta_segments(TWO_PI, radius, scale);
        let samples_l = segs_l + 1;
        let samples_s = segs_s;

        let mut cap = [true; 2];
        for i in 0..2 {
            let con_idx = geo.connections[i];
            if con_idx == INVALID_GEO {
                continue;
            }
            let con = &connections[con_idx];
            if con.flags.has(ConnectionFlags::HAS_CIRCULAR_SIDE) {
                if do_interfaces_match(geo, geo_all, connections, con_idx) {
                    cap[i] = false;
                }
            }
        }

        let t0: Vec<[f32; 2]> = (0..samples_l as usize)
            .map(|i| {
                let a = (angle / (samples_l as f32 - 1.0)) * i as f32;
                [a.cos(), a.sin()]
            })
            .collect();
        let t1: Vec<[f32; 2]> = (0..samples_s as usize)
            .map(|i| {
                let a = (TWO_PI / samples_s as f32) * i as f32 + geo.sample_start_angle;
                [a.cos(), a.sin()]
            })
            .collect();

        let total_verts =
            (samples_l as usize + (if cap[0] { 1 } else { 0 }) + (if cap[1] { 1 } else { 0 }))
                * samples_s as usize;
        let mut verts = Vec::with_capacity(total_verts * 3);
        let mut indices = Vec::new();

        // shell
        for u in 0..samples_l as usize {
            for v in 0..samples_s as usize {
                let x = (radius * t1[v][0] + offset) * t0[u][0];
                let y = (radius * t1[v][0] + offset) * t0[u][1];
                let z = radius * t1[v][1];
                push_vertex(&mut verts, x, y, z);
            }
        }
        // cap0 ring
        if cap[0] {
            for v in 0..samples_s as usize {
                let x = (radius * t1[v][0] + offset) * t0[0][0];
                let y = (radius * t1[v][0] + offset) * t0[0][1];
                let z = radius * t1[v][1];
                push_vertex(&mut verts, x, y, z);
            }
        }
        // cap1 ring
        if cap[1] {
            let last = samples_l as usize - 1;
            for v in 0..samples_s as usize {
                let x = (radius * t1[v][0] + offset) * t0[last][0];
                let y = (radius * t1[v][0] + offset) * t0[last][1];
                let z = radius * t1[v][1];
                push_vertex(&mut verts, x, y, z);
            }
        }

        // shell indices
        let ss = samples_s as u32;
        for u in 0..segs_l as u32 {
            for v in 0..ss - 1 {
                indices.push(ss * (u + 0) + (v + 0));
                indices.push(ss * (u + 1) + (v + 0));
                indices.push(ss * (u + 1) + (v + 1));
                indices.push(ss * (u + 1) + (v + 1));
                indices.push(ss * (u + 0) + (v + 1));
                indices.push(ss * (u + 0) + (v + 0));
            }
            // wrap
            indices.push(ss * (u + 0) + (ss - 1));
            indices.push(ss * (u + 1) + (ss - 1));
            indices.push(ss * (u + 1) + 0);
            indices.push(ss * (u + 1) + 0);
            indices.push(ss * (u + 0) + 0);
            indices.push(ss * (u + 0) + (ss - 1));
        }

        let mut o = (samples_l * ss) as usize;
        if cap[0] {
            let ring: Vec<u32> = (0..ss as usize).map(|i| (o + i) as u32).collect();
            tessellate_circle(&mut indices, ring);
            o += ss as usize;
        }
        if cap[1] {
            let ring: Vec<u32> = (0..ss as usize)
                .map(|i| (o + (ss as usize - 1 - i)) as u32)
                .collect();
            tessellate_circle(&mut indices, ring);
        }

        make_tri(verts, indices)
    }

    // ─── Snout ───────────────────────────────────────────────────────────

    pub fn snout(
        &self,
        geo: &Geometry,
        geo_all: &[Geometry],
        connections: &[Connection],
        scale: f32,
    ) -> Triangulation {
        let (offset, bshear, tshear, radius_b, radius_t, height) = match &geo.shape {
            GeometryShape::Snout {
                offset,
                bshear,
                tshear,
                radius_b,
                radius_t,
                height,
            } => (*offset, *bshear, *tshear, *radius_b, *radius_t, *height),
            _ => unreachable!(),
        };

        // Use radius_max so the segment count is generous enough for both ends.
        // sagitta_segments already rounds to a multiple of 4, so if radius_b == radius_t
        // (equal-end snout / straight section) the rings are vertex-identical to an
        // adjacent cylinder of the same radius and will merge in the dedup pass.
        let segments = self.sagitta_segments(TWO_PI, radius_b.max(radius_t), scale);
        let samples = segments;

        let mut cap = [true; 2];
        for i in 0..2 {
            let con_idx = geo.connections[i];
            if con_idx == INVALID_GEO {
                continue;
            }
            let con = &connections[con_idx];
            if con.flags.has(ConnectionFlags::HAS_CIRCULAR_SIDE) {
                if do_interfaces_match(geo, geo_all, connections, con_idx) {
                    cap[i] = false;
                }
            }
        }

        let t: Vec<[f32; 2]> = (0..samples as usize)
            .map(|i| {
                let a = (TWO_PI / samples as f32) * i as f32 + geo.sample_start_angle;
                [a.cos(), a.sin()]
            })
            .collect();
        let tb: Vec<[f32; 2]> = t
            .iter()
            .map(|tc| [radius_b * tc[0], radius_b * tc[1]])
            .collect();
        let tt: Vec<[f32; 2]> = t
            .iter()
            .map(|tc| [radius_t * tc[0], radius_t * tc[1]])
            .collect();

        let h2 = 0.5 * height;
        let ox = 0.5 * offset[0];
        let oy = 0.5 * offset[1];
        let mb = [bshear[0].tan(), bshear[1].tan()];
        let mt = [tshear[0].tan(), tshear[1].tan()];

        let n = samples as usize;
        let total = 2 * n + (if cap[0] { n } else { 0 }) + (if cap[1] { n } else { 0 });
        let mut verts = Vec::with_capacity(total * 3);
        let mut indices = Vec::new();

        // shell: bottom/top vertex pairs
        for i in 0..n {
            push_vertex(
                &mut verts,
                tb[i][0] - ox,
                tb[i][1] - oy,
                -h2 + mb[0] * tb[i][0] + mb[1] * tb[i][1],
            );
            push_vertex(
                &mut verts,
                tt[i][0] + ox,
                tt[i][1] + oy,
                h2 + mt[0] * tt[i][0] + mt[1] * tt[i][1],
            );
        }
        if cap[0] {
            for i in 0..n {
                push_vertex(
                    &mut verts,
                    tb[i][0] - ox,
                    tb[i][1] - oy,
                    -h2 + mb[0] * tb[i][0] + mb[1] * tb[i][1],
                );
            }
        }
        if cap[1] {
            for i in 0..n {
                push_vertex(
                    &mut verts,
                    tt[i][0] + ox,
                    tt[i][1] + oy,
                    h2 + mt[0] * tt[i][0] + mt[1] * tt[i][1],
                );
            }
        }

        let sn = n as u32;
        for i in 0..sn {
            let ii = (i + 1) % sn;
            quad_indices(&mut indices, 0, 2 * i, 2 * ii, 2 * ii + 1, 2 * i + 1);
        }
        let mut o = (2 * n) as usize;
        if cap[0] {
            let ring: Vec<u32> = (0..n).map(|i| (o + n - 1 - i) as u32).collect();
            tessellate_circle(&mut indices, ring);
            o += n;
        }
        if cap[1] {
            let ring: Vec<u32> = (0..n).map(|i| (o + i) as u32).collect();
            tessellate_circle(&mut indices, ring);
        }

        make_tri(verts, indices)
    }

    // ─── Cylinder ────────────────────────────────────────────────────────

    pub fn cylinder(
        &self,
        geo: &Geometry,
        geo_all: &[Geometry],
        connections: &[Connection],
        scale: f32,
    ) -> Triangulation {
        let (radius, height) = match &geo.shape {
            GeometryShape::Cylinder { radius, height } => (*radius, *height),
            _ => unreachable!(),
        };

        let segments = self.sagitta_segments(TWO_PI, radius, scale);
        let samples = segments;

        let mut cap = [true; 2];
        for i in 0..2 {
            let con_idx = geo.connections[i];
            if con_idx == INVALID_GEO {
                continue;
            }
            let con = &connections[con_idx];
            if con.flags.has(ConnectionFlags::HAS_CIRCULAR_SIDE) {
                if do_interfaces_match(geo, geo_all, connections, con_idx) {
                    cap[i] = false;
                }
            }
        }

        let t: Vec<[f32; 2]> = (0..samples as usize)
            .map(|i| {
                let a = (TWO_PI / samples as f32) * i as f32 + geo.sample_start_angle;
                let ca = a.cos();
                let sa = a.sin();
                [radius * ca, radius * sa]
            })
            .collect();

        let h2 = 0.5 * height;
        let n = samples as usize;
        let total = (if true { 2 * n } else { 0 })
            + (if cap[0] { n } else { 0 })
            + (if cap[1] { n } else { 0 });
        let mut verts = Vec::with_capacity(total * 3);
        let mut indices = Vec::new();

        for i in 0..n {
            push_vertex(&mut verts, t[i][0], t[i][1], -h2);
            push_vertex(&mut verts, t[i][0], t[i][1], h2);
        }
        if cap[0] {
            for i in 0..n {
                push_vertex(&mut verts, t[i][0], t[i][1], -h2);
            }
        }
        if cap[1] {
            for i in 0..n {
                push_vertex(&mut verts, t[i][0], t[i][1], h2);
            }
        }

        let sn = n as u32;
        for i in 0..sn {
            let ii = (i + 1) % sn;
            quad_indices(&mut indices, 0, 2 * i, 2 * ii, 2 * ii + 1, 2 * i + 1);
        }
        let mut o = (2 * n) as usize;
        if cap[0] {
            let ring: Vec<u32> = (0..n).map(|i| (o + n - 1 - i) as u32).collect();
            tessellate_circle(&mut indices, ring);
            o += n;
        }
        if cap[1] {
            let ring: Vec<u32> = (0..n).map(|i| (o + i) as u32).collect();
            tessellate_circle(&mut indices, ring);
        }

        make_tri(verts, indices)
    }

    // ─── Sphere-based shape (sphere, dishes) ─────────────────────────────

    pub fn sphere_based_shape(
        &self,
        geo: &Geometry,
        radius: f32,
        arc: f32,
        shift_z: f32,
        scale_z: f32,
        scale: f32,
    ) -> Triangulation {
        let mut scale_z = scale_z;
        if !scale_z.is_finite() {
            scale_z = 0.0;
        }

        let segments = self.sagitta_segments(TWO_PI, radius, scale);
        let samples = segments;

        let is_sphere = arc >= PI - 1e-3;
        let arc = if is_sphere { PI } else { arc };

        let min_rings = 3u32;
        let rings = min_rings.max((scale_z * samples as f32 * arc * (1.0 / TWO_PI)) as u32);

        let theta_scale = arc / (rings - 1) as f32;
        let cos_theta: Vec<f32> = (0..rings as usize)
            .map(|r| (theta_scale * r as f32).cos())
            .collect();
        let sin_theta: Vec<f32> = (0..rings as usize)
            .map(|r| (theta_scale * r as f32).sin())
            .collect();

        // samples per ring
        let mut ring_n: Vec<usize> = (0..rings as usize)
            .map(|r| (3f32.max(sin_theta[r] * samples as f32)) as usize)
            .collect();
        ring_n[0] = 1;
        if is_sphere {
            *ring_n.last_mut().unwrap() = 1;
        }

        let total_verts: usize = ring_n.iter().sum();
        let mut verts = Vec::with_capacity(total_verts * 3);

        for r in 0..rings as usize {
            let nz = cos_theta[r];
            let z = radius * scale_z * nz + shift_z;
            let w = sin_theta[r];
            let n = ring_n[r];
            let phi_scale = TWO_PI / n as f32;
            for i in 0..n {
                let phi = phi_scale * i as f32 + geo.sample_start_angle;
                let nx = w * phi.cos();
                let ny = w * phi.sin();
                push_vertex(&mut verts, radius * nx, radius * ny, z);
            }
        }

        let mut indices = Vec::new();
        let mut o_c: usize = 0;
        for r in 0..(rings as usize - 1) {
            let n_c = ring_n[r];
            let n_n = ring_n[r + 1];
            let o_n = o_c + n_c;
            if n_c < n_n {
                for i_n in 0..n_n {
                    let ii_n = (i_n + 1) % n_n;
                    let i_c = (n_c * (i_n + 1)) / n_n % n_c;
                    let ii_c = (n_c * (i_n + 1 + 1)) / n_n % n_c;
                    if i_c != ii_c {
                        indices.push((o_c + i_c) as u32);
                        indices.push((o_n + ii_n) as u32);
                        indices.push((o_c + ii_c) as u32);
                    }
                    indices.push((o_c + i_c) as u32);
                    indices.push((o_n + i_n) as u32);
                    indices.push((o_n + ii_n) as u32);
                }
            } else {
                for i_c in 0..n_c {
                    let ii_c = (i_c + 1) % n_c;
                    let i_n = (n_n * i_c) / n_c % n_n;
                    let ii_n = (n_n * (i_c + 1)) / n_c % n_n;
                    indices.push((o_c + i_c) as u32);
                    indices.push((o_n + ii_n) as u32);
                    indices.push((o_c + ii_c) as u32);
                    if i_n != ii_n {
                        indices.push((o_c + i_c) as u32);
                        indices.push((o_n + i_n) as u32);
                        indices.push((o_n + ii_n) as u32);
                    }
                }
            }
            o_c = o_n;
        }

        make_tri(verts, indices)
    }

    // ─── FacetGroup ──────────────────────────────────────────────────────

    pub fn facet_group(&self, geo: &Geometry) -> Triangulation {
        let polygons = match &geo.shape {
            GeometryShape::FacetGroup { polygons } => polygons,
            _ => unreachable!(),
        };

        let mut verts: Vec<f32> = Vec::new();
        let mut indices: Vec<u32> = Vec::new();

        'poly: for poly in polygons {
            // Verify all vertices are finite
            for cont in &poly.contours {
                if cont.vertices.iter().any(|v| !v.is_finite()) {
                    continue 'poly;
                }
            }

            if poly.contours.len() == 1 && poly.contours[0].vertices.len() / 3 == 3 {
                let cont = &poly.contours[0];
                let vo = (verts.len() / 3) as u32;
                verts.extend_from_slice(&cont.vertices);
                indices.extend_from_slice(&[vo, vo + 1, vo + 2]);
            } else if poly.contours.len() == 1 && poly.contours[0].vertices.len() / 3 == 4 {
                let cont = &poly.contours[0];
                let v = &cont.vertices;
                let vo = (verts.len() / 3) as u32;
                verts.extend_from_slice(v);

                // find least-folding diagonal
                let p = |i: usize| -> [f32; 3] { [v[i * 3], v[i * 3 + 1], v[i * 3 + 2]] };
                let sub3 = |a: [f32; 3], b: [f32; 3]| -> [f32; 3] {
                    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
                };
                let cross3 = |a: [f32; 3], b: [f32; 3]| -> [f32; 3] {
                    [
                        a[1] * b[2] - a[2] * b[1],
                        a[2] * b[0] - a[0] * b[2],
                        a[0] * b[1] - a[1] * b[0],
                    ]
                };
                let dot3 =
                    |a: [f32; 3], b: [f32; 3]| -> f32 { a[0] * b[0] + a[1] * b[1] + a[2] * b[2] };
                let v01 = sub3(p(1), p(0));
                let v12 = sub3(p(2), p(1));
                let v23 = sub3(p(3), p(2));
                let v30 = sub3(p(0), p(3));
                let n0 = cross3(v01, v30);
                let n1 = cross3(v12, v01);
                let n2 = cross3(v23, v12);
                let n3 = cross3(v30, v23);
                if dot3(n0, n2) < dot3(n1, n3) {
                    indices.extend_from_slice(&[vo, vo + 1, vo + 2, vo + 2, vo + 3, vo]);
                } else {
                    indices.extend_from_slice(&[vo + 3, vo, vo + 1, vo + 1, vo + 2, vo + 3]);
                }
            } else {
                // General case: use ear-clipping tessellation
                self.tessellate_polygon_general(poly, &mut verts, &mut indices);
            }
        }

        make_tri(verts, indices)
    }

    fn tessellate_polygon_general(
        &self,
        poly: &Polygon,
        verts: &mut Vec<f32>,
        indices: &mut Vec<u32>,
    ) {
        use tess2_rust::{ElementType, TessellatorApi, WindingRule};

        // Compute bbox centre for numerical stability — same as original C++
        let mut bbox = BBox3f::empty();
        for cont in &poly.contours {
            for i in 0..cont.vertices.len() / 3 {
                bbox.engulf_point(Vec3f::from_slice(&cont.vertices[i * 3..]));
            }
        }
        let mx = ((bbox.min.x + bbox.max.x) * 0.5) as f64;
        let my = ((bbox.min.y + bbox.max.y) * 0.5) as f64;
        let mz = ((bbox.min.z + bbox.max.z) * 0.5) as f64;

        let mut tess = TessellatorApi::new();

        let mut any_data = false;
        for cont in &poly.contours {
            let n = cont.vertices.len() / 3;
            if n < 3 {
                continue;
            }

            // Centre vertices around bbox midpoint for numerical stability
            // (tess2-rust uses f64 internally) and *clean* the contour before handing
            // it to libtess2: drop non-finite points and consecutive duplicates
            // (incl. the wrap-around). Degenerate input is the main trigger for the
            // sweep's out-of-bounds / freed-region paths; cleaning here avoids them.
            let mut centred: Vec<f64> = Vec::with_capacity(n * 3);
            for i in 0..n {
                let p = [
                    cont.vertices[i * 3] as f64 - mx,
                    cont.vertices[i * 3 + 1] as f64 - my,
                    cont.vertices[i * 3 + 2] as f64 - mz,
                ];
                if !p.iter().all(|c| c.is_finite()) {
                    continue;
                }
                let dup = centred.len() >= 3
                    && centred[centred.len() - 3] == p[0]
                    && centred[centred.len() - 2] == p[1]
                    && centred[centred.len() - 1] == p[2];
                if !dup {
                    centred.extend_from_slice(&p);
                }
            }
            // Drop a wrap-around duplicate (last == first).
            if centred.len() >= 6 {
                let (f, l) = (&centred[0..3], &centred[centred.len() - 3..]);
                if f == l {
                    centred.truncate(centred.len() - 3);
                }
            }
            if centred.len() < 9 {
                continue; // fewer than 3 distinct points → no area
            }

            // Reject zero-area / collinear contours via Newell's normal (its magnitude
            // is 2×area). A degenerate (arealess) contour is the root cause of the
            // sweep's dangling-edge / INVALID-index crashes — gating it here keeps
            // libtess2 from ever building that corrupt state. Threshold is relative to
            // the contour's own extent so it scales with model units.
            let pts = centred.len() / 3;
            let mut nx = 0.0;
            let mut ny = 0.0;
            let mut nz = 0.0;
            let mut ext = 0.0_f64;
            for i in 0..pts {
                let a = &centred[i * 3..i * 3 + 3];
                let b = &centred[((i + 1) % pts) * 3..((i + 1) % pts) * 3 + 3];
                nx += (a[1] - b[1]) * (a[2] + b[2]);
                ny += (a[2] - b[2]) * (a[0] + b[0]);
                nz += (a[0] - b[0]) * (a[1] + b[1]);
                ext = ext.max(a[0].abs()).max(a[1].abs()).max(a[2].abs());
            }
            let area2 = (nx * nx + ny * ny + nz * nz).sqrt();
            // Areal scale ~ ext²; require the contour's area to be a non-trivial
            // fraction of it (1e-12 of ext² ≈ floating-point noise floor).
            if area2 <= 1e-12 * ext * ext {
                continue;
            }

            tess.add_contour(3, &centred);
            any_data = true;
        }

        if !any_data {
            return;
        }

        let ok = tess.tessellate(
            WindingRule::Odd,
            ElementType::Polygons,
            3,    // poly_size  = triangles
            3,    // vertex_size = x,y,z
            None, // normal: let libtess2 compute it
        );

        if !ok {
            return;
        }

        let vo = (verts.len() / 3) as u32;

        // Add bbox centre back, cast f64 -> f32 when pushing
        let src = tess.vertices();
        let vn = tess.vertex_count();
        for i in 0..vn {
            verts.push((src[i * 3] + mx) as f32);
            verts.push((src[i * 3 + 1] + my) as f32);
            verts.push((src[i * 3 + 2] + mz) as f32);
        }

        // elements() returns flat triangle indices (u32), u32::MAX means TESS_UNDEF
        let elems = tess.elements();
        let tri_count = tess.element_count();
        for e in 0..tri_count {
            let a = elems[e * 3];
            let b = elems[e * 3 + 1];
            let c = elems[e * 3 + 2];
            if a != u32::MAX && b != u32::MAX && c != u32::MAX {
                indices.push(vo + a);
                indices.push(vo + b);
                indices.push(vo + c);
            }
        }
    }
}

#[cfg(test)]
mod line_tests {
    use super::*;

    #[test]
    fn cross_has_two_perpendicular_plates() {
        let t = line_cross(0.0, 2.0, 0.05).expect("non-degenerate line");
        assert_eq!(t.vertices_n, 8, "2 plates * 4 corners");
        assert_eq!(t.triangles_n, 4, "2 plates * 2 triangles");
        assert_eq!(t.indices.len(), 12);
        // Every vertex sits at one of the two endpoints along local Z.
        for v in t.vertices.chunks_exact(3) {
            assert!(v[2] == 0.0 || v[2] == 2.0);
        }
        // Plate 1 lies in the Z-X plane (y==0), plate 2 in the Z-Y plane (x==0).
        let plate1 = &t.vertices[0..12];
        let plate2 = &t.vertices[12..24];
        assert!(plate1.chunks_exact(3).all(|v| v[1] == 0.0));
        assert!(plate2.chunks_exact(3).all(|v| v[0] == 0.0));
    }

    #[test]
    fn zero_length_line_is_skipped() {
        assert!(line_cross(1.0, 1.0, 0.05).is_none());
    }
}

/// Build a thin "+" cross for an RVM Line: two perpendicular quads (plates) of
/// half-width `hw`, each spanning the local **Z**-axis segment `a..b`. Plate 1 lies in
/// the local Z-X plane, plate 2 in the Z-Y plane, so from any view at least one
/// plate presents face-on — giving the otherwise zero-area line clickable surface.
/// Returns None for a zero-length line (a == b), which has no selectable area.
///
/// The line runs along local **Z**, not local X: RVM stores a structural member's
/// matrix with the member's length on its local Z axis (the GENSEC extrusion axis),
/// and the Line is that member's centerline. Building along X (as rvmparser-master's
/// export does) drops the centreline 90° off the member — two parallel girders then
/// collapse into one collinear line. Verified against the girder facet meshes in
/// `file4.rvm`: the member spans 4.45 m along world X, exactly matching `b - a`.
fn line_cross(a: f32, b: f32, hw: f32) -> Option<Triangulation> {
    if (a - b).abs() < 1e-6 {
        return None;
    }
    let mut verts: Vec<f32> = Vec::with_capacity(8 * 3);
    // Plate 1 — Z-X plane (spans X = ±hw).
    push_vertex(&mut verts, -hw, 0.0, a);
    push_vertex(&mut verts, -hw, 0.0, b);
    push_vertex(&mut verts, hw, 0.0, b);
    push_vertex(&mut verts, hw, 0.0, a);
    // Plate 2 — Z-Y plane (spans Y = ±hw).
    push_vertex(&mut verts, 0.0, -hw, a);
    push_vertex(&mut verts, 0.0, -hw, b);
    push_vertex(&mut verts, 0.0, hw, b);
    push_vertex(&mut verts, 0.0, hw, a);

    let mut indices: Vec<u32> = Vec::with_capacity(12);
    for plate in 0..2u32 {
        let base = plate * 4;
        indices.extend_from_slice(&[base, base + 1, base + 2, base, base + 2, base + 3]);
    }
    Some(make_tri(verts, indices))
}

fn make_tri(vertices: Vec<f32>, indices: Vec<u32>) -> Triangulation {
    let vertices_n = (vertices.len() / 3) as u32;
    let triangles_n = (indices.len() / 3) as u32;
    Triangulation {
        vertices,
        normals: Vec::new(),
        indices,
        vertices_n,
        triangles_n,
        id: 0,
        color: 0,
        error: 0.0,
    }
}

// ─── Main tessellate entry point ─────────────────────────────────────────

pub fn tessellate(
    geo: &Geometry,
    geo_all: &[Geometry],
    connections: &[Connection],
    tolerance: f32,
    line_width: f32,
    seg_scale: f32,
    align_segments: bool,
) -> Option<Triangulation> {
    let factory = TriangulationFactory::new(tolerance, align_segments);
    // Scale used for the chord-tolerance segment count. The merged path derives it
    // from the primitive's own matrix; the instanced path tessellates in local space
    // (identity matrix) so it must pass a representative occurrence scale here, or
    // primitives whose matrix carries a unit scale (e.g. mm→m) over-tessellate wildly.
    let scale = seg_scale;

    let mut tri = match geo.kind {
        GeometryKind::Pyramid => factory.pyramid(geo, geo_all, connections),
        GeometryKind::Box => factory.box_shape(geo, geo_all, connections),
        GeometryKind::RectangularTorus => {
            factory.rectangular_torus(geo, geo_all, connections, scale)
        }
        GeometryKind::CircularTorus => factory.circular_torus(geo, geo_all, connections, scale),
        GeometryKind::EllipticalDish => {
            let (base_radius, height) = match &geo.shape {
                GeometryShape::EllipticalDish {
                    base_radius,
                    height,
                } => (*base_radius, *height),
                _ => unreachable!(),
            };
            factory.sphere_based_shape(geo, base_radius, HALF_PI, 0.0, height / base_radius, scale)
        }
        GeometryKind::SphericalDish => {
            let (r_circ, h) = match &geo.shape {
                GeometryShape::SphericalDish {
                    base_radius,
                    height,
                } => (*base_radius, *height),
                _ => unreachable!(),
            };
            let r_sphere = (r_circ * r_circ + h * h) / (2.0 * h);
            let sinval = (r_circ / r_sphere).clamp(-1.0, 1.0);
            let mut arc = sinval.asin();
            if r_circ < h {
                arc = PI - arc;
            }
            factory.sphere_based_shape(geo, r_sphere, arc, h - r_sphere, 1.0, scale)
        }
        GeometryKind::Snout => factory.snout(geo, geo_all, connections, scale),
        GeometryKind::Cylinder => factory.cylinder(geo, geo_all, connections, scale),
        GeometryKind::Sphere => {
            let diameter = match &geo.shape {
                GeometryShape::Sphere { diameter } => *diameter,
                _ => unreachable!(),
            };
            factory.sphere_based_shape(geo, 0.5 * diameter, PI, 0.0, 1.0, scale)
        }
        GeometryKind::FacetGroup => factory.facet_group(geo),
        // RVM has no native line geometry; draw a thin "+" cross of two quads swept
        // along the local Z-axis segment so the line is selectable. The segment is
        // *centred on the matrix origin* (RVM places a member's matrix at the member
        // centre, with the Line as its centreline), so we draw ±(b-a)/2 rather than
        // a..b — otherwise the centreline overhangs one end and misses the other.
        // `line_width` is a world-space size, but the cross is built in local space and
        // then scaled by the primitive's matrix (which often carries a mm→m unit scale).
        // Pre-divide by that scale so the rendered width actually equals `line_width`.
        GeometryKind::Line => {
            let (a, b) = match &geo.shape {
                GeometryShape::Line { a, b } => (*a, *b),
                _ => return None,
            };
            let hw = if scale > 0.0 {
                0.5 * line_width / scale
            } else {
                0.5 * line_width
            };
            let half = 0.5 * (b - a);
            line_cross(-half, half, hw).unwrap_or_default()
        }
    };

    if tri.vertices_n == 0 {
        return None;
    }

    // Apply world transform (double precision)
    let m = Mat3x4d::from_f32(&geo.m_3x4.data);
    for i in 0..tri.vertices_n as usize {
        let u = i * 3;
        let v = Vec3d::from_f32_slice(&tri.vertices[u..u + 3]);
        let t = m.mul_vec(v);
        tri.vertices[u] = t.x as f32;
        tri.vertices[u + 1] = t.y as f32;
        tri.vertices[u + 2] = t.z as f32;
    }

    Some(tri)
}
