// Copyright 2025 Lars Brubaker
// License: SGI Free Software License B (MIT-compatible)
//
// Port of libtess2 geom.c/h
//
// Pure geometric functions operating on vertex (s, t) coordinates.
// These are exact translations of the C functions with identical floating-point
// behavior to ensure mathematical equivalence with the original library.

/// Internal coordinate type used throughout the sweep and all geometric
/// predicates.
///
/// Pinned to **`f64`** — matching the C# agg-sharp `Tesselator` reference and
/// modern libtess2 builds that opt for double precision.  The sweep's
/// stability hinges on edge-sign tests not flipping under small coordinate
/// perturbations (e.g. rotating integer input to floating coordinates):
/// `f32`'s ~7 decimal digits of precision is not enough to keep
/// near-collinear predicates on the same side across rotations.  With `f64`
/// the 15-digit margin absorbs the rounding noise and the topology stays
/// rotation-invariant — which is the whole point of this library.
pub type Real = f64;

/// Returns true if u is lexicographically <= v (s first, then t).
#[inline]
pub fn vert_leq(u_s: Real, u_t: Real, v_s: Real, v_t: Real) -> bool {
    u_s < v_s || (u_s == v_s && u_t <= v_t)
}

/// Returns true if u == v (exact equality).
#[inline]
pub fn vert_eq(u_s: Real, u_t: Real, v_s: Real, v_t: Real) -> bool {
    u_s == v_s && u_t == v_t
}

/// Returns true if u is lexicographically <= v with s and t transposed.
#[inline]
pub fn trans_leq(u_s: Real, u_t: Real, v_s: Real, v_t: Real) -> bool {
    u_t < v_t || (u_t == v_t && u_s <= v_s)
}

/// Given three vertices u,v,w such that vert_leq(u,v) && vert_leq(v,w),
/// evaluates the t-coord of edge uw at the s-coord of v.
/// Returns v.t - (uw)(v.s), the signed distance from uw to v.
/// If uw is vertical (passes through v), returns zero.
/// The calculation is extremely accurate and stable.
pub fn edge_eval(u_s: Real, u_t: Real, v_s: Real, v_t: Real, w_s: Real, w_t: Real) -> Real {
    // debug_assert!(vert_leq(u_s, u_t, v_s, v_t) && vert_leq(v_s, v_t, w_s, w_t));
    let gap_l = v_s - u_s;
    let gap_r = w_s - v_s;
    if gap_l + gap_r > 0.0 {
        if gap_l < gap_r {
            (v_t - u_t) + (u_t - w_t) * (gap_l / (gap_l + gap_r))
        } else {
            (v_t - w_t) + (w_t - u_t) * (gap_r / (gap_l + gap_r))
        }
    } else {
        0.0
    }
}

/// Returns a value whose **sign** matches `edge_eval(u,v,w)`, computed with a
/// cheaper division-free cross-product formula.
///
/// Direct port of libtess2's `tesedgeSign` (`geom.c`).  The point of having
/// this as a separate routine from `edge_eval` is numerical stability: the
/// division inside `edge_eval` (`gap_l / (gap_l + gap_r)`) rounds slightly
/// differently depending on floating-point representation, which flips the
/// sign on nearly-collinear inputs.  Rotating a polygon changes coordinate
/// representations just enough for that flipping to give DIFFERENT
/// topological decisions during the sweep — the classic robustness failure
/// libtess2 was written to avoid.
///
/// The cross-product formula below preserves the same sign as
/// `edge_eval(u,v,w)` (they differ only by a positive scalar `gap_l+gap_r`),
/// but avoids division entirely, so its sign is stable under the small
/// coordinate perturbations that rotations introduce.
#[inline]
pub fn edge_sign(u_s: Real, u_t: Real, v_s: Real, v_t: Real, w_s: Real, w_t: Real) -> Real {
    // libtess2 `#define EdgeSign(u,v,w) tesedgeEval(u,v,w)` (geom.h): the cheap
    // sign-only form (the former `tesedgeSign`, previously inlined here) returns
    // the WRONG SIGN when the s-coordinates are nearly equal — SGI/upstream
    // explicitly switched `EdgeSign` to the accurate normalized evaluator to fix
    // it.  The cheap form made `check_for_right_splice` flip a decision on
    // near-degenerate glyph edges, corrupting the sweep (found by the
    // `fuzz_glyph_discovery` differential test against the C reference).  Match C
    // exactly: sign tests use the same evaluator as `edge_eval`.
    edge_eval(u_s, u_t, v_s, v_t, w_s, w_t)
}

/// Like edge_eval but with s and t transposed.
pub fn trans_eval(u_s: Real, u_t: Real, v_s: Real, v_t: Real, w_s: Real, w_t: Real) -> Real {
    // debug_assert!(trans_leq(u_s, u_t, v_s, v_t) && trans_leq(v_s, v_t, w_s, w_t));
    let gap_l = v_t - u_t;
    let gap_r = w_t - v_t;
    if gap_l + gap_r > 0.0 {
        if gap_l < gap_r {
            (v_s - u_s) + (u_s - w_s) * (gap_l / (gap_l + gap_r))
        } else {
            (v_s - w_s) + (w_s - u_s) * (gap_r / (gap_l + gap_r))
        }
    } else {
        0.0
    }
}

/// Like edge_sign but with s and t transposed.
pub fn trans_sign(u_s: Real, u_t: Real, v_s: Real, v_t: Real, w_s: Real, w_t: Real) -> Real {
    // debug_assert!(trans_leq(u_s, u_t, v_s, v_t) && trans_leq(v_s, v_t, w_s, w_t));
    let gap_l = v_t - u_t;
    let gap_r = w_t - v_t;
    if gap_l + gap_r > 0.0 {
        (v_s - w_s) * gap_l + (v_s - u_s) * gap_r
    } else {
        0.0
    }
}

/// Returns true if (u, v, w) are in CCW (counter-clockwise) order.
#[inline]
pub fn vert_ccw(u_s: Real, u_t: Real, v_s: Real, v_t: Real, w_s: Real, w_t: Real) -> bool {
    u_s * (v_t - w_t) + v_s * (w_t - u_t) + w_s * (u_t - v_t) >= 0.0
}

/// L1 distance between two vertices.
#[inline]
pub fn vert_l1_dist(u_s: Real, u_t: Real, v_s: Real, v_t: Real) -> Real {
    (u_s - v_s).abs() + (u_t - v_t).abs()
}

/// Numerically stable interpolation: returns (b*x + a*y) / (a + b),
/// or (x + y) / 2 if a == b == 0. Requires a, b >= 0 and enforces this.
/// Guarantees MIN(x,y) <= result <= MAX(x,y).
#[inline]
pub fn real_interpolate(mut a: Real, x: Real, mut b: Real, y: Real) -> Real {
    if a < 0.0 {
        a = 0.0;
    }
    if b < 0.0 {
        b = 0.0;
    }
    if a <= b {
        if b == 0.0 {
            x / 2.0 + y / 2.0
        } else {
            x + (y - x) * (a / (a + b))
        }
    } else {
        y + (x - y) * (b / (a + b))
    }
}

/// Compute the intersection point of edges (o1,d1) and (o2,d2).
/// Returns (s, t) of the intersection.
/// The result is guaranteed to lie within the bounding rectangle of both edges.
pub fn edge_intersect(
    o1_s: Real,
    o1_t: Real,
    d1_s: Real,
    d1_t: Real,
    o2_s: Real,
    o2_t: Real,
    d2_s: Real,
    d2_t: Real,
) -> (Real, Real) {
    // Compute s-coordinate of intersection using VertLeq ordering.
    let v_s;
    {
        let (mut a_s, mut a_t) = (o1_s, o1_t);
        let (mut b_s, mut b_t) = (d1_s, d1_t);
        let (mut c_s, mut c_t) = (o2_s, o2_t);
        let (mut d_s, mut d_t) = (d2_s, d2_t);

        if !vert_leq(a_s, a_t, b_s, b_t) {
            core::mem::swap(&mut a_s, &mut b_s);
            core::mem::swap(&mut a_t, &mut b_t);
        }
        if !vert_leq(c_s, c_t, d_s, d_t) {
            core::mem::swap(&mut c_s, &mut d_s);
            core::mem::swap(&mut c_t, &mut d_t);
        }
        if !vert_leq(a_s, a_t, c_s, c_t) {
            core::mem::swap(&mut a_s, &mut c_s);
            core::mem::swap(&mut a_t, &mut c_t);
            core::mem::swap(&mut b_s, &mut d_s);
            core::mem::swap(&mut b_t, &mut d_t);
        }

        if !vert_leq(c_s, c_t, b_s, b_t) {
            v_s = c_s / 2.0 + b_s / 2.0;
        } else if vert_leq(b_s, b_t, d_s, d_t) {
            let mut z1 = edge_eval(a_s, a_t, c_s, c_t, b_s, b_t);
            let mut z2 = edge_eval(c_s, c_t, b_s, b_t, d_s, d_t);
            if z1 + z2 < 0.0 {
                z1 = -z1;
                z2 = -z2;
            }
            v_s = real_interpolate(z1, c_s, z2, b_s);
        } else {
            let mut z1 = edge_sign(a_s, a_t, c_s, c_t, b_s, b_t);
            let mut z2 = -edge_sign(a_s, a_t, d_s, d_t, b_s, b_t);
            if z1 + z2 < 0.0 {
                z1 = -z1;
                z2 = -z2;
            }
            v_s = real_interpolate(z1, c_s, z2, d_s);
        }
    }

    // Compute t-coordinate of intersection using TransLeq ordering.
    let v_t;
    {
        let (mut a_s, mut a_t) = (o1_s, o1_t);
        let (mut b_s, mut b_t) = (d1_s, d1_t);
        let (mut c_s, mut c_t) = (o2_s, o2_t);
        let (mut d_s, mut d_t) = (d2_s, d2_t);

        if !trans_leq(a_s, a_t, b_s, b_t) {
            core::mem::swap(&mut a_s, &mut b_s);
            core::mem::swap(&mut a_t, &mut b_t);
        }
        if !trans_leq(c_s, c_t, d_s, d_t) {
            core::mem::swap(&mut c_s, &mut d_s);
            core::mem::swap(&mut c_t, &mut d_t);
        }
        if !trans_leq(a_s, a_t, c_s, c_t) {
            core::mem::swap(&mut a_s, &mut c_s);
            core::mem::swap(&mut a_t, &mut c_t);
            core::mem::swap(&mut b_s, &mut d_s);
            core::mem::swap(&mut b_t, &mut d_t);
        }

        if !trans_leq(c_s, c_t, b_s, b_t) {
            v_t = c_t / 2.0 + b_t / 2.0;
        } else if trans_leq(b_s, b_t, d_s, d_t) {
            let mut z1 = trans_eval(a_s, a_t, c_s, c_t, b_s, b_t);
            let mut z2 = trans_eval(c_s, c_t, b_s, b_t, d_s, d_t);
            if z1 + z2 < 0.0 {
                z1 = -z1;
                z2 = -z2;
            }
            v_t = real_interpolate(z1, c_t, z2, b_t);
        } else {
            let mut z1 = trans_sign(a_s, a_t, c_s, c_t, b_s, b_t);
            let mut z2 = -trans_sign(a_s, a_t, d_s, d_t, b_s, b_t);
            if z1 + z2 < 0.0 {
                z1 = -z1;
                z2 = -z2;
            }
            v_t = real_interpolate(z1, c_t, z2, d_t);
        }
    }

    (v_s, v_t)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn vert_leq_basic() {
        assert!(vert_leq(0.0, 0.0, 1.0, 0.0));
        assert!(vert_leq(0.0, 0.0, 0.0, 1.0));
        assert!(vert_leq(0.0, 0.0, 0.0, 0.0));
        assert!(!vert_leq(1.0, 0.0, 0.0, 0.0));
    }

    #[test]
    fn trans_leq_basic() {
        assert!(trans_leq(0.0, 0.0, 0.0, 1.0));
        assert!(trans_leq(0.0, 0.0, 1.0, 0.0));
        assert!(!trans_leq(0.0, 1.0, 0.0, 0.0));
    }

    #[test]
    fn edge_eval_horizontal() {
        // u=(0,0), v=(0.5,1), w=(1,0) -- vertical midpoint of unit interval
        // The t-value of the edge uw at s=0.5 is 0 (since u and w both have t=0).
        // But v.t = 1, so signed distance from uw to v = 1 - 0 = 1.
        let r = edge_eval(0.0, 0.0, 0.5, 1.0, 1.0, 0.0);
        assert!((r - 1.0).abs() < 1e-6, "got {}", r);
    }

    #[test]
    fn edge_eval_vertical_returns_zero() {
        // When u.s == v.s == w.s (vertical), result must be 0.
        let r = edge_eval(0.0, 0.0, 0.0, 0.5, 0.0, 1.0);
        assert_eq!(r, 0.0);
    }

    #[test]
    fn vert_ccw_basic() {
        assert!(vert_ccw(0.0, 0.0, 1.0, 0.0, 0.5, 1.0));
        assert!(!vert_ccw(0.0, 0.0, 0.5, 1.0, 1.0, 0.0));
    }

    #[test]
    fn real_interpolate_midpoint() {
        let r = real_interpolate(0.0, 0.0, 0.0, 1.0);
        assert!((r - 0.5).abs() < 1e-6);
    }

    #[test]
    fn real_interpolate_weighted() {
        // a=1, x=0, b=1, y=2 → (1*0 + 1*2) / 2 = 1? No wait:
        // (b*x + a*y) / (a+b) = (1*0 + 1*2) / 2 = 1
        // But formula is: x + (y-x)*(a/(a+b)) = 0 + 2*(0.5) = 1 ✓
        let r = real_interpolate(1.0, 0.0, 1.0, 2.0);
        assert!((r - 1.0).abs() < 1e-6);
    }

    #[test]
    fn edge_intersect_crossing() {
        // Two edges crossing at (0.5, 0.5):
        // Edge 1: (0,0) → (1,1)
        // Edge 2: (0,1) → (1,0)
        let (s, t) = edge_intersect(0.0, 0.0, 1.0, 1.0, 0.0, 1.0, 1.0, 0.0);
        assert!((s - 0.5).abs() < 1e-5, "s={}", s);
        assert!((t - 0.5).abs() < 1e-5, "t={}", t);
    }
}
