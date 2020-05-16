use super::helper::Float;
use geo_types::{Coordinate, Rect};
use std::cmp::Ordering;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum LineIntersection<F>
where
    F: Float,
{
    None,
    Point(Coordinate<F>),
    Overlap(Coordinate<F>, Coordinate<F>),
}

#[inline]
fn _old_get_intersection_bounding_box<F>(
    a1: Coordinate<F>,
    a2: Coordinate<F>,
    b1: Coordinate<F>,
    b2: Coordinate<F>,
) -> (Option<Rect<F>>, (Coordinate<F>, Coordinate<F>, Coordinate<F>, Coordinate<F>), bool)
where
    F: Float,
{
    let (a_start_x, a_end_x) = if a1.x < a2.x { (a1.x, a2.x) } else { (a2.x, a1.x) };
    let (a_start_y, a_end_y) = if a1.y < a2.y { (a1.y, a2.y) } else { (a2.y, a1.y) };
    let (b_start_x, b_end_x) = if b1.x < b2.x { (b1.x, b2.x) } else { (b2.x, b1.x) };
    let (b_start_y, b_end_y) = if b1.y < b2.y { (b1.y, b2.y) } else { (b2.y, b1.y) };
    let interval_start_x = a_start_x.max(b_start_x);
    let interval_start_y = a_start_y.max(b_start_y);
    let interval_end_x = a_end_x.min(b_end_x);
    let interval_end_y = a_end_y.min(b_end_y);
    if interval_start_x <= interval_end_x && interval_start_y <= interval_end_y {
        (Some(Rect {
            min: Coordinate {
                x: interval_start_x,
                y: interval_start_y,
            },
            max: Coordinate {
                x: interval_end_x,
                y: interval_end_y,
            },
        }), (a1, a2, b1, b2), false)
    } else {
        (None, (a1, a2, b1, b2), false)
    }
}

/// Calculate the AABB which a possible intersection must fall in, IE the intersection of the AABBs for segments a and b.
/// Also return a1, a2, b1, b2 reordered such that a1_out and b1_out point +X, or +Y if they have no X component,
/// a1_out.x <= b1_out.x, and if a1_out.x == b1_out.x then a1_out.y <= b1_out.y.
/// This reordering ensures floating point errors propagate in the same way for intersection(a1, a2, b1, b2),
/// intersection(b1, b2, a1, a2), and all other permutations of the arguments which represent the same pair of
/// actual input segments.
#[inline]
fn get_intersection_bounding_box<F>(
    a1: Coordinate<F>,
    a2: Coordinate<F>,
    b1: Coordinate<F>,
    b2: Coordinate<F>,
) -> (Option<Rect<F>>, (Coordinate<F>, Coordinate<F>, Coordinate<F>, Coordinate<F>), bool)
where
    F: Float,
{
    let mut flip_a = false;
    let mut maybe_flip_a = false;
    let mut flip_b = false;
    let mut maybe_flip_b = false;
    let mut flip_lines = false;
    let mut maybe_flip_lines = false;
    let (a_start_x, a_end_x) = match a1.x.partial_cmp(&a2.x) {
        Some(Ordering::Less) => (a1.x, a2.x),
        Some(Ordering::Greater) => {
            flip_a = true;
            (a2.x, a1.x)
        },
        Some(Ordering::Equal) => {
            maybe_flip_a = true;
            (a1.x, a2.x)
        }
        None => (a1.x, a2.x)
    };
    // let (a_start_x, a_end_x) = if a1.x < a2.x { (a1.x, a2.x) } else { (a2.x, a1.x) };
    let (b_start_x, b_end_x) = match b1.x.partial_cmp(&b2.x) {
        Some(Ordering::Less) => (b1.x, b2.x),
        Some(Ordering::Greater) => {
            flip_b = true;
            (b2.x, b1.x)
        },
        Some(Ordering::Equal) => {
            maybe_flip_b = true;
            (b1.x, b2.x)
        }
        None => (b1.x, b2.x)
    };
    // let (b_start_x, b_end_x) = if b1.x < b2.x { (b1.x, b2.x) } else { (b2.x, b1.x) };
    let (a_start_y, a_end_y) = match a1.y.partial_cmp(&a2.y) {
        Some(Ordering::Less) => (a1.y, a2.y),
        Some(Ordering::Greater) => {
            // Flip if X already required it, or if X was equal.
            flip_a = flip_a || maybe_flip_a;
            (a2.y, a1.y)
        },
        Some(Ordering::Equal) => {
            (a1.y, a2.y)
        }
        None => (a1.y, a2.y)
    };
    // let (a_start_y, a_end_y) = if a1.y < a2.y { (a1.y, a2.y) } else { (a2.y, a1.y) };
    let (b_start_y, b_end_y) = match b1.y.partial_cmp(&b2.y) {
        Some(Ordering::Less) => (b1.y, b2.y),
        Some(Ordering::Greater) => {
            // Flip if X already required it, or if X was equal.
            flip_b = flip_b || maybe_flip_b;
            (b2.y, b1.y)
        },
        Some(Ordering::Equal) => {
            (b1.y, b2.y)
        }
        None => (b1.y, b2.y)
    };
    // let (b_start_y, b_end_y) = if b1.y < b2.y { (b1.y, b2.y) } else { (b2.y, b1.y) };
    let interval_start_x = match a_start_x.partial_cmp(&b_start_x) {
        Some(Ordering::Less) => b_start_x,
        Some(Ordering::Greater) => {
            flip_lines = true;
            a_start_x
        },
        Some(Ordering::Equal) => {
            maybe_flip_lines = true;
            a_start_x
        }
        None => a_start_x
    };
    // let interval_start_x = a_start_x.max(b_start_x);
    let interval_start_y = if a_start_y < b_start_y {
        b_start_y
    } else {
        flip_lines = flip_lines || maybe_flip_lines;
        a_start_y
    };
    // let interval_start_y = a_start_y.max(b_start_y);
    let interval_end_x = a_end_x.min(b_end_x);
    let interval_end_y = a_end_y.min(b_end_y);
    if interval_start_x <= interval_end_x && interval_start_y <= interval_end_y {
        (Some(Rect {
            min: Coordinate {
                x: interval_start_x,
                y: interval_start_y,
            },
            max: Coordinate {
                x: interval_end_x,
                y: interval_end_y,
            },
        }), match (flip_a, flip_b, flip_lines) {
            (false, false, false) => (a1, a2, b1, b2),
            (false, false, true) => (b1, b2, a1, a2),
            (false, true, false) => (a1, a2, b2, b1),
            (false, true, true) => (b2, b1, a1, a2),
            (true, false, false) => (a2, a1, b1, b2),
            (true, false, true) => (b1, b2, a2, a1),
            (true, true, false) => (a2, a1, b2, b1),
            (true, true, true) => (b2, b1, a2, a1) 
        }, flip_a || (flip_lines && flip_b))
    } else {
        // Don't bother shuffling with no intersection to find.
        (None, (a1, a2, b1, b2), false)
    }
}

#[inline]
fn constrain_to_bounding_box<F>(p: Coordinate<F>, bb: Rect<F>) -> Coordinate<F>
where
    F: Float,
{
    Coordinate {
        x: if p.x < bb.min.x {
            bb.min.x
        } else if p.x > bb.max.x {
            bb.max.x
        } else {
            p.x
        },
        y: if p.y < bb.min.y {
            bb.min.y
        } else if p.y > bb.max.y {
            bb.max.y
        } else {
            p.y
        },
    }
}

pub fn intersection<F>(
    a1_in: Coordinate<F>,
    a2_in: Coordinate<F>,
    b1_in: Coordinate<F>,
    b2_in: Coordinate<F>,
) -> LineIntersection<F>
where
    F: Float,
{
    // let (bb, (a1, a2, b1, b2), flip) = _old_get_intersection_bounding_box(a1_in, a2_in, b1_in, b2_in);
    let (bb, (a1, a2, b1, b2), flip) = get_intersection_bounding_box(a1_in, a2_in, b1_in, b2_in);
    if let Some(bb) = bb {
        let inter = intersection_impl(a1, a2, b1, b2);
        match inter {
            LineIntersection::None => LineIntersection::None,
            LineIntersection::Point(p) => LineIntersection::Point(constrain_to_bounding_box(p, bb)),
            LineIntersection::Overlap(p1, p2) => {
                if flip {
                    LineIntersection::Overlap(constrain_to_bounding_box(p2, bb), constrain_to_bounding_box(p1, bb))
                } else {
                    LineIntersection::Overlap(constrain_to_bounding_box(p1, bb), constrain_to_bounding_box(p2, bb))
                }
            }
        }
    } else {
        LineIntersection::None
    }
}

fn intersection_impl<F>(
    a1: Coordinate<F>,
    a2: Coordinate<F>,
    b1: Coordinate<F>,
    b2: Coordinate<F>,
) -> LineIntersection<F>
where
    F: Float,
{
    // println!("{:?} {:?} {:?} {:?}", a1, a2, b1, b2);
    let va = Coordinate {
        x: a2.x - a1.x,
        y: a2.y - a1.y,
    };
    let vb = Coordinate {
        x: b2.x - b1.x,
        y: b2.y - b1.y,
    };
    let e = Coordinate {
        x: b1.x - a1.x,
        y: b1.y - a1.y,
    };
    let mut kross = cross_product(va, vb);
    let mut sqr_kross = kross * kross;
    let sqr_len_a = dot_product(va, va);

    if sqr_kross > F::zero() {
        let s = cross_product(e, vb) / kross;
        if s < F::zero() || s > F::one() {
            return LineIntersection::None;
        }
        let t = cross_product(e, va) / kross;
        if t < F::zero() || t > F::one() {
            return LineIntersection::None;
        }

        if s == F::zero() || s == F::one() {
            return LineIntersection::Point(mid_point(a1, s, va));
        }
        if t == F::zero() || t == F::one() {
            return LineIntersection::Point(mid_point(b1, t, vb));
        }

        return LineIntersection::Point(mid_point(a1, s, va));
    }

    kross = cross_product(e, va);
    sqr_kross = kross * kross;

    if sqr_kross > F::zero() {
        return LineIntersection::None;
    }

    let sa = dot_product(va, e) / sqr_len_a;
    let sb = sa + dot_product(va, vb) / sqr_len_a;
    let smin = sa.min(sb);
    let smax = sa.max(sb);

    if smin <= F::one() && smax >= F::zero() {
        if smin == F::one() {
            return LineIntersection::Point(mid_point(a1, smin, va));
        }
        if smax == F::zero() {
            return LineIntersection::Point(mid_point(a1, smax, va));
        }

        return LineIntersection::Overlap(
            mid_point(a1, smin.max(F::zero()), va),
            mid_point(a1, smax.min(F::one()), va),
        );
    }

    LineIntersection::None
}

fn mid_point<F>(p: Coordinate<F>, s: F, d: Coordinate<F>) -> Coordinate<F>
where
    F: Float,
{
    Coordinate {
        x: p.x + s * d.x,
        y: p.y + s * d.y,
    }
}

#[inline]
fn cross_product<F>(a: Coordinate<F>, b: Coordinate<F>) -> F
where
    F: Float,
{
    a.x * b.y - a.y * b.x
}

#[inline]
fn dot_product<F>(a: Coordinate<F>, b: Coordinate<F>) -> F
where
    F: Float,
{
    a.x * b.x + a.y * b.y
}

#[cfg(test)]
mod test {
    use super::super::helper::test::xy;
    use super::*;

    fn rect(min: Coordinate<f64>, max: Coordinate<f64>) -> Rect<f64> {
        Rect { min, max }
    }

    #[test]
    fn test_get_intersection_bounding_box() {
        assert_eq!(
            get_intersection_bounding_box(xy(0, 0), xy(2, 2), xy(1, 1), xy(3, 3)).0,
            Some(Rect {
                min: xy(1, 1),
                max: xy(2, 2)
            }),
        );
        assert_eq!(
            get_intersection_bounding_box(xy(-1, 0), xy(1, 0), xy(0, -1), xy(0, 1)).0,
            Some(Rect {
                min: xy(0, 0),
                max: xy(0, 0)
            }),
        );
        assert_eq!(
            get_intersection_bounding_box(xy(0, 0), xy(1, 1), xy(2, 0), xy(3, 1)).0,
            None,
        );
        assert_eq!(
            get_intersection_bounding_box(xy(3, 0), xy(2, 1), xy(1, 0), xy(0, 1)).0,
            None,
        );
        assert_eq!(
            get_intersection_bounding_box(xy(0, 0), xy(1, 1), xy(0, 2), xy(1, 3)).0,
            None,
        );
        assert_eq!(
            get_intersection_bounding_box(xy(0, 3), xy(1, 2), xy(0, 1), xy(1, 0)).0,
            None,
        );
    }

    #[test]
    fn test_constrain_to_bounding_box() {
        assert_eq!(
            constrain_to_bounding_box(xy(100, 0), rect(xy(-1, -1), xy(1, 1))),
            xy(1, 0),
        );
        assert_eq!(
            constrain_to_bounding_box(xy(-100, 0), rect(xy(-1, -1), xy(1, 1))),
            xy(-1, 0),
        );
        assert_eq!(
            constrain_to_bounding_box(xy(0, 100), rect(xy(-1, -1), xy(1, 1))),
            xy(0, 1),
        );
        assert_eq!(
            constrain_to_bounding_box(xy(0, -100), rect(xy(-1, -1), xy(1, 1))),
            xy(0, -1),
        );
    }

    #[test]
    fn test_intersection() {
        assert_eq!(
            intersection(xy(0, 0), xy(1, 1), xy(1, 0), xy(2, 2)),
            LineIntersection::None
        );
        assert_eq!(
            intersection(xy(0, 0), xy(1, 1), xy(1, 0), xy(10, 2)),
            LineIntersection::None
        );
        assert_eq!(
            intersection(xy(2, 2), xy(3, 3), xy(0, 6), xy(2, 4)),
            LineIntersection::None
        );

        assert_eq!(
            intersection(xy(0, 0), xy(1, 1), xy(1, 0), xy(0, 1)),
            LineIntersection::Point(xy(0.5, 0.5))
        );

        assert_eq!(
            intersection(xy(0, 0), xy(1, 1), xy(0, 1), xy(0, 0)),
            LineIntersection::Point(xy(0, 0))
        );
        assert_eq!(
            intersection(xy(0, 0), xy(1, 1), xy(0, 1), xy(1, 1)),
            LineIntersection::Point(xy(1, 1))
        );

        assert_eq!(
            intersection(xy(0, 0), xy(1, 1), xy(0.5, 0.5), xy(1, 0)),
            LineIntersection::Point(xy(0.5, 0.5))
        );

        assert_eq!(
            intersection(xy(0, 0), xy(10, 10), xy(1, 1), xy(5, 5)),
            LineIntersection::Overlap(xy(1, 1), xy(5, 5))
        );
        assert_eq!(
            intersection(xy(1, 1), xy(10, 10), xy(1, 1), xy(5, 5)),
            LineIntersection::Overlap(xy(1, 1), xy(5, 5))
        );
        assert_eq!(
            intersection(xy(3, 3), xy(10, 10), xy(0, 0), xy(5, 5)),
            LineIntersection::Overlap(xy(3, 3), xy(5, 5))
        );
        assert_eq!(
            intersection(xy(0, 0), xy(1, 1), xy(0, 0), xy(1, 1)),
            LineIntersection::Overlap(xy(0, 0), xy(1, 1))
        );
        assert_eq!(
            intersection(xy(1, 1), xy(0, 0), xy(0, 0), xy(1, 1)),
            LineIntersection::Overlap(xy(1, 1), xy(0, 0))
        );

        assert_eq!(
            intersection(xy(0, 0), xy(1, 1), xy(1, 1), xy(2, 2)),
            LineIntersection::Point(xy(1, 1))
        );
        assert_eq!(
            intersection(xy(1, 1), xy(0, 0), xy(1, 1), xy(2, 2)),
            LineIntersection::Point(xy(1, 1))
        );
        assert_eq!(
            intersection(xy(0, 0), xy(1, 1), xy(2, 2), xy(4, 4)),
            LineIntersection::None
        );
        assert_eq!(
            intersection(xy(0, 0), xy(1, 1), xy(0, -1), xy(1, 0)),
            LineIntersection::None
        );
        assert_eq!(
            intersection(xy(1, 1), xy(0, 0), xy(0, -1), xy(1, 0)),
            LineIntersection::None
        );
        assert_eq!(
            intersection(xy(0, -1), xy(1, 0), xy(0, 0), xy(1, 1)),
            LineIntersection::None
        );

        assert_eq!(
            intersection(xy(0, 0.5), xy(1, 1.5), xy(0, 1), xy(1, 0)),
            LineIntersection::Point(xy(0.25, 0.75))
        );

        assert_eq!(
            intersection(xy(0, 0), xy(1, 0), xy(1, -1), xy(2, 1)),
            LineIntersection::None
        );
    }

    extern crate rand;
    use rand::Rng;

    fn random_coord() -> Coordinate<f32> {
        let mut rng = rand::thread_rng();
        let x: f32 = rng.gen();
        let y: f32 = rng.gen();
        return Coordinate{x: x, y: y}
    }

    #[test]
    fn test_intersection_order_same_value() {
        fn check_all_combinations(a1: Coordinate<f32>,
                                  a2: Coordinate<f32>,
                                  b1: Coordinate<f32>,
                                  b2: Coordinate<f32>) {
            let p1234 = intersection(a1, a2, b1, b2);
            assert_eq!(p1234, intersection(a2, a1, b1, b2));
            assert_eq!(p1234, intersection(a1, a2, b2, b1));
            assert_eq!(p1234, intersection(a2, a1, b2, b1));
            assert_eq!(p1234, intersection(b1, b2, a1, a2));
            assert_eq!(p1234, intersection(b1, b2, a2, a1));
            assert_eq!(p1234, intersection(b2, b1, a1, a2));
            assert_eq!(p1234, intersection(b2, b1, a2, a1));
        }

        for _ in 0..100000 {
            let a1 = random_coord();
            let a2 = random_coord();
            let b1 = random_coord();
            let b2 = random_coord();
            let bm = Coordinate{x: (b1.x + b2.x)/2., y: (b1.y + b2.y)/2.};
            check_all_combinations(a1, a2, b1, b2);
            check_all_combinations(a1, a2, a1, b2);
            check_all_combinations(a1, bm, b1, b2);
            }
    }

    #[test]
    fn test_intersection_order_same_variant() {

        fn check_same_variant(i1: LineIntersection<f32>, i2: LineIntersection<f32>){
            use LineIntersection::*;
            let same = match (i1, i2) {
                (LineIntersection::None, LineIntersection::None) => true,
                (Point(_a), Point(_b)) => true,
                (Overlap(_a, _b), Overlap(_c, _d)) => true,
                _ => false
            };
            assert!(same, "{:?} differs from {:?} with coords:", i1, i2);
            // if !same {
            //     println!("{:?} differs from {:?} with coords:", i1, i2);
            // }
            // same
        }

        fn check_all_combinations(a1: Coordinate<f32>,
                                  a2: Coordinate<f32>,
                                  b1: Coordinate<f32>,
                                  b2: Coordinate<f32>) {
            let p1234 = intersection(a1, a2, b1, b2);
            check_same_variant(p1234, intersection(a2, a1, b1, b2));
            check_same_variant(p1234, intersection(a1, a2, b2, b1));
            check_same_variant(p1234, intersection(a2, a1, b2, b1));
            check_same_variant(p1234, intersection(b1, b2, a1, a2));
            check_same_variant(p1234, intersection(b1, b2, a2, a1));
            check_same_variant(p1234, intersection(b2, b1, a1, a2));
            check_same_variant(p1234, intersection(b2, b1, a2, a1));
        }

        for _ in 0..100000 {
            let a1 = random_coord();
            let a2 = random_coord();
            let b1 = random_coord();
            let b2 = random_coord();
            let bm = Coordinate{x: (b1.x + b2.x)/2., y: (b1.y + b2.y)/2.};
            // Shared endpoint
            check_all_combinations(a1, a2, a1, b2);
            // Endpoint of one in interior of other
            check_all_combinations(a1, bm, b1, b2);
            // Colinear shared endpoint
            check_all_combinations(b1, bm, bm, b2);
            }
    }
}
