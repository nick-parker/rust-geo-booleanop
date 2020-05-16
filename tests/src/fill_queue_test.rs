use super::helper::fixture_shapes;
use geo::{Coordinate, Rect};
use geo_booleanop::boolean::fill_queue::fill_queue;
use geo_booleanop::boolean::Operation;
use num_traits::Float;

#[test]
fn test_two_polygons() {
    let (s, c) = fixture_shapes("two_shapes.geojson");
    let mut sbbox = Rect::new(Coordinate{x: 0f64, y: 0f64}, Coordinate{x: 0f64, y: 0f64});
    let mut cbbox = sbbox;
    let mut q = fill_queue(&[s], &[c], &mut sbbox, &mut cbbox, Operation::Intersection);

    let mut sorted = Vec::new();
    let mut keep_ref = Vec::new();
    while let Some(e) = q.pop() {
        keep_ref.push(e.clone());
        sorted.push((
            e.is_left(),
            e.point.x,
            e.point.y,
            e.get_other_event().unwrap().point.x,
            e.get_other_event().unwrap().point.y,
        ));
    }

    assert_eq!(
        sorted,
        vec![
            (true, 16.0, 282.0, 153.0, 203.5),
            (true, 16.0, 282.0, 298.0, 359.0),
            (true, 56.0, 181.0, 108.5, 120.0),
            (true, 56.0, 181.0, 153.0, 294.5),
            (false, 108.5, 120.0, 56.0, 181.0),
            (true, 108.5, 120.0, 241.5, 229.5),
            (false, 153.0, 203.5, 16.0, 282.0),
            (true, 153.0, 203.5, 298.0, 359.0),
            (false, 153.0, 294.5, 56.0, 181.0),
            (true, 153.0, 294.5, 241.5, 229.5),
            (false, 241.5, 229.5, 108.5, 120.0),
            (false, 241.5, 229.5, 153.0, 294.5),
            (false, 298.0, 359.0, 153.0, 203.5),
            (false, 298.0, 359.0, 16.0, 282.0),
        ]
    );
}

#[test]
fn test_fill_event_queue() {
    fn xy<T: Into<f64>>(x: T, y: T) -> Coordinate<f64> {
        Coordinate {
            x: x.into(),
            y: y.into(),
        }
    }

    let (s, c) = fixture_shapes("two_triangles.geojson");
    let mut sbbox = Rect::new(xy(0f64, 0f64), xy(0f64, 0f64));
    let mut cbbox = sbbox;
    let mut q = fill_queue(&[s], &[c], &mut sbbox, &mut cbbox, Operation::Intersection);

    assert_eq!(
        sbbox,
        Rect::new(
            xy(20.0, -113.5),
            xy(226.5, 74.0)
        ),
    );
    assert_eq!(
        cbbox,
        Rect::new(
            xy(54.5, -198.0),
            xy(239.5, 33.5)
        ),
    );

    let mut sorted = Vec::new();
    let mut keep_ref = Vec::new();
    while let Some(e) = q.pop() {
        keep_ref.push(e.clone());
        sorted.push((
            e.point.x,
            e.point.y,
            e.is_left(),
            e.get_other_event().unwrap().point.x,
            e.get_other_event().unwrap().point.y,
            e.get_other_event().unwrap().is_left(),
        ));
    }
    assert_eq!(
        sorted,
        vec![
            (20.0, -23.5, true, 226.5, -113.5, false),
            (20.0, -23.5, true, 170.0, 74.0, false),
            (54.5, -170.5, true, 239.5, -198.0, false),
            (54.5, -170.5, true, 140.5, 33.5, false),
            (140.5, 33.5, false, 54.5, -170.5, true),
            (140.5, 33.5, true, 239.5, -198.0, false),
            (170.0, 74.0, false, 20.0, -23.5, true),
            (170.0, 74.0, true, 226.5, -113.5, false),
            (226.5, -113.5, false, 20.0, -23.5, true),
            (226.5, -113.5, false, 170.0, 74.0, true),
            (239.5, -198.0, false, 54.5, -170.5, true),
            (239.5, -198.0, false, 140.5, 33.5, true)
        ]
    );
}
