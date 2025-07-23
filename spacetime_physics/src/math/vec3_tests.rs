use super::*;
use parry3d::na::{Point3, Translation3, Unit, Vector3};

#[test]
fn test_vec3_length() {
    let v = Vec3::new(3.0, 4.0, 0.0);
    assert_eq!(v.length(), 5.0);
}

#[test]
fn test_vec3_length_zero() {
    let v = Vec3::new(0.0, 0.0, 0.0);
    assert_eq!(v.length(), 0.0);
}

#[test]
fn test_vec3_length_one() {
    let v = Vec3::new(1.0, 0.0, 0.0);
    assert_eq!(v.length(), 1.0);
}

#[test]
fn test_vec3_length_negative_components() {
    let v = Vec3::new(-3.0, -4.0, 0.0);
    assert_eq!(v.length(), 5.0);
}

#[test]
fn test_vec3_length_unit_x() {
    let v = Vec3::new(1.0, 0.0, 0.0);
    assert_eq!(v.length(), 1.0);
}

#[test]
fn test_vec3_dot_product_orthogonal() {
    let a = Vec3::X;
    let b = Vec3::Y;
    assert_eq!(a.dot(b), 0.0);
}

#[test]
fn test_vec3_dot_product_parallel() {
    let a = Vec3::new(1.0, 2.0, 3.0);
    assert_eq!(a.dot(a), a.length_squared());
}

#[test]
fn test_vec3_cross_product_x_y() {
    let a = Vec3::X;
    let b = Vec3::Y;
    assert_eq!(a.cross(b), Vec3::Z);
}

#[test]
fn test_vec3_cross_product_y_z() {
    let a = Vec3::Y;
    let b = Vec3::Z;
    assert_eq!(a.cross(b), Vec3::X);
}

#[test]
fn test_vec3_cross_product_z_x() {
    let a = Vec3::Z;
    let b = Vec3::X;
    assert_eq!(a.cross(b), Vec3::Y);
}

#[test]
fn test_vec3_normalize() {
    let v = Vec3::new(0.0, 3.0, 4.0);
    let n = v.normalize();
    assert!((n.length() - 1.0).abs() < 1e-6);
}

#[test]
fn test_vec3_normalize_zero_returns_zero() {
    let v = Vec3::ZERO;
    assert_eq!(v.normalize(), Vec3::ZERO);
}

#[test]
fn test_vec3_normalize_or_fallback() {
    let v = Vec3::ZERO;
    let fallback = Vec3::new(1.0, 0.0, 0.0);
    assert_eq!(v.normalize_or(fallback), fallback);
}

#[test]
fn test_vec3_addition() {
    let a = Vec3::new(1.0, 2.0, 3.0);
    let b = Vec3::new(4.0, 5.0, 6.0);
    assert_eq!(a + b, Vec3::new(5.0, 7.0, 9.0));
}

#[test]
fn test_vec3_subtraction() {
    let a = Vec3::new(4.0, 5.0, 6.0);
    let b = Vec3::new(1.0, 2.0, 3.0);
    assert_eq!(a - b, Vec3::new(3.0, 3.0, 3.0));
}

#[test]
fn test_vec3_scalar_multiplication() {
    let v = Vec3::new(1.0, -2.0, 3.0);
    assert_eq!(v * 2.0, Vec3::new(2.0, -4.0, 6.0));
}

#[test]
fn test_vec3_scalar_division() {
    let v = Vec3::new(2.0, 4.0, 6.0);
    assert_eq!(v / 2.0, Vec3::new(1.0, 2.0, 3.0));
}

#[test]
fn test_vec3_negation() {
    let v = Vec3::new(1.0, -2.0, 3.0);
    assert_eq!(-v, Vec3::new(-1.0, 2.0, -3.0));
}

#[test]
fn test_vec3_abs() {
    let v = Vec3::new(-1.0, 2.0, -3.0);
    assert_eq!(v.abs(), Vec3::new(1.0, 2.0, 3.0));
}

#[test]
fn test_vec3_min() {
    let a = Vec3::new(1.0, 4.0, 3.0);
    let b = Vec3::new(2.0, 3.0, 5.0);
    assert_eq!(a.min(b), Vec3::new(1.0, 3.0, 3.0));
}

#[test]
fn test_vec3_project_onto_plane() {
    let v = Vec3::new(1.0, 2.0, 3.0);
    let n = Vec3::Y;
    let projected = v.project_onto_plane(n);
    assert_eq!(projected, Vec3::new(1.0, 0.0, 3.0));
}

#[test]
fn test_vec3_to_vector3() {
    let v = Vec3::new(1.0, 2.0, 3.0);
    let vec: Vector3<f32> = v.into();
    assert_eq!(vec, Vector3::new(1.0, 2.0, 3.0));
}

#[test]
fn test_vector3_to_vec3() {
    let vec = Vector3::new(4.0, 5.0, 6.0);
    let v: Vec3 = vec.into();
    assert_eq!(v, Vec3::new(4.0, 5.0, 6.0));
}

#[test]
fn test_vec3_to_point3() {
    let v = Vec3::new(7.0, 8.0, 9.0);
    let p: Point3<f32> = v.into();
    assert_eq!(p, Point3::new(7.0, 8.0, 9.0));
}

#[test]
fn test_point3_to_vec3() {
    let p = Point3::new(1.0, 2.0, 3.0);
    let v: Vec3 = p.into();
    assert_eq!(v, Vec3::new(1.0, 2.0, 3.0));
}

#[test]
fn test_vec3_to_translation3() {
    let v = Vec3::new(1.5, -2.0, 0.75);
    let t: Translation3<f32> = v.into();
    assert_eq!(t.vector, Vector3::new(1.5, -2.0, 0.75));
}

#[test]
fn test_vec3_to_unit_vector() {
    let v = Vec3::new(0.0, 3.0, 4.0);
    let unit: Unit<Vector3<f32>> = v.into();
    assert!((unit.norm() - 1.0).abs() < 1e-6);
    assert_eq!(unit.into_inner(), Vector3::new(0.0, 0.6, 0.8));
}

#[test]
fn test_unit_vector_to_vec3() {
    let unit = Unit::new_normalize(Vector3::new(0.0, 6.0, 8.0));
    let v: Vec3 = unit.into();
    assert_eq!(v, Vec3::new(0.0, 0.6, 0.8));
}

#[test]
fn test_add_assign_vec3() {
    let mut a = Vec3::new(1.0, 2.0, 3.0);
    let b = Vec3::new(4.0, 5.0, 6.0);
    a += b;
    assert_eq!(a, Vec3::new(5.0, 7.0, 9.0));
}

#[test]
fn test_add_assign_scalar() {
    let mut a = Vec3::new(1.0, 2.0, 3.0);
    a += 1.0;
    assert_eq!(a, Vec3::new(2.0, 3.0, 4.0));
}

#[test]
fn test_sub_assign_vec3() {
    let mut a = Vec3::new(5.0, 7.0, 9.0);
    let b = Vec3::new(1.0, 2.0, 3.0);
    a -= b;
    assert_eq!(a, Vec3::new(4.0, 5.0, 6.0));
}

#[test]
fn test_sub_assign_scalar() {
    let mut a = Vec3::new(5.0, 6.0, 7.0);
    a -= 1.0;
    assert_eq!(a, Vec3::new(4.0, 5.0, 6.0));
}

#[test]
fn test_mul_assign_vec3() {
    let mut a = Vec3::new(1.0, 2.0, 3.0);
    let b = Vec3::new(2.0, 3.0, 4.0);
    a *= b;
    assert_eq!(a, Vec3::new(2.0, 6.0, 12.0));
}

#[test]
fn test_mul_assign_scalar() {
    let mut a = Vec3::new(1.0, -2.0, 3.0);
    a *= 2.0;
    assert_eq!(a, Vec3::new(2.0, -4.0, 6.0));
}

#[test]
fn test_div_assign_vec3() {
    let mut a = Vec3::new(2.0, 6.0, 12.0);
    let b = Vec3::new(2.0, 3.0, 4.0);
    a /= b;
    assert_eq!(a, Vec3::new(1.0, 2.0, 3.0));
}

#[test]
fn test_div_assign_scalar() {
    let mut a = Vec3::new(2.0, 4.0, 6.0);
    a /= 2.0;
    assert_eq!(a, Vec3::new(1.0, 2.0, 3.0));
}
