use super::*;

#[test]
fn test_mat3_identity() {
    let m = Mat3::IDENTITY;
    assert_eq!(m.m11, 1.0);
    assert_eq!(m.m22, 1.0);
    assert_eq!(m.m33, 1.0);
    assert_eq!(m.m12, 0.0);
    assert_eq!(m.m13, 0.0);
    assert_eq!(m.m21, 0.0);
    assert_eq!(m.m23, 0.0);
    assert_eq!(m.m31, 0.0);
    assert_eq!(m.m32, 0.0);
}

#[test]
fn test_mat3_zero() {
    let m = Mat3::ZERO;
    assert_eq!(m, Mat3::new(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
}

#[test]
fn test_mat3_new() {
    let m = Mat3::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
    assert_eq!(m.m11, 1.0);
    assert_eq!(m.m22, 5.0);
    assert_eq!(m.m33, 9.0);
}

#[test]
fn test_mat3_from_diagonal() {
    let diag = Vec3::new(1.0, 2.0, 3.0);
    let m = Mat3::from_diagonal(diag);
    assert_eq!(m.m11, 1.0);
    assert_eq!(m.m22, 2.0);
    assert_eq!(m.m33, 3.0);
    assert_eq!(m.m12, 0.0);
    assert_eq!(m.m13, 0.0);
    assert_eq!(m.m21, 0.0);
    assert_eq!(m.m23, 0.0);
    assert_eq!(m.m31, 0.0);
    assert_eq!(m.m32, 0.0);
}

#[test]
fn test_mat3_transpose() {
    let m = Mat3::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
    let t = m.transpose();
    assert_eq!(t.m12, m.m21);
    assert_eq!(t.m13, m.m31);
    assert_eq!(t.m21, m.m12);
    assert_eq!(t.m31, m.m13);
    assert_eq!(t.m11, m.m11);
    assert_eq!(t.m22, m.m22);
    assert_eq!(t.m33, m.m33);
}

#[test]
fn test_mat3_determinant() {
    let m = Mat3::IDENTITY;
    assert_eq!(m.determinant(), 1.0);

    let singular = Mat3::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
    assert!((singular.determinant() - 0.0).abs() < 1e-6);
}

#[test]
fn test_mat3_inverse_identity() {
    let inv = Mat3::IDENTITY.inverse();
    assert_eq!(inv, Mat3::IDENTITY);
}

#[test]
fn test_mat3_inverse_singular_returns_zero() {
    let m = Mat3::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
    let inv = m.inverse();
    assert_eq!(inv, Mat3::ZERO);
}

#[test]
fn test_mat3_mul_mat3() {
    let a = Mat3::IDENTITY;
    let b = Mat3::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
    let result = a * b;
    assert_eq!(result, b);

    let result2 = b * a;
    assert_eq!(result2, b);
}

#[test]
fn test_mat3_mul_vec3() {
    let m = Mat3::IDENTITY;
    let v = Vec3::new(1.0, 2.0, 3.0);
    let result = m * v;
    assert_eq!(result, v);
}

#[test]
fn test_mat3_mul_scalar() {
    let m = Mat3::IDENTITY;
    let result = 2.0 * m;
    assert_eq!(
        result,
        Mat3::new(2.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 2.0)
    );
}
