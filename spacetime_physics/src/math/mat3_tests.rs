use super::*;

fn assert_approx_eq(a: Mat3, b: Mat3, epsilon: f32) {
    let Mat3 {
        m11,
        m12,
        m13,
        m21,
        m22,
        m23,
        m31,
        m32,
        m33,
    } = (a - b).abs();
    assert!(m11 < epsilon && m12 < epsilon && m13 < epsilon);
    assert!(m21 < epsilon && m22 < epsilon && m23 < epsilon);
    assert!(m31 < epsilon && m32 < epsilon && m33 < epsilon);
}

#[test]
fn test_identity_matrix_properties() {
    let m = Mat3::IDENTITY;
    assert_eq!(m * Mat3::IDENTITY, Mat3::IDENTITY);

    let v = Vec3::new(1.0, 2.0, 3.0);
    assert_eq!(m * v, v);
}

#[test]
fn test_zero_matrix_properties() {
    let m = Mat3::ZERO;
    let v = Vec3::new(1.0, 2.0, 3.0);
    assert_eq!(m * m, Mat3::ZERO);
    assert_eq!(m * v, Vec3::ZERO);
}

#[test]
fn test_from_diagonal() {
    let diag = Vec3::new(2.0, 3.0, 4.0);
    let m = Mat3::from_diagonal(diag);
    assert_eq!(m, Mat3::new(2.0, 0.0, 0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 4.0));
}

#[test]
fn test_transpose() {
    let m = Mat3::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
    let t = m.transpose();
    assert_eq!(t, Mat3::new(1.0, 4.0, 7.0, 2.0, 5.0, 8.0, 3.0, 6.0, 9.0));
}

#[test]
fn test_determinant_identity() {
    let det = Mat3::IDENTITY.determinant();
    assert_eq!(det, 1.0);
}

#[test]
fn test_determinant_singular() {
    let m = Mat3::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
    let det = m.determinant();
    assert!((det - 0.0).abs() < f32::EPSILON);
}

#[test]
fn test_inverse_identity() {
    let inv = Mat3::IDENTITY.inverse();
    assert_eq!(inv, Mat3::IDENTITY);
}

#[test]
fn test_inverse_singular_returns_zero() {
    let m = Mat3::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
    assert_eq!(m.inverse(), Mat3::ZERO);
}

#[test]
fn test_inverse_general_case() {
    let m = Mat3::new(1.0, 2.0, 3.0, 0.0, 1.0, 4.0, 5.0, 6.0, 0.0);
    let inv = m.inverse();
    let approx_identity = m * inv;
    assert_approx_eq(approx_identity, Mat3::IDENTITY, 1e-5);
}

#[test]
fn test_mat3_mul_scalar() {
    let m = Mat3::IDENTITY;
    let result = 3.0 * m;
    assert_eq!(
        result,
        Mat3::new(3.0, 0.0, 0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 3.0)
    );
}

#[test]
fn test_mat3_mul_vec3() {
    let m = Mat3::new(1.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 3.0);
    let v = Vec3::new(1.0, 1.0, 1.0);
    let result = m * v;
    assert_eq!(result, Vec3::new(1.0, 2.0, 3.0));
}

#[test]
fn test_mat3_mul_mat3() {
    let a = Mat3::new(1.0, 2.0, 3.0, 0.0, 1.0, 4.0, 5.0, 6.0, 0.0);
    let b = a.inverse();
    let result = a * b;
    assert_approx_eq(result, Mat3::IDENTITY, 1e-5);
}
