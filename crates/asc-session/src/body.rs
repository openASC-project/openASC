use asc_physics::{Body, Error, MassProperties, Result};
use asc_scene::spawns::Spawn;
use glam::{DMat3, DQuat, DVec3};

/// Caller supplies a validated COM position and inertia; marker height and car.ini dimensions
/// are deliberately not inferred to be a solved chassis pose or a measured physical tensor.
pub fn body_with_spawn_basis(
    properties: MassProperties,
    centre_of_mass_m: [f64; 3],
    spawn: &Spawn,
) -> Result<Body> {
    let basis = DMat3::from_cols(
        DVec3::from_array(spawn.left),
        DVec3::from_array(spawn.up),
        DVec3::from_array(spawn.forward),
    );
    let axes = [basis.x_axis, basis.y_axis, basis.z_axis];
    if !basis.is_finite()
        || axes.iter().any(|v| (v.length_squared() - 1.).abs() > 1e-6)
        || [(0, 1), (0, 2), (1, 2)]
            .into_iter()
            .any(|(a, b)| axes[a].dot(axes[b]).abs() > 1e-6)
        || (basis.determinant() - 1.).abs() > 1e-6
        || spawn.up[1] <= 0.5
    {
        return Err(Error::new(
            "physics_spawn_basis",
            "Expected a finite upright rigid spawn basis within 1e-6.",
        ));
    }
    let q = DQuat::from_mat3(&basis).normalize();
    let reconstructed = DMat3::from_quat(q);
    if !q.is_finite()
        || basis
            .to_cols_array()
            .into_iter()
            .zip(reconstructed.to_cols_array())
            .any(|(a, b)| (a - b).abs() > 1e-6)
    {
        return Err(Error::new(
            "physics_spawn_basis",
            "Spawn basis is not rigid within 1e-6; do not repair skew/scale for rigid-body initialization.",
        ));
    }
    Body::new(properties, centre_of_mass_m, q.to_array(), [0.; 3], [0.; 3])
}
