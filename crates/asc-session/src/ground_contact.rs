//! Normal support on finite road-face interiors. No edge/vertex or swept response.
use asc_ground::Ground;
use asc_physics::{
    Error, NormalContact, NormalContactSample, Result, SupportPlane, TangentialContact,
    TangentialContactSample, WheelKinematics,
};
use glam::DVec3;
use serde::Serialize;

#[derive(Debug, Serialize)]
pub struct TrackFace {
    pub triangle: usize,
    pub mesh: String,
    pub model: String,
    pub surface_key: String,
    /// Used as an explicit scale only when the optional friction provider is enabled.
    pub surface_friction: f64,
    /// Metadata only; normal damping is explicitly supplied by the caller.
    pub surface_damping: f64,
    pub valid_track: bool,
    pub pitlane: bool,
    pub surface_source: String,
    pub surface_section: String,
    pub surface_line: usize,
}
#[derive(Debug, Serialize)]
pub struct TrackContact {
    pub face: TrackFace,
    pub sample: NormalContactSample,
    pub tangential: Option<TangentialContactSample>,
}
fn geometry_error(e: asc_config::Error) -> Error {
    Error::new(e.code, &e.to_string())
}
/// Select the smallest signed circle/plane gap among eligible finite faces.
/// Ties within 1e-9 m use source triangle order, so seams never double the force.
/// The caller chooses the minimum upward normal component (1e-4..1).
/// A circle overlapping a face from below is an error; fully overhead faces are ignored.
pub fn face_contact(
    ground: &Ground,
    wheel: &WheelKinematics,
    law: &NormalContact,
    min_normal_y: f64,
) -> Result<Option<TrackContact>> {
    face_contact_with_friction(ground, wheel, law, min_normal_y, None)
}
/// Same selection and geometry limits as face_contact. With friction enabled,
/// selected surface FRICTION scales both explicit tyre coefficients (0..10).
pub fn face_contact_with_friction(
    ground: &Ground,
    wheel: &WheelKinematics,
    law: &NormalContact,
    min_normal_y: f64,
    friction: Option<&TangentialContact>,
) -> Result<Option<TrackContact>> {
    if !min_normal_y.is_finite() || !(1e-4..=1.).contains(&min_normal_y) {
        return Err(Error::new(
            "track_contact_normal",
            "Minimum normal Y must lie within 1e-4..1",
        ));
    }
    let extent = law.radius_m() + law.max_penetration_m();
    let faces = ground
        .faces_in_box(wheel.body.position_m, [extent; 3])
        .map_err(geometry_error)?;
    let mut selected = None;
    let mut selected_gap = f64::INFINITY;
    for face in faces {
        let n = face.normal();
        if n[1] < min_normal_y {
            continue;
        }
        let plane = SupportPlane::new(face.points()[0], n)?;
        let geometry = law.support_geometry(wheel, plane)?;
        if !face
            .contains_point(geometry.plane_point_world_m)
            .map_err(geometry_error)?
        {
            continue;
        }
        let distance = DVec3::from_array(n)
            .dot(DVec3::from_array(wheel.body.position_m) - DVec3::from_array(face.points()[0]));
        if distance < -1e-8 {
            let radial_extent = distance - geometry.gap_m;
            if -distance > radial_extent + 1e-8 {
                continue;
            }
            return Err(Error::new(
                "track_contact_backside",
                &format!(
                    "Wheel overlaps triangle {} from below; backside collision response is unsupported",
                    face.triangle
                ),
            ));
        }
        if geometry.gap_m < selected_gap - 1e-9 {
            selected_gap = geometry.gap_m;
            selected = Some((face, plane));
        }
    }
    selected
        .map(|(face, plane)| {
            let result = if let Some(friction) = friction {
                law.sample_with_friction(wheel, plane, friction, face.surface.friction)
                    .map(|c| (c.normal, Some(c.tangential)))
            } else {
                law.sample(wheel, plane).map(|s| (s, None))
            };
            let (sample, tangential) = result.map_err(|e| {
                Error::new(
                    e.code,
                    &format!(
                        "Track triangle {} ({}): {}",
                        face.triangle, face.mesh, e.message
                    ),
                )
            })?;
            let s = face.surface;
            Ok(TrackContact {
                face: TrackFace {
                    triangle: face.triangle,
                    mesh: face.mesh.into(),
                    model: face.model.into(),
                    surface_key: s.key.clone(),
                    surface_friction: s.friction,
                    surface_damping: s.damping,
                    valid_track: s.valid_track,
                    pitlane: s.pitlane,
                    surface_source: s.source.clone(),
                    surface_section: s.section.clone(),
                    surface_line: s.line,
                },
                sample,
                tangential,
            })
        })
        .transpose()
}
