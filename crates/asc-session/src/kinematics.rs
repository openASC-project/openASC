use asc_physics::{Error, FiveLink, LinkPose, Result, StrutLink};
use asc_vehicle::suspension::Axle;

/// Explicit raw design-coordinate diagnostic. This does not infer AC side
/// mirroring, static camber/toe, BASEY, track placement or wheel force laws.
pub fn dwb_geometry(axle: &Axle, travel_axis: [f64; 3]) -> Result<FiveLink> {
    dwb_geometry_with_reflection(axle, travel_axis, false)
}
/// Reflect source hardpoint X coordinates before constructing a right-handed mechanism.
/// Travel/rack vectors and all other caller-supplied parameters are in the resulting frame.
pub fn dwb_geometry_with_reflection(
    axle: &Axle,
    travel_axis: [f64; 3],
    reflect_x: bool,
) -> Result<FiveLink> {
    if axle.kind != "DWB" {
        return Err(Error::new(
            "suspension_kinematics_type",
            "The five-distance geometry requires explicit DWB source records",
        ));
    }
    let get = |key: &str| {
        axle.hardpoints
            .get(key)
            .copied()
            .map(|p| reflect_point(p, reflect_x))
            .ok_or_else(|| {
                Error::new(
                    "suspension_kinematics_point",
                    &format!("Missing DWB hardpoint {key}"),
                )
            })
    };
    FiveLink::new(
        [
            get("WBCAR_TOP_FRONT")?,
            get("WBCAR_TOP_REAR")?,
            get("WBCAR_BOTTOM_FRONT")?,
            get("WBCAR_BOTTOM_REAR")?,
            get("WBCAR_STEER")?,
        ],
        [
            get("WBTYRE_TOP")?,
            get("WBTYRE_TOP")?,
            get("WBTYRE_BOTTOM")?,
            get("WBTYRE_BOTTOM")?,
            get("WBTYRE_STEER")?,
        ],
        LinkPose::new([0.; 3], [0., 0., 0., 1.])?,
        travel_axis,
    )
}

/// Raw design-coordinate STRUT geometry, with an upright-local telescoping axis
/// passing through STRUT_TYRE and the fixed STRUT_CAR top at the reference pose.
pub fn strut_geometry(axle: &Axle, travel_axis: [f64; 3]) -> Result<StrutLink> {
    strut_geometry_with_reflection(axle, travel_axis, false)
}
/// Reflect all source hardpoints, including both ends of the strut; no improper pose is created.
pub fn strut_geometry_with_reflection(
    axle: &Axle,
    travel_axis: [f64; 3],
    reflect_x: bool,
) -> Result<StrutLink> {
    if axle.kind != "STRUT" {
        return Err(Error::new(
            "suspension_kinematics_type",
            "Strut geometry requires explicit STRUT source records",
        ));
    }
    let get = |key: &str| {
        axle.hardpoints
            .get(key)
            .copied()
            .map(|p| reflect_point(p, reflect_x))
            .ok_or_else(|| {
                Error::new(
                    "suspension_kinematics_point",
                    &format!("Missing STRUT hardpoint {key}"),
                )
            })
    };
    StrutLink::new(
        [
            get("WBCAR_BOTTOM_FRONT")?,
            get("WBCAR_BOTTOM_REAR")?,
            get("WBCAR_STEER")?,
        ],
        [
            get("WBTYRE_BOTTOM")?,
            get("WBTYRE_BOTTOM")?,
            get("WBTYRE_STEER")?,
        ],
        get("STRUT_CAR")?,
        get("STRUT_TYRE")?,
        LinkPose::new([0.; 3], [0., 0., 0., 1.])?,
        travel_axis,
    )
}
fn reflect_point(mut p: [f64; 3], reflect_x: bool) -> [f64; 3] {
    if reflect_x {
        p[0] = -p[0];
    }
    p
}
