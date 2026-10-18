use asc_physics::{Error, FiveLink, LinkPose, Result};
use asc_vehicle::suspension::Axle;

/// Explicit raw design-coordinate diagnostic. This does not infer AC side
/// mirroring, static camber/toe, BASEY, track placement or wheel force laws.
pub fn dwb_geometry(axle: &Axle, travel_axis: [f64; 3]) -> Result<FiveLink> {
    if axle.kind != "DWB" {
        return Err(Error::new(
            "suspension_kinematics_type",
            "The five-distance geometry requires explicit DWB source records",
        ));
    }
    let get = |key: &str| {
        axle.hardpoints.get(key).copied().ok_or_else(|| {
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
