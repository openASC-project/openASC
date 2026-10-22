//! Original vehicle configuration literals shared by native tests.
use std::collections::BTreeMap;
/// Original constrained-dynamics fixture, independent of installed game data.
pub fn suspension_motion_file() -> String {
    let mut text = "[HEADER]\nVERSION=1\n[BASIC]\nWHEELBASE=2.4\nCG_LOCATION=.5\n".to_owned();
    for (axle, kind) in [("FRONT", "STRUT"), ("REAR", "DWB")] {
        text +=
            &format!("[{axle}]\nTYPE={kind}\nTRACK=1.5\nBASEY=-.2\nROD_LENGTH=.1\nHUB_MASS=30\n");
        text += "WBCAR_TOP_FRONT=-.3,.2,.12\nWBCAR_TOP_REAR=-.3,.2,-.12\nWBTYRE_TOP=0,.2,0\nWBCAR_BOTTOM_FRONT=-.3,-.2,.12\nWBCAR_BOTTOM_REAR=-.3,-.2,-.12\nWBTYRE_BOTTOM=0,-.2,0\nWBCAR_STEER=-.3,0,.15\nWBTYRE_STEER=0,0,.15\nSTRUT_CAR=-.2,.4,0\nSTRUT_TYRE=0,-.2,0\n";
        for key in [
            "SPRING_RATE",
            "PROGRESSIVE_SPRING_RATE",
            "BUMP_STOP_RATE",
            "BUMPSTOP_UP",
            "BUMPSTOP_DN",
            "PACKER_RANGE",
            "DAMP_BUMP",
            "DAMP_FAST_BUMP",
            "DAMP_FAST_BUMPTHRESHOLD",
            "DAMP_REBOUND",
            "DAMP_FAST_REBOUND",
            "DAMP_FAST_REBOUNDTHRESHOLD",
            "TOE_OUT",
            "STATIC_CAMBER",
        ] {
            text += &format!("{key}=0\n");
        }
    }
    text
}
pub fn vehicle_files() -> BTreeMap<&'static str, String> {
    [
    ("car.ini","[HEADER]\nVERSION=1\n[BASIC]\nTOTALMASS=900\nINERTIA=1.5,1.2,3.5\nGRAPHICS_OFFSET=0,-.3,.1\nGRAPHICS_PITCH_ROTATION=.5\n[CONTROLS]\nSTEER_LOCK=360\nSTEER_RATIO=12\n"),
    ("suspensions.ini","[HEADER]\nVERSION=1\n[BASIC]\nWHEELBASE=3\nCG_LOCATION=.4\n[FRONT]\nTYPE=STRUT\nTRACK=1.4\nBASEY=-.2\n[REAR]\nTYPE=DWB\nTRACK=1.5\nBASEY=-.25\n"),
    ("tyres.ini","[HEADER]\nVERSION=10\n[COMPOUND_DEFAULT]\nINDEX=0\n[FRONT]\nNAME=Original front\nRADIUS=.3\nWIDTH=.2\n[REAR]\nNAME=Original rear\nRADIUS=.31\nWIDTH=.21\n[FRONT_1]\nNAME=Alternative front\nRADIUS=.32\nWIDTH=.22\n[REAR_1]\nNAME=Alternative rear\nRADIUS=.33\nWIDTH=.23\n"),
    ("engine.ini","[HEADER]\nVERSION=1\nPOWER_CURVE=power.lut\n[ENGINE_DATA]\nMINIMUM=800\nLIMITER=6000\nINERTIA=.15\n"),
    ("drivetrain.ini","[HEADER]\nVERSION=3\n[TRACTION]\nTYPE=RWD\n[GEARS]\nCOUNT=3\nGEAR_R=-3\nGEAR_1=3\nGEAR_2=2\nGEAR_3=1.2\nFINAL=4\n"),
    ("brakes.ini","[HEADER]\nVERSION=1\n[DATA]\nMAX_TORQUE=1200\nFRONT_SHARE=.6\nHANDBRAKE_TORQUE=700\n"),
    ("lods.ini","[LOD_0]\nFILE=original.kn5\n"),("power.lut","0|0\n1000|90\n4000|140\n6500|100\n"),
].into_iter().map(|(name,text)|(name,text.into())).collect()
}
