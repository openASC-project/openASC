"""Original car configuration and ACD fixtures, unrelated to any shipped car."""
from pathlib import Path
import struct

from openasc.acd import folder_key
from kn5_fixtures import file_bytes, mesh

FILES = {
    'car.ini': '[HEADER]\nVERSION=1\n[BASIC]\nTOTALMASS=900\nINERTIA=1.5,1.2,3.5\nGRAPHICS_OFFSET=0,-.3,.1\nGRAPHICS_PITCH_ROTATION=.5\n[CONTROLS]\nSTEER_LOCK=360\nSTEER_RATIO=12\n',
    'suspensions.ini': '[HEADER]\nVERSION=1\n[BASIC]\nWHEELBASE=3\nCG_LOCATION=.4\n[FRONT]\nTYPE=STRUT\nTRACK=1.4\nBASEY=-.2\n[REAR]\nTYPE=DWB\nTRACK=1.5\nBASEY=-.25\n',
    'tyres.ini': '[HEADER]\nVERSION=10\n[COMPOUND_DEFAULT]\nINDEX=0\n[FRONT]\nNAME=Original front\nRADIUS=.3\nWIDTH=.2\n[REAR]\nNAME=Original rear\nRADIUS=.31\nWIDTH=.21\n[FRONT_1]\nNAME=Alternative front\nRADIUS=.32\nWIDTH=.22\n[REAR_1]\nNAME=Alternative rear\nRADIUS=.33\nWIDTH=.23\n',
    'engine.ini': '[HEADER]\nVERSION=1\nPOWER_CURVE=power.lut\n[ENGINE_DATA]\nMINIMUM=800\nLIMITER=6000\nINERTIA=.15\n',
    'drivetrain.ini': '[HEADER]\nVERSION=3\n[TRACTION]\nTYPE=RWD\n[GEARS]\nCOUNT=3\nGEAR_R=-3\nGEAR_1=3\nGEAR_2=2\nGEAR_3=1.2\nFINAL=4\n',
    'brakes.ini': '[HEADER]\nVERSION=1\n[DATA]\nMAX_TORQUE=1200\nFRONT_SHARE=.6\nHANDBRAKE_TORQUE=700\n',
    'lods.ini': '[LOD_0]\nFILE=original.kn5\n',
    'power.lut': '0|0\n1000|90\n4000|140\n6500|100\n',
}


def suspension_source():
    """Original typed-record fixture; hardpoints are not a solved linkage."""
    text = '[HEADER]\nVERSION=1\n[BASIC]\nWHEELBASE=3\nCG_LOCATION=.4\n[ARB]\nFRONT=3000\nREAR=1500\n'
    for section, kind in [('FRONT', 'STRUT'), ('REAR', 'DWB')]:
        text += f'[{section}]\nTYPE={kind}\nTRACK=1.5\nBASEY=-.2\nROD_LENGTH=-.02\nHUB_MASS=30\n'
        for name in ['WBCAR_BOTTOM_FRONT', 'WBCAR_BOTTOM_REAR', 'WBTYRE_BOTTOM', 'WBCAR_STEER', 'WBTYRE_STEER']:
            text += f'{name}=0.3,-0.1,0.2\n'
        for name in (['STRUT_CAR', 'STRUT_TYRE'] if kind == 'STRUT' else ['WBCAR_TOP_FRONT', 'WBCAR_TOP_REAR', 'WBTYRE_TOP']):
            text += f'{name}=0.1,0.2,0\n'
        text += ('SPRING_RATE=0\nPROGRESSIVE_SPRING_RATE=100\nBUMP_STOP_RATE=50000\nBUMPSTOP_UP=.1\nBUMPSTOP_DN=.08\nPACKER_RANGE=.15\n'
                 'DAMP_BUMP=2000\nDAMP_FAST_BUMP=1000\nDAMP_FAST_BUMPTHRESHOLD=.12\nDAMP_REBOUND=3000\n'
                 'DAMP_FAST_REBOUND=1500\nDAMP_FAST_REBOUNDTHRESHOLD=.15\nTOE_OUT=-.001\nSTATIC_CAMBER=-2\n')
    return text


def archive_bytes(files, identifier='fixture', header=None):
    """Test-only encoder. Independent fixed key/record vectors also test the reader."""
    result = b'' if header is None else struct.pack('<ii', -1111, header)
    key = folder_key(identifier)
    for name, value in files:
        name = name.encode('ascii')
        value = value.encode('utf-8') if isinstance(value, str) else value
        result += struct.pack('<i', len(name)) + name + struct.pack('<i', len(value))
        result += b''.join(bytes(((c + key[i % len(key)]) & 255, 0, 0, 0)) for i, c in enumerate(value))
    return result


def install_car(root, identifier='fixture', packed=False, files=None):
    path = Path(root) / 'content/cars' / identifier
    path.mkdir(parents=True, exist_ok=True)
    files = FILES if files is None else files
    if packed:
        (path / 'data.acd').write_bytes(archive_bytes(files.items(), identifier))
    else:
        (path / 'data').mkdir(exist_ok=True)
        for name, text in files.items():
            (path / 'data' / name).write_text(text, encoding='utf-8')
    (path / 'original.kn5').write_bytes(file_bytes())
    points = tuple((x, y, z) for x in (-.7, .7) for y in (.2, 1.2) for z in (-1.7, 1.7))
    indices = (0, 1, 3, 0, 3, 2, 4, 6, 7, 4, 7, 5, 0, 4, 5, 0, 5, 1,
               2, 3, 7, 2, 7, 6, 0, 2, 6, 0, 6, 4, 1, 5, 7, 1, 7, 3)
    (path / 'collider.kn5').write_bytes(file_bytes(mesh(points=points, indices=indices)))
    return path
