"""Original car configuration and ACD fixtures, unrelated to any shipped car."""
from pathlib import Path
import struct

from openasc.acd import folder_key
from kn5_fixtures import file_bytes

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
    return path
