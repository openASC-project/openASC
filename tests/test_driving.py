import http.client
import json
from pathlib import Path
import tempfile
from threading import Thread
import unittest

from kn5_fixtures import base, file_bytes, mesh
from vehicle_fixtures import install_car
from test_dynamics import plane
from openasc.config import ContentError
from openasc.driving import Session, load_drive
from openasc.dynamics import Dynamics
from openasc.kn5 import IDENTITY
from openasc.placement import spawn_transform
from openasc.vehicle import load_vehicle
from openasc.viewer import Preview, make_server


class DrivingTests(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.root = Path(temp.name)
        install_car(self.root, packed=True)
        vehicle = load_vehicle(self.root, 'fixture')
        pose = list(IDENTITY)
        pose[13] = 2
        self.clock = [0.]
        self.session = Session(Dynamics(vehicle, plane(), spawn_transform('AC_PIT_0', 'original.kn5', pose)), clock=lambda: self.clock[0])

    def command(self, action, sequence, **extra):
        return {'token': self.session.token, 'sequence': sequence, 'action': action, **extra}

    def test_clock_steps_are_independent_of_reads_and_expired_input_pauses(self):
        self.session.command(self.command('resume', 1))
        controls = dict(throttle=1, brake=0, steer=0, handbrake=0, gear=1)
        self.session.command(self.command('input', 2, controls=controls))
        self.session.advance(120)
        state = self.session.state()
        self.assertEqual(state['tick'], 120)
        self.assertGreater(state['speed_kmh'], 1)
        for _ in range(20):
            self.assertEqual(self.session.state()['tick'], 120)
        self.clock[0] = .6
        self.session.advance(1)
        state = self.session.state()
        self.assertTrue(state['paused'])
        self.assertIn('heartbeat', state['reason'])
        self.assertEqual(state['controls']['throttle'], 0)
        self.assertEqual(state['tick'], 120)

    def test_invalid_commands_are_atomic_and_replays_are_rejected(self):
        self.session.command(self.command('resume', 1))
        before = self.session.state()
        cases = [(self.command('resume', 1), 'drive_stale'),
                 (self.command('reset', 2) | {'token': 'неверно'}, 'drive_token'),
                 (self.command('input', 2, controls={'throttle': 1}), 'drive_input'),
                 (self.command('reset', True), 'drive_sequence'),
                 (self.command('reset', 2) | {'extra': 1}, 'drive_command')]
        for payload, code in cases:
            with self.subTest(code=code):
                with self.assertRaises(ContentError) as error:
                    self.session.command(payload)
                self.assertEqual(error.exception.diagnostic.code, code)
                self.assertEqual(self.session.state(), before)
        state = self.session.command(self.command('reset', 2))
        self.assertEqual(state['tick'], 0)
        self.assertTrue(state['paused'])
        self.assertEqual(state['sequence'], 2)

    def test_local_protocol_requires_origin_nonce_bounded_schema_and_fresh_sequence(self):
        server = make_server(Preview({'scope': 'original_fixture'}, b''), 0, self.session)
        worker = Thread(target=server.serve_forever, daemon=True)
        worker.start()
        try:
            def request(method='GET', path='/api/drive', body=None, headers=None):
                client = http.client.HTTPConnection('127.0.0.1', server.server_port, timeout=5)
                try:
                    client.request(method, path, body=body, headers=headers or {})
                    response = client.getresponse()
                    return response.status, dict(response.getheaders()), response.read()
                finally:
                    client.close()
            status, headers, body = request()
            self.assertEqual(status, 200)
            self.assertEqual(json.loads(body)['token'], self.session.token)
            self.assertNotIn('Access-Control-Allow-Origin', headers)
            local = {'Content-Type': 'application/json', 'Origin': f'http://127.0.0.1:{server.server_port}'}
            command = json.dumps(self.command('resume', 1))
            self.assertEqual(request('POST', body=command, headers=local)[0], 200)
            self.assertEqual(request('POST', body=command, headers=local)[0], 409)
            self.assertEqual(request('POST', body=command, headers={'Content-Type': 'application/json'})[0], 403)
            self.assertEqual(request('POST', body=command, headers=local | {'Origin': 'https://example.com'})[0], 403)
            self.assertEqual(request('POST', body=command, headers=local | {'Host': 'example.com'})[0], 403)
            self.assertEqual(request('POST', body=json.dumps(self.command('pause', 2) | {'token': 'bad'}), headers=local)[0], 403)
            self.assertEqual(request('POST', body='[' * 1100, headers=local)[0], 400)
            self.assertEqual(request('POST', body='x' * 2049, headers=local)[0], 400)
            self.assertEqual(request('POST', body='{bad', headers=local)[0], 400)
            for controls in (None, [{}], dict(throttle=10 ** 1000, brake=0, steer=0, handbrake=0, gear=1)):
                self.assertEqual(request('POST', body=json.dumps(self.command('input', 2, controls=controls)), headers=local)[0], 400)
            self.assertEqual(request('POST', path='/not-a-command', body=command, headers=local)[0], 404)
            self.assertEqual(request('GET', path='/drive.js')[0], 200)
        finally:
            server.shutdown()
            server.server_close()
            worker.join(timeout=5)

    def test_original_fixture_assembly_has_car_mesh_offsets_and_no_asset_exports(self):
        track = self.root / 'content/tracks/fixture'
        (track / 'data').mkdir(parents=True)
        (track / 'data/surfaces.ini').write_text('[SURFACE_0]\nKEY=ROAD\nFRICTION=.9\nDAMPING=0\nIS_VALID_TRACK=1\nIS_PITLANE=0')
        (track / 'models.ini').write_text('[MODEL_0]\nFILE=original.kn5\nPOSITION=0,0,0\nROTATION=0,0,0')
        pose = list(IDENTITY)
        pose[13] = 2
        road = mesh(name='01ROAD', points=((-20, 0, -20), (20, 0, -20), (-20, 0, 20), (20, 0, 20)), indices=(0, 2, 1, 1, 2, 3))
        (track / 'original.kn5').write_bytes(file_bytes(base([road, base([mesh(name='AC_PIT_0')], name='AC_PIT_0', matrix=pose)])))
        before = set(self.root.rglob('*'))
        preview, session = load_drive(self.root, 'fixture', 'fixture')
        self.assertEqual(preview.metadata['scope'], 'first_drive_prototype')
        self.assertEqual(preview.metadata['drive']['car_meshes'], 1)
        self.assertNotIn('AC_PIT_0', [m['name'] for m in preview.metadata['meshes']])
        self.assertIn('drive_markers_hidden', [d['code'] for d in preview.metadata['diagnostics']])
        self.assertEqual(len(preview.geometry), preview.metadata['geometry_bytes'])
        car = [m for m in preview.metadata['meshes'] if m.get('dynamic') == 'car'][0]
        self.assertEqual(car['vertex_offset'] % 4, 0)
        self.assertGreater(car['material'], 0)
        self.assertTrue(session.state()['paused'])
        self.assertNotIn(str(self.root), json.dumps(preview.metadata))
        self.assertEqual(set(self.root.rglob('*')), before)
