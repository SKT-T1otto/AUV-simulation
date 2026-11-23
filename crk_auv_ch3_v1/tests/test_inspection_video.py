import struct
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from unittest.mock import Mock, patch

from crk_ch3 import inspection_video as video
from crk_ch3.search_world import build_search_world, fixture
from crk_ch3.vehicle import load_profile


class RecordingTests(unittest.TestCase):
    def test_camera_isolated_from_vehicles(self):
        original, _ = build_search_world(fixture('demo'), load_profile())
        world = ET.fromstring(video.camera_world(original)).find('world')
        cam = world.find("model[@name='inspection_observer']")
        self.assertEqual(cam.findtext('static'), 'true')
        self.assertFalse(cam.findall('.//collision'))
        sensor = cam.find("link/sensor[@type='camera']")
        self.assertEqual(sensor.findtext('plugin/service'), video.SERVICE)
        self.assertEqual(sensor.findtext('plugin/use_sim_time'), 'true')
        self.assertEqual(sensor.findtext('camera/image/width'), str(video.WIDTH))
        world.remove(cam)
        world.remove(world.find("plugin[@name='gz::sim::systems::Sensors']"))
        before = ET.fromstring(original).find('world')
        for tree in (world,before):
            ET.indent(tree)
        self.assertEqual(ET.tostring(world), ET.tostring(before))

    def test_service_requests_and_refusal(self):
        with tempfile.TemporaryDirectory() as folder, patch.object(video.subprocess,'run') as run:
            output = Path(folder)
            run.return_value = Mock(returncode=0,stdout='data: true\n',stderr='')
            video.request(output,start=True)
            cmd = run.call_args.args[0]
            self.assertIn(video.SERVICE,cmd)
            self.assertIn('start: true',cmd[-1])
            self.assertIn(video.PARTIAL,cmd[-1])
            video.request(output,start=False)
            self.assertEqual(run.call_args.args[0][-1],'stop: true')
            run.return_value.stdout = 'data: false'
            with self.assertRaises(RuntimeError):
                video.request(output,start=True)

    def test_mp4_index_and_partial_recording(self):
        # Synthetic box metadata only. No media frames are rendered or encoded.
        def box(kind, data):
            return struct.pack('>I4s', len(data)+8, kind) + data
        mdhd = bytes(12)+struct.pack('>II',1000,18800)
        sizes = box(b'stsz',bytes(8)+struct.pack('>I',470))
        media = box(b'mdhd',mdhd)+box(b'hdlr',bytes(8)+b'vide')+box(b'minf',box(b'stbl',sizes))
        track = box(b'tkhd',struct.pack('>II',1280<<16,720<<16))+box(b'mdia',media)
        content = box(b'ftyp',b'isom')+box(b'mdat',b'index-test')+box(b'moov',box(b'trak',track))
        file = Mock()
        file.read_bytes.return_value = content
        self.assertEqual(video.video_track(file),dict(frames=470,width=1280,height=720,duration_s=18.8))
        file.read_bytes.return_value = content[:-10]
        with self.assertRaises(ValueError):
            video.video_track(file)
        with patch.object(video,'video_track',return_value=dict(frames=1,width=1280,height=720,duration_s=.04)):
            with self.assertRaises(ValueError):
                video.verify_recording(Path('.'),18.8)
