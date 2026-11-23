"""Command-line control of Gazebo Harmonic's server-side CameraVideoRecorder."""
import json
import os
import shutil
import struct
import subprocess
import time
import xml.etree.ElementTree as ET

from .motion_world import plugin
from .preview import child

WIDTH, HEIGHT, FPS = 1280, 720, 25
SERVICE = '/inspection_demo/camera/record_video'
PARTIAL = 'inspection.partial.mp4'


def camera_world(xml):
    sdf = ET.fromstring(xml)
    world = sdf.find('world')
    child(plugin(world, 'sensors', 'Sensors'), 'render_engine', 'ogre2')
    model = child(world, 'model', name='inspection_observer')
    child(model, 'static', 'true')
    child(model, 'pose', '0 -16 4 0 0.66 2.05')
    sensor = child(child(model, 'link', name='mount'), 'sensor', name='survey_camera', type='camera')
    child(sensor, 'always_on', 'true')
    child(sensor, 'update_rate', FPS)
    child(sensor, 'topic', '/inspection_demo/camera/image')
    camera = child(sensor, 'camera')
    child(camera, 'horizontal_fov', '1.05')
    image = child(camera, 'image')
    child(image, 'width', WIDTH)
    child(image, 'height', HEIGHT)
    child(image, 'format', 'R8G8B8')
    clip = child(camera, 'clip')
    child(clip, 'near', '.1')
    child(clip, 'far', '100')
    recorder = plugin(sensor, 'camera-video-recorder', 'CameraVideoRecorder')
    child(recorder, 'service', SERVICE)
    child(recorder, 'use_sim_time', 'true')
    child(recorder, 'fps', FPS)
    child(recorder, 'bitrate', 4000000)
    ET.indent(sdf)
    return ET.tostring(sdf, encoding='unicode')


def request(output, *, start):
    # Called by the ROS mission process inside this run's GZ_PARTITION.
    payload = ('start: true, format: "mp4", save_filename: ' +
               json.dumps(str(output / PARTIAL))) if start else 'stop: true'
    command = ['gz', 'service', '-s', SERVICE, '--reqtype', 'gz.msgs.VideoRecord',
               '--reptype', 'gz.msgs.Boolean', '--timeout', '15000', '--req', payload]
    with (output / 'video_recorder.log').open('a', encoding='utf-8') as log:
        print('start' if start else 'stop', file=log, flush=True)
        reply = subprocess.run(command, capture_output=True, text=True, timeout=20)
        print(reply.stdout, reply.stderr, file=log, flush=True)
    if reply.returncode or 'data: true' not in reply.stdout:
        raise RuntimeError('Gazebo camera did not acknowledge the recording request; see video_recorder.log')


def boxes(data):
    offset = 0
    while offset < len(data):
        size, kind = struct.unpack_from('>I4s', data, offset)
        header = 8
        if size == 1:
            size, = struct.unpack_from('>Q', data, offset + 8)
            header = 16
        if size == 0:
            size = len(data) - offset
        if size < header or offset + size > len(data):
            raise ValueError('Truncated MP4 box')
        yield kind, data[offset+header:offset+size]
        offset += size


def video_track(path):
    # Read the MP4 index even on ROS images which only ship the libav libraries.
    top = dict(boxes(memoryview(path.read_bytes())))
    if not top.get(b'mdat') or b'ftyp' not in top:
        raise ValueError('Missing MP4 media data')
    for kind, track in boxes(top[b'moov']):
        if kind != b'trak':
            continue
        parts = dict(boxes(track))
        media = dict(boxes(parts[b'mdia']))
        if bytes(media[b'hdlr'][8:12]) != b'vide':
            continue
        header = media[b'mdhd']
        if header[0] == 1:
            scale, duration = struct.unpack_from('>IQ', header, 20)
        else:
            scale, duration = struct.unpack_from('>II', header, 12)
        table = dict(boxes(dict(boxes(media[b'minf']))[b'stbl']))
        count, = struct.unpack_from('>I', table[b'stsz'], 8)
        width, height = struct.unpack('>II', parts[b'tkhd'][-8:])
        return dict(frames=count, width=width >> 16, height=height >> 16,
                    duration_s=duration / scale)
    raise ValueError('No MP4 video track')


def verify_recording(output, task_duration):
    path = output / PARTIAL
    track = video_track(path)
    tolerance = max(.4, task_duration * .05)
    if (task_duration <= 0 or (track['width'], track['height']) != (WIDTH, HEIGHT)
            or abs(track['duration_s'] - task_duration) > tolerance
            or abs(track['frames']/FPS - task_duration) > tolerance):
        raise ValueError(f'Camera movie does not cover the task: {track}; task duration {task_duration}')
    track['full_decode_checked'] = False
    ffmpeg = shutil.which('ffmpeg')
    if ffmpeg:
        subprocess.run([ffmpeg, '-v', 'error', '-xerror', '-nostdin', '-i', str(path),
                        '-map', '0:v:0', '-f', 'null', '-'], timeout=90, check=True)
        track['full_decode_checked'] = True
    path.rename(output / 'inspection.mp4')
    return track


def stop_recording(output):
    request(output, start=False)
    # The service acknowledges the request before the rendering thread flushes and
    # moves the temporary MP4. Physics stays paused during this wait.
    until = time.monotonic() + 8
    while not (output / PARTIAL).is_file():
        if time.monotonic() >= until:
            # Some paused renderers finish the encoder during orderly server exit.
            # Keep the native encoder's inode before its destructor removes the
            # temporary name. The supervisor checks the index only after teardown.
            pending = [p for p in output.glob('*.mp4') if p.stem.isdecimal() and p.is_file()]
            if len(pending) == 1:
                try:
                    os.link(pending[0], output / PARTIAL)
                except (FileNotFoundError, FileExistsError):
                    if not (output / PARTIAL).is_file():
                        raise
                return 'native_server_shutdown'
            raise TimeoutError('Gazebo acknowledged stop but has not finalized the MP4')
        time.sleep(.1)
    return 'native_recorder'
