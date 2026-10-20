"""Render a 30-degree studio orbit (−15° to +15°) of the exported Newton trajectory.

Run --preview to inspect the framing before encoding the full 1080p film.
Only visual geometry and lighting change; physics is never recomputed here.
"""
import argparse
from functools import partial
from http.server import ThreadingHTTPServer
import json
import math
from pathlib import Path
import shutil
import subprocess
import sys
import threading
from urllib.parse import urlencode

from playwright.sync_api import sync_playwright
from render_preview import QuietHandler, ROOT


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--preview', action='store_true')
    parser.add_argument('--simulation-dir', type=Path, default=ROOT/'output')
    parser.add_argument('--playback-speed', type=float, default=.75)
    args = parser.parse_args()
    if not math.isfinite(args.playback_speed) or args.playback_speed <= 0:
        parser.error('playback-speed must be finite and positive')
    out = ROOT / 'output'
    simulation_dir = args.simulation_dir.resolve()
    relative = simulation_dir.relative_to(ROOT)
    destination = simulation_dir / 'cinematic_30deg'
    destination.mkdir(exist_ok=True)
    if not (out / 'meshes_cinematic.json').exists():
        sys.path.insert(0, str(ROOT))
        from revo3_cat_cradle.hand import Hand
        print('Exporting detailed visual meshes...', flush=True)
        meshes = Hand().visuals(max_faces=12000)
        (out / 'meshes_cinematic.json').write_text(json.dumps(meshes, separators=(',', ':')))
    data = json.loads((simulation_dir / 'trajectory.json').read_text())
    if data.get('validation', {}).get('deformation_bounds_passed') is False:
        raise RuntimeError('Simulation failed deformation checks; inspect its metrics before rendering')
    source_duration = data['frames'][-1]['time']
    motion_speed = data['config'].get('motion_speed', 1.)
    fps, speed, hold = 30, args.playback_speed, 1.0
    duration = source_duration / speed + 2 * hold
    count = math.ceil(duration * fps - 1e-4)
    server = ThreadingHTTPServer(('127.0.0.1', 0), partial(QuietHandler, directory=str(ROOT)))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    try:
        with sync_playwright() as playwright:
            options = dict(headless=True, args=['--enable-webgl', '--use-gl=angle',
                           '--use-angle=swiftshader', '--enable-unsafe-swiftshader'])
            chrome = Path('/Applications/Google Chrome.app/Contents/MacOS/Google Chrome')
            if chrome.exists():
                options['executable_path'] = str(chrome)
            browser = playwright.chromium.launch(**options)
            page = browser.new_page(viewport={'width':1920, 'height':1080}, device_scale_factor=1)
            errors = []
            page.on('pageerror', lambda error: errors.append(str(error)))
            query = urlencode(dict(capture=1, cinematic=1,
                              trajectory=f'../{relative.as_posix()}/trajectory.json', playback=speed))
            page.goto(f'http://127.0.0.1:{server.server_port}/web/?{query}', wait_until='networkidle')
            page.wait_for_function('window.viewerReady === true', timeout=120000)
            info = page.evaluate('window.revo3')
            assert info['anchors'] == 0 and info['interaction'] == 'contact_friction_only'

            def render(index):
                time = max(0, min(source_duration, (index / fps - hold) * speed))
                page.evaluate('([time,progress])=>window.renderCinematic(time,progress)', [time,index/(count-1)])

            for name, index in [('start',0), ('pickup',round((4.8/motion_speed/speed+hold)*fps)),
                                ('middle',count//2), ('end',count-1)]:
                render(index)
                page.screenshot(path=str(destination / f'{name}.png'))
            if not args.preview:
                ffmpeg = shutil.which('ffmpeg')
                if not ffmpeg:
                    raise RuntimeError('Install ffmpeg to encode MP4')
                video = destination / 'revo3_cat_cradle_30deg.mp4'
                command = [ffmpeg,'-y','-hide_banner','-loglevel','error','-f','image2pipe',
                           '-vcodec','png','-framerate',str(fps),'-i','-',
                           '-c:v','libx264','-preset','slow','-crf','17','-pix_fmt','yuv420p',
                           '-movflags','+faststart',str(video)]
                process = subprocess.Popen(command, stdin=subprocess.PIPE)
                try:
                    for index in range(count):
                        render(index)
                        process.stdin.write(page.screenshot(type='png'))
                        if index % 30 == 0:
                            print(f'Rendered {index}/{count}', flush=True)
                    process.stdin.close()
                    if process.wait() != 0:
                        raise RuntimeError('ffmpeg encoding failed')
                finally:
                    if process.poll() is None:
                        process.terminate()
                        process.wait()
                manifest = {**info, 'video':video.name, 'width':1920, 'height':1080,
                            'fps':fps, 'frames':count, 'duration_s':count/fps,
                            'source_duration_s':source_duration, 'playback_speed':speed,
                            'motion_speed':motion_speed, 'simulation_dir':relative.as_posix(),
                            'orbit_degrees':30, 'start_azimuth_degrees':-15, 'end_azimuth_degrees':15}
                (destination/'render.json').write_text(json.dumps(manifest, indent=2))
                print(json.dumps(manifest), flush=True)
            browser.close()
            if errors:
                raise RuntimeError('\n'.join(errors))
    finally:
        server.shutdown()


if __name__ == '__main__':
    main()
