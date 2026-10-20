"""Record two synchronized fixed views in a 1080 x 1920 portrait video."""
import argparse
from functools import partial
from http.server import ThreadingHTTPServer
import json
import io
from pathlib import Path
import shutil
import subprocess
import threading
from urllib.parse import urlencode

from playwright.sync_api import sync_playwright
from PIL import Image
from render_preview import QuietHandler, ROOT


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--simulation-dir',type=Path,default=ROOT/'output/speed_1_5')
    parser.add_argument('--preview',action='store_true')
    parser.add_argument('--output-dir',type=Path,default=None)
    parser.add_argument('--loop',action='store_true',help='No cover; blend the last quiet 0.6 s into the settled first frame')
    args=parser.parse_args()
    simulation=args.simulation_dir.resolve()
    relative=simulation.relative_to(ROOT)
    data=json.loads((simulation/'trajectory.json').read_text())
    if not data['validation']['deformation_bounds_passed']:
        raise RuntimeError('Simulation failed deformation checks')
    if not (ROOT/'output/meshes_cinematic.json').exists():
        raise RuntimeError('Run render_cinematic.py --preview to export detailed meshes first')
    out=args.output_dir.resolve() if args.output_dir else simulation/'portrait'
    out.mkdir(parents=True,exist_ok=True)
    duration=data['frames'][-1]['time']
    fps=30
    cover_frames,opening_frames,closing_frames=24,30,30
    count=cover_frames+opening_frames+round(duration*fps)+closing_frames
    if args.loop:
        validation=json.loads((simulation/'cycle_validation.json').read_text())
        if validation['seam_p95_m']>=.005 or validation['end_crossings']!=0:
            raise RuntimeError('Physical loop endpoints are not close enough for the quiet-pose dissolve')
        cover_frames=opening_frames=closing_frames=0
        count=round(duration*fps)+1
    server=ThreadingHTTPServer(('127.0.0.1',0),partial(QuietHandler,directory=str(ROOT)))
    threading.Thread(target=server.serve_forever,daemon=True).start()
    try:
        with sync_playwright() as p:
            options=dict(headless=True,args=['--enable-webgl','--use-gl=angle',
                         '--use-angle=swiftshader','--enable-unsafe-swiftshader'])
            chrome=Path('/Applications/Google Chrome.app/Contents/MacOS/Google Chrome')
            if chrome.exists():options['executable_path']=str(chrome)
            browser=p.chromium.launch(**options)
            page=browser.new_page(viewport={'width':1080,'height':1920},device_scale_factor=1)
            errors=[]
            page.on('pageerror',lambda error:errors.append(str(error)))
            query=urlencode(dict(trajectory=f'../{relative.as_posix()}/trajectory.json'))
            page.goto(f'http://127.0.0.1:{server.server_port}/web/stacked.html?{query}',wait_until='networkidle')
            page.wait_for_function('window.stackedReady?.()===true',timeout=120000)
            info=page.evaluate('window.stackedInfo()')
            assert info['source']['anchors']==0
            cover_time=data['motion'].get('forward_end_seconds',duration) if args.loop else duration
            pickup_time=data['motion'].get('opening_hold_seconds',0)+4.8/data['config'].get('motion_speed',1)
            for name,time in [('cover',cover_time),('start',0),('pickup',pickup_time)]:
                page.evaluate('time=>window.renderStacked(time)',time)
                page.screenshot(path=str(out/f'{name}.png'))
            if not args.preview:
                ffmpeg=shutil.which('ffmpeg')
                if not ffmpeg:raise RuntimeError('ffmpeg is required')
                video=out/('revo3_cycle_portrait.mp4' if args.loop else 'revo3_dual_view_portrait.mp4')
                page.evaluate('time=>window.renderStacked(time)',0)
                first_image=Image.open(io.BytesIO(page.screenshot(type='png'))).convert('RGB')
                command=[ffmpeg,'-y','-hide_banner','-loglevel','error','-f','image2pipe',
                         '-vcodec','png','-framerate',str(fps),'-i','-',
                         '-c:v','libx264','-preset','slow','-crf','17','-pix_fmt','yuv420p',
                         '-movflags','+faststart',str(video)]
                encoder=subprocess.Popen(command,stdin=subprocess.PIPE)
                try:
                    for i in range(count):
                        time=duration if i<cover_frames else max(0,min(duration,(i-cover_frames-opening_frames)/fps))
                        page.evaluate('time=>window.renderStacked(time)',time)
                        png=page.screenshot(type='png')
                        if args.loop and i>=count-19:
                            # Editorial seam treatment only. The simulated rope
                            # states remain untouched in the trajectory export.
                            u=(i-(count-19))/18
                            u=u*u*(3-2*u)
                            blended=Image.blend(Image.open(io.BytesIO(png)).convert('RGB'),first_image,u)
                            buffer=io.BytesIO();blended.save(buffer,format='PNG');png=buffer.getvalue()
                        encoder.stdin.write(png)
                        if i%30==0:print(f'Rendered {i}/{count}',flush=True)
                    encoder.stdin.close()
                    if encoder.wait()!=0:raise RuntimeError('Video encoding failed')
                finally:
                    if encoder.poll() is None:encoder.terminate();encoder.wait()
                metadata=dict(width=1080,height=1920,fps=fps,frames=count,duration_s=count/fps,
                              simulation_seconds=duration,motion_speed=data['config'].get('motion_speed',1),
                              playback_speed=1,cover_seconds=cover_frames/fps,fixed_azimuths=[0,-45],
                              synchronized=True,simulation_dir=str(relative))
                if args.loop:
                    metadata.update(loop=True,preroll_seconds=data['motion']['preroll_seconds'],
                        seam_dissolve_seconds=.6,seam='Quiet final pose dissolves into settled first frame; physics states are not modified')
                (out/'render.json').write_text(json.dumps(metadata,indent=2))
                print(json.dumps(metadata),flush=True)
            browser.close()
            if errors:raise RuntimeError('\n'.join(errors))
    finally:
        server.shutdown()


if __name__=='__main__':main()
