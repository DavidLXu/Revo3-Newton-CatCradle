"""Render the actual Newton trajectory in Chrome; encode at source duration."""
from functools import partial
from http.server import ThreadingHTTPServer, SimpleHTTPRequestHandler
from pathlib import Path
import argparse
import json
import shutil
import subprocess
import threading
from playwright.sync_api import sync_playwright

ROOT = Path(__file__).resolve().parents[1]


class QuietHandler(SimpleHTTPRequestHandler):
    def log_message(self, *_):
        pass


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--video", action="store_true")
    parser.add_argument("--front", action="store_true")
    parser.add_argument("--chrome", default=None)
    args = parser.parse_args()
    out = ROOT/"output"
    handler = partial(QuietHandler, directory=str(ROOT))
    server = ThreadingHTTPServer(("127.0.0.1", 0), handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    try:
        with sync_playwright() as p:
            chrome = args.chrome or "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome"
            options = dict(headless=True, args=["--enable-webgl", "--use-gl=angle", "--use-angle=swiftshader", "--enable-unsafe-swiftshader"])
            if Path(chrome).exists():
                options["executable_path"] = chrome
            browser = p.chromium.launch(**options)
            page = browser.new_page(viewport={"width":1280,"height":900}, device_scale_factor=1)
            errors=[]
            page.on("pageerror", lambda e: errors.append(str(e)))
            page.goto(f"http://127.0.0.1:{server.server_port}/web/?capture=1",wait_until="networkidle")
            page.wait_for_function("window.viewerReady === true", timeout=60000)
            if args.front:
                page.evaluate("window.setFront()")
            info = page.evaluate("window.revo3")
            assert page.locator("#seek").input_value() == "0"
            assert page.locator("#play").inner_text() == "播放"
            for name, frame in (("preview",0),("pose_start",round(info['fps']*.5)),("pose_1",info['fps']),
                                ("pose_2",2*info['fps']),("pose_3",3*info['fps']),
                                ("little_entry",round(4.5*info['fps'])),("little_inserted",5*info['fps']),
                                ("pose_final",info['frameCount']-1)):
                page.evaluate("f=>window.setFrame(f)",min(frame,info["frameCount"]-1))
                page.screenshot(path=str(out/f"{name}.png"))
            page.evaluate("window.setFrame(0); window.setSide()")
            page.screenshot(path=str(out/"initial_side.png"))
            page.evaluate("window.setFront()")
            if not args.front:
                page.locator('#camera').click()
            # Exercise scrubbing and camera controls; no anchor overlay remains.
            assert info['anchors'] == 0
            assert info['interaction'] == 'contact_friction_only'
            assert page.locator('#pins').count() == 0
            page.locator('#camera').click()
            page.locator('#seek').fill(str(min(round(info['fps']*.5),info['frameCount']-1)))
            assert '0.50' in page.locator('#time').inner_text() or info['frameCount'] < 16
            trajectory=json.loads((out/'trajectory.json').read_text())
            if trajectory['config'].get('adaptive_length'):
                index=min(round(info['fps']*.5),info['frameCount']-1)
                length=trajectory['frames'][index]['rope_rest_length_m']*100
                caption=page.locator('#source').inner_text()
                assert '自动收放绳' in caption and f'{length:.1f} cm' in caption
            page.locator('#camera').click()
            if args.video:
                if not shutil.which("ffmpeg"):
                    raise RuntimeError("ffmpeg is needed for MP4 export")
                folder = out/"frames"
                folder.mkdir(exist_ok=True)
                # The final endpoint is retained in JSON, not encoded twice.
                for i in range(info["frameCount"]-1):
                    page.evaluate("f=>window.setFrame(f)",i)
                    page.screenshot(path=str(folder/f"{i:04d}.png"))
                    if i%30 == 0:
                        print(f"Rendered {i}/{info['frameCount']-1}", flush=True)
                fps=json.loads((out/"trajectory.json").read_text())["config"]["fps"]
                subprocess.run(["ffmpeg","-y","-hide_banner","-loglevel","error","-framerate",str(fps),
                    "-i",str(folder/"%04d.png"),"-frames:v",str(info["frameCount"]-1),
                    "-c:v","libx264","-crf","18","-pix_fmt","yuv420p","-movflags","+faststart",str(out/"revo3_cat_cradle.mp4")],check=True)
            browser.close()
            if errors:
                raise RuntimeError("Browser errors: " + "\n".join(errors))
            print(json.dumps({"browser_validation":"passed", **info}))
    finally:
        server.shutdown()


if __name__ == "__main__":
    main()
