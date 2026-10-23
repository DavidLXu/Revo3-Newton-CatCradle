"""Record Revotron overview and hand-detail views, without changing physics."""

import argparse, json, subprocess, threading, shutil, sys
from functools import partial
from http.server import ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlencode
from playwright.sync_api import sync_playwright
from render_preview import QuietHandler, ROOT


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--preview", action="store_true")
    p.add_argument(
        "--draft",
        action="store_true",
        help="Render a labelled draft and retain failed validation results",
    )
    p.add_argument(
        "--simulation-dir", type=Path, default=ROOT / "output/revotron_bimanual"
    )
    p.add_argument("--single-player", action="store_true")
    p.add_argument(
        "--software",
        action="store_true",
        help="Use SwiftShader instead of Metal on macOS",
    )
    a = p.parse_args()
    out = a.simulation_dir.resolve()
    data = json.loads((out / "trajectory.json").read_text())
    duration = data["frames"][-1]["time"]
    dest = out / "portrait"
    dest.mkdir(exist_ok=True)
    if not (out / "meshes.json").exists():
        sys.path.insert(0, str(ROOT))
        from revo3_cat_cradle.bimanual import RevoTron

        (out / "meshes.json").write_text(
            json.dumps(RevoTron().visuals(max_faces=8000), separators=(",", ":"))
        )
    if (
        not a.preview
        and not a.draft
        and not json.loads((out / "validation.json").read_text())["passed"]
    ):
        raise RuntimeError("Bimanual validation must pass before recording")
    server = ThreadingHTTPServer(
        ("127.0.0.1", 0), partial(QuietHandler, directory=str(ROOT))
    )
    threading.Thread(target=server.serve_forever, daemon=True).start()
    try:
        with sync_playwright() as p:
            options = dict(
                headless=True,
                args=(
                    ["--use-angle=metal"]
                    if sys.platform == "darwin" and not a.software
                    else [
                        "--use-gl=angle",
                        "--use-angle=swiftshader",
                        "--enable-unsafe-swiftshader",
                    ]
                ),
            )
            chrome = Path(
                "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome"
            )
            if chrome.exists():
                options["executable_path"] = str(chrome)
            browser = p.chromium.launch(**options)
            page = browser.new_page(viewport={"width": 1080, "height": 1920})
            errors = []
            page.on("pageerror", lambda e: errors.append(str(e)))
            query = urlencode(
                dict(
                    capture="1",
                    trajectory="../" + str(out.relative_to(ROOT) / "trajectory.json"),
                    meshes="../" + str(out.relative_to(ROOT) / "meshes.json"),
                    figure="single-player" if a.single_player else "crossed-loop",
                    draft="1" if a.draft else "0",
                )
            )
            page.goto(
                f"http://127.0.0.1:{server.server_port}/web/revotron.html?{query}",
                wait_until="networkidle",
            )
            page.wait_for_function("window.revotronReady?.()===true", timeout=120000)
            for name, t in [
                ("start", 0),
                ("crossed", min(8, duration)),
                ("pickup", min(13, duration)),
                ("final", duration),
            ]:
                page.evaluate("t=>window.renderRevotron(t)", t)
                page.screenshot(path=str(dest / (name + ".png")))
            if not a.preview:
                ffmpeg = shutil.which("ffmpeg")
                if not ffmpeg:
                    raise RuntimeError("ffmpeg is required")
                count = round(duration * 30) + 1
                video = dest / (
                    "revotron_opening_a_draft.mp4"
                    if a.draft
                    else "revotron_bimanual_cat_cradle.mp4"
                )
                enc = subprocess.Popen(
                    [
                        ffmpeg,
                        "-y",
                        "-v",
                        "error",
                        "-f",
                        "image2pipe",
                        "-vcodec",
                        "png",
                        "-framerate",
                        "30",
                        "-i",
                        "-",
                        "-c:v",
                        "libx264",
                        "-preset",
                        "slow",
                        "-crf",
                        "17",
                        "-pix_fmt",
                        "yuv420p",
                        "-movflags",
                        "+faststart",
                        str(video),
                    ],
                    stdin=subprocess.PIPE,
                )
                try:
                    for i in range(count):
                        page.evaluate("t=>window.renderRevotron(t)", i / 30)
                        enc.stdin.write(page.screenshot(type="png"))
                        if i % 30 == 0:
                            print(f"Rendered {i}/{count}", flush=True)
                    enc.stdin.close()
                    if enc.wait() != 0:
                        raise RuntimeError("Encoding failed")
                finally:
                    if enc.poll() is None:
                        enc.terminate()
                        enc.wait()
                (dest / "render.json").write_text(
                    json.dumps(
                        dict(
                            width=1080,
                            height=1920,
                            fps=30,
                            frames=count,
                            duration_s=count / 30,
                            physics_playback_speed=1,
                            draft=a.draft,
                            validation=json.loads(
                                (out / "validation.json").read_text()
                            ),
                            views=["whole robot", "hand detail"],
                        ),
                        indent=2,
                    )
                )
            browser.close()
            if errors:
                raise RuntimeError("\n".join(errors))
    finally:
        server.shutdown()


if __name__ == "__main__":
    main()
