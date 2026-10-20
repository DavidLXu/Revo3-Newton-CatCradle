"""Trim two seconds of relaxation and add a one-second star-frame hold.

This is an editorial change to the recorded movie, not a new physics run.
The existing 0.6 s loop dissolve is rebuilt at the shortened endpoint.
"""
import argparse
import json
from pathlib import Path
import subprocess

from PIL import Image


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source',type=Path)
    parser.add_argument('destination',type=Path)
    args=parser.parse_args()
    info=json.loads(subprocess.check_output(['ffprobe','-v','error','-show_streams','-of','json',str(args.source)]))['streams'][0]
    width,height=int(info['width']),int(info['height'])
    if info['avg_frame_rate']!='30/1' or int(info['nb_frames'])!=576:
        raise ValueError('Expected the 576-frame, 30 fps continuous cycle recording')
    fps=30;keep_frames=516;star_frame=245;hold_frames=30
    args.destination.parent.mkdir(parents=True,exist_ok=True)
    decoder=subprocess.Popen(['ffmpeg','-v','error','-i',str(args.source),'-f','rawvideo','-pix_fmt','rgb24','-'],stdout=subprocess.PIPE)
    encoder=subprocess.Popen(['ffmpeg','-y','-v','error','-f','rawvideo','-pix_fmt','rgb24',
        '-s',f'{width}x{height}','-r',str(fps),'-i','-','-an','-c:v','libx264',
        '-preset','slow','-crf','17','-pix_fmt','yuv420p','-movflags','+faststart',str(args.destination)],stdin=subprocess.PIPE)
    first=None
    try:
        for i in range(576):
            raw=decoder.stdout.read(width*height*3)
            if len(raw)!=width*height*3:
                raise RuntimeError(f'Incomplete source frame {i}')
            if i==0:first=Image.frombytes('RGB',(width,height),raw)
            if i>=keep_frames:continue
            if i>=keep_frames-19:
                u=(i-(keep_frames-19))/18
                u=u*u*(3-2*u)
                raw=Image.blend(Image.frombytes('RGB',(width,height),raw),first,u).tobytes()
            encoder.stdin.write(raw)
            if i==star_frame:
                for _ in range(hold_frames):encoder.stdin.write(raw)
        decoder.stdout.close();encoder.stdin.close()
        if decoder.wait()!=0 or encoder.wait()!=0:
            raise RuntimeError('Video decoding or encoding failed')
    finally:
        for process in (decoder,encoder):
            if process.poll() is None:process.terminate();process.wait()
    report=dict(source=str(args.source.resolve()),frames=keep_frames+hold_frames,fps=fps,
        duration_s=(keep_frames+hold_frames)/fps,trimmed_tail_seconds=2,
        added_star_hold_seconds=1,star_source_time_s=star_frame/fps,
        seam_dissolve_seconds=.6,physics_changed=False)
    args.destination.with_suffix('.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
