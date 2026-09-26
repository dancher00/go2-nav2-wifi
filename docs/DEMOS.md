# Demo recordings

## 3D Point-LIO mapping — September 16, 2026

[![Robot and live 3D map](media/go2-3d-slam-demo.jpg)](media/go2-3d-slam-demo.mp4)

The external camera and RViz recording show the same session. The edit starts
at 01:06 in the external recording and 00:22 in the screen recording. The
approximately 44-second offset was checked against the start of walking and
subsequent turns. The previous 54-second offset derived from filenames was
incorrect: the robot moved while RViz was still starting. This edit begins
with a visible map and omits the initialization cover. The recordings are
visually aligned, not synchronized by hardware timestamps.
Both views play at 2× speed. After about 31 seconds, the edit switches to a
larger RViz view, followed by a short hold. The export is 37.2 seconds,
1920 × 720, H.264 at 30 fps.
Desktop chrome and terminal setup are cropped or omitted. No audio is included.

This is a mapping demonstration. Walking and a visible trajectory do not by
themselves demonstrate autonomous navigation, loop closure, or measured accuracy.

Rebuild locally with `bash demo3d/render-demo.sh /tmp/go2-demo` (FFmpeg required). The original
recordings are local files in `demo3d/`; they are not needed to watch the export.
The script refuses to overwrite an existing export.

The corrected export is versioned at `docs/media/go2-3d-slam-demo.mp4` and linked
from the README preview. The old GitHub attachment contains the obsolete edit.

## Earlier 2D demos

These demonstrate the previous 2D workflow and are retained for reference.

### Nav A → B

https://github.com/user-attachments/assets/44ae54a9-09f1-490c-ab3b-6291595e3324

### LiDAR + RViz

https://github.com/user-attachments/assets/2c817478-9fc5-4000-8211-b8b47e07eafb

### SLAM mapping

https://github.com/user-attachments/assets/16ffa9da-6469-4384-a56e-00d0343bb375
