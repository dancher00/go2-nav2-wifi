#!/usr/bin/env bash
# Rebuild the README demo from the two original recordings. Requires ffmpeg.
set -euo pipefail
cd "$(dirname "$0")/.."
# Match visible motion, not recording filenames: camera 66 s = RViz 22 s.
# At this point the first map is visible and the robot is about to start walking.
# Keep a single offset throughout; both views use the same 2x playback speed.
output_dir="${1:-docs/media}"
[[ $# -le 1 ]] || { echo 'Usage: bash demo3d/render-demo.sh [output-directory]' >&2; exit 2; }
mkdir -p "$output_dir"
for name in go2-3d-slam-demo.mp4 go2-3d-slam-demo.jpg; do
  [[ ! -e "$output_dir/$name" ]] || { echo "Export already exists: $output_dir/$name" >&2; exit 1; }
done
ffmpeg -hide_banner -n \
  -ss 66 -t 62 -i demo3d/jabra-recording-2026-09-16-223243.mkv \
  -ss 22 -t 71 -i 'demo3d/Screencast from 2026-09-16 22-33-37.mp4' \
  -filter_complex "
    [0:v]setpts=(PTS-STARTPTS)/2,fps=30,scale=924:520,
      pad=948:520:0:0:color=0x101820,setsar=1[robot];
    [1:v]split=2[screen][ending];
    [screen]trim=duration=62,setpts=(PTS-STARTPTS)/2,fps=30,
      crop=1792:896:88:140,scale=924:462,pad=924:520:0:29:color=0x222627,setsar=1[rviz];
    [robot][rviz]hstack=inputs=2,pad=1920:720:24:116:color=0x101820,
      drawtext=text='GO2 / 3D LiDAR MAPPING':x=24:y=22:fontsize=34:fontcolor=white,
      drawtext=text='ROBOT':x=24:y=83:fontsize=20:fontcolor=0x8fcddd,
      drawtext=text='LIVE MAP / RViz':x=972:y=83:fontsize=20:fontcolor=0x8fcddd,
      drawtext=text='Point-LIO  |  Onboard LiDAR + IMU  |  Wi-Fi':x=24:y=670:fontsize=23:fontcolor=white,
      drawtext=text='2x speed':x=w-tw-24:y=670:fontsize=23:fontcolor=0x8fcddd[main];
    [ending]trim=start=62:duration=9,setpts=(PTS-STARTPTS)/2,fps=30,
      crop=1792:896:88:140,scale=1160:580,setsar=1,
      pad=1920:720:380:92:color=0x101820,
      drawtext=text='GO2 / RECONSTRUCTED 3D MAP':x=24:y=22:fontsize=34:fontcolor=white,
      drawtext=text='Point-LIO  |  Map + estimated trajectory':x=24:y=682:fontsize=22:fontcolor=white,
      drawtext=text='2x speed':x=w-tw-24:y=682:fontsize=22:fontcolor=0x8fcddd,
      tpad=stop_mode=clone:stop_duration=1.5[end];
    [main][end]concat=n=2:v=1:a=0,format=yuv420p[out]
  " \
  -map '[out]' -an -c:v libx264 -preset medium -crf 23 \
  -maxrate 1700k -bufsize 3400k -movflags +faststart \
  "$output_dir/go2-3d-slam-demo.mp4"
ffmpeg -hide_banner -n -ss 26 -i "$output_dir/go2-3d-slam-demo.mp4" \
  -frames:v 1 -update 1 "$output_dir/go2-3d-slam-demo.jpg"
