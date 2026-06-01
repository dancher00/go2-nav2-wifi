// Stream JPEG frames from Go2 VideoClient to stdout: [4-byte BE size][jpeg bytes]...
// Build: bash ~/robot-build-camera-cli.sh
// Used by robot_front_camera_bridge.py (no unitree_sdk2_python required).

#include <unitree/robot/go2/video/video_client.hpp>
#include <cstdint>
#include <iostream>
#include <unistd.h>
#include <vector>

static void write_u32_be(uint32_t n) {
  unsigned char b[4] = {
      static_cast<unsigned char>((n >> 24) & 0xff),
      static_cast<unsigned char>((n >> 16) & 0xff),
      static_cast<unsigned char>((n >> 8) & 0xff),
      static_cast<unsigned char>(n & 0xff),
  };
  std::cout.write(reinterpret_cast<char *>(b), 4);
}

int main() {
  std::ios::sync_with_stdio(false);
  std::cin.tie(nullptr);

  unitree::robot::ChannelFactory::Instance()->Init(0);
  unitree::robot::go2::VideoClient client;
  client.SetTimeout(1.0f);
  client.Init();

  std::vector<uint8_t> jpeg;
  while (true) {
    jpeg.clear();
    const int ret = client.GetImageSample(jpeg);
    if (ret != 0 || jpeg.empty()) {
      continue;
    }
    const uint32_t len = static_cast<uint32_t>(jpeg.size());
    write_u32_be(len);
    std::cout.write(reinterpret_cast<const char *>(jpeg.data()), jpeg.size());
    std::cout.flush();
    usleep(50000);  // ~20 fps cap — bridge throttles via GO2_CAM_FPS (default 10)
  }
  return 0;
}
