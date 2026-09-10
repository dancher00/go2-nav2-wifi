"""Small live camera image for RViz; JPEG decoding and resizing run on Jetson."""
import time
from pathlib import Path
import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage,Image

rclpy.init(args=[])
node=rclpy.create_node('camera_fusion_preview')
pub=node.create_publisher(Image,'/camera_fusion/camera_preview',qos_profile_sensor_data)
last=0.
cv2.setNumThreads(1)

def receive(msg):
    global last
    now=time.monotonic()
    if now-last<.2 or not pub.get_subscription_count():return
    last=now
    picture=cv2.imdecode(np.frombuffer(bytes(msg.data),np.uint8),cv2.IMREAD_COLOR)
    if picture is None:return
    picture=cv2.resize(picture,(960,540),interpolation=cv2.INTER_AREA)
    folders=sorted(Path('/data').glob('intrinsics-*'))
    if folders:
        count=len(list(folders[-1].glob('*.npz')))
        cv2.putText(picture,f'Calibration: {count}/25 views',(12,28),cv2.FONT_HERSHEY_SIMPLEX,.7,(0,255,255),2)
    out=Image(header=msg.header,height=540,width=960,encoding='bgr8',is_bigendian=False,step=960*3,data=picture.tobytes())
    pub.publish(out)

node.create_subscription(CompressedImage,'/go2_stock_camera/image/compressed',receive,qos_profile_sensor_data)
try:rclpy.spin(node)
except KeyboardInterrupt:pass
finally:node.destroy_node();rclpy.try_shutdown()
