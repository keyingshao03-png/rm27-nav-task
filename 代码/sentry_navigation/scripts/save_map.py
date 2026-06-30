import rclpy, cv2, numpy as np, yaml
from nav_msgs.msg import OccupancyGrid
rclpy.init()
n = rclpy.create_node("mapsaver")
msgs = []
n.create_subscription(OccupancyGrid, "/map", lambda m: msgs.append(m), 10)
print("Waiting for /map...")
for i in range(30):
    rclpy.spin_once(n, timeout_sec=1.0)
    if msgs: break
    print(f"  {i+1}s...")
if not msgs:
    print("NO MAP DATA - drive robot around first")
else:
    m = msgs[-1]
    d = np.array(m.data, dtype=np.int8).reshape(m.info.height, m.info.width)
    img = np.where(d >= 0, 255 - d, 205).astype(np.uint8)
    cv2.imwrite("map.pgm", img)
    yaml.dump({"image":"map.pgm","mode":"trinary","resolution":float(m.info.resolution),
     "origin":[float(m.info.origin.position.x),float(m.info.origin.position.y),0.0],
     "negate":0,"occupied_thresh":0.65,"free_thresh":0.25}, open("map.yaml","w"))
    print(f"SAVED {m.info.width}x{m.info.height} -> map.pgm + map.yaml")
rclpy.shutdown()