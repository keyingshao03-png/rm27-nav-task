#!/usr/bin/env bash
# Gazebo-native moving obstacle. No ROS service dependency.

set +e

SDF="/tmp/rm27_moving_box.sdf"
cat > "$SDF" <<'EOF'
<?xml version="1.0"?>
<sdf version="1.6">
  <model name="moving_box">
    <static>false</static>
    <link name="link">
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.1</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>0.1</iyy><iyz>0</iyz><izz>0.1</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry><box><size>0.6 0.6 1.0</size></box></geometry>
      </collision>
      <visual name="visual">
        <geometry><box><size>0.6 0.6 1.0</size></box></geometry>
        <material>
          <ambient>1 0 0 1</ambient>
          <diffuse>1 0 0 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
EOF

echo "[box] deleting old moving_box if exists"
gz model -m moving_box -d >/dev/null 2>&1 || true
sleep 1

echo "[box] spawning moving_box"
gz model -m moving_box -f "$SDF" -x -1.2 -y -2.2 -z 0.5
sleep 1

echo "[box] moving obstacle in a loop. Press Ctrl+C to stop."
t=0
while true; do
  x=$(awk -v t="$t" 'BEGIN { printf "%.3f", -1.2 + 0.9*cos(t*0.12) }')
  y=$(awk -v t="$t" 'BEGIN { printf "%.3f", -2.2 + 0.9*sin(t*0.12) }')
  yaw=$(awk -v t="$t" 'BEGIN { printf "%.3f", t*0.12 }')
  gz model -m moving_box -x "$x" -y "$y" -z 0.5 -Y "$yaw" >/dev/null 2>&1
  t=$((t + 1))
  sleep 0.15
done
