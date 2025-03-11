# VisuaLocalization-GroundTruth
This GroundTruth Repo is for robot localization with visual method.

# Ceiling Camera   
 --- 
## motioneye    
```
sudo systemctl start motioneye
```
```
sudo systemctl stop motioneye
```
## Gstreamer  RTSP stream   
### On Ceiling   
```
./cam-rtsp "( v4l2src device=/dev/video0 ! videoconvert! videoscale ! video/x-raw,format=I420,width=640,height=480 ! x264enc tune="ze
rolatency" threads=4 ! rtph264pay name=pay0 pt=96 )"
```
```
./cam-rtsp "( v4l2src device=/dev/video0 ! image/jpeg, width=1280, height=720, framerate=30/1 ! rtpjpegpay name=pay0 pt=96 )"

```
```
./cam-rtsp "( v4l2src device=/dev/video0 io-mode=2 ! image/jpeg, width=1024, height=768, framerate=30/1 ! rtpjpegpay name=pay0 pt=96 )"
```
- System Service   
   
```
sudo systemctl status cam-rtsp

```
### On Client    
```
ffplay -rtsp_transport udp rtsp://192.168.50.60:8554/camera

```
   
```
# FROM 192.168.50.60 to 192.168.50.152
Frame 248766: 1442 bytes on wire (11536 bits), 1442 bytes captured (11536 bits) on interface wlo1, id 0
    Section number: 1
    Interface id: 0 (wlo1)
    Encapsulation type: Ethernet (1)
    Arrival Time: Feb 23, 2025 01:11:32.757133514 CST
    UTC Arrival Time: Feb 22, 2025 17:11:32.757133514 UTC
    Epoch Arrival Time: 1740244292.757133514
    [Time shift for this packet: 0.000000000 seconds]
    [Time delta from previous captured frame: 0.000000095 seconds]
    [Time delta from previous displayed frame: 0.000000095 seconds]
    [Time since reference or first frame: 47.032052970 seconds]
    Frame Number: 248766
    Frame Length: 1442 bytes (11536 bits)
    Capture Length: 1442 bytes (11536 bits)
    [Frame is marked: False]
    [Frame is ignored: False]
    [Protocols in frame: eth:ethertype:ip:udp:data]
    [Coloring Rule Name: UDP]
    [Coloring Rule String: udp]
Ethernet II, Src: RaspberryPiT_56:bd:7e (e4:5f:01:56:bd:7e), Dst: Intel_ea:86:61 (00:93:37:ea:86:61)
Internet Protocol Version 4, Src: 192.168.50.60, Dst: 192.168.50.152
User Datagram Protocol, Src Port: 36668, Dst Port: 8934
Data (1400 bytes)

# FROM # FROM 192.168.50.152 to 192.168.50.60 (RTCP REPORT)
Frame 248767: 90 bytes on wire (720 bits), 90 bytes captured (720 bits) on interface wlo1, id 0
    Section number: 1
    Interface id: 0 (wlo1)
    Encapsulation type: Ethernet (1)
    Arrival Time: Feb 23, 2025 01:11:32.757202738 CST
    UTC Arrival Time: Feb 22, 2025 17:11:32.757202738 UTC
    Epoch Arrival Time: 1740244292.757202738
    [Time shift for this packet: 0.000000000 seconds]
    [Time delta from previous captured frame: 0.000069224 seconds]
    [Time delta from previous displayed frame: 0.036835846 seconds]
    [Time since reference or first frame: 47.032122194 seconds]
    Frame Number: 248767
    Frame Length: 90 bytes (720 bits)
    Capture Length: 90 bytes (720 bits)
    [Frame is marked: False]
    [Frame is ignored: False]
    [Protocols in frame: eth:ethertype:ip:udp:rtcp]
    [Coloring Rule Name: UDP]
    [Coloring Rule String: udp]
Ethernet II, Src: Intel_ea:86:61 (00:93:37:ea:86:61), Dst: RaspberryPiT_56:bd:7e (e4:5f:01:56:bd:7e)
Internet Protocol Version 4, Src: 192.168.50.152, Dst: 192.168.50.60
User Datagram Protocol, Src Port: 8935, Dst Port: 36669
Real-time Transport Control Protocol (Receiver Report)
Real-time Transport Control Protocol (Source description)

```
封包延遲約為👇    
```
延遲（Latency） = 目標端收到時間 - 來源端發送時間
                 = 1740244292.757202738 - 1740244292.757133514
                 = 0.000069224 (s)
                 = 69.2 (µs)

```
### Gstreamer Client   
```
gst-launch-1.0 rtspsrc location=rtsp://192.168.50.60:8554/camera latency=0 ! decodebin ! autovideosink
```
### ROS config   
```
export GSCAM_CONFIG="rtspsrc location=rtsp://192.168.50.60:8554/camera latency=0 ! decodebin ! autovideosink"
```
```
ros2 run gstcam gst_video_publisher 
ros2 run image_proc image_proc --ros-args --remap __ns:=/ceiling

```
   
   
## On Host    
```
sudo modprobe v4l2loopback devices=1 card_label="Visual Ceiling Webcam" exclusive_caps=1
```
remove virtualcam   
```
sudo modprobe -r v4l2loopback

```
```
gst-launch-1.0 rtspsrc location=rtsp://192.168.50.60:8554/camera latency=0 ! decodebin ! videosink device=/dev/video0
```