#!/usr/bin/env python

import json
import rospy
from std_msgs.msg import String
import socket
import ctypes
import struct

# Define COCO class names
coco_names = ["person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat",
              "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog",
              "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag",
              "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat",
              "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
              "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot",
              "hot dog", "pizza", "donut", "cake", "chair", "couch", "potted plant", "bed", "dining table",
              "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave", "oven",
              "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier",
              "toothbrush"]

class NmsBbox(ctypes.Structure):
    _fields_ = [
        ("x1", ctypes.c_uint),
        ("y1", ctypes.c_uint),
        ("x2", ctypes.c_uint),
        ("y2", ctypes.c_uint),
        ("dx1", ctypes.c_uint),
        ("dy1", ctypes.c_uint),
        ("dx2", ctypes.c_uint),
        ("dy2", ctypes.c_uint),
        ("conf", ctypes.c_float),
        ("class_id", ctypes.c_uint),
        ("arr", ctypes.c_float * 32)
    ]
        
UDP_IP = "0.0.0.0"  # Listen on all interfaces
UDP_PORT = 5000
offset = 4

def parse_data(socket):
    pub = rospy.Publisher('camera_od_topic', String, queue_size=10)
    rospy.init_node('camera_object_detector', anonymous=True)
    try:
        while True:
            try:
                data, _ = socket.recvfrom(3364)
                deserialized_data = data
                bbox_num = struct.unpack('<i', deserialized_data[:4])[0]

                parsed_data = []

                for bbox_id in range(0, bbox_num):
                    offset = 4
                    bbox_size = ctypes.sizeof(NmsBbox)
                    start = offset + (bbox_size * bbox_id)
                    end = start + bbox_size
                    bbox_bytes = deserialized_data[start: end]
                    bbox = ctypes.cast(bbox_bytes, ctypes.POINTER(NmsBbox)).contents
                    class_name = coco_names[bbox.class_id]

                    bbox_info = {
                        'bbox_id': bbox_id,
                        'x1': bbox.x1,
                        'y1': bbox.y1,
                        'x2': bbox.x2,
                        'y2': bbox.y2,
                        'dx1': bbox.dx1,
                        'dy1': bbox.dy1,
                        'dx2': bbox.dx2,
                        'dy2': bbox.dy2,
                        'class_name': class_name,
                        'confidence': bbox.conf
                    }

                    parsed_data.append(bbox_info)

                # Publish parsed data to ROS topic
                pub.publish(json.dumps(parsed_data))

            except Exception as e:
                rospy.logerr("Error parsing data: %s", str(e))
    except Exception as e:
        rospy.logerr("Error in websocket connection: %s", str(e))

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    parse_data(sock)
    

if __name__ == "__main__":
    try:
        rospy.loginfo("Starting camera object detector node...")
        main()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("Unexpected error: %s", str(e))