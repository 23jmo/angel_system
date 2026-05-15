import rospy
import cv2
import pyaudio
import wave

import time
from threading import Event, Thread

from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

import numpy as np

from angel_utils import declare_and_get_parameters, RateTracker
from angel_utils import make_default_main

from angel_msgs.msg import HeadsetAudioData  # Add this import at the top of the file
from builtin_interfaces.msg import Time

# Topic for publishing image data
PARAM_PV_IMAGES_TOPIC = "image_topic"

# Topic for publishing image timestamp data
PARAM_PV_IMAGES_TS_TOPIC = "image_ts_topic"

# Topic for publishing audio data
PARAM_AUDIO_TOPIC = "audio_topic"

# IP address of the device
PARAM_IP_ADDR = "ip_addr"

# Width of the captured video frame
PARAM_PV_WIDTH = "pv_width"

# Height of the captured video frame
PARAM_PV_HEIGHT = "pv_height"

# Frame rate of the captured video
PARAM_PV_FRAMERATE = "pv_framerate"

DISABLE_TOPIC_STR = "disable"

class LaptopROSBridge(Node):
    
    def __init__(self):
        super().__init__(self.__class__.__name__)
        log = self.get_logger()
        param_values = declare_and_get_parameters(
            self,
            [
                (PARAM_PV_IMAGES_TOPIC,),
                (PARAM_PV_IMAGES_TS_TOPIC,),
                (PARAM_AUDIO_TOPIC,),
                (PARAM_IP_ADDR,),
                (PARAM_PV_WIDTH,),
                (PARAM_PV_HEIGHT,),
                (PARAM_PV_FRAMERATE,),
            ],
        )
        
        self._image_topic = param_values[PARAM_PV_IMAGES_TOPIC]
        self._image_ts_topic = param_values[PARAM_PV_IMAGES_TS_TOPIC]
        self._audio_topic = param_values[PARAM_AUDIO_TOPIC]
        self.ip_addr = param_values[PARAM_IP_ADDR]
        self.pv_width = param_values[PARAM_PV_WIDTH]
        self.pv_height = param_values[PARAM_PV_HEIGHT]
        self.pv_framerate = param_values[PARAM_PV_FRAMERATE]

        if self._image_topic != DISABLE_TOPIC_STR:
            # Create publisher for image data 
            self.ros_frame_publisher = self.create_publisher(
                Image, self._image_topic, 1
            )
            # Create publisher for image timestamp data
            self.ros_frame_ts_publisher = self.create_publisher(
                Time, self._image_ts_topic, 1
            )
            # Start the frame publishing thread
            self._pv_active = Event()
            self._pv_active.set()
            self._pv_rate_tracker = RateTracker()
            self._pv_thread = Thread(target=self.pv_publisher, name="publish_pv")
            self._pv_thread.daemon = True
            self._pv_thread.start()

        if self._audio_topic != DISABLE_TOPIC_STR:
            self.ros_audio_publisher = self.create_publisher(
                HeadsetAudioData, self._audio_topic, 1
            )
            # Start the audio data thread
            self._audio_active = Event()
            self._audio_active.set()
            self._audio_rate_tracker = RateTracker()
            self._audio_thread = Thread(
                target=self.audio_publisher, name="publish_audio"
            )
            self._audio_thread.daemon = True
            self._audio_thread.start()
            
            
    def pv_publisher(self):
        cap = cv2.VideoCapture(0)
        bridge = CvBridge()
        while self._pv_active.is_set():
            ret, frame = cap.read()
            if ret:
                image_message = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                self.ros_frame_publisher.publish(image_message)
                self.ros_frame_ts_publisher.publish(image_message.header.stamp)
    
    def audio_publisher(self):
        
        log = self.get_logger()
        pa = pyaudio.PyAudio()
        
        AUDIO_CHANNELS = 1
        AUDIO_RATE = 48000
        
        while self._audio_active.is_set():
            stream = pa.open(
                format=pyaudio.paInt16, 
                channels=AUDIO_CHANNELS, 
                rate=AUDIO_RATE, 
                output=True, 
                stream_callback=self.audio_callback
            )
        
    
    def audio_callback(self, in_data, frame_count, time_info, status):
        audio_data = np.frombuffer(in_data, dtype=np.int16)
        
        msg = HeadsetAudioData()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "laptop_microphone"
        msg.channels = 1
        msg.sample_rate = self.get_parameter('audio_rate').value
        msg.sample_duration = 1.0 / msg.sample_rate
        msg.data = audio_data.tolist()

        self.ros_audio_publisher.publish(msg)
       
def shutdown_clients(self):
    log = self.get_logger()
    
    if self._image_topic != DISABLE_TOPIC_STR:
        self._pv_active.clear()
        self._pv_thread.join()
        log.info("PV thread closed")

    if self._audio_topic != DISABLE_TOPIC_STR:
        self._audio_active.clear()
        self._audio_thread.join()
        log.info("Audio thread closed")

def destroy_node(self):
    self.shutdown_clients()
    
main = make_default_main(LaptopROSBridge)

if __name__ == "__main__":
    main()


