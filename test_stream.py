import subprocess
import numpy as np
from camera import *

class RTSPStreamer:
    def __init__(self, rtsp_url, width=1280, height=720, frame_rate=30, codec='libx264', bitrate='3M', preset='fast'):
        self.rtsp_url = rtsp_url
        self.width = width
        self.height = height
        self.frame_rate = frame_rate
        self.codec = codec
        self.bitrate = bitrate
        self.preset = preset
        self.process = None

    def start_stream(self):
        """Start the ffmpeg process with specified configurations."""
        ffmpeg_command = [
            'ffmpeg',
            '-y',  # Overwrite output files without asking
            '-f', 'rawvideo',  # Input format
            '-vcodec', 'rawvideo',  # Input codec
            '-pix_fmt', 'bgr24',  # Input pixel format
            '-s', f'{self.width}x{self.height}',  # Input resolution
            '-r', str(self.frame_rate),  # Input frame rate
            '-i', '-',  # Input from stdin
            '-c:v', self.codec,  # Output codec
            '-pix_fmt', 'yuv420p',  # Output pixel format
            '-b:v', self.bitrate,  # Set bitrate
            '-preset', self.preset,  # Encoding preset
            '-tune', 'zerolatency',  # Minimize latency
            '-bufsize', '500k',  # Lower buffer size
            '-g', str(self.frame_rate),  # Set GOP size (keyint)
            '-f', 'flv',  # Output format
            self.rtsp_url  # Output URL
        ]

        # Start the ffmpeg process
        self.process = subprocess.Popen(ffmpeg_command, stdin=subprocess.PIPE)

    def stop_stream(self):
        """Stop the streaming process."""
        if self.process:
            self.process.terminate()
            self.process.wait()

# Example usage:
if __name__ == "__main__":
    rtmp_url = 'rtmp://103.167.198.50/live/albert1'
    camera = Camera(resolution=(1280, 720))
    camera.set_continuous_autofocus()
    # Configure stream parameters
    streamer = RTSPStreamer(
        rtsp_url=rtmp_url,
        width=1280,
        height=720,
        frame_rate=30,
        codec='libx264',
        bitrate='1M',
        preset='ultrafast'  # You can use 'ultrafast' for even faster encoding at the expense of compression efficiency
    )

    streamer.start_stream()
    
    try:
        while True:
            try:
                frame = camera.read_camera()
                if frame is None:
                    break
                # Write frame to ffmpeg process
                streamer.process.stdin.write(frame.tobytes())
            except KeyboardInterrupt:
                print("Streaming interrupted by user.")
                break
    finally:
        streamer.stop_stream()
