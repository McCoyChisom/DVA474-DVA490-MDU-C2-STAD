gst-launch-1.0 udpsrc port=5600 caps="application/x-rtp" ! rtph264depay ! avdec_h264 ! autovideosink
gst-launch-1.0 udpsrc port=5600 caps="application/x-rtp" ! rtph264depay ! avdec_h264 ! videorate ! video/x-raw,framerate=1/1 ! videoconvert ! pngenc ! multifilesink location="frame.png"
