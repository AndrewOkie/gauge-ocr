#!/bin/bash

./ocr -in "rtmpsrc location=\"rtmp://192.168.88.241/stream/cam live=1\" ! decodebin ! videoconvert ! videoscale sharpen=1 ! appsink"
