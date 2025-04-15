#!/bin/bash
# can_init.sh
sudo ip link set can0 type can bitrate 500000 restart-ms 100
sudo ip link set can0 up