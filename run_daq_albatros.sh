#!/bin/bash
python albatros_daq.py "localhost" -f /home/pi/firmware/quad_input_poco_2018-09-08_1002.fpg -a 393216 -s 0xFFFF -l /home/pi/logs/ -o ../test_data/ -t 30
