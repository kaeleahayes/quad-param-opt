# quad-param-opt
Code and documentation for quadcopter rotor drag parameter optimization

### Processing flight data
After recording required flight data, store the resulting rosbag in raw_flight_data. The script parse_rosbag.py reads the topics from the rosbag and reformats the data into .csv files usable by the parameter optimization code.

Usage - 
- source ~/[QUADCOPTER_WS]/install/setup.bash to get any custom message types
- python3 parse_rosbag.py raw_flight_data/[ROSBAG_FOLDER] -o processed_data/[SUBDIRECTORY_NAME]