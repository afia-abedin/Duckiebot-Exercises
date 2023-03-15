Download the folder Exercise4-duckiebot
There is two folders launchers and packages
Launchers > default.sh file > launches duckietown_detection_node
Packages>LAB4>launch>duckietown_detection_node.launch
Three .py files in Packages>LAB4>src
In duckietown_detection_node.launch we are launching the three files duckiebot_detection_node.py,duckiebot_distance_node.py and follower.py
For execution:
Go to Exercise4-duckiebot 
Open Terminal
Run 
dts devel build -f -H csc22932.local --no-cache
dts devel run -f -H csc22932.local
