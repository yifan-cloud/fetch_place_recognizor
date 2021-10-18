# fetch_place_recognizor
Place recognizor using DBoW2 on Fetch Robot

Only test it on the simulation env.

## Get started
Build DBoW2 first

`cd include`
`mkdir build`
`cd build`
`cmake ..`
`make`

Then catkin_make the rospackage

`cd ../`
`catkin_make`

Start Fetch simulation first

`roslaunch fetch_gazebo playground.launch`

Then start the place_recognition node

`rosrun place_recognition ple_recognition`
