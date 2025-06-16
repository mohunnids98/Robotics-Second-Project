define kinect ranger 
(
  sensor
  (
    range_max 100.0
    fov 360.0
    samples 1242
  )
  # generic model properties
  color "black"
  size [ 0.06 0.15 0.03 ]
)

define turtlebot position
(
  pose [ 0.0 0.0 0.0 0.0 ] 

  odom_error [0.01 0.01 0.01 0.01 0.01 0.01]

  size [ 0.54 0.40 0.40 ]
  origin [ 0.0 0.0 0.0 0.0 ]
  gui_nose 1
  drive "diff"
  color "gray"
  
  kinect(pose [ 0.0 0.0 0.0 0.0 ])
)
