define rplidar ranger
(
  sensor(       
    # laser-specific properties
    # factory settings for LMS200  
    range [ 0.1  6.0 ]
    fov 360.0
    samples 270
  )
  model
  (
    # generic model properties
    size [ 0.07 0.07 0.05 ]  
    color "blue"
  )
)

define runner position
(
  pose [ 0.0 0.0 0.0 0.0 ]

  odom_error [0.03 0.03 999999 999999 999999 0.02]

  size [ 0.2552 0.2552 0.40 ]
  origin [ 0.0 0.0 0.0 0.0 ]
  gui_nose 1
  drive "diff"
  color "gray"
  
  rplidar(pose [ -0.1 0.0 -0.11 0.0 ])
)
