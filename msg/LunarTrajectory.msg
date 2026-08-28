# Atomic local trajectory. Each twist is the desired effective body-frame velocity at
# the pose with the same index, before skid-steer inverse-command compensation.
nav_msgs/Path path
geometry_msgs/Twist[] twists
