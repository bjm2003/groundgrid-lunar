# Atomic local trajectory. Each twist is the desired effective body-frame velocity at
# the pose with the same index, before skid-steer inverse-command compensation.
# path.header.seq is the planner's execution goal_id (0 means unscoped/startup).
# path.header.stamp remains the publication/freshness time, NOT the requested-goal time.
nav_msgs/Path path
geometry_msgs/Twist[] twists
