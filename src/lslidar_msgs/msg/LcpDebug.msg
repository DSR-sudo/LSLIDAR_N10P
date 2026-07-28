# LCP geometric debug data.  All distances are metres.
#
# This message is published only while status is 2 and pose_valid is true.
# The four directional distances are ray distances from the laser origin in
# the current body frame: front=+X, left=+Y, rear=-X, right=-Y.  They are not
# the signed lcp_map position and remain positive even when the vehicle moves.

std_msgs/Header header

uint8 status
bool map_locked
bool pose_valid

# Current laser pose in lcp_map, duplicated here so the distances and pose
# always describe exactly the same processed scan.
float32 position_x_m
float32 position_y_m
float32 yaw_rad

float32 front_distance_m
float32 rear_distance_m
float32 left_distance_m
float32 right_distance_m

# Dimensions of the locked rectangular LCP map along lcp_map +X and +Y.
float32 map_size_x_m
float32 map_size_y_m
