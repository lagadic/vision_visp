# Stamped list of KLT points positions.
#
# KLT points positions associated with a particular timestamp.
# Used by the viewer to display KLT points positions and allow
# tracking debug.

std_msgs/Header header                        # Header (required for synchronization).
visp_tracker/KltPoint[] klt_points_positions      # List of KLT points positions.
