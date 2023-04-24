# Stamped list of moving edge positions.
#
# Moving edge positions associated with a particular timestamp.
# Used by the viewer to display moving edge positions and allow
# tracking debug.

std_msgs/Header header                       # Header (required for synchronization).
visp_tracker/MovingEdgeSite[] moving_edge_sites  # List of moving dge sites (i.e. positions).
