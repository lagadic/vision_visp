# Object pose at a particular point of time.
#
# If the tracking fails (is_tracking is false), the
# cMo attribute value can be anything and must be ignored.

Header header               # Header (required for synchornization).
bool is_tracking            # Did the tracking succeed?
geometry_msgs/Transform cMo # Object pose (translation and quaternion).
