# This message contains tracking parameters.
#
# These parameters determine how precise, how fast and how
# reliable will be the tracking.
#
# It should be tuned carefully and can be changed dynamically.
#
# For more details, see the ViSP documentation:
# http://www.irisa.fr/lagadic/visp/publication.html

# Klt Parameters.

int64 max_features      # Maximum number of features
int64 window_size       # Window size
float64 quality         # Quality of the tracker
float64 min_distance      # Minimum distance betwenn two points
float64 harris          # Harris free parameters
int64 size_block        # Block size
int64 pyramid_lvl       # Pyramid levels
int64 mask_border       # Mask Border

