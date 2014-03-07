# Moving edge position.

float64 x      # X position in the image
float64 y      # Y position in the image
int32 suppress # Is the moving edge valid?
               # - 0:   yes
	       # - 1:   no, constrast check has failed.
	       # - 2:   no, threshold check has failed.
	       # - 3:   no, M-estimator check has failed.
	       # - >=4: no, undocumented reason.
