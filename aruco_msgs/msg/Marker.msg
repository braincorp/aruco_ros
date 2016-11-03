# Error status
uint32 NO_ERROR                            = 0
uint32 CODE_UPSIDE_DOWN                    = 1
uint32 CODE_TWISTED                        = 2
uint32 ANGLE_TOO_STEEP                     = 4
uint32 CODE_NOT_FLAT                       = 8
uint32 TOO_CLOSE                           = 16
uint32 TOO_FAR                             = 32
uint32 MORE_THAN_ONE_CODE                  = 64

Header header
uint32 id
uint32 home_id
uint32 error_code
string error_message
geometry_msgs/PoseWithCovariance pose