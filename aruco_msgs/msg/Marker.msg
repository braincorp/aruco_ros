# Error status
string CODE_UPSIDE_DOWN_MESSAGE            = Code is mounted upside down
string CODE_TWISTED_MESSAGE                = Code is twisted
string ANGLE_TOO_STEEP_MESSAGE             = Park parallel to the code
string CODE_NOT_FLAT_MESSAGE               = Code must be mounted flat
string TOO_CLOSE_MESSAGE                   = Emma is too close to code
string TOO_FAR_MESSAGE                     = Emma is too far from code
string MORE_THAN_ONE_CODE_MESSAGE          = Emma detects multiple codes

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