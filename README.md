Controls:

|Controller|Input|Number|Direction|Function
|---|---|---|---|---
|Driver|LS-Y axis|1|Forward is positive|Move the robot forward/backwards
|Driver|LS-X axis|0|Right is positive|Move the robot left/right while staying oriented to the field
|Driver|RS-X axis|4|Right is positive|Turn the robot left/right in position

Gyro: Left is Positive, Right is Negative.  Value between -180 to 180, 179.5* + 1* = -179.5*
Swerve Modules: Left is negative, Right is Positive.  AbsEncoderRaw must be in degrees, but can go forever angle

Things to lookup:
Motor configurations
Gearings
Encoder counts
Absolute sensor
Wheel Size
SwerveModuleState takes care of swerve rotation to keep us between 0-90* (just drives the motor backwards to flip it)
