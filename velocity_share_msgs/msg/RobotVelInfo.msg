# A representation of a robot's name, its pose and the current velocity represented
# as a point relative to the global frame.
# The "high_prio" determines whether the reported robot has a higher prio than the current one.

string robot_name
bool high_prio
nav_msgs/Path path
