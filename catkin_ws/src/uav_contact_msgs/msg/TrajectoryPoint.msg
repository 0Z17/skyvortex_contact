# Single target point in config-space and derived commands.

std_msgs/Header header

# q = [x, y, z, psi, theta]
float64 x
float64 y
float64 z
float64 psi
float64 theta

# dq/dt = [dx, dy, dz, dpsi, dtheta]
float64 vx
float64 vy
float64 vz
float64 vpsi
float64 vtheta

# Contact normal vector n = [nx, ny, nz]
float64 nx
float64 ny
float64 nz
