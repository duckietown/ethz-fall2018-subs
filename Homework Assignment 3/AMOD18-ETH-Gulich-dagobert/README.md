### Methodology

An discrete time LQR-controller on the state vector [d_err, phi_err] was synthesised using matlab. The resulting K-matrix was then ported to the controller implementation on the duckiebot.

## Videos
### 1st Video
The first video shows the LQR with Weight matrix Q = [40 0; 0 5] and R = 1. Clearly it can be seen that the weight on the distance is not big enough as the duck always drifts around the centerline.

### 2nd Video
In the 2nd Video this issue was treated by increasing the weight on the distance to Q = [180 0; 0 10]. Now the distance is kept well but the robot is oscillating.

### 3rd Video
Thus in the 3rd video the weight was again decreased to Q = [10 0; 0 10] which shows to be a good trade-off.

### 3rd Video
Clip 4 shows the controller's performance on an inner lane.
