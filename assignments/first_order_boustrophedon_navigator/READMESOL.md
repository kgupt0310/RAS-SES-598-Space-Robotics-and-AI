Final parameters:

                ('Kp_linear', 12.0),
                ('Kd_linear', 0.03),
                ('Kp_angular', 8.0),
                ('Kd_angular', 0.1),
                ('spacing', 0.5)

Results: - 
Average cross-track error  = 0.107 units
Maximum cross-track error  = 0.258  units


The results show that the error is well within the range for the mover to run smoothly enough.
Methodology used for tuning: 

To tune the parameters I start with an initial set of parameters that are ideal for a straight line motion which means high Kp for linear motion and low Kp for angular motion since for most of the path our motion is linear with very low change in orientation of the turtle. Now, to smoothen the curve along the corners, I increase Kp for angular motion slightly to ensure a smooth turn. The Kd is kept low for linear motion since the change in velocity would be minimal throughout the motion even at the curves . Kd for angular motion is slightly increased to ensure smooth motion along the curve where velocity would increase and decrease sharply as the turtle moves along the curve. At each iteration the final average error and max avg error is measured along with the path traced by the turtle. The spacing is decreased slightly at every iteration to increase the coverage while making sure it does not cause a sharp change in direction at the corners.

challanges:

Parameter Tuning Complexity: 
-Difficulty in finding the optimal balance between speed and accuracy
-Interdependence of parameters leading to unexpected behaviors when adjusted


Oscillation and Stability Issues:
-Overcoming oscillations in the robot's path, especially during turns
-Maintaining stability at higher speeds without compromising accuracy
Corner Handling:

-Achieving smooth transitions at corners without overshooting
-Balancing the trade-off between sharp turns and maintaining speed

Cross-Track Error Minimization:

-Consistently keeping the cross-track error below the required threshold (0.2 units average, 0.5 units maximum)
-Dealing with sudden increases in cross-track error due to external factors or sharp turns