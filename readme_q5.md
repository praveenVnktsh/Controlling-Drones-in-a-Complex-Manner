### Question 5

```
Gains: 
KXatt = [Kp_phi, Kp_theta, Kp_psi]
KXpos = [Kp_x, Kp_y, Kp_z]
"kpatt": [  190,    190,    70],
"kdatt": [    30,    30,    18],
"kppos": [    20,    20,    18],
"kdpos": [    8,    8,    9],
```
The actual poses of the drone are as follows:
![](outputs/5/a_states_trackPlot.png)

<p style="page-break-after: always;">&nbsp;</p>

The error plots for the requested scenario are as follows:

![](outputs/5/a_stateErrors_trackPlot.png)

![](outputs/5/a_settlingandrisetimes.png)

- Performance parameters:
    - Rise Time:
    - Settling Time:
    - Overshoot: 


<p style="page-break-after: always;">&nbsp;</p>


Ensuring that the performance is physically plausible using the thrust to weight plot for this scenario:
 
![](outputs/5/a_fbyw.png)

We can see that the performance is well within physical limits.

<p style="page-break-after: always;">&nbsp;</p>

The trajectory in 3D can be seen like this:

![](outputs/5/a_trajectory.png)

<p style="page-break-after: always;">&nbsp;</p>

#### Ablation study 

##### Modifying Kd_positions (Outer Loop)
Here the Kd of the outer loop has been increased.

```
Gains: 
KXatt = [Kp_phi, Kp_theta, Kp_psi]
KXpos = [Kp_x, Kp_y, Kp_z]
params['kpatt'] = [190, 190, 20]
params['kdatt'] = [30, 30, 18]
params['kppos'] = [20, 20, 10]
params['kdpos'] = [13, 13, 19]
```

The actual poses of the drone are as follows:
![](outputs/5/gains2_kdpos_states.png)


The error plots for the requested scenario are as follows:

![](outputs/5/gains2_kdpos_stateErrors.png)

- We can see that the system reaches the steady state value in a much more delayed fashion - it takes a lot more time for the drone to converge to the desired height and actually fails to do so in the actual amount of time that we give it!

<p style="page-break-after: always;">&nbsp;</p>


Ensuring that the performance is physically plausible using the thrust to weight plot for this scenario:
 
![](outputs/5/gains2_kdpos_fbyw.png)

We can see that the performance is well within physical limits.

<p style="page-break-after: always;">&nbsp;</p>


##### Modifying Kp_positions (Outer Loop)

Here, the Kp of the outer loop has been increased.

```
Gains: 
KXatt = [Kp_phi, Kp_theta, Kp_psi]
KXpos = [Kp_x, Kp_y, Kp_z]
params['kpatt'] = [190, 190, 20]
params['kdatt'] = [30, 30, 18]
params['kppos'] = [25, 25, 10]
params['kdpos'] = [8, 8, 9]
```
The actual poses of the drone are as follows:
![](outputs/5/gains3_kppos_states.png)

The error plots for the requested scenario are as follows:

![](outputs/5/gains3_kppos_stateErrors.png)

- We can see that the overshoot has been increased and the drone reaches the desired x-location much quicker than previously.

Ensuring that the performance is physically plausible using the thrust to weight plot for this scenario:
 
![](outputs/5/gains3_kppos_fbyw.png)

We can see that the performance is well within physical limits.


<p style="page-break-after: always;">&nbsp;</p>


##### Modifying Kp_attitude (Inner Loop)

Here the Kp of the attitude controller has been increased

```
Gains: 
KXatt = [Kp_phi, Kp_theta, Kp_psi]
KXpos = [Kp_x, Kp_y, Kp_z]
params['kpatt'] = [220, 220, 20]
params['kdatt'] = [30, 30, 18]
params['kppos'] = [25, 25, 10]
params['kdpos'] = [8, 8, 9]
        
```
The actual poses of the drone are as follows:
![](outputs/5/gains3_kpatt_states.png)

The error plots for the requested scenario are as follows:

![](outputs/5/gains3_kpatt_stateErrors.png)

- We can see that the overshoot in the attitude has been increased significantly, but the system responds faster.

Ensuring that the performance is physically plausible using the thrust to weight plot for this scenario:
 
![](outputs/5/gains3_kppos_fbyw.png)

We can see that the performance is well within physical limits.

<p style="page-break-after: always;">&nbsp;</p>


##### Modifying Kd_attitude (Inner Loop)

Here the Kd of the attitude controller has been increased

```
Gains: 
KXatt = [Kp_phi, Kp_theta, Kp_psi]
KXpos = [Kp_x, Kp_y, Kp_z]
params['kpatt'] = [190, 190, 20]
params['kdatt'] = [35, 35, 20]
params['kppos'] = [25, 25, 10]
params['kdpos'] = [8, 8, 9]
        
```
The actual poses of the drone are as follows:
![](outputs/5/gains4_kdatt_states.png)

The error plots for the requested scenario are as follows:

![](outputs/5/gains4_kdatt_stateErrors.png)

- We can see that the overshoot in the attitude has been decreased, and the drone is smoother in its response.

Ensuring that the performance is physically plausible using the thrust to weight plot for this scenario:
 
![](outputs/5/gains3_kppos_fbyw.png)

We can see that the performance is well within physical limits.
