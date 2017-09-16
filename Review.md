## Model Predictive Control

**Given:**
The simulator with a car riding around the track.
 During the ride the simulator emits arrays of
 waypoints aligned to the world coordinate system.
 
**Task:**
- Calculate Cross track error `CTE`
- Calculate PSI error `EPSI`
- Implement **MPC**
>> - Implement basic kinematic model. 

**Solution:**

Calculation of the cross track error can be represented as:
```
for (int i = 0; i < coeffs.size(); i++) {
   result += coeffs[i] * pow(x, i);
}
```
Where `coeffs` is the coefficients of the polynomial fitted to the waypoints.
The cross track error for the prediction calcualted in the following way:

`cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt`
Where:
`x` - X coordinate
`y` - Y coordinate
`v` - V speed
`dt` - time between measurements

`EPSI` was calculated by applying this formula:
`epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt`

The initial `epsi` was calculated using `arctan` from the first derivative.  

To finish the kinematic model of the car the future `x`, `y`, `psi` 
coordinates were calculated using equations below.

```
x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
v_[t+1] = v[t] + a[t] * dt
```

## Timestep Length and Elapsed Duration

With timestamp length (N) equals to 20 and delta time (Dt) 100ms the car successfully gets over the track.

During the development I tried to bring N to a higher value like 30. Such increase requires more computational power and
is unnecessary, since the car can go safely with speed up to 50mph. Smaller `Dt` leads to smaller time horizon and higher error rate.
So the car's behaviour is unstable.  

## Model Predictive Control with Latency

The first step is to take the previous calculation into account. Using the calculations from the previous calculations.
Second step to apply cost function introduced in the lesson and using velocity and it's delta. 
The best result achieved with limiting speed. 

To deal with latency we simply make the receiver sleep for 100ms before new entries. 

```
this_thread::sleep_for(chrono::milliseconds(100));
ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
```