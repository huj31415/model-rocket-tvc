import math
import matplotlib.pyplot as plt

MAX_T = 50 # time to stop sim
CP = 0  # neutral stability
TVC_PIVOT_DIST = 25  # TVC motor mount pivot distance
TVC_LENGTH = 5  # Length of motor
MAX_TV_DEFLECTION = 5  # Max motor angular deflection in degrees
MAX_DEG_PER_SEC = 10  # Max motor deflection speed in deg/s
MOI = 5  # moment of inertia
MASS = 1
G = 9.81
dt = 0.01  # timestep

# Simulation state variables
t = [0]
x = [0]
y = [0]
dx = [0]
dy = [0]
d2x = [0]
d2y = [0]
pitch = [0]  # degrees, starts at 95
dpitch = [0]
d2pitch = [0]
tvc_deflection = [0]  # motor pitch relative to rocket
setPitch = [90]  # pitch setpoint

# PID vars - Ziegler-Nichols method
i = 0
Ku = 0.1
Tu = 8.5
KP = Ku * 0.6
KI = Ku * 1.2 / Tu
KD = 3 * Ku * Tu / 40
err = [0]
# prev_err = 0

def PID(setpoint, position):
  global i # , prev_err
  prev_err = err[-1]
  new_err = setpoint - position
  err.append(new_err)
  p = KP * new_err
  i += KI * new_err * dt
  d = KD * (new_err - prev_err) / dt
  # prev_err = err  # Update for next cycle
  c = p + i + d
  return max(-MAX_TV_DEFLECTION, min(c, MAX_TV_DEFLECTION))  # Clamp the output

# Returns thrust in N
def thrust(t):
  return 100 if t <= MAX_T else 0

# Simulation loop
def simloop():
  # Increment time
  t.append(t[-1] + dt)

  # Update TVC angle with max deflection speed
  prev_tvc_angle = tvc_deflection[-1]
  desired_tvc_angle = PID(setPitch[-1], pitch[-1])
  max_deflection_step = MAX_DEG_PER_SEC * dt

  # Limit the rate of change of the TVC deflection
  if desired_tvc_angle > prev_tvc_angle + max_deflection_step:
    tvc_angle = prev_tvc_angle + max_deflection_step
  elif desired_tvc_angle < prev_tvc_angle - max_deflection_step:
    tvc_angle = prev_tvc_angle - max_deflection_step
  else:
    tvc_angle = desired_tvc_angle

  tvc_angle = max(-MAX_TV_DEFLECTION, min(MAX_TV_DEFLECTION, tvc_angle))
  tvc_deflection.append(tvc_angle)

  # Get thrust and forces
  F = thrust(t[-1])
  pitch_rad = math.radians(pitch[-1])
  tvc_rad = math.radians(tvc_deflection[-1])
  
  # Forces in local coordinates
  localF = F * math.cos(tvc_rad)
  d2x.append(localF * math.cos(pitch_rad) / MASS)
  d2y.append(localF * math.sin(pitch_rad) / MASS - G)

  # Angular acceleration
  d2pitch.append(F * math.sin(tvc_rad) * TVC_PIVOT_DIST / MOI)

  # Integrate translational dynamics
  dx.append(dx[-1] + d2x[-1] * dt)
  dy.append(dy[-1] + d2y[-1] * dt)
  x.append(x[-1] + dx[-1] * dt)
  y.append(y[-1] + dy[-1] * dt)

  # Integrate rotational dynamics
  dpitch.append(dpitch[-1] + d2pitch[-1] * dt)
  pitch.append(pitch[-1] + dpitch[-1] * dt)


while t[-1] < MAX_T: # and x[-1] >= 0:
  setPitch.append(90 if t[-1] <= 25 else 0)
  simloop()

# Plotting
nplots = 3
plt.figure(figsize=(12, 8))

# Plot pitch
plt.subplot(nplots, 1, 1)
plt.plot(t, pitch, label="Pitch (degrees)")
plt.plot(t, setPitch, label="Pitch setpoint (deg)")
# plt.axhline(setPitch, color='r', linestyle='--', label="Setpoint")
plt.xlabel("Time (s)")
plt.ylabel("Pitch (deg)")
plt.axis([0, MAX_T, -180, 180])
plt.legend()
plt.grid()

# Plot TVC deflection
plt.subplot(nplots, 1, 2)
plt.plot(t, dpitch, label="Pitch rate")
plt.xlabel("Time (s)")
plt.ylabel("Pitch rate (deg/s)")
# plt.axis([0, MAX_T, -MAX_TV_DEFLECTION - 1, MAX_TV_DEFLECTION + 1])
plt.legend()
plt.grid()

# Plot y displacement
plt.subplot(nplots, 1, 3)
plt.plot(t, y, label="Vertical Displacement (m)")
plt.xlabel("Time (s)")
plt.ylabel("Displacement (m)")
plt.legend()
plt.grid()

# plt.subplot(nplots, 1, 4)
fig, ax1 = plt.subplots()

# Plot error
ax1.set_xlabel("Time (s)")
ax1.set_ylabel("Error (deg)", color="tab:blue")
ax1.plot(t, err, label="Error", color="tab:blue")
ax1.tick_params(axis="y", labelcolor="tab:blue")
ax1.set_ylim(-90, 90)

# Create second y-axis for TVC correction
ax2 = ax1.twinx()
ax2.set_ylabel("TVC Correction (deg)", color="tab:orange")
ax2.plot(t, tvc_deflection, label="TVC Correction", color="tab:orange")
ax2.tick_params(axis="y", labelcolor="tab:orange")
ax2.set_ylim(-(MAX_TV_DEFLECTION + 1), (MAX_TV_DEFLECTION + 1))

plt.tight_layout()
plt.grid()
plt.show()
