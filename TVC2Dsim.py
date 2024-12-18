import math
import random
import matplotlib.pyplot as plt


dt = 0.01  # timestep

MAX_T = 50 # time to stop sim
CP = 0  # neutral stability
TVC_PIVOT_DIST = 25  # TVC motor mount pivot distance
TVC_LENGTH = 5  # Length of motor
MAX_TV_DEFLECTION = 5  # Max motor angular deflection in degrees
MAX_DEG_PER_SEC = 50  # Max motor deflection speed in deg/s
TVC_DELAY_DT = 2     # how many dt to delay reaction by
MOI = 5  # moment of inertia
MASS = 1
G = 9.81
DRAG_FACTOR = 1 # drag coeff or area or smth
RHO = 1.204 # kg/m^3

# Simulation state variables
t = [0]
x = [0]
y = [0]
dx = [0]
dy = [0]
d2x = [0]
d2y = [0]
pitch = [90]
dpitch = [0]
d2pitch = [0]
AoA = [0] # rad
drag = [0]
vNet = [0]
vAngle = [0]
tvc_deflection = [0]  # motor pitch relative to rocket
setPitch = [90]  # pitch setpoint

# PID vars - Ziegler-Nichols method
i = 0
Ku = 0.5
Tu = 18.2
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
  # return 100 if t <= MAX_T else 0
  return math.sqrt(t) * 100

# Simulation loop
def simloop():
  # Increment time
  t.append(t[-1] + dt)

  # Update TVC angle with max deflection speed
  prev_tvc_angle = tvc_deflection[-1]
  desired_tvc_angle = PID(setPitch[-1], pitch[max(0,len(pitch) - TVC_DELAY_DT)])
  max_deflection_step = MAX_DEG_PER_SEC * dt

  # Limit the rate of change of the TVC deflection
  if desired_tvc_angle > prev_tvc_angle + max_deflection_step:
    tvc_angle = prev_tvc_angle + max_deflection_step
  elif desired_tvc_angle < prev_tvc_angle - max_deflection_step:
    tvc_angle = prev_tvc_angle - max_deflection_step
  else:
    tvc_angle = desired_tvc_angle

  tvc_angle = max(-MAX_TV_DEFLECTION, min(MAX_TV_DEFLECTION, tvc_angle)) # if t[-1] >= TVC_DELAY_DT * dt else 0
  tvc_deflection.append(tvc_angle)

  # Get thrust and forces
  F = thrust(t[-1])
  pitch_rad = math.radians(pitch[-1])
  tvc_rad = math.radians(tvc_deflection[-1])
  
  # Forces in local coordinates
  # Drag force
  vAngle.append(math.atan2(dy[-1], dx[-1]))
  vNet.append(math.hypot(dx[-1], dy[-1]))
  AoA.append(math.radians(pitch[-1]) - vAngle[-1])
  drag.append(0.5 * DRAG_FACTOR * math.cos(AoA[-1]) * vNet[-1] * vNet[-1] * RHO)
  # Add rocket force - drag force
  localF = F * math.cos(tvc_rad)# - drag[-1]
  
  # Acceleration from force
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
  # setPitch.append(90 if t[-1] <= 10 else 0)
  setPitch.append(random.randrange(45, 135) if t[-1] % 20 <= 0.001 else setPitch[-1])
  simloop()

# Plotting
nrows = 3
plt.figure(figsize=(12, 8))

# Plot pitch
plt.subplot(nrows, 2, 1)
plt.plot(t, pitch, label="Pitch (degrees)")
plt.plot(t, setPitch, label="Pitch setpoint (deg)")
# plt.axhline(setPitch, color='r', linestyle='--', label="Setpoint")
plt.xlabel("Time (s)")
plt.ylabel("Pitch (deg)")
plt.axis([0, MAX_T, 0, 180])
plt.legend()
plt.grid()


# Plot pitch rate
plt.subplot(nrows, 2, 2)
plt.plot(t, dpitch, label="Pitch rate")
plt.xlabel("Time (s)")
plt.ylabel("Pitch rate (deg/s)")
plt.legend()
plt.grid()

# Plot y displacement
plt.subplot(nrows, 2, 3)
plt.plot(t, y, label="Vertical Displacement (m)")
plt.xlabel("Time (s)")
plt.ylabel("Displacement (m)")
plt.legend()
plt.grid()

# Plot x displacement
plt.subplot(nrows, 2, 4)
plt.plot(t, x, label="Horizontal Displacement (m)")
plt.xlabel("Time (s)")
plt.ylabel("Displacement (m)")
plt.legend()
plt.grid()

# plot drag
plt.subplot(nrows, 2, 5)
plt.plot(t, drag, label="drag")
plt.xlabel("Time (s)")
plt.ylabel("drag")
plt.legend()
plt.grid()

# plot AoA
plt.subplot(nrows, 2, 6)
plt.plot(t, AoA, label="AoA (rad)")
plt.xlabel("Time (s)")
plt.ylabel("AoA (rad)")
plt.legend()
plt.grid()


plt.tight_layout()

# plt.subplot(nrows, 2, 6)
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
