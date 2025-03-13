import math
import random
import matplotlib.pyplot as plt
from perlin_noise import PerlinNoise



dt = 0.01  # timestep

MAX_T = 50 #50 # time to stop sim, 5
BURN_TIME = 20 #20 # 2
CP = 0  # neutral stability
TVC_PIVOT_DIST = 0.3  # TVC motor mount pivot distance
TVC_LENGTH = 0.05  # Length of motor
MAX_TV_DEFLECTION = 15  # Max motor angular deflection in degrees
MAX_DEG_PER_SEC = 600 #50  # Max motor deflection speed in deg/s
MAX_DEFLECTION_STEP = MAX_DEG_PER_SEC * dt
TVC_DELAY_DT = 5     # how many dt to delay reaction by
TVC_OFFSET = 0 #1        # offset due to build inaccuracies
MOI = 0.015 # 5  # moment of inertia
MASS = 0.25 # 1
G = 9.81
DRAG_FACTOR = .01 # for the drag equation
RHO = 1.204 # kg/m^3
ROT_DAMPING = 1 # 0.999  # rotation damping due to drag per dt

WIND_FORCE = 1
WIND_VEL = 5

PITCH_INIT = 85

pn = PerlinNoise(octaves=5)
pnoise = [pn(0)]

noise = [random.random()]

# Simulation state variables
t = [0]
x = [0]
y = [0]
dx = [0]
dy = [0]
d2x = [0]
d2y = [0]
pitch = [PITCH_INIT]
dpitch = [1]
d2pitch = [0]
AoA = [0] # rad
drag = [0]
localF = [0]
vNet = [0]
vAngle = [0]
tvc_deflection = [0]  # motor deflection relative to rocket
setPitch = [90]  # pitch setpoint

# PID vars
pitchI = 0       # integral error value

# Tuning - Ziegler-Nichols method / Relay method
ctrlAmp = 5 # amplitude of control oscillation
outAmp = 2 # amplitude of output oscillation
Ku = 4 * ctrlAmp / (math.pi * outAmp) #1.44    # oscillating KP value
Tu = 4.55 #4   # oscillation period
KP = Ku * 0.6 # Ku * 0.6
KI = KP * 0.5 * Tu # Ku * 1.2 / Tu
KD = KP * 0.12 * Tu # 3 * Ku * Tu / 40
# KP = 1.44
# KI = 0
# KD = 0
pitchErr = [setPitch[0] - pitch[0]]

dxErr = [0]
dxI = 0
dxPID = [0]
dxKu = 7.27
dxTu = 1.9
dxKP = dxKu * 0.6 #- 3
dxKI = dxKu * 1.2 / dxTu
dxKD = -3 * dxKu * dxTu / 40

def PID(setpoint:float, position:float, KP:float, KI:float, KD:float, err:list[float], i:float):
  # global i # , prev_err
  prev_err = err[-1]
  new_err = setpoint - position
  err.append(new_err)
  p = KP * new_err
  i += KI * new_err * dt
  d = KD * (new_err - prev_err) / dt
  return p + i + d

# Returns thrust in N
def thrust(t):
  # return 10 if t <= MAX_T else 0     # constant thrust
  # return math.sqrt(t) * 100           # sqrt thrust
  return math.exp(-t * 10) * 10 + 5 if t <= BURN_TIME else 0   # thrustcurve approximation

# Simulation loop
def simloop():
  # Increment time
  t.append(t[-1] + dt)
  
  # add next perlin noise step
  pnoise.append(pn(t[-1] / MAX_T))
  noise.append(random.random())
  
  dxPID.append(PID(0, dx[-1], dxKP, dxKI, dxKD, dxErr, dxI))
  setPitch.append(90 - dxPID[-1]) #PID(0, dy[-1], 1, 0, 0, dyErr, dyI))

  # Update TVC angle with max deflection speed, including noise
  prev_tvc_angle = tvc_deflection[-1]
  desired_tvc_angle = PID(setPitch[-1], pitch[max(0,len(pitch) - TVC_DELAY_DT)], KP, KI, KD, pitchErr, pitchI) #pitch + noise[-1]

  # Limit the rate of change of the TVC deflection
  if desired_tvc_angle > prev_tvc_angle + MAX_DEFLECTION_STEP:
    tvc_angle = prev_tvc_angle + MAX_DEFLECTION_STEP
  elif desired_tvc_angle < prev_tvc_angle - MAX_DEFLECTION_STEP:
    tvc_angle = prev_tvc_angle - MAX_DEFLECTION_STEP
  else:
    tvc_angle = desired_tvc_angle

  # actuate TVC
  # tvc_angle = max(-MAX_TV_DEFLECTION, min(MAX_TV_DEFLECTION, tvc_angle))
  # tvc_deflection.append(math.copysign(ctrlAmp, 90 - pitch[-1])) # For bang-bang control tuning - Ku = 4(output change)/pi(amplitude), Tu = period
  tvc_deflection.append(max(-MAX_TV_DEFLECTION, min(tvc_angle + TVC_OFFSET, MAX_TV_DEFLECTION))) # For running

  # Get thrust and forces
  F = thrust(t[-1])
  pitch_rad = math.radians(pitch[-1])
  tvc_rad = math.radians(tvc_deflection[-1])
  
  # Forces in local coordinates
  # Drag force
  vAngle.append(math.atan2(dy[-1], dx[-1]))
  # vNet.append(math.hypot(dx[-1], dy[-1]))
  AoA.append(math.radians(pitch[-1]) - vAngle[-1])
  # drag.append(0.5 * DRAG_FACTOR * math.cos(AoA[-1]) * vNet[-1] * vNet[-1] * RHO)
  # Add rocket force - drag force
  localF.append(F * math.cos(tvc_rad))# - drag[-1]
  
  # Acceleration from force
  d2x.append(localF[-1] * math.cos(pitch_rad) / MASS + WIND_FORCE / MASS * (WIND_VEL * pnoise[-1] - dx[-1]))
  d2y.append(localF[-1] * math.sin(pitch_rad) / MASS - G)

  # Angular acceleration
  d2pitch.append(F * math.sin(tvc_rad) * TVC_PIVOT_DIST / MOI)# + 1 * math.sin(AoA[-1]) * TVC_PIVOT_DIST / MOI)

  # Integrate translational dynamics
  dx.append(dx[-1] + d2x[-1] * dt)
  dy.append(dy[-1] + d2y[-1] * dt)
  x.append(x[-1] + dx[-1] * dt)
  y.append(y[-1] + dy[-1] * dt)

  # Integrate rotational dynamics
  dpitch.append(dpitch[-1] * ROT_DAMPING + d2pitch[-1] * dt)
  pitch.append(pitch[-1] + dpitch[-1] * dt)


while t[-1] < MAX_T and y[-1] >= 0: # and x[-1] >= 0:
  # setPitch.append(90)
  # setPitch.append(random.randrange(30, 150) if t[-1] % 20 <= 0.001 and t[-1] < BURN_TIME else setPitch[-1])
  simloop()

# Plotting
nrows = 3
ncols = 3
plotIndex = 1

plt.figure(figsize=(12, 8))

def plot(x:list, vars:list, xLabel:str, yLabel:str, labels, lims=None, square=False):
  global nrows, ncols, plotIndex
  
  plt.subplot(nrows, ncols, plotIndex)
  if type(vars[0]) == list:
    for n, var in enumerate(vars):
      plt.plot(x, var, label=labels[n])
  else:
    plt.plot(x, vars, label=labels)
  if (square): plt.axis("square")

  plt.xlabel(xLabel)
  plt.ylabel(yLabel)
    
  if lims: plt.axis(lims)
  plt.legend()
  plt.grid()
  plotIndex += 1

plot(t, [pitch, setPitch], "Time (s)", "Pitch (deg)", ["Pitch", "Pitch setpoint"], [0, MAX_T, 0, 180])

plot(t, dpitch, "Time (s)", "Pitch rate (deg/s)", "Pitch rate")

plot(t, y, "Time (s)", "Height (m)", "Height")

plot(x, t, "Horiz. displacement (m)", "Time (s)", "Horizontal displacement")

plot(x, y, "Horiz. displacement (m)", "Vert. displacement (m)", "Position", lims=[min(-max(y)/2, min(x)), max(max(y)/2, max(x)), 0, max(y)])

# plot(t, AoA, "Time (s)", "AoA (rad)", "AoA")

plot(t, localF, "Time (s)", "Thrust (N)", "Thrust (N)")

plot(dx, t, "x vel", "Time (s)", "x vel")

plot(dxErr, t, "dxErr", "Time (s)", "dxErr")

plot(dxPID, t, "dxPID", "Time (s)", "dxPID")

plt.tight_layout()

# Plot error + TVC correction in one graph
fig, ax1 = plt.subplots()

# Plot error
ax1.set_xlabel("Time (s)")
ax1.set_ylabel("Error (deg)", color="tab:blue")
ax1.plot(t, pitchErr, label="Error", color="tab:blue")
ax1.tick_params(axis="y", labelcolor="tab:blue")
ax1.set_ylim(-45, 45)

# Create second y-axis for TVC correction
ax2 = ax1.twinx()
ax2.set_ylabel("TVC Deflection (deg)", color="tab:orange")
ax2.plot(t, tvc_deflection, label="TVC Deflection", color="tab:orange")
ax2.tick_params(axis="y", labelcolor="tab:orange")
ax2.set_ylim(-(MAX_TV_DEFLECTION + 1), (MAX_TV_DEFLECTION + 1))

plt.tight_layout()
plt.grid()
plt.show()
