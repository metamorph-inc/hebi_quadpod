import math as m

# "quick" script to calculate some inverse kinematics for a three-dof
# reference: http://web.eecs.umich.edu/~ocj/courses/autorob/autorob_10_ik_closedform.pdf <-- WARNING: this has errors

def hip_to_knee_coord_xform(x0, y0, z0, theta, thigh_len):
  x1, yz, z1 = None, None, None
  if theta >= 0:
    x1 = -x0*m.cos(theta) + y0*m.sin(theta)
    y1 = x0*m.sin(theta) + y0*m.cos(theta) - thigh_len
    z1 = z0
  else:
    x1 = x0*m.cos(theta) + y0*m.sin(theta)
    y1 = x0*m.sin(theta) + y0*m.cos(theta) - thigh_len
    z1 = z0
  return x1, y1, z1

def inverse_kinematics(x0_f, y0_f, z0_f, thigh_len, shin_len, foot_len):
  print("\n")
  print("x0_f, y0_f, z0_f", x0_f, y0_f, z0_f)
  theta1 = m.atan2(x0_f, y0_f)
  print("theta1:", theta1)
  x1_f, y1_f, z1_f = hip_to_knee_coord_xform(x0_f, y0_f, z0_f, theta1, thigh_len)
  print("x1_f, y1_f, z1_f:", x1_f, y1_f, z1_f)
  x0_k = thigh_len*m.sin(theta1)
  y0_k = thigh_len*m.cos(theta1)
  z0_k = 0.0
  print("x0_k, y0_k, z0_k:", x0_k, y0_k, z0_k)
  x1_k, y1_k, z1_k = hip_to_knee_coord_xform(x0_k, y0_k, z0_k, theta1, thigh_len)
  print("x1_k, y1_k, z1_k:", x1_k, y1_k, z1_k)
  phi3 = m.acos((shin_len**2 + foot_len**2 - (y1_f - y1_k)**2 - (z1_f - z1_k)**2) / (2*shin_len*foot_len))
  theta3 = m.pi - phi3
  theta2 = m.atan2(-(z1_f - z1_k), y1_f - y1_k) - m.atan2(shin_len*m.sin(theta3), shin_len + foot_len*m.cos(theta3))
  hip_angle, knee_angle, ankle_angle = theta1, theta2, theta3
  return hip_angle, knee_angle, ankle_angle

# Do some quick trig to figure out your start/in-between/stop points
# It's your job to avoid out-of-reach coordinates / problem cases
x_end_eff_vals = [-0.0, -0.00757, -0.01514, -0.02271, -0.03028, -0.03784, -0.04541, -0.05298, -0.06055, -0.06812]
y_end_eff_vals = [0.34688] * 10
z_end_eff_vals = [-0.346884] * 10
thigh_len = 0.2032
shin_len = 0.2032
foot_len = 0.2032

target_hip_angle = []
target_knee_angle = []
target_ankle_angle = []

for x, y, z in zip(x_end_eff_vals, y_end_eff_vals, z_end_eff_vals):
  hip_result, knee_result, ankle_result = inverse_kinematics(x, y, z, thigh_len, shin_len, foot_len)
  target_hip_angle.append(hip_result)
  target_knee_angle.append(knee_result)
  target_ankle_angle.append(ankle_result)


target_hip_angle = [round(x, 4) for x in target_hip_angle]  # be wary: https://docs.python.org/2/tutorial/floatingpoint.html#tut-fp-issues
target_knee_angle = [round(x, 4) for x in target_knee_angle]
target_ankle_angle = [round(x, 4) for x in target_ankle_angle]

print("Target hip angle sequence: " + str(target_hip_angle))
print("Target knee angle sequence: " + str(target_knee_angle))
print("Target ankle angle sequence: " + str(target_ankle_angle))
