import math as m

# "quick" script to calculate some inverse kinematics for a two-dof target_ankle_angle
# reference: http://web.eecs.umich.edu/~ocj/courses/autorob/autorob_10_ik_closedform.pdf <-- WARNING: this has errors

def inverse_kinematics(z, x):
  print("z, x, m.acos argument", z, x, (0.2032**2 + 0.2032**2 - z**2 - x**2) / (2*0.2032*0.2032))
  gamma2 = m.acos((0.2032**2 + 0.2032**2 - z**2 - x**2) / (2*0.2032*0.2032))
  theta2 = m.pi - gamma2
  target_ankle_angle = -theta2

  print("gamma2:", gamma2)
  print("theta1", -m.atan2((0.2032*m.sin(gamma2)),(0.2032 + 0.2032*m.cos(gamma2))) + m.atan2(x,z))
  theta1 = -m.atan2((0.2032*m.sin(gamma2)), (0.2032 + 0.2032*m.cos(gamma2))) + m.atan2(x, z)
  target_knee_angle = theta1

  print("\n")
  return target_knee_angle, target_ankle_angle


z_vals = [0.0, 0.03854266666, 0.07708533332, 0.11562799998, 0.15417066664,
          0.1927133333, 0.23125599996, 0.26979866662, 0.30834133328, 0.34688399994]
x_vals = [0.143684] * 10
target_knee_angle = []
target_ankle_angle = []

for z, x in zip(z_vals, x_vals):
  knee_result, ankle_result = inverse_kinematics(z, x)
  target_knee_angle.append(knee_result)
  target_ankle_angle.append(ankle_result)

target_knee_angle = [round(x, 4) for x in target_knee_angle]  # be wary: https://docs.python.org/2/tutorial/floatingpoint.html#tut-fp-issues
target_ankle_angle = [round(x, 4) for x in target_ankle_angle]

print("Target knee angle sequence: " + str(target_knee_angle))
print("Target ankle angle sequence: " + str(target_ankle_angle))
