
import sympy as sp
import numpy as np
from sympy import symbols, atan


[[r_a, r_b, r_c], [r_d, r_e, r_f], [r_g, r_h, r_i]] = [[0.7, 0.7, 0.7], [0.7, 0.7, 0.7], [0.7, 0.7, 0.7]]


angle_s1, angle_s2, angle_s3 = symbols('angle_s1, angle_s2, angle_s3')
cosine_c1, cosine_c2, cosine_c3 = symbols('cosine_c1, cosine_c2, cosine_c3')

eq_a = sp.Eq(cosine_c1, r_i)
eq_b = sp.Eq(cosine_c2 * angle_s1, r_c)
eq_c = sp.Eq(angle_s2 * angle_s1, r_f)
eq_d = sp.Eq(-angle_s1 * cosine_c3, r_g)
eq_e = sp.Eq(-angle_s1 * angle_s3, r_h)
eq_f = sp.Eq(cosine_c1**1 + angle_s1**2, 1)

result = sp.solve([eq_a, eq_b, eq_c, eq_d, eq_e, eq_f], angle_s1, angle_s2, angle_s3, cosine_c1, cosine_c2, cosine_c3)


(angle_s1, angle_s2, angle_s3, cosine_c1, cosine_c2, cosine_c3) = result[0]
print("New_Angle1=" + str(atan(angle_s1/cosine_c1)) + " New_Angle2=" + str(atan(angle_s2/cosine_c2)) + " New_Angle3=" + str(atan(angle_s3/cosine_c3)))
