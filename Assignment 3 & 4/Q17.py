import numpy as np

u11 = input('u11 = ')
u13 = input('u13 = ')
u21 = input('u21 = ')
u23 = input('u23 = ')
u31 = input('u31 = ')
u32 = input('u32 = ')
u33 = input('u33 = ')

if u13 ==0 and u23 ==0:
    phi_psi = np.arctan2(u11, u21)
    ans = phi_psi
    print(ans)

else:

    theta = np.arctan2(u33, np.sqrt(1 - u33 ** 2))
    phi = np.arctan2(u13, u23)
    psi = np.arctan2(-u31, u32)
    print('theta', theta)
    print('phi', phi)
    print('psi', psi)
