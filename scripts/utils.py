import numpy as np


def SerializeTransform(tm):
    flatten_tm = np.array(tm[0:3, 0:4]).T.flatten()
    return ' '.join([' %.5f' % v for v in flatten_tm])


def RPY2Transform(psi, theta, phi, x, y, z):
    cPsi = np.cos(psi)
    sPsi = np.sin(psi)
    cTheta = np.cos(theta)
    sTheta = np.sin(theta)
    cPhi = np.cos(phi)
    sPhi = np.sin(phi)
    return np.mat([[cTheta * cPhi, sPsi * sTheta * cPhi - cPsi * sPhi, cPsi * sTheta * cPhi + sPsi * sPhi, x],
                   [sTheta * sPhi, sPsi * sTheta * sPhi + cPsi * cPhi, cPsi * sTheta * sPhi - sPsi * cPhi, y],
                   [-sTheta, sPsi * cTheta, cPsi * cTheta, z],
                   [0, 0, 0, 1]])


def quat2Transform(a, b, c, d, x, y, z):
    # a+bi+cj+dk
    assert 1 - 1e-4 < a ** 2 + b ** 2 + c ** 2 + d ** 2 < 1 + 1e-4
    return np.mat([[a ** 2 + b ** 2 - c ** 2 - d ** 2, 2 * b * c - 2 * a * d, 2 * b * d + 2 * a * c, x],
                   [2 * b * c + 2 * a * d, a ** 2 - b ** 2 + c ** 2 - d ** 2, 2 * c * d - 2 * a * b, y],
                   [2 * b * d - 2 * a * c, 2 * c * d + 2 * a * b, a * 2 - b ** 2 - c ** 2 + d ** 2, z],
                   [0, 0, 0, 1]])
