"""独立复算 MATLAB 候选模型、float32 调度闭环和旧系数的对应关系。
运行：python tests/check_lqr.py（需要 numpy / scipy）。
"""
import json
from pathlib import Path
import numpy as np
from scipy.linalg import expm, solve_discrete_are, solve_continuous_are

ROOT = Path(__file__).resolve().parents[1]
M, m, r, D, Iw, Iyaw, g = 0.4011 - 2 * 0.0594, 0.0594, 0.0263, 0.0571, 4.746798875e-6, 2.924e-4, 9.81
samples = np.array([
    [-2.062074067, 30.581185585], [-2.344496804, 35.948556341],
    [-2.644809395, 41.285729291], [-2.95266012, 47.122124913],
    [-3.206381809, 52.954859608], [-3.436791379, 60.726054458],
    [-3.592282470, 68.957943589], [-3.870786882, 76.847155053],
    [-4.338690063, 81.641435889]])
jc = np.polyfit(np.linalg.norm(samples, axis=1) * 1e-3,
    np.array([310700,315200,320500,327000,334300,345200,358300,372200,381300]) * 1e-9, 3)
Q = np.diag(1 / np.array([np.deg2rad(5), np.deg2rad(150), .30, 2., .08, .30]) ** 2)
Ru = np.eye(2) / .020 ** 2


def model(L):
    J = np.polyval(jc, L)
    den = 2*J*Iw + 2*Iw*L*L*M + J*M*r*r + 2*J*r*r*m + 2*L*L*M*r*r*m
    A, B = np.zeros((6,6)), np.zeros((6,2))
    A[0,1] = 1
    A[1,0] = L*M*g*(2*Iw + M*r*r + 2*r*r*m)/den
    A[2,0] = -L*L*M*M*r*r*g/den
    A[4,2], A[5,3] = -1, -1
    B[1,:] = (2*Iw + M*r*r + 2*r*r*m + L*M*r)/den
    B[2,:] = -r*(M*L*L + M*r*L + J)/den
    c = D*r/(m*D*D*r*r + Iw*D*D + 2*Iyaw*r*r)
    B[3,:] = [c, -c]
    zoh = expm(np.block([[A, B], [np.zeros((2,8))]]) * .001)
    return zoh[:6,:6], zoh[:6,6:]


heights = np.linspace(.031,.082,200)
gains = []
for height in heights:
    Ad, Bd = model(height)
    P = solve_discrete_are(Ad, Bd, Q, Ru)
    gains.append(-np.linalg.solve(Ru + Bd.T@P@Bd, Bd.T@P@Ad))
gains = np.array(gains)
poly = np.empty((2,6,4))
for side in range(2):
    for state in range(6):
        y = gains[:,side,state]
        poly[side,state] = [0,0,0,y.mean()] if np.ptp(y) < 1e-9 else np.polyfit(heights,y,3)
poly = poly.astype(np.float32)
worst_radius = 0
worst_error = 0
# 更密的独立高度采样，按固件 Horner 运算顺序和 float32 验证。
for height in np.linspace(.031,.082,1001):
    Ad, Bd = model(height)
    h = np.float32(height)
    fitted = ((poly[:,:,0]*h + poly[:,:,1])*h + poly[:,:,2])*h + poly[:,:,3]
    radius = np.max(np.abs(np.linalg.eigvals(Ad + Bd @ fitted)))
    worst_radius = max(worst_radius, radius)
    assert radius < 1, (height, radius)
for i, height in enumerate(heights):
    fitted = np.array([[np.polyval(poly[s,c],height) for c in range(6)] for s in range(2)])
    worst_error = max(worst_error, np.max(np.abs(fitted-gains[i])))
assert np.allclose(poly[0,:3],poly[1,:3])
assert np.allclose(poly[0,[3,5]],-poly[1,[3,5]])
result = {'version':1,'balance':{'gain_poly':poly.tolist(), 'height_min_m':.031,
    'height_max_m':.082,'model_height_m':.048,'torque_scale':1},'motor':{'torque_limit_Nm':.025}}
output = ROOT/'docs/gain_poly_candidate.json'
output.write_text(json.dumps(result,indent=2)+'\n')
print(f'1001 heights stable; max pole radius={worst_radius:.9f}; max fit error={worst_error:.6g}')
print('At 0.048 m:',np.array([[np.polyval(poly[s,c],.048) for c in range(6)] for s in range(2)]))
print('Candidate JSON:',output)
