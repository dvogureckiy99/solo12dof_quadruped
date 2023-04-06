import numpy as np
import math
def ikin (p,angls):
    l_zfoot = 0.0159
    l_zhip_fe = 0.01965
    l_zULM = 0.051
    l_zLLM = 0.00795
    l2 = 0.16
    l3 = 0.16
    l_x_hip_aa = 0.2141
    l_y_hip_aa = 0.0875
    lCM_hip_fe = l_x_hip_aa-l_zhip_fe
    lzlowLLM = l_zLLM + l_zULM
    p=np.array([[0.00000,0.00000,0.00000],[0.00000,0.00000,0.00000],[0.00000,0.00000,0.00000],[0.00000,0.00000,0.00000]])
    for i in range (0,4):
        p[i,2] = p [i,2] + l_zfoot
        print(p)
    if nargin>1:
        if angls(3)==0:
            s_y=0
            c_y=1
        else:
            s_y=math.sin(-angls(3))
            c_y=math.cos(-angls(3))
        Rz = np.array([[c_y, -s_y, 0],
                        [s_y, c_y, 0],
                        [0, 0, 1]])
        if angls(2) == 0:
            s_p = 0
            c_p = 1
        else:
            s_p = math.sin(-angls(2))
            c_p = math.cos(-angls(2))
        Ry = np.array([[c_p, 0, s_p],
                        [0,1,0],
                        [-s_p,0,c_p]])
        if angls(1) == 0:
            s_r = 0;
            c_r = 1;
        else:
            s_r = math.sin(-angls(1));
            c_r = math.cos(-angls(1));
        Rx = np.array([[1, 0, 0],
                        [0,c_r, -s_r],
                        [0,s_r, c_r]])
        RCM_CMWF = Rz.dot(Ry).dot(Rx)
        # for i in range (0,4):
                # p1 = np.array([[RCM_CMWF], [0, 0, 0], [0, 0, 0], [0, 1]]) @ np.hstack([p[i, :], 1])
                # p[i, :] = p1[:3]

    q_1 = np.array[[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]
    q_2 = np.array[[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]
    # for i in range(4): z_foot = p[i, 2]
    #     l4 = np.sqrt(lzlowLLM ** 2 + (l2 + l3) ** 2)
    #     l23 = np.sqrt((z_foot) ** 2 + (abs(p[i, 1]) - l_y_hip_aa) ** 2)
    #     l23_2 = np.sqrt(l23 ** 2 - lzlowLLM ** 2)
    #     l_xfoot = abs(p[i, 0]) - lCM_hip_fe
    #     d_l34_1 = round(np.sqrt(l_xfoot ** 2 + l23_2 ** 2) * 1e4) / 1e4
    #     d_l34_1 = np.round(d_l34_1, 5)
    #     alpha = 2 * np.arcsin(d_l34_1 / 2 / l2)
    #     gamma = np.arctan(lzlowLLM / (l23_2))
    #     beta = np.arccos(abs(z_foot) / l23)
    # if abs(p[i, 1]) >= l_y_hip_aa and z_foot < 0:
    #     q1 = -beta + gamma
    # elif abs(p[i, 1]) >= l_y_hip_aa and z_foot >= 0:
    #     q1 = -math.pi + beta + gamma
    # else:
    #     q1 = beta + gamma
