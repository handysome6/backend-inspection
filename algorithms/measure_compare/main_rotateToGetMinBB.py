import cloudComPy as cc
from icecream import ic
import numpy as np
cc.initCC()

def vis_cloudcompy(clouds):
    for cloud in clouds:
        cc.addToRenderScene(cloud)
    cc.setOrthoView()

    # cc.setCustomView((0, 0, 1), (0, -1, 0))
    cc.setGlobalZoom()
    cc.setTopView()

    cc.setBackgroundColor(False, 255, 255, 255)
    cc.render("cc_render.png", 3000,3000, isInteractive=True)

def rotate_cloud_z_axis(cloud, degree):
    transform1 = cc.ccGLMatrix()
    transform1.initFromParameters(degree * np.pi/180, (0, 0, 1), (0, 0, 0))
    cloud.applyRigidTransformation(transform1)
    return cloud

def get_bb_dim(bb):
    max_corner = bb.maxCorner()
    min_corner = bb.minCorner()
    return ((max_corner[0] - min_corner[0])*1000, 
            (max_corner[1] - min_corner[1])*1000, 
            (max_corner[2] - min_corner[2])*1000)


def rotate_and_get_bb_dim(cloud, degree_start, degree_end, step):
    # rotate the cloud to the start degree
    rotate_cloud_z_axis(cloud, degree_start)
    # 保存结果和对应角度
    result = []
    # 根据给定的起始角度、结束角度和步长进行旋转
    num_steps = int((degree_end - degree_start) / step)
    for i in range(num_steps):
        current_degree = degree_start + i * step
        # print("当前角度: ", f"{current_degree:.2f}")
        rotate_cloud_z_axis(cloud, step)
        bb = cloud.getOwnBB()         # 获取边界框
        x, y, z = get_bb_dim(bb)
        # print(f"x: {x:.4f}, y: {y:.4f}, z: {z:.4f}")
        result.append((current_degree, x, y, z))
    xs = [item[1] for item in result]
    ys = [item[2] for item in result]
    degrees = [item[0] for item in result]
    
    argmax_x = np.argmax(xs)
    # argmax_y = np.argmax(ys)

    xs_argmax_x = xs[argmax_x]      
    # ys_argmax_x = ys[argmax_x]   
    # xs_argmax_y = xs[argmax_y]
    # ys_argmax_y = ys[argmax_y]

    degree_argmax_x = degrees[argmax_x]
    # degree_argmax_y = degrees[argmax_y]

    ic(xs_argmax_x, degree_argmax_x)
    return xs_argmax_x, degree_argmax_x
