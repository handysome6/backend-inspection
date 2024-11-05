import numpy as np
import cv2
import math
from .Support import B2I,I2B,PointAffineTransform,my_getAffineTransform
from .circle_detect import CircleEdgeDrawDetector
from icecream import ic


#判断给定图像中是否有CCT
def CCT_or_not(cct_bimg):
#    plt.imshow(cct_bimg)
#    plt.show()
#    print("正在判断该区域是否包含CCT......")
    sample_num=36    
    #得到cct图像的尺寸
    shape=np.shape(cct_bimg)
    height=shape[0] 
    width=shape[1]
    #得到中心点坐标，该坐标可以视为CCT中心坐标
    X0=width/2
    Y0=height/2
    #得到CCT中心⚪的半径
    r1=X0/3.0
    #存放三个圆周上采样点像素值的列表
    pixels_0_5_r=[]
#    pixels_0_7_5_r=[]
    pixels_1_5_r=[]
    pixels_2_5_r=[]
#    pixels_3_5_r=[]
    #遍历三个圆周处的采样点像素
    for j in range(sample_num):
        xi=math.cos(10.0*j/180*math.pi)
        yi=math.sin(10.0*j/180*math.pi)
        
        x_0_5_r=0.5*r1*xi+X0
        y_0_5_r=0.5*r1*yi+Y0
        
        x_1_5_r=1.5*r1*xi+X0
        y_1_5_r=1.5*r1*yi+Y0
        
        x_2_5_r=2.5*r1*xi+X0
        y_2_5_r=2.5*r1*yi+Y0
        
        #好吧，我认输，以后访问像素记住【rows,cols】，先行后列
        pixel_value_0_5_r=cct_bimg[round(y_0_5_r)][round(x_0_5_r)]
        pixel_value_1_5_r=cct_bimg[round(y_1_5_r)][round(x_1_5_r)]
        pixel_value_2_5_r=cct_bimg[round(y_2_5_r)][round(x_2_5_r)]
        
        pixels_0_5_r.append(pixel_value_0_5_r)
        pixels_1_5_r.append(pixel_value_1_5_r)
        pixels_2_5_r.append(pixel_value_2_5_r)

    if sum(pixels_0_5_r)==sample_num and sum(pixels_1_5_r)==0 and sum(pixels_2_5_r)>=2:
#        print('判断完成：True')
        return True
    elif sum(pixels_0_5_r)==0 and sum(pixels_1_5_r)==sample_num and sum(pixels_2_5_r)<=sample_num-2:
        return True
    else:
#        print('判断完成：False')
        return False
    
def CCT_Decode(cct_img,N,color):
    # plt.imshow(cct_img)
    # plt.show()
    #得到cct图像的尺寸
    shape=np.shape(cct_img)
    height=shape[0] 
    width=shape[1]
    #得到中心点坐标，该坐标可以视为CCT中心坐标
    X0=width/2
    Y0=height/2
    #得到CCT的三个半径
    r1=X0*0.333333
    #存放所有码值的list
    code_all=[]
    #如果是12位编码，那就转30圈，每圈加一度
    for j in range(int(360/N)):
        code_j=[]#存放单圈码值的list
        #以N为标准在2.5倍r1的圆环上均匀采样
        for k in range(N):
            x=2.5*r1*math.cos((360.0/N*k+j)/180*math.pi)+X0
            y=2.5*r1*math.sin((360.0/N*k+j)/180*math.pi)+Y0
            #访问像素记住【rows,cols】，先行后列
            pixel_value=cct_img[round(y)][round(x)]
            code_j.append(pixel_value)
        #将每一圈得到的编码都转换为最小编码
        temp1=B2I(code_j,N)
        temp2=I2B(temp1,N)
        code_all.append(temp2) 
    code=np.asarray(code_all)
#    print(code)
    #对列求平均
    code=np.mean(code,0)    
#    print(code)
    #对cct编码进行二值化
    result=[]         
    for i in code:
        if i>0.5:
            result.append(1)
        if i<=0.5:
            result.append(0)
    if color=='black':
        result=swap0and1(result)
    #调用DrawCCT中的解码函数进行解码
    return B2I(result,len(result))

#反转编码的函数：0->1,1->0,为黑色码带设计
def swap0and1(code_list):
    result=[]
    for i in code_list:
        if i==0:
            result.append(1)
        if i==1:
            result.append(0)
    return result

def CCT_extract(img:np.ndarray,N,color='white'):
    img = img.copy()
         
    #存放解码结果的list
    CodeTable={}
    '''
    image.shape[0], 图片垂直尺寸
    image.shape[1], 图片水平尺寸
    image.shape[2], 图片通道数
    '''
    img_shape=img.shape
    img_height=img_shape[0]
    img_width=img_shape[1]
    
#    print('img_width=',img_width)
#    print('img_height=',img_height)

    #将输入图像转换为灰度图
    detector = CircleEdgeDrawDetector(img)
    ellipse_list = detector._detect_ellipse()

    for ellipse in ellipse_list:
        #得到拟合的椭圆参数：中心点坐标，尺寸，旋转角
        center = (ellipse[0][0], ellipse[0][1])
        a, b = (int(ellipse[0][2])+int(ellipse[0][3]),int(ellipse[0][2])+int(ellipse[0][4]))
        axes = (2*a, 2*b)
        angle = ellipse[0][5]
        box1=tuple([center,axes,angle])
        #print('box1:',box1)       
        box2=tuple([box1[0],tuple([box1[1][0]*2,box1[1][1]*2]),box1[2]])
        box3=tuple([box1[0],tuple([box1[1][0]*3,box1[1][1]*3]),box1[2]])
        #求得最外层椭圆的最小外接矩形的四个顶点，顺时针方向
        minRect = cv2.boxPoints(box3)
        #计算椭圆的长轴
        a=max(box3[1][0],box3[1][1])
        s=a
        #在原图像中裁剪CCT所在的区域
        cct_roi=None
        row_min=round(box1[0][1]-s/2)
        row_max=round(box1[0][1]+s/2)
        col_min=round(box1[0][0]-s/2)
        col_max=round(box1[0][0]+s/2)
#            print('判断该ROI是否超出边界......')
#            print([row_min,row_max,col_min,col_max])
        #判断cct_roi是否超出原图像边界
        if row_min>=0 and row_max<=img_height and col_min>=0 and col_max<=img_width:
            #从原图像中将cct_roi截取出来            
            cct_roi=img[row_min:row_max,col_min:col_max]
            #cct_roi相对于原始影像的偏移量
            dx=box1[0][0]-s/2
            dy=box1[0][1]-s/2            
            #对CCT椭圆区域进行仿射变换将其变为正圆
            src=np.float32([[minRect[0][0]-dx,minRect[0][1]-dy],[minRect[1][0]-dx,minRect[1][1]-dy],
                            [minRect[2][0]-dx,minRect[2][1]-dy],[minRect[3][0]-dx,minRect[3][1]-dy],
                            [box1[0][0]-dx,box1[0][1]-dy]])
            dst=np.float32([[box1[0][0]-a/2-dx,box1[0][1]-a/2-dy],[box1[0][0]+a/2-dx,box1[0][1]-a/2-dy],
                            [box1[0][0]+a/2-dx,box1[0][1]+a/2-dy],[box1[0][0]-a/2-dx,box1[0][1]+a/2-dy],
                            [box1[0][0]-dx,box1[0][1]-dy]])
            #得到仿射变换矩阵
            #M=cv2.getAffineTransform(src,dst)
            M=my_getAffineTransform(src,dst)
            if isinstance(M,int):
                continue
            #计算仿射变换后的中心点坐标
            X0,Y0=PointAffineTransform(M,[box1[0][0]-dx,box1[0][1]-dy])
            #print('X0=',X0,'  ','Y0=',Y0)
            CCT_img=None
            #对cct_roi进行仿射变换
            cct_roi_size=np.shape(cct_roi)
            if cct_roi_size[0]>0 and cct_roi_size[1]>0:
                CCT_img=cv2.warpAffine(cct_roi,M,(round(s),round(s)))
            #print('cct img shape=',np.shape(CCT_img))
            #对仿射变换后的CCT进行缩放
            CCT_large = cv2.resize(CCT_img, (0, 0), fx=200.0/s, fy=200.0/s, interpolation=cv2.INTER_LANCZOS4)                            
            #将放大后的CCT转换为灰度图
            CCT_gray=cv2.cvtColor(CCT_large,cv2.COLOR_BGR2GRAY)
#           #对该灰度图进行自适应二值化
            retval,CCT_bina=cv2.threshold(CCT_gray,0,1,cv2.THRESH_BINARY+cv2.THRESH_OTSU)            
            kernel=cv2.getStructuringElement(cv2.MORPH_RECT,(3,3))
#           #执行腐蚀
            CCT_eroded=cv2.erode(CCT_bina,kernel)
            #plt.imshow(CCT_bina)
            #plt.show()
            #判断这个区域里是不是CCT
            if CCT_or_not(CCT_eroded):
                #调用解码函数进行解码
                code=CCT_Decode(CCT_eroded,N,color)
                CodeTable[code]=[box1[0][0],box1[0][1]]
                #将编码在原图像中绘制出来.各参数依次是：图片，添加的文字，左上角坐标，字体，字体大小，颜色，字体粗细
                cv2.putText(img,str(code),(int(box3[0][0]-0.25*s),int(box1[0][1]+0.5*s)),cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 2)
                #绘制拟合出的椭圆
                cv2.ellipse(img,box1,(0,255,0),1)
                cv2.ellipse(img,box2,(0,255,0),1)
                cv2.ellipse(img,box3,(0,255,0),1)   
            else:
                pass
                # print('该区域不包含CCT')
                # print(box1)
                
                # show
                # cv2.ellipse(img,box1,(0,0,255),1)
                # cv2.ellipse(img,box2,(0,0,255),1)
                # cv2.ellipse(img,box3,(0,0,255),1)
                # cv2.namedWindow('img', cv2.WINDOW_NORMAL)
                # cv2.imshow('img', img)
                # cv2.waitKey(0)
    return CodeTable,img


if __name__ == "__main__":
    img = cv2.imread(r"C:\workspace\data\2.85m\2.85m_2\4\img\Image.png")
    N=8
    color='white'
    CodeTable, res_img = CCT_extract(img,N,color)
    from icecream import ic
    ic(CodeTable)
    # imshow
    cv2.namedWindow('img', cv2.WINDOW_NORMAL)
    cv2.imshow('img', res_img)
    cv2.waitKey(0)