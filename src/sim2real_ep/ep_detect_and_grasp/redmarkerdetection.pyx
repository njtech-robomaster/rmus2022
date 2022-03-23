import numpy as np
import cv2
cimport numpy as np
cimport cython

DTYPE = np.uint8
ctypedef np.uint8_t DTYPE_t
ctypedef np.int DTYPE_i

cdef unsigned char absSub(unsigned char v1, unsigned char v2):
    return v1-v2 if v1>v2 else v2-v1

@cython.boundscheck(False)
@cython.wraparound(False)

# instruction:
# python3 setup.py build_ext --inplace
# python3 detector_python.py

def red_segmentation(np.ndarray[DTYPE_t, ndim=2] image,np.ndarray[DTYPE_t, ndim=3] hsv_image,np.ndarray[DTYPE_t, ndim=1] seg_papram):

    cdef int height, width, i, j
    height = image.shape[0]
    width = image.shape[1]

    hsv_0 = 0
    hsv_1 = 0
    hsv_2 = 0

    for i in range(height):
        for j in range(width):
            hsv_0 = hsv_image[i,j,0]
            hsv_1 = hsv_image[i,j,1]
            hsv_2 = hsv_image[i,j,2]

            if (not((((hsv_0 >= seg_papram[0]) and (hsv_0 <= seg_papram[1]))
                 or (hsv_0 >= seg_papram[2]) and (hsv_0 <= seg_papram[3])) 
                 and (hsv_2>=seg_papram[4]) and (hsv_1>=seg_papram[5]))):
                image[i,j] = 0
            else:
                image[i,j] = 255
                
def img_sum(np.ndarray[DTYPE_t, ndim=2] image):
    cdef int height, width, i, j
    height = image.shape[0]
    width = image.shape[1]
    cdef int sum = 0
    for i in range(height):
        for j in range(width):
            sum += image[i,j]
    return sum

class point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

def cross(x1,y1,x2,y2):
    return x1*y2-x2*y1

def compare(a=point(0,0),b=point(0,0),c=point(0,0)):
    return cross((b.x-a.x),(b.y-a.y),(c.x-a.x),(c.y-a.y))

def cmp2(a,b):
    c = point(0,0)
    if(compare(c,a,b)==0):
        return a.x<b.x
    else:
        return compare(c,a,b)>0

def sort_contour(cnt):

    if not len(cnt) == 4:
        return None

    cx = (cnt[0,0,0]+cnt[1,0,0]+cnt[2,0,0]+cnt[3,0,0])/4
    cy = (cnt[0,0,1]+cnt[1,0,1]+cnt[2,0,1]+cnt[3,0,1])/4

    cnt_norm = cnt.copy()
    for i in range(4):
        cnt_norm[i,0,0] = cnt[i,0,0] - cx
        cnt_norm[i,0,1] = cnt[i,0,1] - cy

    for t in range(10):
        for i in range(3):
            p1 = point(cnt_norm[i,0,0],cnt_norm[i,0,1])
            p2 = point(cnt_norm[i+1,0,0],cnt_norm[i+1,0,1])
            if cmp2(p1,p2):
                cnt_norm[i,0,0],cnt_norm[i+1,0,0] = cnt_norm[i+1,0,0],cnt_norm[i,0,0]
                cnt_norm[i,0,1],cnt_norm[i+1,0,1] = cnt_norm[i+1,0,1],cnt_norm[i,0,1]

    for i in range(4):
        cnt_norm[i,0,0] = cnt_norm[i,0,0] + cx
        cnt_norm[i,0,1] = cnt_norm[i,0,1] + cy

    return cnt_norm

templates = []

def load_template():
    tpl_path = "./tpl/"
    for i in range(6):
        tpl = cv2.imread(tpl_path + str(i) + ".png", 0)
        print(tpl.shape)
        templates.append(tpl)

def marker_detection(np.ndarray[DTYPE_t, ndim=3] frame,np.ndarray[DTYPE_t, ndim=1] seg_papram):

    r = 0.045
    model_object = np.array([(0-0.5*r,0-0.5*r, 0.0),
                            (r-0.5*r, 0-0.5*r, 0.0),
                            (r-0.5*r, r-0.5*r, 0.0),
                            (0-0.5*r, r-0.5*r, 0.0)])
    camera_matrix = np.array([
                 (617.3054000792732, 0.0, 424.0),
                 (0.0, 617.3054000792732, 240.0),
                 (0,0,1)],
				 dtype="double")
    dist_coeffs = np.array([[0,0,0,0]], dtype="double")

    hsvImg = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    grayImg = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    match_threshold = 150000

    red_segmentation(grayImg,hsvImg,seg_papram)

    contours, hierarchy = cv2.findContours(grayImg,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)  

    quads = []
    quads_f = []
    quads_prj = []

    for i in range(len(contours)):
        cnt = contours[i]
        area = cv2.contourArea(cnt)
        bbox = cv2.boundingRect(cnt)
        if area >=30 :
            approx = cv2.approxPolyDP(cnt,15,True)
            if len(approx) == 4:
                approx_sort = sort_contour(approx)
                quads.append(approx_sort)
                quads_f.append(approx_sort.astype(float))

    rvec_list = []
    tvec_list = []
    for i in range(len(quads_f)):
        model_image = np.array([(quads_f[i][0,0,0],quads_f[i][0,0,1]),
                                (quads_f[i][1,0,0],quads_f[i][1,0,1]),
                                (quads_f[i][2,0,0],quads_f[i][2,0,1]),
                                (quads_f[i][3,0,0],quads_f[i][3,0,1])])
        ret, rvec, tvec = cv2.solvePnP(model_object, model_image, camera_matrix, dist_coeffs)
        projectedPoints,_ = cv2.projectPoints(model_object, rvec, tvec, camera_matrix, dist_coeffs)

        err = 0
        for t in range(len(projectedPoints)):
            err += np.linalg.norm(projectedPoints[t]-model_image[t])

        area = cv2.contourArea(quads[i])
        if err/area < 0.005:
            quads_prj.append(projectedPoints.astype(int))
            rvec_list.append(rvec)
            tvec_list.append(tvec)

    quads_prj_draw = []
    quads_ID = []
    for i in range(len(quads_prj)):
        points_src = np.array([[(quads_prj[i][0,0,0],quads_prj[i][0,0,1])],
                                [(quads_prj[i][1,0,0],quads_prj[i][1,0,1])],
                                [(quads_prj[i][2,0,0],quads_prj[i][2,0,1])],
                                [(quads_prj[i][3,0,0],quads_prj[i][3,0,1])]],dtype = "float32")
        points_dst = np.array([[50,0],
                                [0,0],
                                [0,50],
                                [50,50]],dtype = "float32")
        M = cv2.getPerspectiveTransform(points_src,points_dst)
        out_img = cv2.warpPerspective(frame,M,(50,50))
        out_img = cv2.cvtColor(out_img, cv2.COLOR_BGR2GRAY)
        out_img = cv2.threshold(out_img, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)[1]

        match_candidate = []
        match_candidate.append(out_img)
        match_candidate.append(cv2.rotate(out_img, cv2.ROTATE_180))
        match_candidate.append(cv2.rotate(out_img, cv2.ROTATE_90_CLOCKWISE))
        match_candidate.append(cv2.rotate(out_img, cv2.ROTATE_90_COUNTERCLOCKWISE))

        min_diff = 100000000000
        min_diff_target = 0

        for t in range(6):
            for tt in range(4):
                diff_img = cv2.absdiff(templates[t], match_candidate[tt])
                sum = img_sum(diff_img)
                if min_diff > sum:
                    min_diff = sum
                    min_diff_target = t

        if min_diff < match_threshold:
            quads_ID.append(min_diff_target)
            quads_prj_draw.append(quads_prj[i])
        else:
            quads_ID.append(-1)

    for i in range(len(quads_prj)):
        bbox = cv2.boundingRect(quads_prj[i])
        if quads_ID[i] == 0:
            cv2.putText(frame, '1', (bbox[0], bbox[1]), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 2)
        elif quads_ID[i] == 1:
            cv2.putText(frame, '2', (bbox[0], bbox[1]), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 2)
        elif quads_ID[i] == 2:
            cv2.putText(frame, '3', (bbox[0], bbox[1]), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 2)
        elif quads_ID[i] == 3:
            cv2.putText(frame, 'B', (bbox[0], bbox[1]), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 2)
        elif quads_ID[i] == 4:
            cv2.putText(frame, 'O', (bbox[0], bbox[1]), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 2)
        elif quads_ID[i] == 5:
            cv2.putText(frame, 'X', (bbox[0], bbox[1]), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 2)

    cv2.drawContours(frame,quads_prj_draw,-1,(0,255,0),3)

    return quads_ID,tvec_list,rvec_list