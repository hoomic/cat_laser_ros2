import numpy as np
import cv2

MIN_MATCH_COUNT = 10
sift = cv2.SIFT_create()

def get_difference_image(img1, img2):
  try:
    #Since the camera can move, first try to fix the perspective between the two images
    # find the keypoints and descriptors with SIFT
    kp1, des1 = sift.detectAndCompute(img1,None)
    kp2, des2 = sift.detectAndCompute(img2,None)
    if des1 is None or des2 is None:
      return None
    FLANN_INDEX_KDTREE = 1
    index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
    search_params = dict(checks = 50)
    flann = cv2.FlannBasedMatcher(index_params, search_params)
    matches = flann.knnMatch(des1,des2,k=2)
    # store all the good matches as per Lowe's ratio test.
    good = []
    for m,n in matches:
        if m.distance < 0.7*n.distance:
            good.append(m)
    if len(good)>MIN_MATCH_COUNT:
      src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
      dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

      # find the homography between the first and second frame
      M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
      
      # warp the first frame so that it has the same perspective as the second frame
      warped = cv2.warpPerspective(img1, M, (img1.shape[1], img1.shape[0]))

      # now that the images "line up" take the absolute difference
      absdiff = cv2.absdiff(warped, img2)

      # Set the locations that weren't warped to 0
      absdiff[warped == 0] = 0

      # finally use gaussian blur to remove noisy differences
      return cv2.GaussianBlur(absdiff, (5,5), 5)
    else:
      return None
  except Exception as e:
    print("The following exception occurred in get_difference_image:")
    print(e.what())
    return None