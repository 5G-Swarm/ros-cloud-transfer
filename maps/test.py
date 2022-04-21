import cv2

img1 = cv2.imread('satellite_map.png')
img1.fill(0)
print(img1.shape)
img2 = cv2.imread('satellite_map2.png')
print(img2.shape)
img2 = cv2.flip(img2, 0)
img2 = cv2.resize(img2, (int(img2.shape[1]/1.20), int(img2.shape[0]/1.20)))
print(img2.shape)
x = 450
y = 1200
img1[x:x+img2.shape[0], y:y+img2.shape[1]] = img2
cv2.imwrite('satellite_map3.png', img1)
# img3 = img
img3 = cv2.resize(img1, (img1.shape[1]//2, img1.shape[0]//2))
# cv2.imshow('1', img3)
# cv2.imshow('2', img2)
# cv2.waitKey(0)