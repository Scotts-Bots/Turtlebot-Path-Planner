from skimage import io
import numpy as np
from skimage.color import rgb2lab, lab2rgb, rgba2rgb
from skimage.morphology import thin
from skimage.transform import rescale, resize

'''IMAGE PROCESSING'''
#read in image of map
test_map = io.imread("test_mapUp2.png")

#resize image
image_resized = resize(test_map, (200, 150),anti_aliasing=True)
test_map = image_resized

#convert from rgba to rgb only
test_map_rgb = test_map[:,:,0:3]

#convert rgb to grayscale image
im_lab=rgb2lab(test_map_rgb)
im_lab[...,1]=im_lab[...,2]=0
gray_test_map_temp=lab2rgb(im_lab)

#convert grayscale image with rgb values to single intensity values
gray_test_map = np.zeros((gray_test_map_temp.shape[0],gray_test_map_temp.shape[1]))
for i in range(gray_test_map_temp.shape[0]):
    for j in range(gray_test_map_temp.shape[1]):
        gray_test_map[i][j] = np.average(gray_test_map_temp[i][j])

#binary thresholding to separate open floors from obstacles
binary_test_map = gray_test_map > 0.67

#morphological erosion to make barriers / obstacles thicker
    #binary_test_map =  np.pad(binary_test_map,100,"edge")
binary_thick_map = thin(binary_test_map, max_num_iter=7)

print(np.shape(binary_thick_map))
np.savetxt("map.txt",binary_thick_map)
io.imsave('map.png',binary_thick_map)