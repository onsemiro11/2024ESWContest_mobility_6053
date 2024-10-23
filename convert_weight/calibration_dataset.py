import cv2
import os
import numpy as np

w, h, c = 640, 640, 3

data_dir = './road_mark/training'

images_list = os.listdir(data_dir)

calib_dataset = np.zeros((len(images_list), w, h, c))

for idx, filename in enumerate(os.listdir(data_dir)):
    if not filename.endswith('.jpg'):
        continue
    filepath = os.path.join(data_dir, filename)
    img = cv2.imread(filepath)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    x = img.shape[1]/2 - w/2
    y = img.shape[0]/2 - h/2
    crop_img = img[int(y):int(y+h), int(x):int(x+w)]
    calib_dataset[idx, :, :, :] = crop_img

np.save('./road_mark/calib_dataset.npy', calib_dataset)
