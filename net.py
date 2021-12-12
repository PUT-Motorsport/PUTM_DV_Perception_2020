import pandas as pd
import numpy as np
import cv2 as cv
import math
from scipy.interpolate import interp1d
from sklearn.model_selection import train_test_split
import pickle


SCANNER_COLS = 400
SCANNER_ROWS = 15

MAX_V_ANGLE = 15
MIN_V_ANGLE = -15

slope_vert = SCANNER_ROWS / (MIN_V_ANGLE - MAX_V_ANGLE)
# slope_horiz = SCANNER_COLS / (80.0 - (-80.0))
slope_horiz = SCANNER_COLS / (80.0 - (-80.0))
slope_range = 255.0 / 50.0

MAX_V_ANGLE = 15
MIN_V_ANGLE = -15

data = pd.read_parquet('cones.prqt')
colors = [None, 'yellow', 'blue', 'orange']
intensity_interp = interp1d([0, 100], [0, 255])


def to_image(data_row):
    heights, widths = [], []

    slope_vert = len(data_row['x']) / (MIN_V_ANGLE - MAX_V_ANGLE)

    for x, y, z in zip(data_row['x'], data_row['y'], data_row['z']):
        # Calculate horizontal and vertical angle of the laser beam
        vert_angle = math.degrees(math.atan2(z, np.sqrt(pow(x, 2.0) + pow(y, 2.0))))
        horiz_angle = math.degrees(math.atan2(y, x))

        # Calculate the pixel location in the image
        # heights.append(int(np.round(slope_vert * (vert_angle - MAX_V_ANGLE))))
        # widths.append(int(np.round(SCANNER_COLS - (slope_horiz * (horiz_angle - (-80.0))))))

        heights.append(int(np.round(slope_vert * (vert_angle - MAX_V_ANGLE))))
        widths.append(int(np.round(slope_horiz * (horiz_angle - (-80.0)))))

    heights = np.array(heights)
    widths = np.array(widths)

    widths = widths - np.min(widths)
    heights = heights - np.min(heights)

    img = np.zeros((np.max(heights) + 1, np.max(widths) + 1, 1), np.uint8)

    for height, width, intens in zip(heights, widths, data_row['intensity']):
        img[height, width] = intensity_interp(intens)

    return img


def create_dataset(imgs, labels):
    labels = np.array(labels)
    X_train, X_test, y_train, y_test = train_test_split(imgs, labels, test_size=0.3, random_state=42, stratify=labels)
    dataset = {}
    dataset['train'] = {'X': X_train, 'y': y_train}
    dataset['test'] = {'X': X_test, 'y': y_test}
    return dataset


def save_dataset(dataset, dataset_name):
    with open(dataset_name + '.pkl', 'wb') as file:
        pickle.dump(dataset, file)


def load_dataset(dataset_name):
    with open(dataset_name + '.pkl', 'rb') as file:
        dataset = pickle.load(file)
    return dataset


def show_img_and_wait(img):
    cv.imshow('image', img)

    while True:
        key = cv.waitKey(0)
        if key == 27:
            cv.destroyAllWindows()
            break


if __name__ == '__main__':
    i = 0
    imgs = []
    for data_row in data.iterrows():
        data_row = data_row[1]
        img = to_image(data_row)
        imgs.append(img)
    #     # if i >= 15:
    #     #     break
    #     # i += 1
        show_img_and_wait(img)
    #
    # dataset = create_dataset(imgs, data['color'])
    # save_dataset(dataset, '1st_try')
