import pandas as pd
import numpy as np
import cv2 as cv
from scipy.interpolate import interp1d
from sklearn.model_selection import train_test_split
import pickle

# specified FoV in cone detection
MAX_H_ANGLE = 80
MIN_H_ANGLE = -80
H_RESOLUTION = 0.1  # (0.1 - 0.4)
NUM_OF_POINTS_H = (MAX_H_ANGLE - MIN_H_ANGLE) / H_RESOLUTION

# from lidar specs
MAX_V_ANGLE = 15
MIN_V_ANGLE = -15
V_RESOLUTION = 2
NUM_OF_LASER_LEVELS_V = int((MAX_V_ANGLE - MIN_V_ANGLE) / V_RESOLUTION)

IMG_ROWS = NUM_OF_LASER_LEVELS_V    # 15
IMG_COLS = 12

slope_vert = IMG_ROWS / (MIN_V_ANGLE - MAX_V_ANGLE)

data = pd.read_parquet('cones.prqt')
colors = [None, 'yellow', 'blue', 'orange']
intensity_interp = interp1d([0, 100], [0, 255])


def to_image(cone_cloud_df):
    # -- vertical pos --
    cone_cloud_df['vert_angle'] = np.degrees(np.arctan2(cone_cloud_df['z'], np.sqrt(pow(cone_cloud_df['x'], 2.0) +
                                                                                    pow(cone_cloud_df['y'], 2.0))))

    cone_cloud_df['img_pos_v'] = (np.round(slope_vert * (cone_cloud_df['vert_angle'] - MIN_V_ANGLE))).astype(int)

    # -- horizontal  pos --
    cone_cloud_df['horiz_angle'] = np.degrees(np.arctan2(cone_cloud_df['y'], cone_cloud_df['x']))
    min_horiz_angle = np.min(cone_cloud_df['horiz_angle'])
    max_horiz_angle = np.max(cone_cloud_df['horiz_angle'])
    # horizontal slope is dependent on range of current angle, cuz horizontal resolution of lidar in not regular,
    # so when it is calculated like vertical one, there are blank pixels in between points (horizontally), which
    # I think could be a problem while teaching neural net.
    slope_horiz = (IMG_COLS - 1) / (max_horiz_angle - min_horiz_angle)
    cone_cloud_df['img_pos_h'] = (np.round(slope_horiz * (cone_cloud_df['horiz_angle'] - min_horiz_angle))).astype(int)

    image = np.zeros((IMG_ROWS, IMG_COLS, 1), np.uint8)

    image[cone_cloud_df['img_pos_v'], cone_cloud_df['img_pos_h'], 0] = intensity_interp(cone_cloud_df['intensity'])

    return image


def create_dataset(images, labels):
    labels = np.array(labels)
    x_train, x_test, y_train, y_test = train_test_split(images, labels, test_size=0.3, random_state=42, stratify=labels)
    dataset = {'train': {'x': x_train, 'y': y_train}, 'test': {'x': x_test, 'y': y_test}}
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
    imgs = []
    for data_row in data.iterrows():
        data_row = data_row[1]
        img = to_image(data_row)
        imgs.append(img)
        show_img_and_wait(img)

    # dataset = create_dataset(imgs, data['color'])
    # save_dataset(dataset, '2nd_try')