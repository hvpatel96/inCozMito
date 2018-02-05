import numpy as np
import re
from sklearn import svm, metrics, tree, neighbors
from skimage import io, feature, filters, exposure, color
from sklearn.ensemble import GradientBoostingClassifier

imageClf = None

class ImageClassifier:

    def __init__(self):
        self.classifer = None

    def imread_convert(self, f):
        return io.imread(f).astype(np.uint8)

    def load_data_from_folder(self, dir):
        # read all images into an image collection
        ic = io.ImageCollection(dir+"*.bmp", load_func=self.imread_convert)

        #create one large array of image data
        data = io.concatenate_images(ic)

        #extract labels from image names
        labels = np.array(ic.files)
        for i, f in enumerate(labels):
            m = re.search("_", f)
            labels[i] = f[len(dir):m.start()]

        return(data,labels)

    def extract_image_features(self, data):
        l = []
        for im in data:
            im_gray = color.rgb2gray(im)

            im_gray = filters.gaussian(im_gray, sigma=0.4)

            f = feature.hog(im_gray, orientations=10, pixels_per_cell=(48, 48), cells_per_block=(4, 4), feature_vector=True, block_norm='L2-Hys')
            l.append(f)


        feature_data = np.array(l)
        return(feature_data)

    def train_classifier(self, train_data, train_labels):
        self.classifer = svm.LinearSVC()
        self.classifer.fit(train_data, train_labels)

    def predict_labels(self, data):
        predicted_labels = self.classifer.predict(data)
        return predicted_labels

def startUp():
    print("Creating and training classifier")
    img_clf = ImageClassifier()

    # load images
    (train_raw, train_labels) = img_clf.load_data_from_folder('./train/')
    # (test_raw, test_labels) = img_clf.load_data_from_folder('./test/')

    # convert images into features
    train_data = img_clf.extract_image_features(train_raw)
    # test_data = img_clf.extract_image_features(test_raw)

    # train model
    img_clf.train_classifier(train_data, train_labels)

    # test model
    print("Classifier Trained")
    return img_clf

''' Take a picture

Takes most recent image from Cozmo
Converts it to 8-bit black and white
Saves to destination
'''

import sys
import cozmo
import datetime
import time
import os
from cozmo.util import degrees, distance_mm, speed_mmps

class StateMachine:
    def run(self):
        startUp()
        self.state = Idle
        while True:
            self.state.run().run()

class State(object):
    def run(self):
        assert 0
    def next(self):
        assert 0

class Idle(State):
    def run(self):
        robot = sdk_conn.wait_for_robot()
        robot.camera.image_stream_enabled = True
        robot.camera.color_image_enabled = False
        robot.camera.enable_auto_exposure()

        robot.set_head_angle(cozmo.util.degrees(0)).wait_for_completed()

        # myargs = sys.argv[1:]

        # if len(myargs) <= 1:
        #     sys.exit("Incorrect arguments")

        num_images_per_type = 5  # number of images to take of each type of object
        imgType = "Environment"

        print("Taking ", num_images_per_type, "images each of ", imgType)

        for i in range(num_images_per_type):
            robot.set_head_angle(cozmo.util.degrees(0)).wait_for_completed()
            time.sleep(.5)
            latest_image = robot.world.latest_image
            new_image = latest_image.raw_image

            robot.say_text("Taking Picture").wait_for_completed()

            timestamp = datetime.datetime.now().strftime("%dT%H%M%S%f")

            new_image.save("./test/" + str(imgType) + "_" + timestamp + ".bmp")

            print("Predicting: ", end="")
            imgArr = [np.asarray(new_image)]
            # Asking for label of new image
            # (test_raw, test_labels) = imageClf.load_data_from_folder('./test/')
            test_data = imageClf.extract_image_features(imgArr)
            predicted_labels = imageClf.predict_labels(test_data)

            for label in predicted_labels:
                print(label)
                robot.say_text(label).wait_for_completed()
                if label == "drone":
                    return Drone
                elif label == "order":
                    return Order
                elif label == "inspection":
                    return Inspection
                else:
                    return Idle

class Drone(State):
    def run(self):

class Order(State):
    def run(self):
        robot.drive_wheels(20, 80)
        time.sleep(10)

class Inspection(State):
    def run(self):
        robot.drive_wheels(20, 80)
        time.sleep(10)

# def run(sdk_conn):
    # robot = sdk_conn.wait_for_robot()
    # robot.camera.image_stream_enabled = True
    # robot.camera.color_image_enabled = False
    # robot.camera.enable_auto_exposure()

    # robot.set_head_angle(cozmo.util.degrees(0)).wait_for_completed()

    # # myargs = sys.argv[1:]

    # # if len(myargs) <= 1:
    # #     sys.exit("Incorrect arguments")

    # num_images_per_type = 5  # number of images to take of each type of object
    # imgType = "Environment"

    # print("Taking ", num_images_per_type, "images each of ", imgType)

    # for i in range(num_images_per_type):
    #     robot.set_head_angle(cozmo.util.degrees(0)).wait_for_completed()
    #     time.sleep(.5)
    #     latest_image = robot.world.latest_image
    #     new_image = latest_image.raw_image

    #     robot.say_text("Taking Picture").wait_for_completed()

    #     timestamp = datetime.datetime.now().strftime("%dT%H%M%S%f")

    #     new_image.save("./test/" + str(imgType) + "_" + timestamp + ".bmp")

    #     print("Predicting: ", end="")
    #     imgArr = [np.asarray(new_image)]
    #     # Asking for label of new image
    #     # (test_raw, test_labels) = imageClf.load_data_from_folder('./test/')
    #     test_data = imageClf.extract_image_features(imgArr)
    #     predicted_labels = imageClf.predict_labels(test_data)

    #     for label in predicted_labels:
    #         print(label)
    #         robot.say_text(label).wait_for_completed()
    #         if label == "drone":


        # Cleans up test directory
        # os.system("del test\*.bmp")

    # robot = sdk_conn.wait_for_robot()
    # robot.camera.image_stream_enabled = True
    # robot.camera.color_image_enabled = False
    # robot.camera.enable_auto_exposure()

    # robot.set_head_angle(cozmo.util.degrees(0)).wait_for_completed()

    # # myargs = sys.argv[1:]

    # # if len(myargs) <= 1:
    # #     sys.exit("Incorrect arguments")

    # num_images_per_type = 5  # number of images to take of each type of object
    # imgType = "Environment"

    # print("Taking ", num_images_per_type, "images each of ", imgType)

    # for i in range(num_images_per_type):
    #     robot.set_head_angle(cozmo.util.degrees(0)).wait_for_completed()
    #     time.sleep(.5)
    #     latest_image = robot.world.latest_image
    #     new_image = latest_image.raw_image

    #     robot.say_text("Taking Picture").wait_for_completed()

    #     timestamp = datetime.datetime.now().strftime("%dT%H%M%S%f")

    #     new_image.save("./test/" + str(imgType) + "_" + timestamp + ".bmp")

    #     print("Predicting: ", end="")
    #     imgArr = [np.asarray(new_image)]
    #     # Asking for label of new image
    #     # (test_raw, test_labels) = imageClf.load_data_from_folder('./test/')
    #     test_data = imageClf.extract_image_features(imgArr)
    #     predicted_labels = imageClf.predict_labels(test_data)

    #     for label in predicted_labels:
    #         print(label)
    #         robot.say_text(label).wait_for_completed()
    #         if label == "drone":
    #             for _ in range(4):
    #                 robot.drive_straight(distance_mm(60), speed_mmps(20)).wait_for_completed()
    #                 robot.turn_in_place(degrees(90)).wait_for_completed()

        # Cleans up test directory
        # os.system("del test\*.bmp")

if __name__ == '__main__':
    cozmo.setup_basic_logging()

    try:
        sm = StateMachine()
        cozmo.connect(sm.run(self))
    except cozmo.ConnectionError as e:
        sys.exit("A connection error occurred: %s" % e)
