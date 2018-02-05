import numpy as np
import re
from sklearn import svm, metrics, tree, neighbors
from skimage import io, feature, filters, exposure, color
from sklearn.ensemble import GradientBoostingClassifier

IMG_CLASSIFIER = None

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


import sys
import cozmo
import datetime
import time
import os
from cozmo.util import degrees, distance_mm, speed_mmps

ROBOT_INSTANCE = None

class StateMachine:

    def __init__(self):
        self.state = Idle()

    def start(self):
        global IMG_CLASSIFIER
        IMG_CLASSIFIER = startUp()

        while True:
            print("While loop")
            self.state.run(self)

class State(object):

    def run(self, stateMachine):
        assert 0

class Idle(State):
    def run(self, stateMachine):
        global IMG_CLASSIFIER
        global ROBOT_INSTANCE

        robot = ROBOT_INSTANCE
        imageClf = IMG_CLASSIFIER

        robot.camera.image_stream_enabled = True
        robot.camera.color_image_enabled = False
        robot.camera.enable_auto_exposure()

        robot.set_head_angle(cozmo.util.degrees(0)).wait_for_completed()

        time.sleep(.5)
        latest_image = robot.world.latest_image
        new_image = latest_image.raw_image

        robot.say_text("Taking Picture").wait_for_completed()

        print("Predicting: ", end="")
        imgArr = [np.asarray(new_image)]
        # Asking for label of new image
        # (test_raw, test_labels) = imageClf.load_data_from_folder('./test/')
        test_data = imageClf.extract_image_features(imgArr)
        predicted_label = imageClf.predict_labels(test_data)[0]
        if predicted_label == "drone":
            stateMachine.state = Drone()
        elif predicted_label == "order":
            stateMachine.state = Order()
        elif predicted_label == "inspection":
            stateMachine.state = Inspection()
        else:
            stateMachine.state = Idle()

class Order(State):
    def run(self, stateMachine):
        global ROBOT_INSTANCE
        robot = ROBOT_INSTANCE

        robot.say_text("Running Order").wait_for_completed()

        robot.drive_wheels(20, 80, duration=10).wait_for_completed()
        stateMachine.state = Idle()

class Inspection(State):
    def run(self, stateMachine):
        global ROBOT_INSTANCE
        robot = ROBOT_INSTANCE

        robot.say_text("Running Inspection").wait_for_completed()
        # Add moving lift up and down functionality
        for _ in range(4):
            robot.drive_straight(distance_mm(200), speed_mmps(50)).wait_for_completed()
            robot.turn_in_place(degrees(90)).wait_for_completed()
        stateMachine.state = Idle()

class Drone(State):
    def run(self, stateMachine):
        global ROBOT_INSTANCE
        robot = ROBOT_INSTANCE

        robot.say_text("Running Drone").wait_for_completed()
        # Add functionality to move and pick up block
        lookaround = robot.start_behavior(cozmo.behavior.BehaviorTypes.LookAroundInPlace)
        cubes = robot.world.wait_until_observe_num_objects(num=1, object_type=cozmo.objects.LightCube, timeout=60)
        lookaround.stop()

        current_action = robot.pickup_object(cubes[0], num_retries=3)
        current_action.wait_for_completed()

        robot.drive_straight(distance_mm(100), speed_mmps(10)).wait_for_completed()

        current_action = robot.place_object_on_ground_here(cubes[0])
        current_action.wait_for_completed()

        robot.drive_straight(distance_mm(-100), speed_mmps(10)).wait_for_completed()

        stateMachine.state = Idle()


def begin(sdk_conn):
    global ROBOT_INSTANCE
    ROBOT_INSTANCE = sdk_conn.wait_for_robot()
    sm = StateMachine()
    sm.start()

if __name__ == '__main__':
    cozmo.setup_basic_logging()

    try:
        cozmo.connect(begin)
    except cozmo.ConnectionError as e:
        sys.exit("A connection error occurred: %s" % e)
