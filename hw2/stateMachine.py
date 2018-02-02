''' Take a picture

Takes most recent image from Cozmo
Converts it to 8-bit black and white
Saves to destination
'''

import sys
import cozmo

currentState = Idle

class State(object):
    action = ""
class Idle(State):
    action =
class Drone(State):
    action =
class Order(State):
    action =
class Inspection(State):
    action =
def transition(next):
    currentState = Idle
    return Idle

def stateDecider(input):
    return {
        'drone' : Drone
        'order' : Order
        'inspection' : Inspection
        'none' : Idle
    }[input]
def transition(input):
    currentState = stateDecider(input)
    currentState = Idle

import numpy as np
import re
from sklearn import svm, metrics, tree, neighbors
from skimage import io, feature, filters, exposure, color
from sklearn.ensemble import GradientBoostingClassifier

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

    img_clf = ImageClassifier()

    # load images
    (train_raw, train_labels) = img_clf.load_data_from_folder('./train/')

    # convert images into features
    train_data = img_clf.extract_image_features(train_raw)

    # train model
    img_clf.train_classifier(train_data, train_labels)
    # predicted_labels = img_clf.predict_labels(train_data)
    # print("\nTraining results")
    # print("=============================")
    # print("Confusion Matrix:\n",metrics.confusion_matrix(train_labels, predicted_labels))
    # print("Accuracy: ", metrics.accuracy_score(train_labels, predicted_labels))
    # print("F1 score: ", metrics.f1_score(train_labels, predicted_labels, average='micro'))

    # # test model
    # predicted_labels = img_clf.predict_labels(test_data)
    # print("\nTest results")
    # print("=============================")
    # print("Confusion Matrix:\n",metrics.confusion_matrix(test_labels, predicted_labels))
    # print("Accuracy: ", metrics.accuracy_score(test_labels, predicted_labels))
    # print("F1 score: ", metrics.f1_score(test_labels, predicted_labels, average='micro'))