#!/usr/bin/env python

##############
#### Your name: Jackson Cook
##############

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
        # Please do not modify the header above
        # print(len(data))
        # print((data[0]))
        # print(len(data[0][0]))
        # print(len(data[0][0][0]))
        greyData = [color.rgb2gray(x) for x in data]

        # extract feature vector from image data

        ########################
        ######## YOUR CODE HERE
        ########################
        feature_data = [feature.hog(img, orientations=8, pixels_per_cell=(24, 24), cells_per_block=(9, 9), block_norm="L2-Hys", transform_sqrt=False, feature_vector=True) for img in greyData]

        # Please do not modify the return type below
        return(feature_data)

    def train_classifier(self, train_data, train_labels):
        # Please do not modify the header above

        # train model and save the trained model to self.classifier
        # decTree = tree.DecisionTreeClassifier()
        # boostedTree = GradientBoostingClassifier(n_estimators=50, learning_rate=0.1, max_depth=1, random_state=0)
        # neigh = neighbors.KNeighborsClassifier(8)
        # supp = svm.SVC(C=1.0, cache_size=200, class_weight=None, coef0=0.0,
        #     decision_function_shape='ovr', degree=8, gamma='auto', kernel='linear',
        #     max_iter=-1, probability=False, random_state=None, shrinking=True,
        #     tol=0.001, verbose=False)
        supp = svm.SVC(kernel='linear');
        self.classifier = supp.fit(train_data, train_labels)
        # self.classifier = neigh.fit(train_data, train_labels)

        ########################
        ######## YOUR CODE HERE
        ########################

    def predict_labels(self, data):
        # Please do not modify the header

        predicted_labels = self.classifier.predict(data)
        # predict labels of test data using trained model in self.classifier
        # the code below expects output to be stored in predicted_labels

        ########################
        ######## YOUR CODE HERE
        ########################

        # Please do not modify the return type below
        return predicted_labels


def main():

    img_clf = ImageClassifier()

    # load images
    (train_raw, train_labels) = img_clf.load_data_from_folder('./train/')
    (test_raw, test_labels) = img_clf.load_data_from_folder('./test/')

    # convert images into features
    train_data = img_clf.extract_image_features(train_raw)
    test_data = img_clf.extract_image_features(test_raw)

    # train model and test on training data
    img_clf.train_classifier(train_data, train_labels)
    predicted_labels = img_clf.predict_labels(train_data)
    print("\nTraining results")
    print("=============================")
    print("Confusion Matrix:\n",metrics.confusion_matrix(train_labels, predicted_labels))
    print("Accuracy: ", metrics.accuracy_score(train_labels, predicted_labels))
    print("F1 score: ", metrics.f1_score(train_labels, predicted_labels, average='micro'))

    # test model
    predicted_labels = img_clf.predict_labels(test_data)
    print("\nTesting results")
    print("=============================")
    print("Confusion Matrix:\n",metrics.confusion_matrix(test_labels, predicted_labels))
    print("Accuracy: ", metrics.accuracy_score(test_labels, predicted_labels))
    print("F1 score: ", metrics.f1_score(test_labels, predicted_labels, average='micro'))


if __name__ == "__main__":
    main()
