#!/usr/bin/env python

import sys
import os.path

# This is a tiny script to help you creating a CSV file from a face
# database with a similar hierarchie:
#
#  philipp@mango:~/facerec/data/at$ tree
#  .
#  |-- README
#  |-- s1
#  |   |-- 1.pgm
#  |   |-- ...
#  |   |-- 10.pgm
#  |-- s2
#  |   |-- 1.pgm
#  |   |-- ...
#  |   |-- 10.pgm
#  ...
#  |-- s40
#  |   |-- 1.pgm
#  |   |-- ...
#  |   |-- 10.pgm
#

def create_csv_func():

    BASE_PATH = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    SEPARATOR = ";"
    BASE_PATH = BASE_PATH + "/images"
    # count = 0

    label = 0
    csv_file = open(BASE_PATH + "/images.csv", "w")
    person_label_file = open(BASE_PATH + "/person_label.txt", "w")
    label_person_dict = {}

    for dirname, dirnames, filenames in os.walk(BASE_PATH):
        for subdirname in dirnames:
            subject_path = os.path.join(dirname, subdirname)
            for filename in os.listdir(subject_path):
                abs_path = "%s/%s" % (subject_path, filename)
                csv_file.write("%s%s%d\n"%(abs_path, SEPARATOR, label))
                # print "%s%s%d" % (abs_path, SEPARATOR, label)
                label_person_dict[label] = os.path.basename(subject_path)
                # count = count + 1
            label = label + 1
    # print count
    for label_name in label_person_dict:
        person_label_file.write("%d=%s\n"%(label_name, label_person_dict[label_name]))
    csv_file.close()
    person_label_file.close()
    print ".csv file create"
