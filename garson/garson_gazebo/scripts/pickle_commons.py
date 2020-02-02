#! /usr/bin/env python
# -*- coding: utf-8 -*-
import pickle

def update_pickle_file(data, pickle_file_path):
    pickle.dump(data, open(pickle_file_path, "wb"))

def read_pickle_file(pickle_file_path):
    try:
        return pickle.load(open(pickle_file_path, "rb"))
    except IOError:
        return None