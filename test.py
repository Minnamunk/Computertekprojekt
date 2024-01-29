import os

def create_empty_file(path):
    with open(path, 'a'):
        os.utime(path, None)
