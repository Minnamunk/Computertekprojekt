import os
from os.path import expanduser

def create_empty_file(folder_path, file_name):


    # Combine the folder path and file name
    file_path = os.path.join(folder_path, file_name)

    # Create an empty file
    # with open(file_path, 'w') as file:
    #    pass
    file = open(expanduser("~")+file_path,'w')

    print(f"Empty file '{file_name}' created in '{folder_path}'.")

# Example usage:
folder_path = '/Desktop'
file_name = 'Computerteknologiprojekt1.txt'

create_empty_file(folder_path, file_name)
