import os

def create_empty_file(folder_path, file_name):


    # Combine the folder path and file name
    file_path = os.path.join(folder_path, file_name)

    # Create an empty file
    with open(file_path, 'w') as file:
        pass

    print(f"Empty file '{file_name}' created in '{folder_path}'.")

# Example usage:
folder_path = '/path/to/your/folder'
file_name = 'empty_file.txt'

create_empty_file(folder_path, file_name)
