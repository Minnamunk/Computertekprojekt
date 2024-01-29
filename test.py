import os

def create_empty_file(folder_path, file_name):


    # Combine the folder path and file name
    file_path = os.path.join(folder_path, file_name)

    # Create an empty file
    with open(file_path, 'w') as file:
        pass

    print(f"Empty file '{file_name}' created in '{folder_path}'.")

# Example usage:
folder_path = 'C:/Users/idavi/Documents/Uni/2.semester/Computerteknologiprojekt1'
file_name = 'Computerteknologiprojekt.txt'

create_empty_file(folder_path, file_name)
