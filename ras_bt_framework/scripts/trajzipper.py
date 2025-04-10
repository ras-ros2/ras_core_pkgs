"""
Copyright (C) 2024 Harsh Davda

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Affero General Public License as published
by the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU Affero General Public License for more details.

You should have received a copy of the GNU Affero General Public License
along with this program. If not, see <https://www.gnu.org/licenses/>.

For inquiries or further information, you may contact:
Harsh Davda
Email: info@opensciencestack.org
"""

import os
import zipfile

def zip_xml_directory():
    # Get the directory of the current script
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    # Define the path to the xml directory
    xml_dir_path = os.path.join(script_dir, '..', 'xml')
    
    # Check if the directory exists
    if not os.path.isdir(xml_dir_path):
        raise FileNotFoundError(f"The directory {xml_dir_path} does not exist.")
    
    # Define the path for the output zip file
    zip_file_path = os.path.join(script_dir, '..', 'xml', 'xml_directory.zip')
    
    # Create a zip file and add all files in the xml directory to it
    with zipfile.ZipFile(zip_file_path, 'w') as zipf:
        for root, dirs, files in os.walk(xml_dir_path):
            for file in files:
                file_path = os.path.join(root, file)
                arcname = os.path.relpath(file_path, start=xml_dir_path)
                zipf.write(file_path, arcname)
    
    return zip_file_path

if __name__ == "__main__":
    try:
        zip_path = zip_xml_directory()
        print(f"The xml directory has been zipped to: {zip_path}")
    except FileNotFoundError as e:
        print(e)