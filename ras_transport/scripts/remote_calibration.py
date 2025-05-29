#!/usr/bin/env python3
from ras_transport.interfaces.TransportWrapper import TransportServiceClient
import json
import time
import os
import datetime
import threading
import sys
import subprocess
from ras_common.globals import RAS_CONFIGS_PATH
from ras_logging.ras_logger import RasLogger

def save_calibration_data(calibration_data, custom_filename=None):
    """Save calibration data to a JSON file with a unique name in the calibration_configs folder"""
    logger = RasLogger()
    try:
        # Create the calibration_configs directory inside the configs directory
        calibration_dir = os.path.join(RAS_CONFIGS_PATH, "calibration_configs")
        os.makedirs(calibration_dir, exist_ok=True)
        
        # Generate a unique filename based on timestamp or use custom filename if provided
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        
        if custom_filename and custom_filename.strip():
            # Use the custom filename provided by the user
            # Make sure it has the .json extension
            if not custom_filename.lower().endswith('.json'):
                custom_filename += '.json'
            filename = custom_filename
            
            # Check if the file already exists
            file_path = os.path.join(calibration_dir, filename)
            while os.path.exists(file_path):
                print(f"Error: A file named '{filename}' already exists.")
                print("Please enter a different filename (or press Enter for default timestamp-based name):")
                new_filename = input().strip()
                
                if not new_filename:
                    # User chose to use default timestamp-based name
                    filename = f"xarm_calibration_{timestamp}.json"
                    break
                else:
                    # Use the new filename provided by the user
                    if not new_filename.lower().endswith('.json'):
                        new_filename += '.json'
                    filename = new_filename
                    file_path = os.path.join(calibration_dir, filename)
        else:
            # Use the default filename with timestamp
            filename = f"xarm_calibration_{timestamp}.json"
            
        file_path = os.path.join(calibration_dir, filename)
        
        # Create the data structure
        data = {
            "calibration": calibration_data,
            "saved_at": datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        }
        
        # Save to calibration_configs directory
        with open(file_path, 'w') as f:
            json.dump(data, f, indent=4)
            
        logger.log_info(f"✅ Calibration data saved to:")
        logger.log_info(f"  - {file_path}")
        
        return filename
    except Exception as e:
        logger.log_error(f"❌ Error saving calibration data: {e}")
        return None

# Global variables
polling_active = False
rqt_process = None

def poll_calibration_data(client):
    """Continuously poll for calibration data"""
    global polling_active
    
    while polling_active:
        try:
            # Request the current calibration data
            get_data_payload = {
                "command": "get_data"
            }
            req = json.dumps(get_data_payload)
            res = client.call(req)
            
            # Parse the response
            response_data = json.loads(res)
            
            if response_data.get("success", False) and "calibration_data" in response_data:
                calibration_data = response_data["calibration_data"]
                
                # Clear the current line and print the data on the same line
                # Use carriage return (\r) to return to the beginning of the line without a newline
                # Use end='' to prevent print from adding a newline
                print(f"\rCurrent calibration: X={calibration_data['x']:.2f} cm, Y={calibration_data['y']:.2f} cm, Z={calibration_data['z']:.2f} cm", end='    ')
                sys.stdout.flush()
        except Exception as e:
            # Don't print errors, just continue
            pass
            
        # Sleep for a short time to avoid flooding
        time.sleep(0.5)

def main():
    logger = RasLogger()
    logger.log_info("ArUco Calibration Control Client")
    logger.log_info("--------------------------------")
    
    # Connect to the service
    transport_service_client = TransportServiceClient("remote_calibration")
    transport_service_client.connect_with_retries()
    transport_service_client.loop()
    
    # Access the global variable
    global polling_active, rqt_process
    
    # Start the ArUco calibration process
    start_payload = {
        "command": "start"
    }
    req = json.dumps(start_payload)
    res = transport_service_client.call(req)
    
    # Parse the response
    try:
        response_data = json.loads(res)
        success = response_data.get("success", False)
        message = response_data.get("message", "No message")
        
        if success:
            logger.log_info(f"✅ ArUco calibration started: {message}")
            
            # Start a thread to continuously poll for calibration data
            polling_active = True
            polling_thread = threading.Thread(target=poll_calibration_data, args=(transport_service_client,))
            polling_thread.daemon = True
            polling_thread.start()
            
            # Launch rqt_image_view to show the processed image with ArUco markers
            logger.log_info("Launching rqt_image_view to show the processed image with ArUco markers...")
            try:
                # Start rqt_image_view using ros2 run command
                rqt_process = subprocess.Popen(
                    ["ros2", "run", "rqt_image_view", "rqt_image_view"],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE
                )
                logger.log_info("✅ rqt_image_view launched successfully")
                logger.log_info("   Please select the topic /aruco_calibration/processed_image/compressed from the dropdown")
            except Exception as e:
                logger.log_error(f"❌ Failed to launch rqt_image_view: {e}")
                rqt_process = None
            
            logger.log_info("ArUco calibration is running. Position the markers in view of the camera.")
            logger.log_info("You will see live calibration data below. Press Enter when you're satisfied to save the current values.")
        else:
            logger.log_error(f"❌ Failed to start ArUco calibration: {message}")
            return
    except Exception as e:
        logger.log_error(f"Error parsing response: {e}")
        logger.log_error(f"Raw response: {res}")
        return
    
    # Wait for user to press Enter to stop the calibration
    print("\nPress Enter to save the current calibration values...")
    input()
    
    # Stop the polling thread
    polling_active = False
    
    # Clear the line
    print("\r" + " " * 80)
    
    # Ask the user for a custom filename
    print("Enter a custom filename for the calibration data (or press Enter for default):")
    custom_filename = input().strip()
    
    # Terminate the rqt_image_view process if it exists
    if rqt_process is not None:
        logger.log_info("Closing rqt_image_view...")
        try:
            if rqt_process:
                rqt_process.terminate()
                rqt_process.wait(timeout=5)
                logger.log_info("✅ rqt_image_view closed successfully")
        except Exception as e:
            logger.log_error(f"❌ Error closing rqt_image_view: {e}")
            try:
                # Force kill if terminate fails
                rqt_process.kill()
                logger.log_info("✅ rqt_image_view force-closed")
            except:
                pass
    
    # Stop the ArUco calibration process
    logger.log_info("Stopping ArUco calibration and getting calibration data...")
    stop_payload = {
        "command": "stop"
    }
    req = json.dumps(stop_payload)
    
    # Add try-except block around the service call
    try:
        res = transport_service_client.call(req)
    except Exception as e:
        logger.log_error(f"Error calling service: {e}")
        logger.log_info("Attempting to read calibration data directly from file")
        
        # If service call fails, try to read the calibration data directly from the file
        temp_calibration_file = os.path.join(RAS_CONFIGS_PATH, "temp_calibration.json")
        if os.path.exists(temp_calibration_file):
            try:
                with open(temp_calibration_file, 'r') as f:
                    calibration_data = json.load(f)
                    
                # Round the X, Y, Z values to integers
                calibration_data['x'] = round(float(calibration_data['x']))
                calibration_data['y'] = round(float(calibration_data['y']))
                calibration_data['z'] = round(float(calibration_data['z']))
                
                logger.log_info(f"Calibration data read from file:")
                logger.log_info(f"  X: {calibration_data['x']} cm (rounded)")
                logger.log_info(f"  Y: {calibration_data['y']} cm (rounded)")
                logger.log_info(f"  Z: {calibration_data['z']} cm (rounded)")
                logger.log_info(f"  Timestamp: {calibration_data['timestamp']}")
                
                # Save the calibration data
                save_calibration_data(calibration_data, custom_filename)
            except Exception as e:
                logger.log_error(f"Error reading calibration data from file: {e}")
        else:
            logger.log_error("Calibration file not found")
        
        return
    
    # Parse the response
    try:
        # First check if the response is valid JSON
        try:
            response_data = json.loads(res)
        except json.JSONDecodeError as e:
            logger.log_error(f"Invalid JSON response: {e}")
            logger.log_error(f"Raw response: {res}")
            return
            
        success = response_data.get("success", False)
        message = response_data.get("message", "No message")
        logger = RasLogger()
        
        if success:
            logger.log_info(f"✅ ArUco calibration stopped: {message}")
            
            # Check if calibration data is included in the response
            if "calibration_data" in response_data:
                calibration_data = response_data["calibration_data"]
                
                # Round the X, Y, Z values to integers
                calibration_data['x'] = round(float(calibration_data['x']))
                calibration_data['y'] = round(float(calibration_data['y']))
                calibration_data['z'] = round(float(calibration_data['z']))
                
                logger.log_info(f"Calibration data received:")
                logger.log_info(f"  X: {calibration_data['x']} cm (rounded)")
                logger.log_info(f"  Y: {calibration_data['y']} cm (rounded)")
                logger.log_info(f"  Z: {calibration_data['z']} cm (rounded)")
                logger.log_info(f"  Timestamp: {calibration_data['timestamp']}")
                
                # Save the calibration data
                save_calibration_data(calibration_data, custom_filename)
            else:
                logger.log_error("No calibration data received. Make sure ArUco markers are visible.")
        else:
            logger.log_error(f"Failed to stop ArUco calibration: {message}")
    except Exception as e:
        logger.log_error(f"Error parsing response: {e}")
        logger.log_error(f"Raw response: {res}")
    
    # Clean up
    transport_service_client.__del__()
    logger.log_info("Client terminated.")
    
    
if __name__ == "__main__":
    main()
