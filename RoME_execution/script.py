import pandas as pd
import os
import time

def write(df, output_file_path, elapsed_time):
    # Find the index where the timestamp is greater than elapsed time
    index = (df['time'] > elapsed_time).idxmax()

    # If no index is found or the index is 0, break the loop
    if pd.isna(index) or index == 0:
        return False
        
    # Get the corresponding biomedical data
    data_to_save = df.loc[index - 1, 'biomedical_data']

    # Save only the last biomedical data to a new file
    with open(output_file_path, 'w') as f:
        f.write(str(data_to_save))

    output_file_path = os.path.splitext(os.path.basename(output_file_path))[0]
    print(f"Last {output_file_path} data saved: {data_to_save}")
    return True

def process_real_time():
    # Define the directory to save files
    output_folder = "read_files"
    os.makedirs(output_folder, exist_ok=True)  # Create the directory if it doesn't exist

    # Read the CSV file into a pandas DataFrame
    heart_beat_path = "heart_beat_adjusted.csv"
    oxigenation_path = "oxigenation_adjusted.csv"
    respiration_path = "respiration_adjusted.csv"
    heart_beat = pd.read_csv(heart_beat_path, names=['time', 'biomedical_data']) 
    oxigenation = pd.read_csv(oxigenation_path, names=['time', 'biomedical_data'])
    respiration = pd.read_csv(respiration_path, names=['time', 'biomedical_data'])

    # Convert 'time' column to numeric type
    heart_beat['time'] = pd.to_numeric(heart_beat['time'])
    oxigenation['time'] = pd.to_numeric(oxigenation['time'])
    respiration['time'] = pd.to_numeric(respiration['time'])

    # Start time of the script
    start_time = time.time()

    # Generate output file paths within the output folder
    heart_beat_output_file_path = output_folder + "/hr.csv"
    oxigenation_output_file_path = output_folder + "/spo2.csv"
    respiration_output_file_path = output_folder + "/resp.csv"

    # Infinite loop to run in real-time
    while True:
        # Calculate elapsed time since the start of the script
        elapsed_time = (time.time() - start_time) * 100

        if write(heart_beat, heart_beat_output_file_path, elapsed_time) == False:
            break
        if write(oxigenation, oxigenation_output_file_path, elapsed_time) == False:
            break
        if write(respiration, respiration_output_file_path, elapsed_time) == False:
            break

        # Sleep for a short interval before checking again
        time.sleep(0.5)  # Adjust this interval as needed

process_real_time()
print("Finished reading")
