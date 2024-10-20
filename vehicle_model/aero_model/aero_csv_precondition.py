import pandas as pd
import sys
import os

# Check if the user provided the CSV path as an argument
if len(sys.argv) != 2:
    print("Usage: python precondition_csv.py <path_to_csv>")
    sys.exit(1)

# Get the CSV file path from the command-line argument
csv_path = sys.argv[1]

# Check if the file exists
if not os.path.isfile(csv_path):
    print(f"Error: The file {csv_path} does not exist.")
    sys.exit(1)

# Load the CSV data
data = pd.read_csv(csv_path)

# Create an empty DataFrame to store new rows with negated values
new_rows = []

# Iterate through each row in the original data
for index, row in data.iterrows():
    Roll = row['Roll']
    Pitch = row['Pitch']
    Yaw = row['Yaw']
    
    # Add a new row for -Roll and -Yaw (negate mx, mz, and cy)
    new_rows.append({
        'Roll': -Roll,
        'Pitch': Pitch,
        'Yaw': -Yaw,
        'cx': row['cx'],
        'cy': -row['cy'],  # negate cy for -Roll
        'cz': row['cz'],
        'mx': -row['mx'],  # negate mx for -Roll
        'my': row['my'],
        'mz': -row['mz']
    })

# Convert new rows into a DataFrame
new_data = pd.DataFrame(new_rows)

# Combine the original data with the new rows
augmented_data = pd.concat([data, new_data], ignore_index=True)

# Create the new file path with _preconditioned.csv
file_name, file_extension = os.path.splitext(csv_path)
new_csv_path = f"{file_name}_preconditioned{file_extension}"

# Save the augmented data to the new CSV file
augmented_data.to_csv(new_csv_path, index=False)

print(f"Augmented CSV file saved as {new_csv_path} successfully!")
