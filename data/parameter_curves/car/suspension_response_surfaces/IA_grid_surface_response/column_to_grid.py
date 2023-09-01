import pandas as pd
import csv

# Make sure you're in /simulation-2024/data/parameter_curves/car/suspension_response_surfaces/IA_grid_surface_response
# when running this script

########################################
######### INPUT FILE NAME HERE #########
########################################
input_csv = "../IA_column_surface_response/RR_IA_response_surface.csv"

########################################
######### OUTPUT FILE NAME HERE ########
########################################
csv_name = "RR_IA_response_surface.csv"

df = pd.read_csv(input_csv)

# x in top row, y in left column, min (x, y) in top left
x_vals = sorted(list(df["heave"].unique()))
y_vals = sorted(list(df["roll"].unique()))

# Initialize data array
data = [[0 for i in range(len(x_vals))] for j in range(len(y_vals))]

for index, row in df.iterrows():
    i = x_vals.index(row["heave"])
    j = y_vals.index(row["roll"])

    data[i][j] = row["IA_change"]

# First row
rows = [[""] + x_vals]

rows += [[y_vals[i]] + data[i] for i in range(len(y_vals))]

with open(csv_name, 'w') as csvfile: 
    # creating a csv writer object 
    csvwriter = csv.writer(csvfile, lineterminator = "\n") 
        
    csvwriter.writerows(rows)