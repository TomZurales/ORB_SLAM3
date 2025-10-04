import os
import csv

directory = os.path.dirname(__file__)

for filename in ["0.csv"]:
    if filename.endswith('.csv'):
        filepath = os.path.join(directory, filename)
        with open(filepath, 'r', newline='') as f:
            reader = list(csv.reader(f))
            if not reader or not reader[0]:
                continue  # skip empty files
            try:
                offset = float(reader[0][0])
            except ValueError:
                continue  # skip if first value is not a number

            new_rows = []
            for row in reader:
                if not row:
                    new_rows.append(row)
                    continue
                try:
                    new_val = float(row[0]) - offset
                    new_row = [str(new_val)] + row[1:]
                except ValueError:
                    print("FAILED TO CONVERT", row)
                    exit(1)
                new_rows.append(new_row)

                tum_filepath = os.path.splitext(filepath)[0] + '.tum'
        with open(tum_filepath, 'w') as tum_file:
            for row in new_rows:
                if row:
                    tum_file.write(' '.join(row) + '\n')