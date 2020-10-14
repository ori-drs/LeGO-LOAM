import csv
csv_file = input('Enter the name of your input file: ')
txt_file = input('Enter the name of your output file: ')
with open(txt_file, "w") as my_output_file:
    with open(csv_file, "r") as my_input_file:
        for row in csv.reader(my_input_file):
            my_output_file.write(str(float(row[0])+1e-9*float(row[1])) + " " + 
                                           str(float(row[2])) + " " + 
                                           row[3] + " " + 
					   row[4] + " " + 
                                           row[5] + " " + 
                                           row[6] + " " + 
                                           row[7] + " " + 
                                           row[8] +'\n') 
    my_output_file.close()
