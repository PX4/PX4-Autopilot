import sys

def replace_lines_in_file(input_filename, output_filename, vehicle_id):
    try:
        with open(input_filename, 'r', encoding='utf-8') as file:
            lines = file.readlines()
        
        updated_lines = []
        for line in lines:
            if line in ['\n', '\r\n']: #if line is empty
                continue
            parts = line.strip().split()
            if len(parts) >= 4 and "param" in parts[0]:
                comment = " " + " ".join(parts[4:]) if len(parts) > 4 else ""
                updated_lines.append(f"{vehicle_id}\t1\t{parts[2]}\t{parts[3]}\tidk\t{comment}\n")
            else:
                updated_lines.append(line)
        
        with open(output_filename, 'w', encoding='utf-8') as file:
            file.writelines(updated_lines)
        
        print(f"File '{output_filename}' created successfully.")
    except Exception as e:
        print(f"Error processing file '{input_filename}': {e}")

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python3 run.py <input_filename> <output_filename.params> <Vehicle-id>")
    else:
        replace_lines_in_file(sys.argv[1], sys.argv[2], sys.argv[3])
