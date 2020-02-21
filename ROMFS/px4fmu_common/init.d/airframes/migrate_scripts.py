from os import walk

file_names = []
for (dirpath, dirnames, filenames) in walk("."):
    file_names.extend(filenames)
    break

in_autoconfig_section = False;
was_comment_line = True

for file_name in file_names:
    autostart_id = file_name.split("_")[0]
    
    f_read = open(file_name, "r")
    f_write = None
    
    for line in f_read.readlines():

        if "$AUTOCNF" in line:
            in_autoconfig_section = True
        elif in_autoconfig_section and line.strip() == "fi":
            in_autoconfig_section = False

        if line.strip().startswith("#") and in_autoconfig_section:
            if f_write is None:
                f_write = open("./param_files/" + autostart_id + ".param", "w+")

            if was_comment_line:
                f_write.write(line.strip() + "\n")
            else:
                f_write.write("\n" + line.strip() + "\n")
            was_comment_line = True
            continue

        if line.strip().startswith("param set"):
            
            if f_write is None:
                f_write = open("./param_files/" + autostart_id + ".param", "w+")
            
            line = line.replace("\t", " ")
            
            line_item_list = line.rstrip("\n").strip().split("#")[0].split(" ")
                                        
            pruned_line_list = []
            
            for item in line_item_list:
                if item is not "" and item is not "\t":
                    pruned_line_list.append(item.strip())
            
            
            param_name = pruned_line_list[-2]
            param_value = pruned_line_list[-1]
            
            f_write.write(param_name + " " + param_value + "\n")
            was_comment_line = False
    
    if f_write is not None:
        f_write.close()
    f_read.close()
            

print(file_names)