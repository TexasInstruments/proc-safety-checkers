import re
import sys
from datetime import datetime

def extract_fwl_data(input_filename, fwl_dict):
    macro_names = []
    macro_values = []

    with open(input_filename, "r") as input_file:
        pattern = r"^#define (\S+)\s+\((\S+U)\)$"

        for line in input_file:
           match_line = re.match(pattern, line)
           if match_line:
                csl_macro, macro_value = match_line.groups()
                macro_name = re.split("_ID|_MMR_BASE|_NUM_REGIONS|_NUM_PRIV_IDS_PER_REGION",csl_macro)

                if len(macro_name) == 2:
                    macro_name = macro_name[0]
                    if macro_name not in macro_names and macro_value not in macro_values:
                        if csl_macro.endswith("_ID"):
                            valid_id = int(macro_value[:-1])
                            if valid_id < 512 or valid_id > 1024:
                                macro_names.append(macro_name)
                                macro_values.append(macro_value)
                                fwl_dict[macro_name] = {}
                                fwl_dict[macro_name]["fwl_id"] = macro_value
                    elif macro_name in macro_names:
                        if csl_macro.endswith("_NUM_REGIONS"):
                            fwl_dict[macro_name]["num_regions"] =  macro_value
                            fwl_dict[macro_name]["max_num_regions"] =  macro_value
                            fwl_dict[macro_name]["registers"] = ['0x0U'] * 8

def print_fwl_data(output_filename, fwl_dict):
    with open(output_filename, "w") as output_file:
        output_file.write("/** Auto-generated cfg using the command 'python create_fwl_config.py " + args[0] + "' on " + datetime.now().strftime("%d/%m/%Y %H:%M:%S") + " **/\n\n")
        output_file.write("#ifndef TIFS_CHECKERS_FWL_CONFIG_H_\n")
        output_file.write("#define TIFS_CHECKERS_FWL_CONFIG_H_\n\n")
        output_file.write("#include <safety_checkers_soc.h>\n\n")
        output_file.write("#ifdef __cplusplus\n")
        output_file.write("extern \"C\" {\n")
        output_file.write("#endif\n\n")

        output_file.write("SafetyCheckers_TifsFwlConfig gSafetyCheckers_TifsFwlConfig[TIFS_CHECKER_FWL_MAX_NUM] = {\n")

        for keys in fwl_dict:
            output_file.write("{\n")
            for key,value in fwl_dict[keys].items():
                if key == "fwl_id":
                    output_file.write("\t" + value + "," + "\t/* fwl id */" + "\n")
                elif key == "num_regions":
                    output_file.write("\t" + value + "," + "\t/* num of regions */" + "\n")
                elif key == "max_num_regions":
                    output_file.write("\t" + value + "," + "\t/* max num of regions */" + "\n")
                elif type(value) is list:
                    output_file.write("\t{ /* fwl registers for each region : {control_reg, priv_id0, priv_id1, priv_id2, \n\t\t\t\t\t\t\t\t\t\t  start_addr_low, start_addr_high, end_addr_low, end_addr_high} */\n")
                    n = int(fwl_dict[keys]["num_regions"][:-1])
                    for i in range(n):
                        output_file.write("\t\t{")
                        for ele in value:
                            output_file.write("0x0U, ")
                        output_file.seek(output_file.tell()-2)
                        output_file.truncate()
                        output_file.write("},\n")
                    output_file.write("\t},\n")
            output_file.write("},\n")
        output_file.seek(output_file.tell()-2)
        output_file.truncate()
        output_file.write("\n};")

        output_file.write("\n\n#ifdef __cplusplus\n")
        output_file.write("}\n")
        output_file.write("#endif\n\n")
        output_file.write("#endif  /* #ifndef TIFS_CHECKERS_FWL_CONFIG_H_ */")

if __name__ == "__main__":
    args = sys.argv[1:]
    if len(args) == 1:
        if args[0] == 'am62x':
            input_filename = "source/drivers/hw_include/am62x/csl_soc_firewalls.h"
            output_filename = "source/safety_checkers/examples/tifs_checkers_app/soc/am62x/tifs_checkers_fwl_config.h"
        elif args[0] == 'j784s4':
            input_filename = "packages/ti/csl/soc/j784s4/src/csl_soc_firewalls.h"
            output_filename = "packages/ti/safety_checkers/examples/tifs_checkers_app/soc/j784s4/tifs_checkers_fwl_config.h"

    fwl_dict = {}
    fwl_dict["dummy"] = '0'
    fwl_attrib_dict = {"fwl_id": 0, "num_regions": 0, "max_num_regions": 0, "registers":['0x0']}
    fwl_dict["dummy"] = fwl_attrib_dict
    del fwl_dict["dummy"]

    extract_fwl_data(input_filename, fwl_dict)
    num_entries = len(fwl_dict)
    print_fwl_data(output_filename, fwl_dict)
