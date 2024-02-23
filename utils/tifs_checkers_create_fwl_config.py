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
        writeBanner(output_file)
        output_file.write("\n/* \n * Auto-generated cfg using the command 'python create_fwl_config.py " + args[0] + "' on " + datetime.now().strftime("%d/%m/%Y %H:%M:%S") + " \n */\n\n")
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
                    output_file.write("\t" + value + "," + "\t/* fwlId */" + "\n")
                elif key == "num_regions":
                    output_file.write("\t" + value + "," + "\t/* numRegions */" + "\n")
                elif key == "max_num_regions":
                    output_file.write("\t" + value + "," + "\t/* maxNumRegions */" + "\n")
                elif type(value) is list:
                    output_file.write("\t{ \t/* Firewall registers for a given region : {controlReg, privId0, privId1, privId2, startAddrLow, startAddrHigh, endAddrLow, endAddrHigh} */\n")
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

def writeBanner(file):
        file.write('/*\n')
        file.write(' * Copyright (C) 2024 Texas Instruments Incorporated\n')
        file.write(' *  \n')
        file.write(' *  Redistribution and use in source and binary forms, with or without \n')
        file.write(' *  modification, are permitted provided that the following conditions\n')
        file.write(' *  are met:\n')
        file.write(' *\n')
        file.write(' *    Redistributions of source code must retain the above copyright\n')
        file.write(' *    notice, this list of conditions and the following disclaimer.\n')
        file.write(' *\n')
        file.write(' *    Redistributions in binary form must reproduce the above copyright\n')
        file.write(' *    notice, this list of conditions and the following disclaimer in the\n')
        file.write(' *    documentation and/or other materials provided with the\n')
        file.write(' *    distribution.\n')
        file.write(' *\n')
        file.write(' *    Neither the name of Texas Instruments Incorporated nor the names of\n')
        file.write(' *    its contributors may be used to endorse or promote products derived\n')
        file.write(' *    from this software without specific prior written permission.\n')
        file.write(' *\n')
        file.write(' *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS \n')
        file.write(' *  \"AS IS\" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT \n')
        file.write(' *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR \n')
        file.write(' *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT \n')
        file.write(' *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, \n')
        file.write(' *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT \n')
        file.write(' *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,\n')
        file.write(' *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY\n')
        file.write(' *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT\n')
        file.write(' *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE\n')
        file.write(' *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.\n')
        file.write(' */\n')

if __name__ == "__main__":
    args = sys.argv[1:]
    if len(args) == 1:
        if args[0] == 'am62x':
            input_filename = "source/drivers/hw_include/am62x/csl_soc_firewalls.h"
            output_filename = "source/safety_checkers/examples/tifs_checkers_app/soc/am62x/tifs_checkers_fwl_config.h"
        elif args[0] == 'am62a':
            input_filename = "source/drivers/hw_include/am62ax/csl_soc_firewalls.h"
            output_filename = "source/safety_checkers/examples/tifs_checkers_app/soc/am62a/tifs_checkers_fwl_config.h"
        elif args[0] == 'am62p':
            input_filename = "source/drivers/hw_include/am62px/csl_soc_firewalls.h"
            output_filename = "source/safety_checkers/examples/tifs_checkers_app/soc/am62p/tifs_checkers_fwl_config.h"
        elif args[0] == 'j722s':
            input_filename = "source/drivers/hw_include/j722s/csl_soc_firewalls.h"
            output_filename = "source/safety_checkers/examples/tifs_checkers_app/soc/j722s/tifs_checkers_fwl_config.h"
        elif args[0] == 'j784s4':
            input_filename = "packages/ti/csl/soc/j784s4/src/csl_soc_firewalls.h"
            output_filename = "packages/ti/safety_checkers/examples/tifs_checkers_app/soc/j784s4/tifs_checkers_fwl_config.h"
        elif args[0] == 'j721e':
            input_filename = "packages/ti/csl/soc/j721e/src/csl_soc_firewalls.h"
            output_filename = "packages/ti/safety_checkers/examples/tifs_checkers_app/soc/j721e/tifs_checkers_fwl_config.h"
        elif args[0] == 'j7200':
            input_filename = "packages/ti/csl/soc/j7200/src/csl_soc_firewalls.h"
            output_filename = "packages/ti/safety_checkers/examples/tifs_checkers_app/soc/j7200/tifs_checkers_fwl_config.h"
        elif args[0] == 'j721s2':
            input_filename = "packages/ti/csl/soc/j721s2/src/csl_soc_firewalls.h"
            output_filename = "packages/ti/safety_checkers/examples/tifs_checkers_app/soc/j721s2/tifs_checkers_fwl_config.h"

    fwl_dict = {}
    fwl_dict["dummy"] = '0'
    fwl_attrib_dict = {"fwl_id": 0, "num_regions": 0, "max_num_regions": 0, "registers":['0x0']}
    fwl_dict["dummy"] = fwl_attrib_dict
    del fwl_dict["dummy"]

    extract_fwl_data(input_filename, fwl_dict)
    num_entries = len(fwl_dict)
    print_fwl_data(output_filename, fwl_dict)
