## THIS IS A GENERATED FILE -- DO NOT EDIT
.configuro: .libraries,rm4f linker.cmd package/cfg/ble_release_prm4f.orm4f

# To simplify configuro usage in makefiles:
#     o create a generic linker command file name 
#     o set modification times of compiler.opt* files to be greater than
#       or equal to the generated config header
#
linker.cmd: package/cfg/ble_release_prm4f.xdl
	$(SED) 's"^\"\(package/cfg/ble_release_prm4fcfg.cmd\)\"$""\"C:/ti/simplelink_cc13x2_26x2_sdk_2_40_00_81/examples/rtos/CC26X2R1_LAUNCHXL/ble5stack/CC2642_Impedance_3/tirtos/iar/config/configPkg/\1\""' package/cfg/ble_release_prm4f.xdl > $@
	-$(SETDATE) -r:max package/cfg/ble_release_prm4f.h compiler.opt compiler.opt.defs
