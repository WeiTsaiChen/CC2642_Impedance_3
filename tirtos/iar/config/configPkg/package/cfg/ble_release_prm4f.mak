#
#  Do not edit this file.  This file is generated from 
#  package.bld.  Any modifications to this file will be 
#  overwritten whenever makefiles are re-generated.
#
#  target compatibility key = iar.targets.arm.M4F{1,0,8.20,2
#
ifeq (,$(MK_NOGENDEPS))
-include package/cfg/ble_release_prm4f.orm4f.dep
package/cfg/ble_release_prm4f.orm4f.dep: ;
endif

package/cfg/ble_release_prm4f.orm4f: | .interfaces
package/cfg/ble_release_prm4f.orm4f: package/cfg/ble_release_prm4f.c package/cfg/ble_release_prm4f.mak
	@$(RM) $@.dep
	$(RM) $@
	@$(MSG) clrm4f $< ...
	LC_ALL=C $(iar.targets.arm.M4F.rootDir)/bin/iccarm    -D DeviceFamily_CC26X2   -I C:/ti/simplelink_cc13x2_26x2_sdk_2_40_00_81/examples/rtos/CC26X2R1_LAUNCHXL/ble5stack/CC2642_Impedance_3/tirtos/iar/app/../../../../../../../../source/ti/ble5stack/controller/cc26xx/inc/   -I C:/ti/simplelink_cc13x2_26x2_sdk_2_40_00_81/examples/rtos/CC26X2R1_LAUNCHXL/ble5stack/CC2642_Impedance_3/tirtos/iar/app/../../../../../../../../source/ti/ble5stack/inc/   -I C:/ti/simplelink_cc13x2_26x2_sdk_2_40_00_81/examples/rtos/CC26X2R1_LAUNCHXL/ble5stack/CC2642_Impedance_3/tirtos/iar/app/../../../../../../../../source/ti/ble5stack/common/cc26xx/   -I C:/ti/simplelink_cc13x2_26x2_sdk_2_40_00_81/examples/rtos/CC26X2R1_LAUNCHXL/ble5stack/CC2642_Impedance_3/tirtos/iar/app/../../../src/app/   -I C:/ti/simplelink_cc13x2_26x2_sdk_2_40_00_81/examples/rtos/CC26X2R1_LAUNCHXL/ble5stack/CC2642_Impedance_3/tirtos/iar/app/../../../../../../../../source/ti/ble5stack/icall/inc/   -I C:/ti/simplelink_cc13x2_26x2_sdk_2_40_00_81/examples/rtos/CC26X2R1_LAUNCHXL/ble5stack/CC2642_Impedance_3/tirtos/iar/app/../../../../../../../../source/ti/ble5stack/inc/   -I C:/ti/simplelink_cc13x2_26x2_sdk_2_40_00_81/examples/rtos/CC26X2R1_LAUNCHXL/ble5stack/CC2642_Impedance_3/tirtos/iar/app/../../../../../../../../source/ti/ble5stack/profiles/dev_info/   -I C:/ti/simplelink_cc13x2_26x2_sdk_2_40_00_81/examples/rtos/CC26X2R1_LAUNCHXL/ble5stack/CC2642_Impedance_3/tirtos/iar/app/../../../../../../../../source/ti/ble5stack/profiles/simple_profile/   -I C:/ti/simplelink_cc13x2_26x2_sdk_2_40_00_81/examples/rtos/CC26X2R1_LAUNCHXL/ble5stack/CC2642_Impedance_3/tirtos/iar/app/../../../../../../../../source/ti/ble5stack/profiles/simple_profile/cc26xx/   -I C:/ti/simplelink_cc13x2_26x2_sdk_2_40_00_81/examples/rtos/CC26X2R1_LAUNCHXL/ble5stack/CC2642_Impedance_3/tirtos/iar/app/../../../../../../../../source/ti/ble5stack/target/   -I C:/ti/simplelink_cc13x2_26x2_sdk_2_40_00_81/examples/rtos/CC26X2R1_LAUNCHXL/ble5stack/CC2642_Impedance_3/tirtos/iar/app/../../../../../../../../source/ti/ble5stack/hal/src/inc/   -I C:/ti/simplelink_cc13x2_26x2_sdk_2_40_00_81/examples/rtos/CC26X2R1_LAUNCHXL/ble5stack/CC2642_Impedance_3/tirtos/iar/app/../../../../../../../../source/ti/ble5stack/hal/src/target/_common/   -I C:/ti/simplelink_cc13x2_26x2_sdk_2_40_00_81/examples/rtos/CC26X2R1_LAUNCHXL/ble5stack/CC2642_Impedance_3/tirtos/iar/app/../../../../../../../../source/ti/ble5stack/hal/src/target/_common/cc26xx/   -I C:/ti/simplelink_cc13x2_26x2_sdk_2_40_00_81/examples/rtos/CC26X2R1_LAUNCHXL/ble5stack/CC2642_Impedance_3/tirtos/iar/app/../../../../../../../../source/ti/ble5stack/hal/src/target/cc2650/rom/   -I C:/ti/simplelink_cc13x2_26x2_sdk_2_40_00_81/examples/rtos/CC26X2R1_LAUNCHXL/ble5stack/CC2642_Impedance_3/tirtos/iar/app/../../../../../../../../source/ti/ble5stack/heapmgr/   -I C:/ti/simplelink_cc13x2_26x2_sdk_2_40_00_81/examples/rtos/CC26X2R1_LAUNCHXL/ble5stack/CC2642_Impedance_3/tirtos/iar/app/../../../../../../../../source/ti/ble5stack/icall/src/inc/   -I C:/ti/simplelink_cc13x2_26x2_sdk_2_40_00_81/examples/rtos/CC26X2R1_LAUNCHXL/ble5stack/CC2642_Impedance_3/tirtos/iar/app/../../../../../../../../source/ti/ble5stack/osal/src/inc/   -I C:/ti/simplelink_cc13x2_26x2_sdk_2_40_00_81/examples/rtos/CC26X2R1_LAUNCHXL/ble5stack/CC2642_Impedance_3/tirtos/iar/app/../../../../../../../../source/ti/ble5stack/services/src/saddr/   -I C:/ti/simplelink_cc13x2_26x2_sdk_2_40_00_81/examples/rtos/CC26X2R1_LAUNCHXL/ble5stack/CC2642_Impedance_3/tirtos/iar/app/../../../../../../../../source/ti/ble5stack/services/src/sdata/   -I C:/ti/simplelink_cc13x2_26x2_sdk_2_40_00_81/examples/rtos/CC26X2R1_LAUNCHXL/ble5stack/CC2642_Impedance_3/tirtos/iar/app/../../../../../../../../source/ti/ble5stack/services/src/nv/   -I C:/ti/simplelink_cc13x2_26x2_sdk_2_40_00_81/examples/rtos/CC26X2R1_LAUNCHXL/ble5stack/CC2642_Impedance_3/tirtos/iar/app/../../../../../../../../source/ti/ble5stack/services/src/nv/cc26xx/   -I C:/ti/simplelink_cc13x2_26x2_sdk_2_40_00_81/examples/rtos/CC26X2R1_LAUNCHXL/ble5stack/CC2642_Impedance_3/tirtos/iar/app/../../../../../../../../source/ti/devices/cc13x2_cc26x2/   -I C:/ti/simplelink_cc13x2_26x2_sdk_2_40_00_81/examples/rtos/CC26X2R1_LAUNCHXL/ble5stack/CC2642_Impedance_3/tirtos/iar/app/../../../../../../../../source/   --silent --aeabi --cpu=Cortex-M4F --diag_suppress=Pa050,Go005 --endian=little -e --fpu=VFPv4_sp --thumb   -Dxdc_cfg__xheader__='"configPkg/package/cfg/ble_release_prm4f.h"'  -Dxdc_target_name__=M4F -Dxdc_target_types__=iar/targets/arm/std.h -Dxdc_bld__profile_release -Dxdc_bld__vers_1_0_8_20_2 -Ohs --dlib_config $(iar.targets.arm.M4F.rootDir)/inc/c/DLib_Config_Normal.h  $(XDCINCS)  -o $@  $<
	
	-@$(FIXDEP) $@.dep $@.dep
	

package/cfg/ble_release_prm4f.srm4f: | .interfaces
package/cfg/ble_release_prm4f.srm4f: package/cfg/ble_release_prm4f.c package/cfg/ble_release_prm4f.mak
	@$(RM) $@.dep
	$(RM) $@
	@$(MSG) clrm4f $< ...
	LC_ALL=C $(iar.targets.arm.M4F.rootDir)/bin/iccarm    -D DeviceFamily_CC26X2   -I C:/ti/simplelink_cc13x2_26x2_sdk_2_40_00_81/examples/rtos/CC26X2R1_LAUNCHXL/ble5stack/CC2642_Impedance_3/tirtos/iar/app/../../../../../../../../source/ti/ble5stack/controller/cc26xx/inc/   -I C:/ti/simplelink_cc13x2_26x2_sdk_2_40_00_81/examples/rtos/CC26X2R1_LAUNCHXL/ble5stack/CC2642_Impedance_3/tirtos/iar/app/../../../../../../../../source/ti/ble5stack/inc/   -I C:/ti/simplelink_cc13x2_26x2_sdk_2_40_00_81/examples/rtos/CC26X2R1_LAUNCHXL/ble5stack/CC2642_Impedance_3/tirtos/iar/app/../../../../../../../../source/ti/ble5stack/common/cc26xx/   -I C:/ti/simplelink_cc13x2_26x2_sdk_2_40_00_81/examples/rtos/CC26X2R1_LAUNCHXL/ble5stack/CC2642_Impedance_3/tirtos/iar/app/../../../src/app/   -I C:/ti/simplelink_cc13x2_26x2_sdk_2_40_00_81/examples/rtos/CC26X2R1_LAUNCHXL/ble5stack/CC2642_Impedance_3/tirtos/iar/app/../../../../../../../../source/ti/ble5stack/icall/inc/   -I C:/ti/simplelink_cc13x2_26x2_sdk_2_40_00_81/examples/rtos/CC26X2R1_LAUNCHXL/ble5stack/CC2642_Impedance_3/tirtos/iar/app/../../../../../../../../source/ti/ble5stack/inc/   -I C:/ti/simplelink_cc13x2_26x2_sdk_2_40_00_81/examples/rtos/CC26X2R1_LAUNCHXL/ble5stack/CC2642_Impedance_3/tirtos/iar/app/../../../../../../../../source/ti/ble5stack/profiles/dev_info/   -I C:/ti/simplelink_cc13x2_26x2_sdk_2_40_00_81/examples/rtos/CC26X2R1_LAUNCHXL/ble5stack/CC2642_Impedance_3/tirtos/iar/app/../../../../../../../../source/ti/ble5stack/profiles/simple_profile/   -I C:/ti/simplelink_cc13x2_26x2_sdk_2_40_00_81/examples/rtos/CC26X2R1_LAUNCHXL/ble5stack/CC2642_Impedance_3/tirtos/iar/app/../../../../../../../../source/ti/ble5stack/profiles/simple_profile/cc26xx/   -I C:/ti/simplelink_cc13x2_26x2_sdk_2_40_00_81/examples/rtos/CC26X2R1_LAUNCHXL/ble5stack/CC2642_Impedance_3/tirtos/iar/app/../../../../../../../../source/ti/ble5stack/target/   -I C:/ti/simplelink_cc13x2_26x2_sdk_2_40_00_81/examples/rtos/CC26X2R1_LAUNCHXL/ble5stack/CC2642_Impedance_3/tirtos/iar/app/../../../../../../../../source/ti/ble5stack/hal/src/inc/   -I C:/ti/simplelink_cc13x2_26x2_sdk_2_40_00_81/examples/rtos/CC26X2R1_LAUNCHXL/ble5stack/CC2642_Impedance_3/tirtos/iar/app/../../../../../../../../source/ti/ble5stack/hal/src/target/_common/   -I C:/ti/simplelink_cc13x2_26x2_sdk_2_40_00_81/examples/rtos/CC26X2R1_LAUNCHXL/ble5stack/CC2642_Impedance_3/tirtos/iar/app/../../../../../../../../source/ti/ble5stack/hal/src/target/_common/cc26xx/   -I C:/ti/simplelink_cc13x2_26x2_sdk_2_40_00_81/examples/rtos/CC26X2R1_LAUNCHXL/ble5stack/CC2642_Impedance_3/tirtos/iar/app/../../../../../../../../source/ti/ble5stack/hal/src/target/cc2650/rom/   -I C:/ti/simplelink_cc13x2_26x2_sdk_2_40_00_81/examples/rtos/CC26X2R1_LAUNCHXL/ble5stack/CC2642_Impedance_3/tirtos/iar/app/../../../../../../../../source/ti/ble5stack/heapmgr/   -I C:/ti/simplelink_cc13x2_26x2_sdk_2_40_00_81/examples/rtos/CC26X2R1_LAUNCHXL/ble5stack/CC2642_Impedance_3/tirtos/iar/app/../../../../../../../../source/ti/ble5stack/icall/src/inc/   -I C:/ti/simplelink_cc13x2_26x2_sdk_2_40_00_81/examples/rtos/CC26X2R1_LAUNCHXL/ble5stack/CC2642_Impedance_3/tirtos/iar/app/../../../../../../../../source/ti/ble5stack/osal/src/inc/   -I C:/ti/simplelink_cc13x2_26x2_sdk_2_40_00_81/examples/rtos/CC26X2R1_LAUNCHXL/ble5stack/CC2642_Impedance_3/tirtos/iar/app/../../../../../../../../source/ti/ble5stack/services/src/saddr/   -I C:/ti/simplelink_cc13x2_26x2_sdk_2_40_00_81/examples/rtos/CC26X2R1_LAUNCHXL/ble5stack/CC2642_Impedance_3/tirtos/iar/app/../../../../../../../../source/ti/ble5stack/services/src/sdata/   -I C:/ti/simplelink_cc13x2_26x2_sdk_2_40_00_81/examples/rtos/CC26X2R1_LAUNCHXL/ble5stack/CC2642_Impedance_3/tirtos/iar/app/../../../../../../../../source/ti/ble5stack/services/src/nv/   -I C:/ti/simplelink_cc13x2_26x2_sdk_2_40_00_81/examples/rtos/CC26X2R1_LAUNCHXL/ble5stack/CC2642_Impedance_3/tirtos/iar/app/../../../../../../../../source/ti/ble5stack/services/src/nv/cc26xx/   -I C:/ti/simplelink_cc13x2_26x2_sdk_2_40_00_81/examples/rtos/CC26X2R1_LAUNCHXL/ble5stack/CC2642_Impedance_3/tirtos/iar/app/../../../../../../../../source/ti/devices/cc13x2_cc26x2/   -I C:/ti/simplelink_cc13x2_26x2_sdk_2_40_00_81/examples/rtos/CC26X2R1_LAUNCHXL/ble5stack/CC2642_Impedance_3/tirtos/iar/app/../../../../../../../../source/   --silent --aeabi --cpu=Cortex-M4F --diag_suppress=Pa050,Go005 --endian=little -e --fpu=VFPv4_sp --thumb   -Dxdc_cfg__xheader__='"configPkg/package/cfg/ble_release_prm4f.h"'  -Dxdc_target_name__=M4F -Dxdc_target_types__=iar/targets/arm/std.h -Dxdc_bld__profile_release -Dxdc_bld__vers_1_0_8_20_2 -Ohs --dlib_config $(iar.targets.arm.M4F.rootDir)/inc/c/DLib_Config_Normal.h  $(XDCINCS)  -o $@  $<
	
	-@$(FIXDEP) $@.dep $@.dep
	

clean,rm4f ::
	-$(RM) package/cfg/ble_release_prm4f.orm4f
	-$(RM) package/cfg/ble_release_prm4f.srm4f

ble_release.prm4f: package/cfg/ble_release_prm4f.orm4f package/cfg/ble_release_prm4f.mak

clean::
	-$(RM) package/cfg/ble_release_prm4f.mak
