# invoke SourceDir generated makefile for ble_release.prm4f
ble_release.prm4f: .libraries,ble_release.prm4f
.libraries,ble_release.prm4f: package/cfg/ble_release_prm4f.xdl
	$(MAKE) -f C:\ti\simplelink_cc13x2_26x2_sdk_2_40_00_81\examples\rtos\CC26X2R1_LAUNCHXL\ble5stack\CC2642_Impedance_3\tirtos/src/makefile.libs

clean::
	$(MAKE) -f C:\ti\simplelink_cc13x2_26x2_sdk_2_40_00_81\examples\rtos\CC26X2R1_LAUNCHXL\ble5stack\CC2642_Impedance_3\tirtos/src/makefile.libs clean

