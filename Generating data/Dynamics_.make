#
# Simcenter Amesim system make file
#

# This variable can be used in -L statements
# or otherwise to separate machine dependent code
# The legal values are : sun, ibm, hp, sgi, lnx, win32

# This makefile has been created using the following cathegory path list
#	$AME
#	$AME/libsig
#	$AME/libmec
#	$AME/libdv
#	$AME/libm6dof
# End category path list

MACHINETYPE = win64

# Special make specs for platform

###
# No options needed currently for WIN64 - same options as for WIN32
# just a different compiler
#


# Then the object files
OBJECTS = \
	"c:/program files/simcenter/v1700/amesim/libmec/submodels/win64/T000.obj" \
	"c:/program files/simcenter/v1700/amesim/libmec/submodels/win64/VDSRF00.obj" \
	"c:/program files/simcenter/v1700/amesim/libmec/submodels/win64/V001.obj" \
	"c:/program files/simcenter/v1700/amesim/libsig/submodels/win64/SSINK.obj" \
	"c:/program files/simcenter/v1700/amesim/libsig/submodels/win64/CONS00.obj" \
	"c:/program files/simcenter/v1700/amesim/libdv/submodels/win64/VDCAR15DOF1.obj" \
	"c:/program files/simcenter/v1700/amesim/libdv/submodels/win64/VDTIRKIN00.obj" \
	"c:/program files/simcenter/v1700/amesim/libdv/submodels/win64/VDSLIP001.obj" \
	"c:/program files/simcenter/v1700/amesim/libdv/submodels/win64/VDTIRE001A.obj" \
	"c:/program files/simcenter/v1700/amesim/libdv/submodels/win64/VDELASTO_V2.obj" \
	"c:/program files/simcenter/v1700/amesim/libdv/submodels/win64/VDSSINK0.obj" \
	"c:/program files/simcenter/v1700/amesim/libmec/submodels/win64/MECFR1R0A.obj" \
	"c:/program files/simcenter/v1700/amesim/libdv/submodels/win64/VDAERO01.obj" \
	"c:/program files/simcenter/v1700/amesim/libdv/submodels/win64/VDSSINK1.obj" \
	"c:/program files/simcenter/v1700/amesim/libm6dof/submodels/win64/VDSSEUX0A.obj" \
	"c:/program files/simcenter/v1700/amesim/libm6dof/submodels/win64/VDSSEUV0A.obj" \
	"c:/program files/simcenter/v1700/amesim/libm6dof/submodels/win64/VDSSLAV0A.obj" \
	"c:/program files/simcenter/v1700/amesim/libm6dof/submodels/win64/VDSSMAT0A.obj" \
	"c:/program files/simcenter/v1700/amesim/libm6dof/submodels/win64/VDSSLAA0A.obj" \
	"c:/program files/simcenter/v1700/amesim/libdv/submodels/win64/VDSSLIP0A.obj" \
	"c:/program files/simcenter/v1700/amesim/libdv/submodels/win64/VDSSTIREEFF0A.obj" \
	"c:/program files/simcenter/v1700/amesim/libsig/submodels/win64/UD00.obj" \
	"c:/program files/simcenter/v1700/amesim/libmec/submodels/win64/TORQC.obj" \
	"c:/program files/simcenter/v1700/amesim/libsig/submodels/win64/GA00.obj" \
	"c:/program files/simcenter/v1700/amesim/libsig/submodels/win64/SPLT0.obj" \
	"c:/program files/simcenter/v1700/amesim/libsig/submodels/win64/SAT0.obj" \
	"c:/program files/simcenter/v1700/amesim/libtrdv/submodels/win64/TRVDDIFF01.obj" \
	"c:/program files/simcenter/v1700/amesim/libmec/submodels/win64/W000.obj" \
	"c:/program files/simcenter/v1700/amesim/submodels/win64/RSTAT.obj" \
	"c:/program files/simcenter/v1700/amesim/libm6dof/submodels/win64/VDSSLAX0A.obj" \
	"c:/program files/simcenter/v1700/amesim/libm6dof/submodels/win64/VDMATRIXID3.obj" \
	"c:/program files/simcenter/v1700/amesim/libmec/submodels/win64/LSTP00A.obj" \
	"c:/program files/simcenter/v1700/amesim/libmec/submodels/win64/ARM002A.obj" \
	"c:/program files/simcenter/v1700/amesim/libmec/submodels/win64/RSPR00.obj" \
	"c:/program files/simcenter/v1700/amesim/libmec/submodels/win64/LML012.obj" \
	"c:/program files/simcenter/v1700/amesim/libmec/submodels/win64/LML021.obj" \
	"c:/program files/simcenter/v1700/amesim/libmec/submodels/win64/SPR000A.obj" \
	"c:/program files/simcenter/v1700/amesim/libdv/submodels/win64/VDDAMP0.obj" \
	"c:/program files/simcenter/v1700/amesim/libdv/submodels/win64/VDRACK00A.obj" \
	"c:/program files/simcenter/v1700/amesim/libmec/submodels/win64/RCON00A.obj" \
	"c:/program files/simcenter/v1700/amesim/libmec/submodels/win64/RSD00A.obj" \
	"c:/program files/simcenter/v1700/amesim/libsig/submodels/win64/FX00.obj" \
	"c:/program files/simcenter/v1700/amesim/libmec/submodels/win64/WTC001.obj" \
	"c:/program files/simcenter/v1700/amesim/libtrdv/submodels/win64/VDADHER00.obj" \
	"c:/program files/simcenter/v1700/amesim/libtrdv/submodels/win64/VDROAD00.obj" \
	"c:/program files/simcenter/v1700/amesim/libdv/submodels/win64/VDSSIDES0A.obj" \
	"c:/program files/simcenter/v1700/amesim/libdv/submodels/win64/VDSSIDEFR.obj" \
	"c:/program files/simcenter/v1700/amesim/libsig/submodels/win64/JUN3M.obj" \
	"c:/program files/simcenter/v1700/amesim/libmec/submodels/win64/MECADS1A.obj" \
	"c:/program files/simcenter/v1700/amesim/libmec/submodels/win64/MECTS1A.obj" \
	"c:/program files/simcenter/v1700/amesim/libmec/submodels/win64/LMECHN1.obj" \
	"c:/program files/simcenter/v1700/amesim/libsig/submodels/win64/DYNDUP2.obj" \
	"c:/program files/simcenter/v1700/amesim/libsig/submodels/win64/DYNDMUX2.obj" \
	"c:/program files/simcenter/v1700/amesim/libsig/submodels/win64/DYNMUX2.obj" \
	"c:/program files/simcenter/v1700/amesim/libsig/submodels/win64/LAG1.obj"

Dynamics_.mexw64: $(OBJECTS) Dynamics_.obj
	@echo Dynamics_.make.link_args =
	@type Dynamics_.make.link_args
	"$(AME)\interfaces\simulink\win32\amemex" $(LDFLAGS) -o Dynamics_.mexw64 Dynamics_.obj @"Dynamics_.make.link_args" $(AMELIBS)

Dynamics_.obj: Dynamics_.c
	"$(AME)\interfaces\simulink\win32\amemexcompile" $(CC) -c -DWIN32 -I"$(MATLAB)/extern/include" -I"$(MATLAB)/simulink/include" -I"$(AME)/interfaces/simulink"  -I"$(AME)/interfaces"  $(CFLAGS) -o Dynamics_.obj Dynamics_.c

.c.obj:
	@echo
	@echo "Warning: \"$<\" is newer than the object."
	@echo ""

.f.obj:
	@echo
	@echo "Warning: \"$<\" is newer than the object."
	@echo ""

# End of file

