###### CONFIGURATION ######

### List of applications to be build
APPLICATIONS = robot_igh
SRC_FILES = test.cpp elmo_client.cpp ethercat_manager.cpp jmk.cpp lib/libethercat_rtdm.a
all::
	g++ -std=gnu++11 -I/usr/xenomai/include/trank -D__XENO_COMPAT__ -I/usr/xenomai/include/cobalt 																										\
	-I/usr/xenomai/include -D_GNU_SOURCE -D_REENTRANT -fasynchronous-unwind-tables -D__COBALT__ 																				\
	-I/usr/xenomai/include/alchemy   -I$(PWD)/include -Xlinker -rpath -Xlinker /usr/xenomai/lib   $(SRC_FILES) 													\
	-lm -Wl,--no-as-needed -ltrank -Wl,@/usr/xenomai/lib/modechk.wrappers -lalchemy -lcopperplate /usr/xenomai/lib/xenomai/bootstrap.o 	\
	-Wl,--wrap=main -Wl,--dynamic-list=/usr/xenomai/lib/dynlist.ld -L/usr/xenomai/lib -lcobalt -lmodechk -lpthread -lrt    							\
	-Wl,--no-as-needed -Wl,@/usr/xenomai/lib/cobalt.wrappers -Wl,@/usr/xenomai/lib/modechk.wrappers    																	\
	-L/usr/xenomai/lib -lcobalt -lmodechk -lpthread -lrt    -o $(APPLICATIONS)
clean::
	$(RM) $(APPLICATIONS) *.o


