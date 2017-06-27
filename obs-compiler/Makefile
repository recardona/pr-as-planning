CC = g++
BISON = bison
FLEX = flex

NFF_TARGET = pr2plan

#CXXFLAGS = -g -Wall -Imod-metric-ff -DDEBUG -DPDDL_TYPE_CHECKING
CXXFLAGS = -O3 -Wall -Imod-metric-ff -DNDEBUG
LDFLAGS = -Lmod-metric-ff/
#EFENCE = -lefence
STATIC = -static
LIBS = -lff $(EFENCE)

NFF_SOURCES = PDDL.cxx \
	pddl_fluent_set.cxx \
	pddl_string_table.cxx \
	bitarray.cxx \
	global_options.cxx \
	options.cxx \
	pr_obs_reader.cxx \
	pr_strips_mapping.cxx \
	strips_writer.cxx \
	act_obs.cxx \
	main.cxx

NFF_OBJECTS = $(NFF_SOURCES:.cxx=.o) 

# Implicit rules
#
.SUFFIXES:

.SUFFIXES: .cxx .C .o

.cxx.o:; $(CC) -o $@ -c $(CXXFLAGS) $<
.C.o:; $(CC) -o $@ -c $(CXXFLAGS) $<

# build rules

all : $(NFF_TARGET) 

$(NFF_TARGET) : $(PARSER_OBJ) $(NFF_OBJECTS)
	$(CC) -o $(NFF_TARGET) $(PARSER_OBJ) $(NFF_OBJECTS) $(LDFLAGS) $(LIBS)

# dependencies
depend:
	# Updating dependencies
	@makedepend -- $(CXXFLAGS) -- $(NFF_SOURCES) $(MAIN_NFF_SOURCES)

# Cleaning
clean:
	rm -rf $(NFF_OBJECTS) $(NFF_TARGET) $(PARSER_SRC)  $(PARSER_HDR) *.o *.hh
# DO NOT DELETE

PDDL.o: PDDL.hxx pddl_basic_types.hxx pddl_string_table.hxx
PDDL.o: pddl_fluent_set.hxx bitarray.hxx nff_logic.hxx /usr/include/stdint.h
PDDL.o: /usr/include/sys/_types/_int8_t.h /usr/include/sys/_types/_int16_t.h
PDDL.o: /usr/include/sys/_types/_int32_t.h /usr/include/sys/_types/_int64_t.h
PDDL.o: /usr/include/_types/_uint8_t.h /usr/include/_types/_uint16_t.h
PDDL.o: /usr/include/_types/_uint32_t.h /usr/include/_types/_uint64_t.h
PDDL.o: /usr/include/sys/_types.h /usr/include/sys/cdefs.h
PDDL.o: /usr/include/sys/_symbol_aliasing.h
PDDL.o: /usr/include/sys/_posix_availability.h /usr/include/machine/_types.h
PDDL.o: /usr/include/i386/_types.h /usr/include/sys/_pthread/_pthread_types.h
PDDL.o: /usr/include/sys/_types/_intptr_t.h
PDDL.o: /usr/include/sys/_types/_uintptr_t.h /usr/include/_types/_intmax_t.h
PDDL.o: /usr/include/_types/_uintmax_t.h utils.hxx /usr/include/sys/times.h
PDDL.o: /usr/include/sys/appleapiopts.h /usr/include/sys/_types/_clock_t.h
PDDL.o: /usr/include/sys/resource.h /usr/include/Availability.h
PDDL.o: /usr/include/AvailabilityInternal.h
PDDL.o: /usr/include/sys/_types/_timeval.h /usr/include/sys/_types/_id_t.h
PDDL.o: /usr/include/unistd.h /usr/include/_types.h /usr/include/sys/unistd.h
PDDL.o: /usr/include/sys/_types/_posix_vdisable.h
PDDL.o: /usr/include/sys/_types/_seek_set.h /usr/include/sys/_types/_size_t.h
PDDL.o: /usr/include/sys/_types/_ssize_t.h /usr/include/sys/_types/_uid_t.h
PDDL.o: /usr/include/sys/_types/_gid_t.h /usr/include/sys/_types/_off_t.h
PDDL.o: /usr/include/sys/_types/_pid_t.h
PDDL.o: /usr/include/sys/_types/_useconds_t.h /usr/include/sys/_types/_null.h
PDDL.o: /usr/include/sys/select.h /usr/include/sys/_types/_fd_def.h
PDDL.o: /usr/include/sys/_types/_timespec.h /usr/include/sys/_types/_time_t.h
PDDL.o: /usr/include/sys/_types/_suseconds_t.h
PDDL.o: /usr/include/sys/_types/_sigset_t.h
PDDL.o: /usr/include/sys/_types/_fd_setsize.h
PDDL.o: /usr/include/sys/_types/_fd_set.h /usr/include/sys/_types/_fd_clr.h
PDDL.o: /usr/include/sys/_types/_fd_isset.h
PDDL.o: /usr/include/sys/_types/_fd_zero.h /usr/include/sys/_types/_fd_copy.h
PDDL.o: /usr/include/sys/_select.h /usr/include/sys/_types/_dev_t.h
PDDL.o: /usr/include/sys/_types/_mode_t.h /usr/include/sys/_types/_uuid_t.h
PDDL.o: /usr/include/gethostuuid.h options.hxx mod-metric-ff/libff.h
PDDL.o: mod-metric-ff/ff.h /usr/include/stdlib.h /usr/include/sys/wait.h
PDDL.o: /usr/include/sys/signal.h /usr/include/machine/signal.h
PDDL.o: /usr/include/i386/signal.h /usr/include/machine/_mcontext.h
PDDL.o: /usr/include/i386/_mcontext.h /usr/include/mach/i386/_structs.h
PDDL.o: /usr/include/sys/_pthread/_pthread_attr_t.h
PDDL.o: /usr/include/sys/_types/_sigaltstack.h
PDDL.o: /usr/include/sys/_types/_ucontext.h /usr/include/machine/endian.h
PDDL.o: /usr/include/i386/endian.h /usr/include/sys/_endian.h
PDDL.o: /usr/include/libkern/_OSByteOrder.h
PDDL.o: /usr/include/libkern/i386/_OSByteOrder.h /usr/include/alloca.h
PDDL.o: /usr/include/sys/_types/_ct_rune_t.h
PDDL.o: /usr/include/sys/_types/_rune_t.h /usr/include/sys/_types/_wchar_t.h
PDDL.o: /usr/include/machine/types.h /usr/include/i386/types.h
PDDL.o: /usr/include/stdio.h /usr/include/sys/_types/_va_list.h
PDDL.o: /usr/include/sys/stdio.h /usr/include/secure/_stdio.h
PDDL.o: /usr/include/secure/_common.h /usr/include/strings.h
PDDL.o: /usr/include/string.h /usr/include/sys/_types/_rsize_t.h
PDDL.o: /usr/include/sys/_types/_errno_t.h /usr/include/secure/_string.h
PDDL.o: /usr/include/ctype.h /usr/include/runetype.h
PDDL.o: /usr/include/sys/_types/_wint_t.h /usr/include/sys/types.h
PDDL.o: /usr/include/sys/_types/_blkcnt_t.h
PDDL.o: /usr/include/sys/_types/_blksize_t.h
PDDL.o: /usr/include/sys/_types/_in_addr_t.h
PDDL.o: /usr/include/sys/_types/_in_port_t.h /usr/include/sys/_types/_ino_t.h
PDDL.o: /usr/include/sys/_types/_ino64_t.h /usr/include/sys/_types/_key_t.h
PDDL.o: /usr/include/sys/_types/_nlink_t.h
PDDL.o: /usr/include/sys/_pthread/_pthread_cond_t.h
PDDL.o: /usr/include/sys/_pthread/_pthread_condattr_t.h
PDDL.o: /usr/include/sys/_pthread/_pthread_mutex_t.h
PDDL.o: /usr/include/sys/_pthread/_pthread_mutexattr_t.h
PDDL.o: /usr/include/sys/_pthread/_pthread_once_t.h
PDDL.o: /usr/include/sys/_pthread/_pthread_rwlock_t.h
PDDL.o: /usr/include/sys/_pthread/_pthread_rwlockattr_t.h
PDDL.o: /usr/include/sys/_pthread/_pthread_t.h
PDDL.o: /usr/include/sys/_pthread/_pthread_key_t.h
PDDL.o: /usr/include/sys/_types/_fsblkcnt_t.h
PDDL.o: /usr/include/sys/_types/_fsfilcnt_t.h /usr/include/sys/timeb.h
pddl_fluent_set.o: pddl_fluent_set.hxx bitarray.hxx
pddl_string_table.o: pddl_string_table.hxx pddl_basic_types.hxx
bitarray.o: bitarray.hxx
global_options.o: global_options.hxx
options.o: options.hxx /usr/include/getopt.h /usr/include/sys/cdefs.h
options.o: /usr/include/sys/_symbol_aliasing.h
options.o: /usr/include/sys/_posix_availability.h /usr/include/unistd.h
options.o: /usr/include/_types.h /usr/include/sys/_types.h
options.o: /usr/include/machine/_types.h /usr/include/i386/_types.h
options.o: /usr/include/sys/_pthread/_pthread_types.h
options.o: /usr/include/sys/unistd.h
options.o: /usr/include/sys/_types/_posix_vdisable.h
options.o: /usr/include/sys/_types/_seek_set.h
options.o: /usr/include/sys/_types/_size_t.h /usr/include/_types/_uint64_t.h
options.o: /usr/include/Availability.h /usr/include/AvailabilityInternal.h
options.o: /usr/include/sys/_types/_ssize_t.h
options.o: /usr/include/sys/_types/_uid_t.h /usr/include/sys/_types/_gid_t.h
options.o: /usr/include/sys/_types/_intptr_t.h
options.o: /usr/include/sys/_types/_off_t.h /usr/include/sys/_types/_pid_t.h
options.o: /usr/include/sys/_types/_useconds_t.h
options.o: /usr/include/sys/_types/_null.h /usr/include/sys/select.h
options.o: /usr/include/sys/appleapiopts.h /usr/include/sys/_types/_fd_def.h
options.o: /usr/include/sys/_types/_timespec.h
options.o: /usr/include/sys/_types/_timeval.h
options.o: /usr/include/sys/_types/_time_t.h
options.o: /usr/include/sys/_types/_suseconds_t.h
options.o: /usr/include/sys/_types/_sigset_t.h
options.o: /usr/include/sys/_types/_fd_setsize.h
options.o: /usr/include/sys/_types/_fd_set.h
options.o: /usr/include/sys/_types/_fd_clr.h
options.o: /usr/include/sys/_types/_fd_isset.h
options.o: /usr/include/sys/_types/_fd_zero.h
options.o: /usr/include/sys/_types/_fd_copy.h /usr/include/sys/_select.h
options.o: /usr/include/sys/_types/_dev_t.h /usr/include/sys/_types/_mode_t.h
options.o: /usr/include/sys/_types/_uuid_t.h /usr/include/gethostuuid.h
pr_obs_reader.o: pr_obs_reader.hxx act_obs.hxx string_ops.hxx PDDL.hxx
pr_obs_reader.o: pddl_basic_types.hxx pddl_string_table.hxx
pr_obs_reader.o: pddl_fluent_set.hxx bitarray.hxx nff_logic.hxx
pr_obs_reader.o: /usr/include/stdint.h /usr/include/sys/_types/_int8_t.h
pr_obs_reader.o: /usr/include/sys/_types/_int16_t.h
pr_obs_reader.o: /usr/include/sys/_types/_int32_t.h
pr_obs_reader.o: /usr/include/sys/_types/_int64_t.h
pr_obs_reader.o: /usr/include/_types/_uint8_t.h
pr_obs_reader.o: /usr/include/_types/_uint16_t.h
pr_obs_reader.o: /usr/include/_types/_uint32_t.h
pr_obs_reader.o: /usr/include/_types/_uint64_t.h /usr/include/sys/_types.h
pr_obs_reader.o: /usr/include/sys/cdefs.h /usr/include/sys/_symbol_aliasing.h
pr_obs_reader.o: /usr/include/sys/_posix_availability.h
pr_obs_reader.o: /usr/include/machine/_types.h /usr/include/i386/_types.h
pr_obs_reader.o: /usr/include/sys/_pthread/_pthread_types.h
pr_obs_reader.o: /usr/include/sys/_types/_intptr_t.h
pr_obs_reader.o: /usr/include/sys/_types/_uintptr_t.h
pr_obs_reader.o: /usr/include/_types/_intmax_t.h
pr_obs_reader.o: /usr/include/_types/_uintmax_t.h
pr_strips_mapping.o: pr_strips_mapping.hxx strips_writer.hxx act_obs.hxx
pr_strips_mapping.o: PDDL.hxx pddl_basic_types.hxx pddl_string_table.hxx
pr_strips_mapping.o: pddl_fluent_set.hxx bitarray.hxx nff_logic.hxx
pr_strips_mapping.o: /usr/include/stdint.h /usr/include/sys/_types/_int8_t.h
pr_strips_mapping.o: /usr/include/sys/_types/_int16_t.h
pr_strips_mapping.o: /usr/include/sys/_types/_int32_t.h
pr_strips_mapping.o: /usr/include/sys/_types/_int64_t.h
pr_strips_mapping.o: /usr/include/_types/_uint8_t.h
pr_strips_mapping.o: /usr/include/_types/_uint16_t.h
pr_strips_mapping.o: /usr/include/_types/_uint32_t.h
pr_strips_mapping.o: /usr/include/_types/_uint64_t.h
pr_strips_mapping.o: /usr/include/sys/_types.h /usr/include/sys/cdefs.h
pr_strips_mapping.o: /usr/include/sys/_symbol_aliasing.h
pr_strips_mapping.o: /usr/include/sys/_posix_availability.h
pr_strips_mapping.o: /usr/include/machine/_types.h /usr/include/i386/_types.h
pr_strips_mapping.o: /usr/include/sys/_pthread/_pthread_types.h
pr_strips_mapping.o: /usr/include/sys/_types/_intptr_t.h
pr_strips_mapping.o: /usr/include/sys/_types/_uintptr_t.h
pr_strips_mapping.o: /usr/include/_types/_intmax_t.h
pr_strips_mapping.o: /usr/include/_types/_uintmax_t.h options.hxx
strips_writer.o: strips_writer.hxx PDDL.hxx pddl_basic_types.hxx
strips_writer.o: pddl_string_table.hxx pddl_fluent_set.hxx bitarray.hxx
strips_writer.o: nff_logic.hxx /usr/include/stdint.h
strips_writer.o: /usr/include/sys/_types/_int8_t.h
strips_writer.o: /usr/include/sys/_types/_int16_t.h
strips_writer.o: /usr/include/sys/_types/_int32_t.h
strips_writer.o: /usr/include/sys/_types/_int64_t.h
strips_writer.o: /usr/include/_types/_uint8_t.h
strips_writer.o: /usr/include/_types/_uint16_t.h
strips_writer.o: /usr/include/_types/_uint32_t.h
strips_writer.o: /usr/include/_types/_uint64_t.h /usr/include/sys/_types.h
strips_writer.o: /usr/include/sys/cdefs.h /usr/include/sys/_symbol_aliasing.h
strips_writer.o: /usr/include/sys/_posix_availability.h
strips_writer.o: /usr/include/machine/_types.h /usr/include/i386/_types.h
strips_writer.o: /usr/include/sys/_pthread/_pthread_types.h
strips_writer.o: /usr/include/sys/_types/_intptr_t.h
strips_writer.o: /usr/include/sys/_types/_uintptr_t.h
strips_writer.o: /usr/include/_types/_intmax_t.h
strips_writer.o: /usr/include/_types/_uintmax_t.h string_ops.hxx
act_obs.o: act_obs.hxx pddl_string_table.hxx pddl_basic_types.hxx PDDL.hxx
act_obs.o: pddl_fluent_set.hxx bitarray.hxx nff_logic.hxx
act_obs.o: /usr/include/stdint.h /usr/include/sys/_types/_int8_t.h
act_obs.o: /usr/include/sys/_types/_int16_t.h
act_obs.o: /usr/include/sys/_types/_int32_t.h
act_obs.o: /usr/include/sys/_types/_int64_t.h /usr/include/_types/_uint8_t.h
act_obs.o: /usr/include/_types/_uint16_t.h /usr/include/_types/_uint32_t.h
act_obs.o: /usr/include/_types/_uint64_t.h /usr/include/sys/_types.h
act_obs.o: /usr/include/sys/cdefs.h /usr/include/sys/_symbol_aliasing.h
act_obs.o: /usr/include/sys/_posix_availability.h
act_obs.o: /usr/include/machine/_types.h /usr/include/i386/_types.h
act_obs.o: /usr/include/sys/_pthread/_pthread_types.h
act_obs.o: /usr/include/sys/_types/_intptr_t.h
act_obs.o: /usr/include/sys/_types/_uintptr_t.h
act_obs.o: /usr/include/_types/_intmax_t.h /usr/include/_types/_uintmax_t.h
main.o: /usr/include/signal.h /usr/include/sys/cdefs.h
main.o: /usr/include/sys/_symbol_aliasing.h
main.o: /usr/include/sys/_posix_availability.h /usr/include/_types.h
main.o: /usr/include/sys/_types.h /usr/include/machine/_types.h
main.o: /usr/include/i386/_types.h /usr/include/sys/_pthread/_pthread_types.h
main.o: /usr/include/sys/signal.h /usr/include/sys/appleapiopts.h
main.o: /usr/include/Availability.h /usr/include/AvailabilityInternal.h
main.o: /usr/include/machine/signal.h /usr/include/i386/signal.h
main.o: /usr/include/machine/_mcontext.h /usr/include/i386/_mcontext.h
main.o: /usr/include/mach/i386/_structs.h
main.o: /usr/include/sys/_pthread/_pthread_attr_t.h
main.o: /usr/include/sys/_types/_sigaltstack.h
main.o: /usr/include/sys/_types/_ucontext.h /usr/include/sys/_types/_pid_t.h
main.o: /usr/include/sys/_types/_sigset_t.h /usr/include/sys/_types/_size_t.h
main.o: /usr/include/sys/_types/_uid_t.h
main.o: /usr/include/sys/_pthread/_pthread_t.h utils.hxx
main.o: /usr/include/sys/times.h /usr/include/sys/_types/_clock_t.h
main.o: /usr/include/sys/resource.h /usr/include/stdint.h
main.o: /usr/include/sys/_types/_int8_t.h /usr/include/sys/_types/_int16_t.h
main.o: /usr/include/sys/_types/_int32_t.h /usr/include/sys/_types/_int64_t.h
main.o: /usr/include/_types/_uint8_t.h /usr/include/_types/_uint16_t.h
main.o: /usr/include/_types/_uint32_t.h /usr/include/_types/_uint64_t.h
main.o: /usr/include/sys/_types/_intptr_t.h
main.o: /usr/include/sys/_types/_uintptr_t.h /usr/include/_types/_intmax_t.h
main.o: /usr/include/_types/_uintmax_t.h /usr/include/sys/_types/_timeval.h
main.o: /usr/include/sys/_types/_id_t.h /usr/include/unistd.h
main.o: /usr/include/sys/unistd.h /usr/include/sys/_types/_posix_vdisable.h
main.o: /usr/include/sys/_types/_seek_set.h
main.o: /usr/include/sys/_types/_ssize_t.h /usr/include/sys/_types/_gid_t.h
main.o: /usr/include/sys/_types/_off_t.h
main.o: /usr/include/sys/_types/_useconds_t.h /usr/include/sys/_types/_null.h
main.o: /usr/include/sys/select.h /usr/include/sys/_types/_fd_def.h
main.o: /usr/include/sys/_types/_timespec.h /usr/include/sys/_types/_time_t.h
main.o: /usr/include/sys/_types/_suseconds_t.h
main.o: /usr/include/sys/_types/_fd_setsize.h
main.o: /usr/include/sys/_types/_fd_set.h /usr/include/sys/_types/_fd_clr.h
main.o: /usr/include/sys/_types/_fd_isset.h
main.o: /usr/include/sys/_types/_fd_zero.h /usr/include/sys/_types/_fd_copy.h
main.o: /usr/include/sys/_select.h /usr/include/sys/_types/_dev_t.h
main.o: /usr/include/sys/_types/_mode_t.h /usr/include/sys/_types/_uuid_t.h
main.o: /usr/include/gethostuuid.h PDDL.hxx pddl_basic_types.hxx
main.o: pddl_string_table.hxx pddl_fluent_set.hxx bitarray.hxx nff_logic.hxx
main.o: options.hxx pr_obs_reader.hxx act_obs.hxx pr_strips_mapping.hxx
main.o: strips_writer.hxx string_ops.hxx
