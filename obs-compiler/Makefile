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
	$(CC) -o $(NFF_TARGET) $(STATIC) $(PARSER_OBJ) $(NFF_OBJECTS) $(LDFLAGS) $(LIBS)

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
PDDL.o: /usr/include/features.h /usr/include/sys/cdefs.h
PDDL.o: /usr/include/bits/wordsize.h /usr/include/gnu/stubs.h
PDDL.o: /usr/include/gnu/stubs-32.h /usr/include/bits/wchar.h utils.hxx
PDDL.o: /usr/include/sys/times.h /usr/include/time.h
PDDL.o: /usr/include/bits/types.h /usr/include/bits/typesizes.h
PDDL.o: /usr/include/sys/resource.h /usr/include/bits/resource.h
PDDL.o: /usr/include/bits/time.h /usr/include/unistd.h
PDDL.o: /usr/include/bits/posix_opt.h /usr/include/bits/confname.h
PDDL.o: /usr/include/getopt.h options.hxx mod-metric-ff/libff.h
PDDL.o: mod-metric-ff/ff.h /usr/include/stdlib.h /usr/include/sys/types.h
PDDL.o: /usr/include/endian.h /usr/include/bits/endian.h
PDDL.o: /usr/include/sys/select.h /usr/include/bits/select.h
PDDL.o: /usr/include/bits/sigset.h /usr/include/sys/sysmacros.h
PDDL.o: /usr/include/bits/pthreadtypes.h /usr/include/alloca.h
PDDL.o: /usr/include/stdio.h /usr/include/libio.h /usr/include/_G_config.h
PDDL.o: /usr/include/wchar.h /usr/include/bits/stdio_lim.h
PDDL.o: /usr/include/bits/sys_errlist.h /usr/include/strings.h
PDDL.o: /usr/include/ctype.h /usr/include/sys/timeb.h
pddl_fluent_set.o: pddl_fluent_set.hxx bitarray.hxx
pddl_string_table.o: pddl_string_table.hxx pddl_basic_types.hxx
bitarray.o: bitarray.hxx
global_options.o: global_options.hxx
options.o: options.hxx /usr/include/getopt.h
pr_obs_reader.o: pr_obs_reader.hxx act_obs.hxx string_ops.hxx PDDL.hxx
pr_obs_reader.o: pddl_basic_types.hxx pddl_string_table.hxx
pr_obs_reader.o: pddl_fluent_set.hxx bitarray.hxx nff_logic.hxx
pr_obs_reader.o: /usr/include/stdint.h /usr/include/features.h
pr_obs_reader.o: /usr/include/sys/cdefs.h /usr/include/bits/wordsize.h
pr_obs_reader.o: /usr/include/gnu/stubs.h /usr/include/gnu/stubs-32.h
pr_obs_reader.o: /usr/include/bits/wchar.h
pr_strips_mapping.o: pr_strips_mapping.hxx strips_writer.hxx act_obs.hxx
pr_strips_mapping.o: PDDL.hxx pddl_basic_types.hxx pddl_string_table.hxx
pr_strips_mapping.o: pddl_fluent_set.hxx bitarray.hxx nff_logic.hxx
pr_strips_mapping.o: /usr/include/stdint.h /usr/include/features.h
pr_strips_mapping.o: /usr/include/sys/cdefs.h /usr/include/bits/wordsize.h
pr_strips_mapping.o: /usr/include/gnu/stubs.h /usr/include/gnu/stubs-32.h
pr_strips_mapping.o: /usr/include/bits/wchar.h options.hxx
strips_writer.o: strips_writer.hxx PDDL.hxx pddl_basic_types.hxx
strips_writer.o: pddl_string_table.hxx pddl_fluent_set.hxx bitarray.hxx
strips_writer.o: nff_logic.hxx /usr/include/stdint.h /usr/include/features.h
strips_writer.o: /usr/include/sys/cdefs.h /usr/include/bits/wordsize.h
strips_writer.o: /usr/include/gnu/stubs.h /usr/include/gnu/stubs-32.h
strips_writer.o: /usr/include/bits/wchar.h string_ops.hxx
act_obs.o: act_obs.hxx pddl_string_table.hxx pddl_basic_types.hxx PDDL.hxx
act_obs.o: pddl_fluent_set.hxx bitarray.hxx nff_logic.hxx
act_obs.o: /usr/include/stdint.h /usr/include/features.h
act_obs.o: /usr/include/sys/cdefs.h /usr/include/bits/wordsize.h
act_obs.o: /usr/include/gnu/stubs.h /usr/include/gnu/stubs-32.h
act_obs.o: /usr/include/bits/wchar.h
main.o: /usr/include/signal.h /usr/include/features.h
main.o: /usr/include/sys/cdefs.h /usr/include/bits/wordsize.h
main.o: /usr/include/gnu/stubs.h /usr/include/gnu/stubs-32.h
main.o: /usr/include/bits/sigset.h /usr/include/bits/types.h
main.o: /usr/include/bits/typesizes.h /usr/include/bits/signum.h
main.o: /usr/include/time.h /usr/include/bits/siginfo.h
main.o: /usr/include/bits/sigaction.h /usr/include/bits/sigcontext.h
main.o: /usr/include/bits/sigstack.h /usr/include/bits/pthreadtypes.h
main.o: /usr/include/bits/sigthread.h utils.hxx /usr/include/sys/times.h
main.o: /usr/include/sys/resource.h /usr/include/bits/resource.h
main.o: /usr/include/bits/time.h /usr/include/unistd.h
main.o: /usr/include/bits/posix_opt.h /usr/include/bits/confname.h
main.o: /usr/include/getopt.h PDDL.hxx pddl_basic_types.hxx
main.o: pddl_string_table.hxx pddl_fluent_set.hxx bitarray.hxx nff_logic.hxx
main.o: /usr/include/stdint.h /usr/include/bits/wchar.h options.hxx
main.o: pr_obs_reader.hxx act_obs.hxx pr_strips_mapping.hxx strips_writer.hxx
main.o: string_ops.hxx
