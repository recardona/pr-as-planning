CC = g++
BISON = bison
FLEX = flex

NFF_TARGET = subopt_PR

CXXFLAGS = -g -Wall -Imod-metric-ff -DPDDL_TYPE_CHECKING
#CXXFLAGS = -O3 -Wall -Imod-metric-ff -DNDEBUG
LDFLAGS = -Lmod-metric-ff/
#EFENCE = -lefence
STATIC = -static
LIBS = -lff $(EFENCE)

NFF_SOURCES = main-PR.cxx \
	nff_options.cxx \
	PDDL.cxx \
	pddl_fluent_set.cxx \
	pddl_string_table.cxx \
	bitarray.cxx \
	nff_h_add_plus.cxx \
	nff_h_add.cxx \
	nff_h2.cxx \
	nff_state.cxx \
	nff_c3_bfs.cxx \
	nff_h1.cxx \
	nff_const_h1.cxx \
	nff_ext_state.cxx \
	nff_prop_util.cxx \
	nff.cxx \
	nff_node.cxx \
	nff_dfs_node.cxx \
	global_options.cxx \
	nff_algobase.cxx \
	nff_c3_at_bfs.cxx \
	nff_relaxed_plan.cxx \
	nff_best_supporter.cxx \
	nff_pb_best_supporter.cxx \
	nff_prim_best_supporter.cxx \
	nff_hadd_best_supporter.cxx \
	nff_hsa_best_supporter.cxx \
	nff_rp_heuristic.cxx \
	nff_heuristic_adapter.cxx \
	nff_harp_adapter.cxx \
	nff_min_adapter.cxx \
	nff_helpful.cxx \
	nff_bfs.cxx


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
	@cp $(NFF_TARGET) ../../plan-libs

# dependencies
depend:
	# Updating dependencies
	@makedepend -- $(CXXFLAGS) -- $(NFF_SOURCES)

# Cleaning
clean:
	rm -rf $(NFF_OBJECTS) $(NFF_TARGET) $(PARSER_SRC) $(PARSER_HDR) *.o *.hh
# DO NOT DELETE

main-PR.o: /usr/include/signal.h /usr/include/features.h
main-PR.o: /usr/include/sys/cdefs.h /usr/include/bits/wordsize.h
main-PR.o: /usr/include/gnu/stubs.h /usr/include/gnu/stubs-32.h
main-PR.o: /usr/include/bits/sigset.h /usr/include/bits/types.h
main-PR.o: /usr/include/bits/typesizes.h /usr/include/bits/signum.h
main-PR.o: /usr/include/time.h /usr/include/bits/siginfo.h
main-PR.o: /usr/include/bits/sigaction.h /usr/include/bits/sigcontext.h
main-PR.o: /usr/include/bits/sigstack.h /usr/include/bits/pthreadtypes.h
main-PR.o: /usr/include/bits/sigthread.h utils.hxx /usr/include/sys/times.h
main-PR.o: /usr/include/sys/resource.h /usr/include/bits/resource.h
main-PR.o: /usr/include/bits/time.h /usr/include/unistd.h
main-PR.o: /usr/include/bits/posix_opt.h /usr/include/bits/confname.h
main-PR.o: /usr/include/getopt.h PDDL.hxx pddl_basic_types.hxx
main-PR.o: pddl_string_table.hxx pddl_fluent_set.hxx bitarray.hxx
main-PR.o: nff_logic.hxx /usr/include/stdint.h /usr/include/bits/wchar.h
main-PR.o: nff_options.hxx nff_h2.hxx ext_math.hxx nff.hxx nff_state.hxx
main-PR.o: nff_hsa_best_supporter.hxx nff_pb_best_supporter.hxx
main-PR.o: nff_best_supporter.hxx nff_relaxed_plan.hxx nff_harp_adapter.hxx
main-PR.o: nff_heuristic_adapter.hxx nff_heuristic.hxx nff_rp_heuristic.hxx
main-PR.o: nff_helpful.hxx nff_ehc.hxx nff_dfs_node.hxx jenkins_12bit.hxx
main-PR.o: pddl_fluent_table.hxx nff_algobase.hxx nff_bfs.hxx nff_node.hxx
nff_options.o: nff_options.hxx global_options.hxx /usr/include/getopt.h
PDDL.o: PDDL.hxx pddl_basic_types.hxx pddl_string_table.hxx
PDDL.o: pddl_fluent_set.hxx bitarray.hxx nff_logic.hxx /usr/include/stdint.h
PDDL.o: /usr/include/features.h /usr/include/sys/cdefs.h
PDDL.o: /usr/include/bits/wordsize.h /usr/include/gnu/stubs.h
PDDL.o: /usr/include/gnu/stubs-32.h /usr/include/bits/wchar.h nff_h2.hxx
PDDL.o: ext_math.hxx nff.hxx nff_state.hxx utils.hxx /usr/include/sys/times.h
PDDL.o: /usr/include/time.h /usr/include/bits/types.h
PDDL.o: /usr/include/bits/typesizes.h /usr/include/sys/resource.h
PDDL.o: /usr/include/bits/resource.h /usr/include/bits/time.h
PDDL.o: /usr/include/unistd.h /usr/include/bits/posix_opt.h
PDDL.o: /usr/include/bits/confname.h /usr/include/getopt.h global_options.hxx
PDDL.o: mod-metric-ff/libff.h mod-metric-ff/ff.h /usr/include/stdlib.h
PDDL.o: /usr/include/sys/types.h /usr/include/endian.h
PDDL.o: /usr/include/bits/endian.h /usr/include/sys/select.h
PDDL.o: /usr/include/bits/select.h /usr/include/bits/sigset.h
PDDL.o: /usr/include/sys/sysmacros.h /usr/include/bits/pthreadtypes.h
PDDL.o: /usr/include/alloca.h /usr/include/stdio.h /usr/include/libio.h
PDDL.o: /usr/include/_G_config.h /usr/include/wchar.h
PDDL.o: /usr/include/bits/stdio_lim.h /usr/include/bits/sys_errlist.h
PDDL.o: /usr/include/strings.h /usr/include/ctype.h /usr/include/sys/timeb.h
pddl_fluent_set.o: pddl_fluent_set.hxx bitarray.hxx
pddl_string_table.o: pddl_string_table.hxx pddl_basic_types.hxx
bitarray.o: bitarray.hxx
nff_h_add_plus.o: nff_h_add_plus.hxx nff_h_add.hxx ext_math.hxx PDDL.hxx
nff_h_add_plus.o: pddl_basic_types.hxx pddl_string_table.hxx
nff_h_add_plus.o: pddl_fluent_set.hxx bitarray.hxx nff_logic.hxx
nff_h_add_plus.o: /usr/include/stdint.h /usr/include/features.h
nff_h_add_plus.o: /usr/include/sys/cdefs.h /usr/include/bits/wordsize.h
nff_h_add_plus.o: /usr/include/gnu/stubs.h /usr/include/gnu/stubs-32.h
nff_h_add_plus.o: /usr/include/bits/wchar.h nff.hxx nff_state.hxx
nff_h_add.o: nff_h_add.hxx ext_math.hxx PDDL.hxx pddl_basic_types.hxx
nff_h_add.o: pddl_string_table.hxx pddl_fluent_set.hxx bitarray.hxx
nff_h_add.o: nff_logic.hxx /usr/include/stdint.h /usr/include/features.h
nff_h_add.o: /usr/include/sys/cdefs.h /usr/include/bits/wordsize.h
nff_h_add.o: /usr/include/gnu/stubs.h /usr/include/gnu/stubs-32.h
nff_h_add.o: /usr/include/bits/wchar.h nff.hxx nff_state.hxx
nff_h2.o: nff_h2.hxx ext_math.hxx PDDL.hxx pddl_basic_types.hxx
nff_h2.o: pddl_string_table.hxx pddl_fluent_set.hxx bitarray.hxx
nff_h2.o: nff_logic.hxx /usr/include/stdint.h /usr/include/features.h
nff_h2.o: /usr/include/sys/cdefs.h /usr/include/bits/wordsize.h
nff_h2.o: /usr/include/gnu/stubs.h /usr/include/gnu/stubs-32.h
nff_h2.o: /usr/include/bits/wchar.h nff.hxx nff_state.hxx nff_h1.hxx
nff_h2.o: square_matrix.hxx nff_options.hxx
nff_state.o: nff_state.hxx PDDL.hxx pddl_basic_types.hxx
nff_state.o: pddl_string_table.hxx pddl_fluent_set.hxx bitarray.hxx
nff_state.o: nff_logic.hxx /usr/include/stdint.h /usr/include/features.h
nff_state.o: /usr/include/sys/cdefs.h /usr/include/bits/wordsize.h
nff_state.o: /usr/include/gnu/stubs.h /usr/include/gnu/stubs-32.h
nff_state.o: /usr/include/bits/wchar.h
nff_c3_bfs.o: nff_c3_bfs.hxx nff_node.hxx nff.hxx PDDL.hxx
nff_c3_bfs.o: pddl_basic_types.hxx pddl_string_table.hxx pddl_fluent_set.hxx
nff_c3_bfs.o: bitarray.hxx nff_logic.hxx /usr/include/stdint.h
nff_c3_bfs.o: /usr/include/features.h /usr/include/sys/cdefs.h
nff_c3_bfs.o: /usr/include/bits/wordsize.h /usr/include/gnu/stubs.h
nff_c3_bfs.o: /usr/include/gnu/stubs-32.h /usr/include/bits/wchar.h
nff_c3_bfs.o: nff_state.hxx jenkins_12bit.hxx nff_h_add_plus.hxx
nff_c3_bfs.o: nff_h_add.hxx ext_math.hxx nff_h2.hxx pddl_fluent_table.hxx
nff_c3_bfs.o: nff_options.hxx
nff_h1.o: nff_h1.hxx ext_math.hxx PDDL.hxx pddl_basic_types.hxx
nff_h1.o: pddl_string_table.hxx pddl_fluent_set.hxx bitarray.hxx
nff_h1.o: nff_logic.hxx /usr/include/stdint.h /usr/include/features.h
nff_h1.o: /usr/include/sys/cdefs.h /usr/include/bits/wordsize.h
nff_h1.o: /usr/include/gnu/stubs.h /usr/include/gnu/stubs-32.h
nff_h1.o: /usr/include/bits/wchar.h nff.hxx nff_state.hxx
nff_const_h1.o: nff_const_h1.hxx ext_math.hxx PDDL.hxx pddl_basic_types.hxx
nff_const_h1.o: pddl_string_table.hxx pddl_fluent_set.hxx bitarray.hxx
nff_const_h1.o: nff_logic.hxx /usr/include/stdint.h /usr/include/features.h
nff_const_h1.o: /usr/include/sys/cdefs.h /usr/include/bits/wordsize.h
nff_const_h1.o: /usr/include/gnu/stubs.h /usr/include/gnu/stubs-32.h
nff_const_h1.o: /usr/include/bits/wchar.h nff.hxx nff_state.hxx
nff_ext_state.o: nff_ext_state.hxx PDDL.hxx pddl_basic_types.hxx
nff_ext_state.o: pddl_string_table.hxx pddl_fluent_set.hxx bitarray.hxx
nff_ext_state.o: nff_logic.hxx /usr/include/stdint.h /usr/include/features.h
nff_ext_state.o: /usr/include/sys/cdefs.h /usr/include/bits/wordsize.h
nff_ext_state.o: /usr/include/gnu/stubs.h /usr/include/gnu/stubs-32.h
nff_ext_state.o: /usr/include/bits/wchar.h nff.hxx nff_state.hxx
nff_prop_util.o: nff_prop_util.hxx nff_ext_state.hxx PDDL.hxx
nff_prop_util.o: pddl_basic_types.hxx pddl_string_table.hxx
nff_prop_util.o: pddl_fluent_set.hxx bitarray.hxx nff_logic.hxx
nff_prop_util.o: /usr/include/stdint.h /usr/include/features.h
nff_prop_util.o: /usr/include/sys/cdefs.h /usr/include/bits/wordsize.h
nff_prop_util.o: /usr/include/gnu/stubs.h /usr/include/gnu/stubs-32.h
nff_prop_util.o: /usr/include/bits/wchar.h nff.hxx nff_state.hxx
nff.o: nff.hxx PDDL.hxx pddl_basic_types.hxx pddl_string_table.hxx
nff.o: pddl_fluent_set.hxx bitarray.hxx nff_logic.hxx /usr/include/stdint.h
nff.o: /usr/include/features.h /usr/include/sys/cdefs.h
nff.o: /usr/include/bits/wordsize.h /usr/include/gnu/stubs.h
nff.o: /usr/include/gnu/stubs-32.h /usr/include/bits/wchar.h
nff_node.o: nff_node.hxx nff.hxx PDDL.hxx pddl_basic_types.hxx
nff_node.o: pddl_string_table.hxx pddl_fluent_set.hxx bitarray.hxx
nff_node.o: nff_logic.hxx /usr/include/stdint.h /usr/include/features.h
nff_node.o: /usr/include/sys/cdefs.h /usr/include/bits/wordsize.h
nff_node.o: /usr/include/gnu/stubs.h /usr/include/gnu/stubs-32.h
nff_node.o: /usr/include/bits/wchar.h nff_state.hxx jenkins_12bit.hxx
nff_dfs_node.o: nff_dfs_node.hxx nff.hxx PDDL.hxx pddl_basic_types.hxx
nff_dfs_node.o: pddl_string_table.hxx pddl_fluent_set.hxx bitarray.hxx
nff_dfs_node.o: nff_logic.hxx /usr/include/stdint.h /usr/include/features.h
nff_dfs_node.o: /usr/include/sys/cdefs.h /usr/include/bits/wordsize.h
nff_dfs_node.o: /usr/include/gnu/stubs.h /usr/include/gnu/stubs-32.h
nff_dfs_node.o: /usr/include/bits/wchar.h nff_state.hxx jenkins_12bit.hxx
global_options.o: global_options.hxx
nff_algobase.o: nff_algobase.hxx nff.hxx PDDL.hxx pddl_basic_types.hxx
nff_algobase.o: pddl_string_table.hxx pddl_fluent_set.hxx bitarray.hxx
nff_algobase.o: nff_logic.hxx /usr/include/stdint.h /usr/include/features.h
nff_algobase.o: /usr/include/sys/cdefs.h /usr/include/bits/wordsize.h
nff_algobase.o: /usr/include/gnu/stubs.h /usr/include/gnu/stubs-32.h
nff_algobase.o: /usr/include/bits/wchar.h
nff_c3_at_bfs.o: nff_c3_at_bfs.hxx nff_node.hxx nff.hxx PDDL.hxx
nff_c3_at_bfs.o: pddl_basic_types.hxx pddl_string_table.hxx
nff_c3_at_bfs.o: pddl_fluent_set.hxx bitarray.hxx nff_logic.hxx
nff_c3_at_bfs.o: /usr/include/stdint.h /usr/include/features.h
nff_c3_at_bfs.o: /usr/include/sys/cdefs.h /usr/include/bits/wordsize.h
nff_c3_at_bfs.o: /usr/include/gnu/stubs.h /usr/include/gnu/stubs-32.h
nff_c3_at_bfs.o: /usr/include/bits/wchar.h nff_state.hxx jenkins_12bit.hxx
nff_c3_at_bfs.o: nff_h_add_plus.hxx nff_h_add.hxx ext_math.hxx nff_h2.hxx
nff_c3_at_bfs.o: pddl_fluent_table.hxx nff_options.hxx
nff_relaxed_plan.o: nff_relaxed_plan.hxx PDDL.hxx pddl_basic_types.hxx
nff_relaxed_plan.o: pddl_string_table.hxx pddl_fluent_set.hxx bitarray.hxx
nff_relaxed_plan.o: nff_logic.hxx /usr/include/stdint.h
nff_relaxed_plan.o: /usr/include/features.h /usr/include/sys/cdefs.h
nff_relaxed_plan.o: /usr/include/bits/wordsize.h /usr/include/gnu/stubs.h
nff_relaxed_plan.o: /usr/include/gnu/stubs-32.h /usr/include/bits/wchar.h
nff_relaxed_plan.o: nff.hxx nff_state.hxx
nff_best_supporter.o: nff_best_supporter.hxx nff.hxx PDDL.hxx
nff_best_supporter.o: pddl_basic_types.hxx pddl_string_table.hxx
nff_best_supporter.o: pddl_fluent_set.hxx bitarray.hxx nff_logic.hxx
nff_best_supporter.o: /usr/include/stdint.h /usr/include/features.h
nff_best_supporter.o: /usr/include/sys/cdefs.h /usr/include/bits/wordsize.h
nff_best_supporter.o: /usr/include/gnu/stubs.h /usr/include/gnu/stubs-32.h
nff_best_supporter.o: /usr/include/bits/wchar.h nff_state.hxx
nff_best_supporter.o: nff_relaxed_plan.hxx nff_options.hxx
nff_pb_best_supporter.o: nff_pb_best_supporter.hxx nff_best_supporter.hxx
nff_pb_best_supporter.o: nff.hxx PDDL.hxx pddl_basic_types.hxx
nff_pb_best_supporter.o: pddl_string_table.hxx pddl_fluent_set.hxx
nff_pb_best_supporter.o: bitarray.hxx nff_logic.hxx /usr/include/stdint.h
nff_pb_best_supporter.o: /usr/include/features.h /usr/include/sys/cdefs.h
nff_pb_best_supporter.o: /usr/include/bits/wordsize.h
nff_pb_best_supporter.o: /usr/include/gnu/stubs.h /usr/include/gnu/stubs-32.h
nff_pb_best_supporter.o: /usr/include/bits/wchar.h nff_state.hxx
nff_pb_best_supporter.o: nff_relaxed_plan.hxx
nff_prim_best_supporter.o: nff_prim_best_supporter.hxx nff_best_supporter.hxx
nff_prim_best_supporter.o: nff.hxx PDDL.hxx pddl_basic_types.hxx
nff_prim_best_supporter.o: pddl_string_table.hxx pddl_fluent_set.hxx
nff_prim_best_supporter.o: bitarray.hxx nff_logic.hxx /usr/include/stdint.h
nff_prim_best_supporter.o: /usr/include/features.h /usr/include/sys/cdefs.h
nff_prim_best_supporter.o: /usr/include/bits/wordsize.h
nff_prim_best_supporter.o: /usr/include/gnu/stubs.h
nff_prim_best_supporter.o: /usr/include/gnu/stubs-32.h
nff_prim_best_supporter.o: /usr/include/bits/wchar.h nff_state.hxx
nff_prim_best_supporter.o: nff_relaxed_plan.hxx
nff_hadd_best_supporter.o: nff_hadd_best_supporter.hxx
nff_hadd_best_supporter.o: nff_pb_best_supporter.hxx nff_best_supporter.hxx
nff_hadd_best_supporter.o: nff.hxx PDDL.hxx pddl_basic_types.hxx
nff_hadd_best_supporter.o: pddl_string_table.hxx pddl_fluent_set.hxx
nff_hadd_best_supporter.o: bitarray.hxx nff_logic.hxx /usr/include/stdint.h
nff_hadd_best_supporter.o: /usr/include/features.h /usr/include/sys/cdefs.h
nff_hadd_best_supporter.o: /usr/include/bits/wordsize.h
nff_hadd_best_supporter.o: /usr/include/gnu/stubs.h
nff_hadd_best_supporter.o: /usr/include/gnu/stubs-32.h
nff_hadd_best_supporter.o: /usr/include/bits/wchar.h nff_state.hxx
nff_hadd_best_supporter.o: nff_relaxed_plan.hxx ext_math.hxx
nff_hsa_best_supporter.o: nff_hsa_best_supporter.hxx
nff_hsa_best_supporter.o: nff_pb_best_supporter.hxx nff_best_supporter.hxx
nff_hsa_best_supporter.o: nff.hxx PDDL.hxx pddl_basic_types.hxx
nff_hsa_best_supporter.o: pddl_string_table.hxx pddl_fluent_set.hxx
nff_hsa_best_supporter.o: bitarray.hxx nff_logic.hxx /usr/include/stdint.h
nff_hsa_best_supporter.o: /usr/include/features.h /usr/include/sys/cdefs.h
nff_hsa_best_supporter.o: /usr/include/bits/wordsize.h
nff_hsa_best_supporter.o: /usr/include/gnu/stubs.h
nff_hsa_best_supporter.o: /usr/include/gnu/stubs-32.h
nff_hsa_best_supporter.o: /usr/include/bits/wchar.h nff_state.hxx
nff_hsa_best_supporter.o: nff_relaxed_plan.hxx ext_math.hxx nff_options.hxx
nff_rp_heuristic.o: nff_rp_heuristic.hxx nff_heuristic.hxx PDDL.hxx
nff_rp_heuristic.o: pddl_basic_types.hxx pddl_string_table.hxx
nff_rp_heuristic.o: pddl_fluent_set.hxx bitarray.hxx nff_logic.hxx
nff_rp_heuristic.o: /usr/include/stdint.h /usr/include/features.h
nff_rp_heuristic.o: /usr/include/sys/cdefs.h /usr/include/bits/wordsize.h
nff_rp_heuristic.o: /usr/include/gnu/stubs.h /usr/include/gnu/stubs-32.h
nff_rp_heuristic.o: /usr/include/bits/wchar.h nff_state.hxx
nff_rp_heuristic.o: nff_best_supporter.hxx nff.hxx nff_relaxed_plan.hxx
nff_heuristic_adapter.o: nff_heuristic_adapter.hxx nff_heuristic.hxx PDDL.hxx
nff_heuristic_adapter.o: pddl_basic_types.hxx pddl_string_table.hxx
nff_heuristic_adapter.o: pddl_fluent_set.hxx bitarray.hxx nff_logic.hxx
nff_heuristic_adapter.o: /usr/include/stdint.h /usr/include/features.h
nff_heuristic_adapter.o: /usr/include/sys/cdefs.h
nff_heuristic_adapter.o: /usr/include/bits/wordsize.h
nff_heuristic_adapter.o: /usr/include/gnu/stubs.h /usr/include/gnu/stubs-32.h
nff_heuristic_adapter.o: /usr/include/bits/wchar.h nff_state.hxx
nff_harp_adapter.o: nff_harp_adapter.hxx nff_heuristic_adapter.hxx
nff_harp_adapter.o: nff_heuristic.hxx PDDL.hxx pddl_basic_types.hxx
nff_harp_adapter.o: pddl_string_table.hxx pddl_fluent_set.hxx bitarray.hxx
nff_harp_adapter.o: nff_logic.hxx /usr/include/stdint.h
nff_harp_adapter.o: /usr/include/features.h /usr/include/sys/cdefs.h
nff_harp_adapter.o: /usr/include/bits/wordsize.h /usr/include/gnu/stubs.h
nff_harp_adapter.o: /usr/include/gnu/stubs-32.h /usr/include/bits/wchar.h
nff_harp_adapter.o: nff_state.hxx nff_rp_heuristic.hxx nff_best_supporter.hxx
nff_harp_adapter.o: nff.hxx nff_relaxed_plan.hxx nff_helpful.hxx
nff_min_adapter.o: nff_min_adapter.hxx nff_heuristic_adapter.hxx
nff_min_adapter.o: nff_heuristic.hxx PDDL.hxx pddl_basic_types.hxx
nff_min_adapter.o: pddl_string_table.hxx pddl_fluent_set.hxx bitarray.hxx
nff_min_adapter.o: nff_logic.hxx /usr/include/stdint.h
nff_min_adapter.o: /usr/include/features.h /usr/include/sys/cdefs.h
nff_min_adapter.o: /usr/include/bits/wordsize.h /usr/include/gnu/stubs.h
nff_min_adapter.o: /usr/include/gnu/stubs-32.h /usr/include/bits/wchar.h
nff_min_adapter.o: nff_state.hxx
nff_helpful.o: nff_state.hxx PDDL.hxx pddl_basic_types.hxx
nff_helpful.o: pddl_string_table.hxx pddl_fluent_set.hxx bitarray.hxx
nff_helpful.o: nff_logic.hxx /usr/include/stdint.h /usr/include/features.h
nff_helpful.o: /usr/include/sys/cdefs.h /usr/include/bits/wordsize.h
nff_helpful.o: /usr/include/gnu/stubs.h /usr/include/gnu/stubs-32.h
nff_helpful.o: /usr/include/bits/wchar.h nff_helpful.hxx nff_relaxed_plan.hxx
nff_helpful.o: nff.hxx nff_best_supporter.hxx
nff_bfs.o: nff_bfs.hxx nff_node.hxx nff.hxx PDDL.hxx pddl_basic_types.hxx
nff_bfs.o: pddl_string_table.hxx pddl_fluent_set.hxx bitarray.hxx
nff_bfs.o: nff_logic.hxx /usr/include/stdint.h /usr/include/features.h
nff_bfs.o: /usr/include/sys/cdefs.h /usr/include/bits/wordsize.h
nff_bfs.o: /usr/include/gnu/stubs.h /usr/include/gnu/stubs-32.h
nff_bfs.o: /usr/include/bits/wchar.h nff_state.hxx jenkins_12bit.hxx
nff_bfs.o: nff_h2.hxx ext_math.hxx pddl_fluent_table.hxx
nff_bfs.o: nff_hsa_best_supporter.hxx nff_pb_best_supporter.hxx
nff_bfs.o: nff_best_supporter.hxx nff_relaxed_plan.hxx nff_harp_adapter.hxx
nff_bfs.o: nff_heuristic_adapter.hxx nff_heuristic.hxx nff_rp_heuristic.hxx
nff_bfs.o: nff_helpful.hxx nff_options.hxx
