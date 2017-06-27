#!/bin/sh
#
# Makefile for FF v 1.0
#


####### FLAGS

TYPE	=
ADDONS	=

CC      = gcc

CFLAGS	= -O3 -Wall -g -ansi $(TYPE) $(ADDONS)
# -g -pg

LIBS    = -lm


####### Files

PDDL_PARSER_SRC	= scan-fct_pddl.tab.c \
	scan-ops_pddl.tab.c \
	scan-probname.tab.c \
	lex.fct_pddl.c \
	lex.ops_pddl.c

PDDL_PARSER_OBJ = scan-fct_pddl.tab.o \
	scan-ops_pddl.tab.o


SOURCES 	= main.c \
	memory.c \
	output.c \
	parse.c \
	expressions.c \
	inst_pre.c \
	inst_easy.c \
	inst_hard.c \
	inst_final.c \
	relax.c \
	search.c \
	utility.c

LIB_SOURCES = libff.c \
	memory.c \
	output.c \
	parse.c \
	expressions.c \
	inst_pre.c \
	inst_easy.c \
	inst_hard.c \
	inst_final.c \
	relax.c \
	search.c \
	utility.c


OBJECTS 	= $(SOURCES:.c=.o)
LIB_OBJECTS	= $(LIB_SOURCES:.c=.o)

####### Implicit rules

.SUFFIXES:

.SUFFIXES: .c .o

.c.o:; $(CC) -c $(CFLAGS) $<

####### Build rules

libff: $(LIB_OBJECTS) $(PDDL_PARSER_OBJ)
	ar cru libff.a $(LIB_OBJECTS) $(PDDL_PARSER_OBJ)

ff: $(OBJECTS) $(PDDL_PARSER_OBJ)
	$(CC) -static -o ff $(OBJECTS) $(PDDL_PARSER_OBJ) $(CFLAGS) $(LIBS)

# pddl syntax
scan-fct_pddl.tab.c: scan-fct_pddl.y lex.fct_pddl.c
	bison -v -pfct_pddl -bscan-fct_pddl scan-fct_pddl.y

scan-ops_pddl.tab.c: scan-ops_pddl.y lex.ops_pddl.c
	bison -v -pops_pddl -bscan-ops_pddl scan-ops_pddl.y

lex.fct_pddl.c: lex-fct_pddl.l
	flex -Pfct_pddl lex-fct_pddl.l

lex.ops_pddl.c: lex-ops_pddl.l
	flex -Pops_pddl lex-ops_pddl.l

test_lib: test_main.cxx
	g++ -static -o test_lib test_main.cxx -lff -L./

# misc
clean:
	rm -f *.o *.bak *~ *% core *_pure_p9_c0_400.o.warnings test_lib \
        \#*\# $(RES_PARSER_SRC) $(PDDL_PARSER_SRC)

veryclean: clean
	rm -f ff H* J* K* L* O* graph.* *.symbex gmon.out \
	$(PDDL_PARSER_SRC) \
	lex.fct_pddl.c lex.ops_pddl.c lex.probname.c \
	*.output

depend:
	makedepend -- $(SOURCES)  $(LIB_SOURCES) $(PDDL_PARSER_SRC) test_main.cxx

lint:
	lclint -booltype Bool $(SOURCES) 2> output.lint

# DO NOT DELETE

main.o: ff.h /usr/include/stdlib.h /usr/include/Availability.h
main.o: /usr/include/AvailabilityInternal.h /usr/include/_types.h
main.o: /usr/include/sys/_types.h /usr/include/sys/cdefs.h
main.o: /usr/include/sys/_symbol_aliasing.h
main.o: /usr/include/sys/_posix_availability.h /usr/include/machine/_types.h
main.o: /usr/include/i386/_types.h /usr/include/sys/_pthread/_pthread_types.h
main.o: /usr/include/sys/wait.h /usr/include/sys/_types/_pid_t.h
main.o: /usr/include/sys/_types/_id_t.h /usr/include/sys/signal.h
main.o: /usr/include/sys/appleapiopts.h /usr/include/machine/signal.h
main.o: /usr/include/i386/signal.h /usr/include/machine/_mcontext.h
main.o: /usr/include/i386/_mcontext.h /usr/include/mach/i386/_structs.h
main.o: /usr/include/sys/_pthread/_pthread_attr_t.h
main.o: /usr/include/sys/_types/_sigaltstack.h
main.o: /usr/include/sys/_types/_ucontext.h
main.o: /usr/include/sys/_types/_sigset_t.h /usr/include/sys/_types/_size_t.h
main.o: /usr/include/sys/_types/_uid_t.h /usr/include/sys/resource.h
main.o: /usr/include/stdint.h /usr/include/sys/_types/_int8_t.h
main.o: /usr/include/sys/_types/_int16_t.h /usr/include/sys/_types/_int32_t.h
main.o: /usr/include/sys/_types/_int64_t.h /usr/include/_types/_uint8_t.h
main.o: /usr/include/_types/_uint16_t.h /usr/include/_types/_uint32_t.h
main.o: /usr/include/_types/_uint64_t.h /usr/include/sys/_types/_intptr_t.h
main.o: /usr/include/sys/_types/_uintptr_t.h /usr/include/_types/_intmax_t.h
main.o: /usr/include/_types/_uintmax_t.h /usr/include/sys/_types/_timeval.h
main.o: /usr/include/machine/endian.h /usr/include/i386/endian.h
main.o: /usr/include/sys/_endian.h /usr/include/libkern/_OSByteOrder.h
main.o: /usr/include/libkern/i386/_OSByteOrder.h /usr/include/alloca.h
main.o: /usr/include/sys/_types/_ct_rune_t.h
main.o: /usr/include/sys/_types/_rune_t.h /usr/include/sys/_types/_wchar_t.h
main.o: /usr/include/sys/_types/_null.h /usr/include/machine/types.h
main.o: /usr/include/i386/types.h /usr/include/sys/_types/_dev_t.h
main.o: /usr/include/sys/_types/_mode_t.h /usr/include/stdio.h
main.o: /usr/include/sys/_types/_va_list.h /usr/include/sys/stdio.h
main.o: /usr/include/sys/_types/_off_t.h /usr/include/sys/_types/_ssize_t.h
main.o: /usr/include/secure/_stdio.h /usr/include/secure/_common.h
main.o: /usr/include/strings.h /usr/include/string.h
main.o: /usr/include/sys/_types/_rsize_t.h /usr/include/sys/_types/_errno_t.h
main.o: /usr/include/secure/_string.h /usr/include/ctype.h
main.o: /usr/include/runetype.h /usr/include/sys/_types/_wint_t.h
main.o: /usr/include/sys/types.h /usr/include/sys/_types/_blkcnt_t.h
main.o: /usr/include/sys/_types/_blksize_t.h /usr/include/sys/_types/_gid_t.h
main.o: /usr/include/sys/_types/_in_addr_t.h
main.o: /usr/include/sys/_types/_in_port_t.h /usr/include/sys/_types/_ino_t.h
main.o: /usr/include/sys/_types/_ino64_t.h /usr/include/sys/_types/_key_t.h
main.o: /usr/include/sys/_types/_nlink_t.h /usr/include/sys/_types/_clock_t.h
main.o: /usr/include/sys/_types/_time_t.h
main.o: /usr/include/sys/_types/_useconds_t.h
main.o: /usr/include/sys/_types/_suseconds_t.h
main.o: /usr/include/sys/_types/_fd_def.h
main.o: /usr/include/sys/_types/_fd_setsize.h
main.o: /usr/include/sys/_types/_fd_set.h /usr/include/sys/_types/_fd_clr.h
main.o: /usr/include/sys/_types/_fd_zero.h
main.o: /usr/include/sys/_types/_fd_isset.h
main.o: /usr/include/sys/_types/_fd_copy.h
main.o: /usr/include/sys/_pthread/_pthread_cond_t.h
main.o: /usr/include/sys/_pthread/_pthread_condattr_t.h
main.o: /usr/include/sys/_pthread/_pthread_mutex_t.h
main.o: /usr/include/sys/_pthread/_pthread_mutexattr_t.h
main.o: /usr/include/sys/_pthread/_pthread_once_t.h
main.o: /usr/include/sys/_pthread/_pthread_rwlock_t.h
main.o: /usr/include/sys/_pthread/_pthread_rwlockattr_t.h
main.o: /usr/include/sys/_pthread/_pthread_t.h
main.o: /usr/include/sys/_pthread/_pthread_key_t.h
main.o: /usr/include/sys/_types/_fsblkcnt_t.h
main.o: /usr/include/sys/_types/_fsfilcnt_t.h /usr/include/sys/timeb.h
main.o: /usr/include/sys/times.h memory.h output.h parse.h expressions.h
main.o: inst_pre.h inst_easy.h inst_hard.h inst_final.h relax.h search.h
memory.o: /usr/include/string.h /usr/include/_types.h
memory.o: /usr/include/sys/_types.h /usr/include/sys/cdefs.h
memory.o: /usr/include/sys/_symbol_aliasing.h
memory.o: /usr/include/sys/_posix_availability.h
memory.o: /usr/include/machine/_types.h /usr/include/i386/_types.h
memory.o: /usr/include/sys/_pthread/_pthread_types.h
memory.o: /usr/include/Availability.h /usr/include/AvailabilityInternal.h
memory.o: /usr/include/sys/_types/_size_t.h /usr/include/sys/_types/_null.h
memory.o: /usr/include/sys/_types/_rsize_t.h
memory.o: /usr/include/sys/_types/_errno_t.h
memory.o: /usr/include/sys/_types/_ssize_t.h /usr/include/strings.h
memory.o: /usr/include/secure/_string.h /usr/include/secure/_common.h ff.h
memory.o: /usr/include/stdlib.h /usr/include/sys/wait.h
memory.o: /usr/include/sys/_types/_pid_t.h /usr/include/sys/_types/_id_t.h
memory.o: /usr/include/sys/signal.h /usr/include/sys/appleapiopts.h
memory.o: /usr/include/machine/signal.h /usr/include/i386/signal.h
memory.o: /usr/include/machine/_mcontext.h /usr/include/i386/_mcontext.h
memory.o: /usr/include/mach/i386/_structs.h
memory.o: /usr/include/sys/_pthread/_pthread_attr_t.h
memory.o: /usr/include/sys/_types/_sigaltstack.h
memory.o: /usr/include/sys/_types/_ucontext.h
memory.o: /usr/include/sys/_types/_sigset_t.h
memory.o: /usr/include/sys/_types/_uid_t.h /usr/include/sys/resource.h
memory.o: /usr/include/stdint.h /usr/include/sys/_types/_int8_t.h
memory.o: /usr/include/sys/_types/_int16_t.h
memory.o: /usr/include/sys/_types/_int32_t.h
memory.o: /usr/include/sys/_types/_int64_t.h /usr/include/_types/_uint8_t.h
memory.o: /usr/include/_types/_uint16_t.h /usr/include/_types/_uint32_t.h
memory.o: /usr/include/_types/_uint64_t.h /usr/include/sys/_types/_intptr_t.h
memory.o: /usr/include/sys/_types/_uintptr_t.h
memory.o: /usr/include/_types/_intmax_t.h /usr/include/_types/_uintmax_t.h
memory.o: /usr/include/sys/_types/_timeval.h /usr/include/machine/endian.h
memory.o: /usr/include/i386/endian.h /usr/include/sys/_endian.h
memory.o: /usr/include/libkern/_OSByteOrder.h
memory.o: /usr/include/libkern/i386/_OSByteOrder.h /usr/include/alloca.h
memory.o: /usr/include/sys/_types/_ct_rune_t.h
memory.o: /usr/include/sys/_types/_rune_t.h
memory.o: /usr/include/sys/_types/_wchar_t.h /usr/include/machine/types.h
memory.o: /usr/include/i386/types.h /usr/include/sys/_types/_dev_t.h
memory.o: /usr/include/sys/_types/_mode_t.h /usr/include/stdio.h
memory.o: /usr/include/sys/_types/_va_list.h /usr/include/sys/stdio.h
memory.o: /usr/include/sys/_types/_off_t.h /usr/include/secure/_stdio.h
memory.o: /usr/include/ctype.h /usr/include/runetype.h
memory.o: /usr/include/sys/_types/_wint_t.h /usr/include/sys/types.h
memory.o: /usr/include/sys/_types/_blkcnt_t.h
memory.o: /usr/include/sys/_types/_blksize_t.h
memory.o: /usr/include/sys/_types/_gid_t.h
memory.o: /usr/include/sys/_types/_in_addr_t.h
memory.o: /usr/include/sys/_types/_in_port_t.h
memory.o: /usr/include/sys/_types/_ino_t.h /usr/include/sys/_types/_ino64_t.h
memory.o: /usr/include/sys/_types/_key_t.h /usr/include/sys/_types/_nlink_t.h
memory.o: /usr/include/sys/_types/_clock_t.h
memory.o: /usr/include/sys/_types/_time_t.h
memory.o: /usr/include/sys/_types/_useconds_t.h
memory.o: /usr/include/sys/_types/_suseconds_t.h
memory.o: /usr/include/sys/_types/_fd_def.h
memory.o: /usr/include/sys/_types/_fd_setsize.h
memory.o: /usr/include/sys/_types/_fd_set.h /usr/include/sys/_types/_fd_clr.h
memory.o: /usr/include/sys/_types/_fd_zero.h
memory.o: /usr/include/sys/_types/_fd_isset.h
memory.o: /usr/include/sys/_types/_fd_copy.h
memory.o: /usr/include/sys/_pthread/_pthread_cond_t.h
memory.o: /usr/include/sys/_pthread/_pthread_condattr_t.h
memory.o: /usr/include/sys/_pthread/_pthread_mutex_t.h
memory.o: /usr/include/sys/_pthread/_pthread_mutexattr_t.h
memory.o: /usr/include/sys/_pthread/_pthread_once_t.h
memory.o: /usr/include/sys/_pthread/_pthread_rwlock_t.h
memory.o: /usr/include/sys/_pthread/_pthread_rwlockattr_t.h
memory.o: /usr/include/sys/_pthread/_pthread_t.h
memory.o: /usr/include/sys/_pthread/_pthread_key_t.h
memory.o: /usr/include/sys/_types/_fsblkcnt_t.h
memory.o: /usr/include/sys/_types/_fsfilcnt_t.h /usr/include/sys/timeb.h
memory.o: /usr/include/sys/times.h memory.h inst_pre.h
output.o: ff.h /usr/include/stdlib.h /usr/include/Availability.h
output.o: /usr/include/AvailabilityInternal.h /usr/include/_types.h
output.o: /usr/include/sys/_types.h /usr/include/sys/cdefs.h
output.o: /usr/include/sys/_symbol_aliasing.h
output.o: /usr/include/sys/_posix_availability.h
output.o: /usr/include/machine/_types.h /usr/include/i386/_types.h
output.o: /usr/include/sys/_pthread/_pthread_types.h /usr/include/sys/wait.h
output.o: /usr/include/sys/_types/_pid_t.h /usr/include/sys/_types/_id_t.h
output.o: /usr/include/sys/signal.h /usr/include/sys/appleapiopts.h
output.o: /usr/include/machine/signal.h /usr/include/i386/signal.h
output.o: /usr/include/machine/_mcontext.h /usr/include/i386/_mcontext.h
output.o: /usr/include/mach/i386/_structs.h
output.o: /usr/include/sys/_pthread/_pthread_attr_t.h
output.o: /usr/include/sys/_types/_sigaltstack.h
output.o: /usr/include/sys/_types/_ucontext.h
output.o: /usr/include/sys/_types/_sigset_t.h
output.o: /usr/include/sys/_types/_size_t.h /usr/include/sys/_types/_uid_t.h
output.o: /usr/include/sys/resource.h /usr/include/stdint.h
output.o: /usr/include/sys/_types/_int8_t.h
output.o: /usr/include/sys/_types/_int16_t.h
output.o: /usr/include/sys/_types/_int32_t.h
output.o: /usr/include/sys/_types/_int64_t.h /usr/include/_types/_uint8_t.h
output.o: /usr/include/_types/_uint16_t.h /usr/include/_types/_uint32_t.h
output.o: /usr/include/_types/_uint64_t.h /usr/include/sys/_types/_intptr_t.h
output.o: /usr/include/sys/_types/_uintptr_t.h
output.o: /usr/include/_types/_intmax_t.h /usr/include/_types/_uintmax_t.h
output.o: /usr/include/sys/_types/_timeval.h /usr/include/machine/endian.h
output.o: /usr/include/i386/endian.h /usr/include/sys/_endian.h
output.o: /usr/include/libkern/_OSByteOrder.h
output.o: /usr/include/libkern/i386/_OSByteOrder.h /usr/include/alloca.h
output.o: /usr/include/sys/_types/_ct_rune_t.h
output.o: /usr/include/sys/_types/_rune_t.h
output.o: /usr/include/sys/_types/_wchar_t.h /usr/include/sys/_types/_null.h
output.o: /usr/include/machine/types.h /usr/include/i386/types.h
output.o: /usr/include/sys/_types/_dev_t.h /usr/include/sys/_types/_mode_t.h
output.o: /usr/include/stdio.h /usr/include/sys/_types/_va_list.h
output.o: /usr/include/sys/stdio.h /usr/include/sys/_types/_off_t.h
output.o: /usr/include/sys/_types/_ssize_t.h /usr/include/secure/_stdio.h
output.o: /usr/include/secure/_common.h /usr/include/strings.h
output.o: /usr/include/string.h /usr/include/sys/_types/_rsize_t.h
output.o: /usr/include/sys/_types/_errno_t.h /usr/include/secure/_string.h
output.o: /usr/include/ctype.h /usr/include/runetype.h
output.o: /usr/include/sys/_types/_wint_t.h /usr/include/sys/types.h
output.o: /usr/include/sys/_types/_blkcnt_t.h
output.o: /usr/include/sys/_types/_blksize_t.h
output.o: /usr/include/sys/_types/_gid_t.h
output.o: /usr/include/sys/_types/_in_addr_t.h
output.o: /usr/include/sys/_types/_in_port_t.h
output.o: /usr/include/sys/_types/_ino_t.h /usr/include/sys/_types/_ino64_t.h
output.o: /usr/include/sys/_types/_key_t.h /usr/include/sys/_types/_nlink_t.h
output.o: /usr/include/sys/_types/_clock_t.h
output.o: /usr/include/sys/_types/_time_t.h
output.o: /usr/include/sys/_types/_useconds_t.h
output.o: /usr/include/sys/_types/_suseconds_t.h
output.o: /usr/include/sys/_types/_fd_def.h
output.o: /usr/include/sys/_types/_fd_setsize.h
output.o: /usr/include/sys/_types/_fd_set.h /usr/include/sys/_types/_fd_clr.h
output.o: /usr/include/sys/_types/_fd_zero.h
output.o: /usr/include/sys/_types/_fd_isset.h
output.o: /usr/include/sys/_types/_fd_copy.h
output.o: /usr/include/sys/_pthread/_pthread_cond_t.h
output.o: /usr/include/sys/_pthread/_pthread_condattr_t.h
output.o: /usr/include/sys/_pthread/_pthread_mutex_t.h
output.o: /usr/include/sys/_pthread/_pthread_mutexattr_t.h
output.o: /usr/include/sys/_pthread/_pthread_once_t.h
output.o: /usr/include/sys/_pthread/_pthread_rwlock_t.h
output.o: /usr/include/sys/_pthread/_pthread_rwlockattr_t.h
output.o: /usr/include/sys/_pthread/_pthread_t.h
output.o: /usr/include/sys/_pthread/_pthread_key_t.h
output.o: /usr/include/sys/_types/_fsblkcnt_t.h
output.o: /usr/include/sys/_types/_fsfilcnt_t.h /usr/include/sys/timeb.h
output.o: /usr/include/sys/times.h utility.h output.h
parse.o: ff.h /usr/include/stdlib.h /usr/include/Availability.h
parse.o: /usr/include/AvailabilityInternal.h /usr/include/_types.h
parse.o: /usr/include/sys/_types.h /usr/include/sys/cdefs.h
parse.o: /usr/include/sys/_symbol_aliasing.h
parse.o: /usr/include/sys/_posix_availability.h /usr/include/machine/_types.h
parse.o: /usr/include/i386/_types.h
parse.o: /usr/include/sys/_pthread/_pthread_types.h /usr/include/sys/wait.h
parse.o: /usr/include/sys/_types/_pid_t.h /usr/include/sys/_types/_id_t.h
parse.o: /usr/include/sys/signal.h /usr/include/sys/appleapiopts.h
parse.o: /usr/include/machine/signal.h /usr/include/i386/signal.h
parse.o: /usr/include/machine/_mcontext.h /usr/include/i386/_mcontext.h
parse.o: /usr/include/mach/i386/_structs.h
parse.o: /usr/include/sys/_pthread/_pthread_attr_t.h
parse.o: /usr/include/sys/_types/_sigaltstack.h
parse.o: /usr/include/sys/_types/_ucontext.h
parse.o: /usr/include/sys/_types/_sigset_t.h
parse.o: /usr/include/sys/_types/_size_t.h /usr/include/sys/_types/_uid_t.h
parse.o: /usr/include/sys/resource.h /usr/include/stdint.h
parse.o: /usr/include/sys/_types/_int8_t.h /usr/include/sys/_types/_int16_t.h
parse.o: /usr/include/sys/_types/_int32_t.h
parse.o: /usr/include/sys/_types/_int64_t.h /usr/include/_types/_uint8_t.h
parse.o: /usr/include/_types/_uint16_t.h /usr/include/_types/_uint32_t.h
parse.o: /usr/include/_types/_uint64_t.h /usr/include/sys/_types/_intptr_t.h
parse.o: /usr/include/sys/_types/_uintptr_t.h /usr/include/_types/_intmax_t.h
parse.o: /usr/include/_types/_uintmax_t.h /usr/include/sys/_types/_timeval.h
parse.o: /usr/include/machine/endian.h /usr/include/i386/endian.h
parse.o: /usr/include/sys/_endian.h /usr/include/libkern/_OSByteOrder.h
parse.o: /usr/include/libkern/i386/_OSByteOrder.h /usr/include/alloca.h
parse.o: /usr/include/sys/_types/_ct_rune_t.h
parse.o: /usr/include/sys/_types/_rune_t.h /usr/include/sys/_types/_wchar_t.h
parse.o: /usr/include/sys/_types/_null.h /usr/include/machine/types.h
parse.o: /usr/include/i386/types.h /usr/include/sys/_types/_dev_t.h
parse.o: /usr/include/sys/_types/_mode_t.h /usr/include/stdio.h
parse.o: /usr/include/sys/_types/_va_list.h /usr/include/sys/stdio.h
parse.o: /usr/include/sys/_types/_off_t.h /usr/include/sys/_types/_ssize_t.h
parse.o: /usr/include/secure/_stdio.h /usr/include/secure/_common.h
parse.o: /usr/include/strings.h /usr/include/string.h
parse.o: /usr/include/sys/_types/_rsize_t.h
parse.o: /usr/include/sys/_types/_errno_t.h /usr/include/secure/_string.h
parse.o: /usr/include/ctype.h /usr/include/runetype.h
parse.o: /usr/include/sys/_types/_wint_t.h /usr/include/sys/types.h
parse.o: /usr/include/sys/_types/_blkcnt_t.h
parse.o: /usr/include/sys/_types/_blksize_t.h
parse.o: /usr/include/sys/_types/_gid_t.h
parse.o: /usr/include/sys/_types/_in_addr_t.h
parse.o: /usr/include/sys/_types/_in_port_t.h
parse.o: /usr/include/sys/_types/_ino_t.h /usr/include/sys/_types/_ino64_t.h
parse.o: /usr/include/sys/_types/_key_t.h /usr/include/sys/_types/_nlink_t.h
parse.o: /usr/include/sys/_types/_clock_t.h /usr/include/sys/_types/_time_t.h
parse.o: /usr/include/sys/_types/_useconds_t.h
parse.o: /usr/include/sys/_types/_suseconds_t.h
parse.o: /usr/include/sys/_types/_fd_def.h
parse.o: /usr/include/sys/_types/_fd_setsize.h
parse.o: /usr/include/sys/_types/_fd_set.h /usr/include/sys/_types/_fd_clr.h
parse.o: /usr/include/sys/_types/_fd_zero.h
parse.o: /usr/include/sys/_types/_fd_isset.h
parse.o: /usr/include/sys/_types/_fd_copy.h
parse.o: /usr/include/sys/_pthread/_pthread_cond_t.h
parse.o: /usr/include/sys/_pthread/_pthread_condattr_t.h
parse.o: /usr/include/sys/_pthread/_pthread_mutex_t.h
parse.o: /usr/include/sys/_pthread/_pthread_mutexattr_t.h
parse.o: /usr/include/sys/_pthread/_pthread_once_t.h
parse.o: /usr/include/sys/_pthread/_pthread_rwlock_t.h
parse.o: /usr/include/sys/_pthread/_pthread_rwlockattr_t.h
parse.o: /usr/include/sys/_pthread/_pthread_t.h
parse.o: /usr/include/sys/_pthread/_pthread_key_t.h
parse.o: /usr/include/sys/_types/_fsblkcnt_t.h
parse.o: /usr/include/sys/_types/_fsfilcnt_t.h /usr/include/sys/timeb.h
parse.o: /usr/include/sys/times.h memory.h output.h parse.h
expressions.o: /usr/include/string.h /usr/include/_types.h
expressions.o: /usr/include/sys/_types.h /usr/include/sys/cdefs.h
expressions.o: /usr/include/sys/_symbol_aliasing.h
expressions.o: /usr/include/sys/_posix_availability.h
expressions.o: /usr/include/machine/_types.h /usr/include/i386/_types.h
expressions.o: /usr/include/sys/_pthread/_pthread_types.h
expressions.o: /usr/include/Availability.h
expressions.o: /usr/include/AvailabilityInternal.h
expressions.o: /usr/include/sys/_types/_size_t.h
expressions.o: /usr/include/sys/_types/_null.h
expressions.o: /usr/include/sys/_types/_rsize_t.h
expressions.o: /usr/include/sys/_types/_errno_t.h
expressions.o: /usr/include/sys/_types/_ssize_t.h /usr/include/strings.h
expressions.o: /usr/include/secure/_string.h /usr/include/secure/_common.h
expressions.o: ff.h /usr/include/stdlib.h /usr/include/sys/wait.h
expressions.o: /usr/include/sys/_types/_pid_t.h
expressions.o: /usr/include/sys/_types/_id_t.h /usr/include/sys/signal.h
expressions.o: /usr/include/sys/appleapiopts.h /usr/include/machine/signal.h
expressions.o: /usr/include/i386/signal.h /usr/include/machine/_mcontext.h
expressions.o: /usr/include/i386/_mcontext.h
expressions.o: /usr/include/mach/i386/_structs.h
expressions.o: /usr/include/sys/_pthread/_pthread_attr_t.h
expressions.o: /usr/include/sys/_types/_sigaltstack.h
expressions.o: /usr/include/sys/_types/_ucontext.h
expressions.o: /usr/include/sys/_types/_sigset_t.h
expressions.o: /usr/include/sys/_types/_uid_t.h /usr/include/sys/resource.h
expressions.o: /usr/include/stdint.h /usr/include/sys/_types/_int8_t.h
expressions.o: /usr/include/sys/_types/_int16_t.h
expressions.o: /usr/include/sys/_types/_int32_t.h
expressions.o: /usr/include/sys/_types/_int64_t.h
expressions.o: /usr/include/_types/_uint8_t.h /usr/include/_types/_uint16_t.h
expressions.o: /usr/include/_types/_uint32_t.h
expressions.o: /usr/include/_types/_uint64_t.h
expressions.o: /usr/include/sys/_types/_intptr_t.h
expressions.o: /usr/include/sys/_types/_uintptr_t.h
expressions.o: /usr/include/_types/_intmax_t.h
expressions.o: /usr/include/_types/_uintmax_t.h
expressions.o: /usr/include/sys/_types/_timeval.h
expressions.o: /usr/include/machine/endian.h /usr/include/i386/endian.h
expressions.o: /usr/include/sys/_endian.h /usr/include/libkern/_OSByteOrder.h
expressions.o: /usr/include/libkern/i386/_OSByteOrder.h /usr/include/alloca.h
expressions.o: /usr/include/sys/_types/_ct_rune_t.h
expressions.o: /usr/include/sys/_types/_rune_t.h
expressions.o: /usr/include/sys/_types/_wchar_t.h
expressions.o: /usr/include/machine/types.h /usr/include/i386/types.h
expressions.o: /usr/include/sys/_types/_dev_t.h
expressions.o: /usr/include/sys/_types/_mode_t.h /usr/include/stdio.h
expressions.o: /usr/include/sys/_types/_va_list.h /usr/include/sys/stdio.h
expressions.o: /usr/include/sys/_types/_off_t.h /usr/include/secure/_stdio.h
expressions.o: /usr/include/ctype.h /usr/include/runetype.h
expressions.o: /usr/include/sys/_types/_wint_t.h /usr/include/sys/types.h
expressions.o: /usr/include/sys/_types/_blkcnt_t.h
expressions.o: /usr/include/sys/_types/_blksize_t.h
expressions.o: /usr/include/sys/_types/_gid_t.h
expressions.o: /usr/include/sys/_types/_in_addr_t.h
expressions.o: /usr/include/sys/_types/_in_port_t.h
expressions.o: /usr/include/sys/_types/_ino_t.h
expressions.o: /usr/include/sys/_types/_ino64_t.h
expressions.o: /usr/include/sys/_types/_key_t.h
expressions.o: /usr/include/sys/_types/_nlink_t.h
expressions.o: /usr/include/sys/_types/_clock_t.h
expressions.o: /usr/include/sys/_types/_time_t.h
expressions.o: /usr/include/sys/_types/_useconds_t.h
expressions.o: /usr/include/sys/_types/_suseconds_t.h
expressions.o: /usr/include/sys/_types/_fd_def.h
expressions.o: /usr/include/sys/_types/_fd_setsize.h
expressions.o: /usr/include/sys/_types/_fd_set.h
expressions.o: /usr/include/sys/_types/_fd_clr.h
expressions.o: /usr/include/sys/_types/_fd_zero.h
expressions.o: /usr/include/sys/_types/_fd_isset.h
expressions.o: /usr/include/sys/_types/_fd_copy.h
expressions.o: /usr/include/sys/_pthread/_pthread_cond_t.h
expressions.o: /usr/include/sys/_pthread/_pthread_condattr_t.h
expressions.o: /usr/include/sys/_pthread/_pthread_mutex_t.h
expressions.o: /usr/include/sys/_pthread/_pthread_mutexattr_t.h
expressions.o: /usr/include/sys/_pthread/_pthread_once_t.h
expressions.o: /usr/include/sys/_pthread/_pthread_rwlock_t.h
expressions.o: /usr/include/sys/_pthread/_pthread_rwlockattr_t.h
expressions.o: /usr/include/sys/_pthread/_pthread_t.h
expressions.o: /usr/include/sys/_pthread/_pthread_key_t.h
expressions.o: /usr/include/sys/_types/_fsblkcnt_t.h
expressions.o: /usr/include/sys/_types/_fsfilcnt_t.h /usr/include/sys/timeb.h
expressions.o: /usr/include/sys/times.h output.h memory.h expressions.h
inst_pre.o: /usr/include/string.h /usr/include/_types.h
inst_pre.o: /usr/include/sys/_types.h /usr/include/sys/cdefs.h
inst_pre.o: /usr/include/sys/_symbol_aliasing.h
inst_pre.o: /usr/include/sys/_posix_availability.h
inst_pre.o: /usr/include/machine/_types.h /usr/include/i386/_types.h
inst_pre.o: /usr/include/sys/_pthread/_pthread_types.h
inst_pre.o: /usr/include/Availability.h /usr/include/AvailabilityInternal.h
inst_pre.o: /usr/include/sys/_types/_size_t.h /usr/include/sys/_types/_null.h
inst_pre.o: /usr/include/sys/_types/_rsize_t.h
inst_pre.o: /usr/include/sys/_types/_errno_t.h
inst_pre.o: /usr/include/sys/_types/_ssize_t.h /usr/include/strings.h
inst_pre.o: /usr/include/secure/_string.h /usr/include/secure/_common.h ff.h
inst_pre.o: /usr/include/stdlib.h /usr/include/sys/wait.h
inst_pre.o: /usr/include/sys/_types/_pid_t.h /usr/include/sys/_types/_id_t.h
inst_pre.o: /usr/include/sys/signal.h /usr/include/sys/appleapiopts.h
inst_pre.o: /usr/include/machine/signal.h /usr/include/i386/signal.h
inst_pre.o: /usr/include/machine/_mcontext.h /usr/include/i386/_mcontext.h
inst_pre.o: /usr/include/mach/i386/_structs.h
inst_pre.o: /usr/include/sys/_pthread/_pthread_attr_t.h
inst_pre.o: /usr/include/sys/_types/_sigaltstack.h
inst_pre.o: /usr/include/sys/_types/_ucontext.h
inst_pre.o: /usr/include/sys/_types/_sigset_t.h
inst_pre.o: /usr/include/sys/_types/_uid_t.h /usr/include/sys/resource.h
inst_pre.o: /usr/include/stdint.h /usr/include/sys/_types/_int8_t.h
inst_pre.o: /usr/include/sys/_types/_int16_t.h
inst_pre.o: /usr/include/sys/_types/_int32_t.h
inst_pre.o: /usr/include/sys/_types/_int64_t.h /usr/include/_types/_uint8_t.h
inst_pre.o: /usr/include/_types/_uint16_t.h /usr/include/_types/_uint32_t.h
inst_pre.o: /usr/include/_types/_uint64_t.h
inst_pre.o: /usr/include/sys/_types/_intptr_t.h
inst_pre.o: /usr/include/sys/_types/_uintptr_t.h
inst_pre.o: /usr/include/_types/_intmax_t.h /usr/include/_types/_uintmax_t.h
inst_pre.o: /usr/include/sys/_types/_timeval.h /usr/include/machine/endian.h
inst_pre.o: /usr/include/i386/endian.h /usr/include/sys/_endian.h
inst_pre.o: /usr/include/libkern/_OSByteOrder.h
inst_pre.o: /usr/include/libkern/i386/_OSByteOrder.h /usr/include/alloca.h
inst_pre.o: /usr/include/sys/_types/_ct_rune_t.h
inst_pre.o: /usr/include/sys/_types/_rune_t.h
inst_pre.o: /usr/include/sys/_types/_wchar_t.h /usr/include/machine/types.h
inst_pre.o: /usr/include/i386/types.h /usr/include/sys/_types/_dev_t.h
inst_pre.o: /usr/include/sys/_types/_mode_t.h /usr/include/stdio.h
inst_pre.o: /usr/include/sys/_types/_va_list.h /usr/include/sys/stdio.h
inst_pre.o: /usr/include/sys/_types/_off_t.h /usr/include/secure/_stdio.h
inst_pre.o: /usr/include/ctype.h /usr/include/runetype.h
inst_pre.o: /usr/include/sys/_types/_wint_t.h /usr/include/sys/types.h
inst_pre.o: /usr/include/sys/_types/_blkcnt_t.h
inst_pre.o: /usr/include/sys/_types/_blksize_t.h
inst_pre.o: /usr/include/sys/_types/_gid_t.h
inst_pre.o: /usr/include/sys/_types/_in_addr_t.h
inst_pre.o: /usr/include/sys/_types/_in_port_t.h
inst_pre.o: /usr/include/sys/_types/_ino_t.h
inst_pre.o: /usr/include/sys/_types/_ino64_t.h
inst_pre.o: /usr/include/sys/_types/_key_t.h
inst_pre.o: /usr/include/sys/_types/_nlink_t.h
inst_pre.o: /usr/include/sys/_types/_clock_t.h
inst_pre.o: /usr/include/sys/_types/_time_t.h
inst_pre.o: /usr/include/sys/_types/_useconds_t.h
inst_pre.o: /usr/include/sys/_types/_suseconds_t.h
inst_pre.o: /usr/include/sys/_types/_fd_def.h
inst_pre.o: /usr/include/sys/_types/_fd_setsize.h
inst_pre.o: /usr/include/sys/_types/_fd_set.h
inst_pre.o: /usr/include/sys/_types/_fd_clr.h
inst_pre.o: /usr/include/sys/_types/_fd_zero.h
inst_pre.o: /usr/include/sys/_types/_fd_isset.h
inst_pre.o: /usr/include/sys/_types/_fd_copy.h
inst_pre.o: /usr/include/sys/_pthread/_pthread_cond_t.h
inst_pre.o: /usr/include/sys/_pthread/_pthread_condattr_t.h
inst_pre.o: /usr/include/sys/_pthread/_pthread_mutex_t.h
inst_pre.o: /usr/include/sys/_pthread/_pthread_mutexattr_t.h
inst_pre.o: /usr/include/sys/_pthread/_pthread_once_t.h
inst_pre.o: /usr/include/sys/_pthread/_pthread_rwlock_t.h
inst_pre.o: /usr/include/sys/_pthread/_pthread_rwlockattr_t.h
inst_pre.o: /usr/include/sys/_pthread/_pthread_t.h
inst_pre.o: /usr/include/sys/_pthread/_pthread_key_t.h
inst_pre.o: /usr/include/sys/_types/_fsblkcnt_t.h
inst_pre.o: /usr/include/sys/_types/_fsfilcnt_t.h /usr/include/sys/timeb.h
inst_pre.o: /usr/include/sys/times.h output.h memory.h expressions.h
inst_pre.o: inst_pre.h
inst_easy.o: ff.h /usr/include/stdlib.h /usr/include/Availability.h
inst_easy.o: /usr/include/AvailabilityInternal.h /usr/include/_types.h
inst_easy.o: /usr/include/sys/_types.h /usr/include/sys/cdefs.h
inst_easy.o: /usr/include/sys/_symbol_aliasing.h
inst_easy.o: /usr/include/sys/_posix_availability.h
inst_easy.o: /usr/include/machine/_types.h /usr/include/i386/_types.h
inst_easy.o: /usr/include/sys/_pthread/_pthread_types.h
inst_easy.o: /usr/include/sys/wait.h /usr/include/sys/_types/_pid_t.h
inst_easy.o: /usr/include/sys/_types/_id_t.h /usr/include/sys/signal.h
inst_easy.o: /usr/include/sys/appleapiopts.h /usr/include/machine/signal.h
inst_easy.o: /usr/include/i386/signal.h /usr/include/machine/_mcontext.h
inst_easy.o: /usr/include/i386/_mcontext.h /usr/include/mach/i386/_structs.h
inst_easy.o: /usr/include/sys/_pthread/_pthread_attr_t.h
inst_easy.o: /usr/include/sys/_types/_sigaltstack.h
inst_easy.o: /usr/include/sys/_types/_ucontext.h
inst_easy.o: /usr/include/sys/_types/_sigset_t.h
inst_easy.o: /usr/include/sys/_types/_size_t.h
inst_easy.o: /usr/include/sys/_types/_uid_t.h /usr/include/sys/resource.h
inst_easy.o: /usr/include/stdint.h /usr/include/sys/_types/_int8_t.h
inst_easy.o: /usr/include/sys/_types/_int16_t.h
inst_easy.o: /usr/include/sys/_types/_int32_t.h
inst_easy.o: /usr/include/sys/_types/_int64_t.h
inst_easy.o: /usr/include/_types/_uint8_t.h /usr/include/_types/_uint16_t.h
inst_easy.o: /usr/include/_types/_uint32_t.h /usr/include/_types/_uint64_t.h
inst_easy.o: /usr/include/sys/_types/_intptr_t.h
inst_easy.o: /usr/include/sys/_types/_uintptr_t.h
inst_easy.o: /usr/include/_types/_intmax_t.h /usr/include/_types/_uintmax_t.h
inst_easy.o: /usr/include/sys/_types/_timeval.h /usr/include/machine/endian.h
inst_easy.o: /usr/include/i386/endian.h /usr/include/sys/_endian.h
inst_easy.o: /usr/include/libkern/_OSByteOrder.h
inst_easy.o: /usr/include/libkern/i386/_OSByteOrder.h /usr/include/alloca.h
inst_easy.o: /usr/include/sys/_types/_ct_rune_t.h
inst_easy.o: /usr/include/sys/_types/_rune_t.h
inst_easy.o: /usr/include/sys/_types/_wchar_t.h
inst_easy.o: /usr/include/sys/_types/_null.h /usr/include/machine/types.h
inst_easy.o: /usr/include/i386/types.h /usr/include/sys/_types/_dev_t.h
inst_easy.o: /usr/include/sys/_types/_mode_t.h /usr/include/stdio.h
inst_easy.o: /usr/include/sys/_types/_va_list.h /usr/include/sys/stdio.h
inst_easy.o: /usr/include/sys/_types/_off_t.h
inst_easy.o: /usr/include/sys/_types/_ssize_t.h /usr/include/secure/_stdio.h
inst_easy.o: /usr/include/secure/_common.h /usr/include/strings.h
inst_easy.o: /usr/include/string.h /usr/include/sys/_types/_rsize_t.h
inst_easy.o: /usr/include/sys/_types/_errno_t.h /usr/include/secure/_string.h
inst_easy.o: /usr/include/ctype.h /usr/include/runetype.h
inst_easy.o: /usr/include/sys/_types/_wint_t.h /usr/include/sys/types.h
inst_easy.o: /usr/include/sys/_types/_blkcnt_t.h
inst_easy.o: /usr/include/sys/_types/_blksize_t.h
inst_easy.o: /usr/include/sys/_types/_gid_t.h
inst_easy.o: /usr/include/sys/_types/_in_addr_t.h
inst_easy.o: /usr/include/sys/_types/_in_port_t.h
inst_easy.o: /usr/include/sys/_types/_ino_t.h
inst_easy.o: /usr/include/sys/_types/_ino64_t.h
inst_easy.o: /usr/include/sys/_types/_key_t.h
inst_easy.o: /usr/include/sys/_types/_nlink_t.h
inst_easy.o: /usr/include/sys/_types/_clock_t.h
inst_easy.o: /usr/include/sys/_types/_time_t.h
inst_easy.o: /usr/include/sys/_types/_useconds_t.h
inst_easy.o: /usr/include/sys/_types/_suseconds_t.h
inst_easy.o: /usr/include/sys/_types/_fd_def.h
inst_easy.o: /usr/include/sys/_types/_fd_setsize.h
inst_easy.o: /usr/include/sys/_types/_fd_set.h
inst_easy.o: /usr/include/sys/_types/_fd_clr.h
inst_easy.o: /usr/include/sys/_types/_fd_zero.h
inst_easy.o: /usr/include/sys/_types/_fd_isset.h
inst_easy.o: /usr/include/sys/_types/_fd_copy.h
inst_easy.o: /usr/include/sys/_pthread/_pthread_cond_t.h
inst_easy.o: /usr/include/sys/_pthread/_pthread_condattr_t.h
inst_easy.o: /usr/include/sys/_pthread/_pthread_mutex_t.h
inst_easy.o: /usr/include/sys/_pthread/_pthread_mutexattr_t.h
inst_easy.o: /usr/include/sys/_pthread/_pthread_once_t.h
inst_easy.o: /usr/include/sys/_pthread/_pthread_rwlock_t.h
inst_easy.o: /usr/include/sys/_pthread/_pthread_rwlockattr_t.h
inst_easy.o: /usr/include/sys/_pthread/_pthread_t.h
inst_easy.o: /usr/include/sys/_pthread/_pthread_key_t.h
inst_easy.o: /usr/include/sys/_types/_fsblkcnt_t.h
inst_easy.o: /usr/include/sys/_types/_fsfilcnt_t.h /usr/include/sys/timeb.h
inst_easy.o: /usr/include/sys/times.h output.h memory.h expressions.h
inst_easy.o: inst_pre.h inst_easy.h
inst_hard.o: ff.h /usr/include/stdlib.h /usr/include/Availability.h
inst_hard.o: /usr/include/AvailabilityInternal.h /usr/include/_types.h
inst_hard.o: /usr/include/sys/_types.h /usr/include/sys/cdefs.h
inst_hard.o: /usr/include/sys/_symbol_aliasing.h
inst_hard.o: /usr/include/sys/_posix_availability.h
inst_hard.o: /usr/include/machine/_types.h /usr/include/i386/_types.h
inst_hard.o: /usr/include/sys/_pthread/_pthread_types.h
inst_hard.o: /usr/include/sys/wait.h /usr/include/sys/_types/_pid_t.h
inst_hard.o: /usr/include/sys/_types/_id_t.h /usr/include/sys/signal.h
inst_hard.o: /usr/include/sys/appleapiopts.h /usr/include/machine/signal.h
inst_hard.o: /usr/include/i386/signal.h /usr/include/machine/_mcontext.h
inst_hard.o: /usr/include/i386/_mcontext.h /usr/include/mach/i386/_structs.h
inst_hard.o: /usr/include/sys/_pthread/_pthread_attr_t.h
inst_hard.o: /usr/include/sys/_types/_sigaltstack.h
inst_hard.o: /usr/include/sys/_types/_ucontext.h
inst_hard.o: /usr/include/sys/_types/_sigset_t.h
inst_hard.o: /usr/include/sys/_types/_size_t.h
inst_hard.o: /usr/include/sys/_types/_uid_t.h /usr/include/sys/resource.h
inst_hard.o: /usr/include/stdint.h /usr/include/sys/_types/_int8_t.h
inst_hard.o: /usr/include/sys/_types/_int16_t.h
inst_hard.o: /usr/include/sys/_types/_int32_t.h
inst_hard.o: /usr/include/sys/_types/_int64_t.h
inst_hard.o: /usr/include/_types/_uint8_t.h /usr/include/_types/_uint16_t.h
inst_hard.o: /usr/include/_types/_uint32_t.h /usr/include/_types/_uint64_t.h
inst_hard.o: /usr/include/sys/_types/_intptr_t.h
inst_hard.o: /usr/include/sys/_types/_uintptr_t.h
inst_hard.o: /usr/include/_types/_intmax_t.h /usr/include/_types/_uintmax_t.h
inst_hard.o: /usr/include/sys/_types/_timeval.h /usr/include/machine/endian.h
inst_hard.o: /usr/include/i386/endian.h /usr/include/sys/_endian.h
inst_hard.o: /usr/include/libkern/_OSByteOrder.h
inst_hard.o: /usr/include/libkern/i386/_OSByteOrder.h /usr/include/alloca.h
inst_hard.o: /usr/include/sys/_types/_ct_rune_t.h
inst_hard.o: /usr/include/sys/_types/_rune_t.h
inst_hard.o: /usr/include/sys/_types/_wchar_t.h
inst_hard.o: /usr/include/sys/_types/_null.h /usr/include/machine/types.h
inst_hard.o: /usr/include/i386/types.h /usr/include/sys/_types/_dev_t.h
inst_hard.o: /usr/include/sys/_types/_mode_t.h /usr/include/stdio.h
inst_hard.o: /usr/include/sys/_types/_va_list.h /usr/include/sys/stdio.h
inst_hard.o: /usr/include/sys/_types/_off_t.h
inst_hard.o: /usr/include/sys/_types/_ssize_t.h /usr/include/secure/_stdio.h
inst_hard.o: /usr/include/secure/_common.h /usr/include/strings.h
inst_hard.o: /usr/include/string.h /usr/include/sys/_types/_rsize_t.h
inst_hard.o: /usr/include/sys/_types/_errno_t.h /usr/include/secure/_string.h
inst_hard.o: /usr/include/ctype.h /usr/include/runetype.h
inst_hard.o: /usr/include/sys/_types/_wint_t.h /usr/include/sys/types.h
inst_hard.o: /usr/include/sys/_types/_blkcnt_t.h
inst_hard.o: /usr/include/sys/_types/_blksize_t.h
inst_hard.o: /usr/include/sys/_types/_gid_t.h
inst_hard.o: /usr/include/sys/_types/_in_addr_t.h
inst_hard.o: /usr/include/sys/_types/_in_port_t.h
inst_hard.o: /usr/include/sys/_types/_ino_t.h
inst_hard.o: /usr/include/sys/_types/_ino64_t.h
inst_hard.o: /usr/include/sys/_types/_key_t.h
inst_hard.o: /usr/include/sys/_types/_nlink_t.h
inst_hard.o: /usr/include/sys/_types/_clock_t.h
inst_hard.o: /usr/include/sys/_types/_time_t.h
inst_hard.o: /usr/include/sys/_types/_useconds_t.h
inst_hard.o: /usr/include/sys/_types/_suseconds_t.h
inst_hard.o: /usr/include/sys/_types/_fd_def.h
inst_hard.o: /usr/include/sys/_types/_fd_setsize.h
inst_hard.o: /usr/include/sys/_types/_fd_set.h
inst_hard.o: /usr/include/sys/_types/_fd_clr.h
inst_hard.o: /usr/include/sys/_types/_fd_zero.h
inst_hard.o: /usr/include/sys/_types/_fd_isset.h
inst_hard.o: /usr/include/sys/_types/_fd_copy.h
inst_hard.o: /usr/include/sys/_pthread/_pthread_cond_t.h
inst_hard.o: /usr/include/sys/_pthread/_pthread_condattr_t.h
inst_hard.o: /usr/include/sys/_pthread/_pthread_mutex_t.h
inst_hard.o: /usr/include/sys/_pthread/_pthread_mutexattr_t.h
inst_hard.o: /usr/include/sys/_pthread/_pthread_once_t.h
inst_hard.o: /usr/include/sys/_pthread/_pthread_rwlock_t.h
inst_hard.o: /usr/include/sys/_pthread/_pthread_rwlockattr_t.h
inst_hard.o: /usr/include/sys/_pthread/_pthread_t.h
inst_hard.o: /usr/include/sys/_pthread/_pthread_key_t.h
inst_hard.o: /usr/include/sys/_types/_fsblkcnt_t.h
inst_hard.o: /usr/include/sys/_types/_fsfilcnt_t.h /usr/include/sys/timeb.h
inst_hard.o: /usr/include/sys/times.h output.h memory.h expressions.h
inst_hard.o: inst_pre.h inst_hard.h
inst_final.o: ff.h /usr/include/stdlib.h /usr/include/Availability.h
inst_final.o: /usr/include/AvailabilityInternal.h /usr/include/_types.h
inst_final.o: /usr/include/sys/_types.h /usr/include/sys/cdefs.h
inst_final.o: /usr/include/sys/_symbol_aliasing.h
inst_final.o: /usr/include/sys/_posix_availability.h
inst_final.o: /usr/include/machine/_types.h /usr/include/i386/_types.h
inst_final.o: /usr/include/sys/_pthread/_pthread_types.h
inst_final.o: /usr/include/sys/wait.h /usr/include/sys/_types/_pid_t.h
inst_final.o: /usr/include/sys/_types/_id_t.h /usr/include/sys/signal.h
inst_final.o: /usr/include/sys/appleapiopts.h /usr/include/machine/signal.h
inst_final.o: /usr/include/i386/signal.h /usr/include/machine/_mcontext.h
inst_final.o: /usr/include/i386/_mcontext.h /usr/include/mach/i386/_structs.h
inst_final.o: /usr/include/sys/_pthread/_pthread_attr_t.h
inst_final.o: /usr/include/sys/_types/_sigaltstack.h
inst_final.o: /usr/include/sys/_types/_ucontext.h
inst_final.o: /usr/include/sys/_types/_sigset_t.h
inst_final.o: /usr/include/sys/_types/_size_t.h
inst_final.o: /usr/include/sys/_types/_uid_t.h /usr/include/sys/resource.h
inst_final.o: /usr/include/stdint.h /usr/include/sys/_types/_int8_t.h
inst_final.o: /usr/include/sys/_types/_int16_t.h
inst_final.o: /usr/include/sys/_types/_int32_t.h
inst_final.o: /usr/include/sys/_types/_int64_t.h
inst_final.o: /usr/include/_types/_uint8_t.h /usr/include/_types/_uint16_t.h
inst_final.o: /usr/include/_types/_uint32_t.h /usr/include/_types/_uint64_t.h
inst_final.o: /usr/include/sys/_types/_intptr_t.h
inst_final.o: /usr/include/sys/_types/_uintptr_t.h
inst_final.o: /usr/include/_types/_intmax_t.h
inst_final.o: /usr/include/_types/_uintmax_t.h
inst_final.o: /usr/include/sys/_types/_timeval.h
inst_final.o: /usr/include/machine/endian.h /usr/include/i386/endian.h
inst_final.o: /usr/include/sys/_endian.h /usr/include/libkern/_OSByteOrder.h
inst_final.o: /usr/include/libkern/i386/_OSByteOrder.h /usr/include/alloca.h
inst_final.o: /usr/include/sys/_types/_ct_rune_t.h
inst_final.o: /usr/include/sys/_types/_rune_t.h
inst_final.o: /usr/include/sys/_types/_wchar_t.h
inst_final.o: /usr/include/sys/_types/_null.h /usr/include/machine/types.h
inst_final.o: /usr/include/i386/types.h /usr/include/sys/_types/_dev_t.h
inst_final.o: /usr/include/sys/_types/_mode_t.h /usr/include/stdio.h
inst_final.o: /usr/include/sys/_types/_va_list.h /usr/include/sys/stdio.h
inst_final.o: /usr/include/sys/_types/_off_t.h
inst_final.o: /usr/include/sys/_types/_ssize_t.h /usr/include/secure/_stdio.h
inst_final.o: /usr/include/secure/_common.h /usr/include/strings.h
inst_final.o: /usr/include/string.h /usr/include/sys/_types/_rsize_t.h
inst_final.o: /usr/include/sys/_types/_errno_t.h
inst_final.o: /usr/include/secure/_string.h /usr/include/ctype.h
inst_final.o: /usr/include/runetype.h /usr/include/sys/_types/_wint_t.h
inst_final.o: /usr/include/sys/types.h /usr/include/sys/_types/_blkcnt_t.h
inst_final.o: /usr/include/sys/_types/_blksize_t.h
inst_final.o: /usr/include/sys/_types/_gid_t.h
inst_final.o: /usr/include/sys/_types/_in_addr_t.h
inst_final.o: /usr/include/sys/_types/_in_port_t.h
inst_final.o: /usr/include/sys/_types/_ino_t.h
inst_final.o: /usr/include/sys/_types/_ino64_t.h
inst_final.o: /usr/include/sys/_types/_key_t.h
inst_final.o: /usr/include/sys/_types/_nlink_t.h
inst_final.o: /usr/include/sys/_types/_clock_t.h
inst_final.o: /usr/include/sys/_types/_time_t.h
inst_final.o: /usr/include/sys/_types/_useconds_t.h
inst_final.o: /usr/include/sys/_types/_suseconds_t.h
inst_final.o: /usr/include/sys/_types/_fd_def.h
inst_final.o: /usr/include/sys/_types/_fd_setsize.h
inst_final.o: /usr/include/sys/_types/_fd_set.h
inst_final.o: /usr/include/sys/_types/_fd_clr.h
inst_final.o: /usr/include/sys/_types/_fd_zero.h
inst_final.o: /usr/include/sys/_types/_fd_isset.h
inst_final.o: /usr/include/sys/_types/_fd_copy.h
inst_final.o: /usr/include/sys/_pthread/_pthread_cond_t.h
inst_final.o: /usr/include/sys/_pthread/_pthread_condattr_t.h
inst_final.o: /usr/include/sys/_pthread/_pthread_mutex_t.h
inst_final.o: /usr/include/sys/_pthread/_pthread_mutexattr_t.h
inst_final.o: /usr/include/sys/_pthread/_pthread_once_t.h
inst_final.o: /usr/include/sys/_pthread/_pthread_rwlock_t.h
inst_final.o: /usr/include/sys/_pthread/_pthread_rwlockattr_t.h
inst_final.o: /usr/include/sys/_pthread/_pthread_t.h
inst_final.o: /usr/include/sys/_pthread/_pthread_key_t.h
inst_final.o: /usr/include/sys/_types/_fsblkcnt_t.h
inst_final.o: /usr/include/sys/_types/_fsfilcnt_t.h /usr/include/sys/timeb.h
inst_final.o: /usr/include/sys/times.h output.h memory.h expressions.h
inst_final.o: inst_pre.h inst_final.h
relax.o: ff.h /usr/include/stdlib.h /usr/include/Availability.h
relax.o: /usr/include/AvailabilityInternal.h /usr/include/_types.h
relax.o: /usr/include/sys/_types.h /usr/include/sys/cdefs.h
relax.o: /usr/include/sys/_symbol_aliasing.h
relax.o: /usr/include/sys/_posix_availability.h /usr/include/machine/_types.h
relax.o: /usr/include/i386/_types.h
relax.o: /usr/include/sys/_pthread/_pthread_types.h /usr/include/sys/wait.h
relax.o: /usr/include/sys/_types/_pid_t.h /usr/include/sys/_types/_id_t.h
relax.o: /usr/include/sys/signal.h /usr/include/sys/appleapiopts.h
relax.o: /usr/include/machine/signal.h /usr/include/i386/signal.h
relax.o: /usr/include/machine/_mcontext.h /usr/include/i386/_mcontext.h
relax.o: /usr/include/mach/i386/_structs.h
relax.o: /usr/include/sys/_pthread/_pthread_attr_t.h
relax.o: /usr/include/sys/_types/_sigaltstack.h
relax.o: /usr/include/sys/_types/_ucontext.h
relax.o: /usr/include/sys/_types/_sigset_t.h
relax.o: /usr/include/sys/_types/_size_t.h /usr/include/sys/_types/_uid_t.h
relax.o: /usr/include/sys/resource.h /usr/include/stdint.h
relax.o: /usr/include/sys/_types/_int8_t.h /usr/include/sys/_types/_int16_t.h
relax.o: /usr/include/sys/_types/_int32_t.h
relax.o: /usr/include/sys/_types/_int64_t.h /usr/include/_types/_uint8_t.h
relax.o: /usr/include/_types/_uint16_t.h /usr/include/_types/_uint32_t.h
relax.o: /usr/include/_types/_uint64_t.h /usr/include/sys/_types/_intptr_t.h
relax.o: /usr/include/sys/_types/_uintptr_t.h /usr/include/_types/_intmax_t.h
relax.o: /usr/include/_types/_uintmax_t.h /usr/include/sys/_types/_timeval.h
relax.o: /usr/include/machine/endian.h /usr/include/i386/endian.h
relax.o: /usr/include/sys/_endian.h /usr/include/libkern/_OSByteOrder.h
relax.o: /usr/include/libkern/i386/_OSByteOrder.h /usr/include/alloca.h
relax.o: /usr/include/sys/_types/_ct_rune_t.h
relax.o: /usr/include/sys/_types/_rune_t.h /usr/include/sys/_types/_wchar_t.h
relax.o: /usr/include/sys/_types/_null.h /usr/include/machine/types.h
relax.o: /usr/include/i386/types.h /usr/include/sys/_types/_dev_t.h
relax.o: /usr/include/sys/_types/_mode_t.h /usr/include/stdio.h
relax.o: /usr/include/sys/_types/_va_list.h /usr/include/sys/stdio.h
relax.o: /usr/include/sys/_types/_off_t.h /usr/include/sys/_types/_ssize_t.h
relax.o: /usr/include/secure/_stdio.h /usr/include/secure/_common.h
relax.o: /usr/include/strings.h /usr/include/string.h
relax.o: /usr/include/sys/_types/_rsize_t.h
relax.o: /usr/include/sys/_types/_errno_t.h /usr/include/secure/_string.h
relax.o: /usr/include/ctype.h /usr/include/runetype.h
relax.o: /usr/include/sys/_types/_wint_t.h /usr/include/sys/types.h
relax.o: /usr/include/sys/_types/_blkcnt_t.h
relax.o: /usr/include/sys/_types/_blksize_t.h
relax.o: /usr/include/sys/_types/_gid_t.h
relax.o: /usr/include/sys/_types/_in_addr_t.h
relax.o: /usr/include/sys/_types/_in_port_t.h
relax.o: /usr/include/sys/_types/_ino_t.h /usr/include/sys/_types/_ino64_t.h
relax.o: /usr/include/sys/_types/_key_t.h /usr/include/sys/_types/_nlink_t.h
relax.o: /usr/include/sys/_types/_clock_t.h /usr/include/sys/_types/_time_t.h
relax.o: /usr/include/sys/_types/_useconds_t.h
relax.o: /usr/include/sys/_types/_suseconds_t.h
relax.o: /usr/include/sys/_types/_fd_def.h
relax.o: /usr/include/sys/_types/_fd_setsize.h
relax.o: /usr/include/sys/_types/_fd_set.h /usr/include/sys/_types/_fd_clr.h
relax.o: /usr/include/sys/_types/_fd_zero.h
relax.o: /usr/include/sys/_types/_fd_isset.h
relax.o: /usr/include/sys/_types/_fd_copy.h
relax.o: /usr/include/sys/_pthread/_pthread_cond_t.h
relax.o: /usr/include/sys/_pthread/_pthread_condattr_t.h
relax.o: /usr/include/sys/_pthread/_pthread_mutex_t.h
relax.o: /usr/include/sys/_pthread/_pthread_mutexattr_t.h
relax.o: /usr/include/sys/_pthread/_pthread_once_t.h
relax.o: /usr/include/sys/_pthread/_pthread_rwlock_t.h
relax.o: /usr/include/sys/_pthread/_pthread_rwlockattr_t.h
relax.o: /usr/include/sys/_pthread/_pthread_t.h
relax.o: /usr/include/sys/_pthread/_pthread_key_t.h
relax.o: /usr/include/sys/_types/_fsblkcnt_t.h
relax.o: /usr/include/sys/_types/_fsfilcnt_t.h /usr/include/sys/timeb.h
relax.o: /usr/include/sys/times.h output.h memory.h expressions.h relax.h
relax.o: search.h
search.o: ff.h /usr/include/stdlib.h /usr/include/Availability.h
search.o: /usr/include/AvailabilityInternal.h /usr/include/_types.h
search.o: /usr/include/sys/_types.h /usr/include/sys/cdefs.h
search.o: /usr/include/sys/_symbol_aliasing.h
search.o: /usr/include/sys/_posix_availability.h
search.o: /usr/include/machine/_types.h /usr/include/i386/_types.h
search.o: /usr/include/sys/_pthread/_pthread_types.h /usr/include/sys/wait.h
search.o: /usr/include/sys/_types/_pid_t.h /usr/include/sys/_types/_id_t.h
search.o: /usr/include/sys/signal.h /usr/include/sys/appleapiopts.h
search.o: /usr/include/machine/signal.h /usr/include/i386/signal.h
search.o: /usr/include/machine/_mcontext.h /usr/include/i386/_mcontext.h
search.o: /usr/include/mach/i386/_structs.h
search.o: /usr/include/sys/_pthread/_pthread_attr_t.h
search.o: /usr/include/sys/_types/_sigaltstack.h
search.o: /usr/include/sys/_types/_ucontext.h
search.o: /usr/include/sys/_types/_sigset_t.h
search.o: /usr/include/sys/_types/_size_t.h /usr/include/sys/_types/_uid_t.h
search.o: /usr/include/sys/resource.h /usr/include/stdint.h
search.o: /usr/include/sys/_types/_int8_t.h
search.o: /usr/include/sys/_types/_int16_t.h
search.o: /usr/include/sys/_types/_int32_t.h
search.o: /usr/include/sys/_types/_int64_t.h /usr/include/_types/_uint8_t.h
search.o: /usr/include/_types/_uint16_t.h /usr/include/_types/_uint32_t.h
search.o: /usr/include/_types/_uint64_t.h /usr/include/sys/_types/_intptr_t.h
search.o: /usr/include/sys/_types/_uintptr_t.h
search.o: /usr/include/_types/_intmax_t.h /usr/include/_types/_uintmax_t.h
search.o: /usr/include/sys/_types/_timeval.h /usr/include/machine/endian.h
search.o: /usr/include/i386/endian.h /usr/include/sys/_endian.h
search.o: /usr/include/libkern/_OSByteOrder.h
search.o: /usr/include/libkern/i386/_OSByteOrder.h /usr/include/alloca.h
search.o: /usr/include/sys/_types/_ct_rune_t.h
search.o: /usr/include/sys/_types/_rune_t.h
search.o: /usr/include/sys/_types/_wchar_t.h /usr/include/sys/_types/_null.h
search.o: /usr/include/machine/types.h /usr/include/i386/types.h
search.o: /usr/include/sys/_types/_dev_t.h /usr/include/sys/_types/_mode_t.h
search.o: /usr/include/stdio.h /usr/include/sys/_types/_va_list.h
search.o: /usr/include/sys/stdio.h /usr/include/sys/_types/_off_t.h
search.o: /usr/include/sys/_types/_ssize_t.h /usr/include/secure/_stdio.h
search.o: /usr/include/secure/_common.h /usr/include/strings.h
search.o: /usr/include/string.h /usr/include/sys/_types/_rsize_t.h
search.o: /usr/include/sys/_types/_errno_t.h /usr/include/secure/_string.h
search.o: /usr/include/ctype.h /usr/include/runetype.h
search.o: /usr/include/sys/_types/_wint_t.h /usr/include/sys/types.h
search.o: /usr/include/sys/_types/_blkcnt_t.h
search.o: /usr/include/sys/_types/_blksize_t.h
search.o: /usr/include/sys/_types/_gid_t.h
search.o: /usr/include/sys/_types/_in_addr_t.h
search.o: /usr/include/sys/_types/_in_port_t.h
search.o: /usr/include/sys/_types/_ino_t.h /usr/include/sys/_types/_ino64_t.h
search.o: /usr/include/sys/_types/_key_t.h /usr/include/sys/_types/_nlink_t.h
search.o: /usr/include/sys/_types/_clock_t.h
search.o: /usr/include/sys/_types/_time_t.h
search.o: /usr/include/sys/_types/_useconds_t.h
search.o: /usr/include/sys/_types/_suseconds_t.h
search.o: /usr/include/sys/_types/_fd_def.h
search.o: /usr/include/sys/_types/_fd_setsize.h
search.o: /usr/include/sys/_types/_fd_set.h /usr/include/sys/_types/_fd_clr.h
search.o: /usr/include/sys/_types/_fd_zero.h
search.o: /usr/include/sys/_types/_fd_isset.h
search.o: /usr/include/sys/_types/_fd_copy.h
search.o: /usr/include/sys/_pthread/_pthread_cond_t.h
search.o: /usr/include/sys/_pthread/_pthread_condattr_t.h
search.o: /usr/include/sys/_pthread/_pthread_mutex_t.h
search.o: /usr/include/sys/_pthread/_pthread_mutexattr_t.h
search.o: /usr/include/sys/_pthread/_pthread_once_t.h
search.o: /usr/include/sys/_pthread/_pthread_rwlock_t.h
search.o: /usr/include/sys/_pthread/_pthread_rwlockattr_t.h
search.o: /usr/include/sys/_pthread/_pthread_t.h
search.o: /usr/include/sys/_pthread/_pthread_key_t.h
search.o: /usr/include/sys/_types/_fsblkcnt_t.h
search.o: /usr/include/sys/_types/_fsfilcnt_t.h /usr/include/sys/timeb.h
search.o: /usr/include/sys/times.h output.h memory.h expressions.h relax.h
search.o: search.h
utility.o: /usr/include/string.h /usr/include/_types.h
utility.o: /usr/include/sys/_types.h /usr/include/sys/cdefs.h
utility.o: /usr/include/sys/_symbol_aliasing.h
utility.o: /usr/include/sys/_posix_availability.h
utility.o: /usr/include/machine/_types.h /usr/include/i386/_types.h
utility.o: /usr/include/sys/_pthread/_pthread_types.h
utility.o: /usr/include/Availability.h /usr/include/AvailabilityInternal.h
utility.o: /usr/include/sys/_types/_size_t.h /usr/include/sys/_types/_null.h
utility.o: /usr/include/sys/_types/_rsize_t.h
utility.o: /usr/include/sys/_types/_errno_t.h
utility.o: /usr/include/sys/_types/_ssize_t.h /usr/include/strings.h
utility.o: /usr/include/secure/_string.h /usr/include/secure/_common.h
utility.o: utility.h ff.h /usr/include/stdlib.h /usr/include/sys/wait.h
utility.o: /usr/include/sys/_types/_pid_t.h /usr/include/sys/_types/_id_t.h
utility.o: /usr/include/sys/signal.h /usr/include/sys/appleapiopts.h
utility.o: /usr/include/machine/signal.h /usr/include/i386/signal.h
utility.o: /usr/include/machine/_mcontext.h /usr/include/i386/_mcontext.h
utility.o: /usr/include/mach/i386/_structs.h
utility.o: /usr/include/sys/_pthread/_pthread_attr_t.h
utility.o: /usr/include/sys/_types/_sigaltstack.h
utility.o: /usr/include/sys/_types/_ucontext.h
utility.o: /usr/include/sys/_types/_sigset_t.h
utility.o: /usr/include/sys/_types/_uid_t.h /usr/include/sys/resource.h
utility.o: /usr/include/stdint.h /usr/include/sys/_types/_int8_t.h
utility.o: /usr/include/sys/_types/_int16_t.h
utility.o: /usr/include/sys/_types/_int32_t.h
utility.o: /usr/include/sys/_types/_int64_t.h /usr/include/_types/_uint8_t.h
utility.o: /usr/include/_types/_uint16_t.h /usr/include/_types/_uint32_t.h
utility.o: /usr/include/_types/_uint64_t.h
utility.o: /usr/include/sys/_types/_intptr_t.h
utility.o: /usr/include/sys/_types/_uintptr_t.h
utility.o: /usr/include/_types/_intmax_t.h /usr/include/_types/_uintmax_t.h
utility.o: /usr/include/sys/_types/_timeval.h /usr/include/machine/endian.h
utility.o: /usr/include/i386/endian.h /usr/include/sys/_endian.h
utility.o: /usr/include/libkern/_OSByteOrder.h
utility.o: /usr/include/libkern/i386/_OSByteOrder.h /usr/include/alloca.h
utility.o: /usr/include/sys/_types/_ct_rune_t.h
utility.o: /usr/include/sys/_types/_rune_t.h
utility.o: /usr/include/sys/_types/_wchar_t.h /usr/include/machine/types.h
utility.o: /usr/include/i386/types.h /usr/include/sys/_types/_dev_t.h
utility.o: /usr/include/sys/_types/_mode_t.h /usr/include/stdio.h
utility.o: /usr/include/sys/_types/_va_list.h /usr/include/sys/stdio.h
utility.o: /usr/include/sys/_types/_off_t.h /usr/include/secure/_stdio.h
utility.o: /usr/include/ctype.h /usr/include/runetype.h
utility.o: /usr/include/sys/_types/_wint_t.h /usr/include/sys/types.h
utility.o: /usr/include/sys/_types/_blkcnt_t.h
utility.o: /usr/include/sys/_types/_blksize_t.h
utility.o: /usr/include/sys/_types/_gid_t.h
utility.o: /usr/include/sys/_types/_in_addr_t.h
utility.o: /usr/include/sys/_types/_in_port_t.h
utility.o: /usr/include/sys/_types/_ino_t.h
utility.o: /usr/include/sys/_types/_ino64_t.h
utility.o: /usr/include/sys/_types/_key_t.h
utility.o: /usr/include/sys/_types/_nlink_t.h
utility.o: /usr/include/sys/_types/_clock_t.h
utility.o: /usr/include/sys/_types/_time_t.h
utility.o: /usr/include/sys/_types/_useconds_t.h
utility.o: /usr/include/sys/_types/_suseconds_t.h
utility.o: /usr/include/sys/_types/_fd_def.h
utility.o: /usr/include/sys/_types/_fd_setsize.h
utility.o: /usr/include/sys/_types/_fd_set.h
utility.o: /usr/include/sys/_types/_fd_clr.h
utility.o: /usr/include/sys/_types/_fd_zero.h
utility.o: /usr/include/sys/_types/_fd_isset.h
utility.o: /usr/include/sys/_types/_fd_copy.h
utility.o: /usr/include/sys/_pthread/_pthread_cond_t.h
utility.o: /usr/include/sys/_pthread/_pthread_condattr_t.h
utility.o: /usr/include/sys/_pthread/_pthread_mutex_t.h
utility.o: /usr/include/sys/_pthread/_pthread_mutexattr_t.h
utility.o: /usr/include/sys/_pthread/_pthread_once_t.h
utility.o: /usr/include/sys/_pthread/_pthread_rwlock_t.h
utility.o: /usr/include/sys/_pthread/_pthread_rwlockattr_t.h
utility.o: /usr/include/sys/_pthread/_pthread_t.h
utility.o: /usr/include/sys/_pthread/_pthread_key_t.h
utility.o: /usr/include/sys/_types/_fsblkcnt_t.h
utility.o: /usr/include/sys/_types/_fsfilcnt_t.h /usr/include/sys/timeb.h
utility.o: /usr/include/sys/times.h output.h
libff.o: libff.h ff.h /usr/include/stdlib.h /usr/include/Availability.h
libff.o: /usr/include/AvailabilityInternal.h /usr/include/_types.h
libff.o: /usr/include/sys/_types.h /usr/include/sys/cdefs.h
libff.o: /usr/include/sys/_symbol_aliasing.h
libff.o: /usr/include/sys/_posix_availability.h /usr/include/machine/_types.h
libff.o: /usr/include/i386/_types.h
libff.o: /usr/include/sys/_pthread/_pthread_types.h /usr/include/sys/wait.h
libff.o: /usr/include/sys/_types/_pid_t.h /usr/include/sys/_types/_id_t.h
libff.o: /usr/include/sys/signal.h /usr/include/sys/appleapiopts.h
libff.o: /usr/include/machine/signal.h /usr/include/i386/signal.h
libff.o: /usr/include/machine/_mcontext.h /usr/include/i386/_mcontext.h
libff.o: /usr/include/mach/i386/_structs.h
libff.o: /usr/include/sys/_pthread/_pthread_attr_t.h
libff.o: /usr/include/sys/_types/_sigaltstack.h
libff.o: /usr/include/sys/_types/_ucontext.h
libff.o: /usr/include/sys/_types/_sigset_t.h
libff.o: /usr/include/sys/_types/_size_t.h /usr/include/sys/_types/_uid_t.h
libff.o: /usr/include/sys/resource.h /usr/include/stdint.h
libff.o: /usr/include/sys/_types/_int8_t.h /usr/include/sys/_types/_int16_t.h
libff.o: /usr/include/sys/_types/_int32_t.h
libff.o: /usr/include/sys/_types/_int64_t.h /usr/include/_types/_uint8_t.h
libff.o: /usr/include/_types/_uint16_t.h /usr/include/_types/_uint32_t.h
libff.o: /usr/include/_types/_uint64_t.h /usr/include/sys/_types/_intptr_t.h
libff.o: /usr/include/sys/_types/_uintptr_t.h /usr/include/_types/_intmax_t.h
libff.o: /usr/include/_types/_uintmax_t.h /usr/include/sys/_types/_timeval.h
libff.o: /usr/include/machine/endian.h /usr/include/i386/endian.h
libff.o: /usr/include/sys/_endian.h /usr/include/libkern/_OSByteOrder.h
libff.o: /usr/include/libkern/i386/_OSByteOrder.h /usr/include/alloca.h
libff.o: /usr/include/sys/_types/_ct_rune_t.h
libff.o: /usr/include/sys/_types/_rune_t.h /usr/include/sys/_types/_wchar_t.h
libff.o: /usr/include/sys/_types/_null.h /usr/include/machine/types.h
libff.o: /usr/include/i386/types.h /usr/include/sys/_types/_dev_t.h
libff.o: /usr/include/sys/_types/_mode_t.h /usr/include/stdio.h
libff.o: /usr/include/sys/_types/_va_list.h /usr/include/sys/stdio.h
libff.o: /usr/include/sys/_types/_off_t.h /usr/include/sys/_types/_ssize_t.h
libff.o: /usr/include/secure/_stdio.h /usr/include/secure/_common.h
libff.o: /usr/include/strings.h /usr/include/string.h
libff.o: /usr/include/sys/_types/_rsize_t.h
libff.o: /usr/include/sys/_types/_errno_t.h /usr/include/secure/_string.h
libff.o: /usr/include/ctype.h /usr/include/runetype.h
libff.o: /usr/include/sys/_types/_wint_t.h /usr/include/sys/types.h
libff.o: /usr/include/sys/_types/_blkcnt_t.h
libff.o: /usr/include/sys/_types/_blksize_t.h
libff.o: /usr/include/sys/_types/_gid_t.h
libff.o: /usr/include/sys/_types/_in_addr_t.h
libff.o: /usr/include/sys/_types/_in_port_t.h
libff.o: /usr/include/sys/_types/_ino_t.h /usr/include/sys/_types/_ino64_t.h
libff.o: /usr/include/sys/_types/_key_t.h /usr/include/sys/_types/_nlink_t.h
libff.o: /usr/include/sys/_types/_clock_t.h /usr/include/sys/_types/_time_t.h
libff.o: /usr/include/sys/_types/_useconds_t.h
libff.o: /usr/include/sys/_types/_suseconds_t.h
libff.o: /usr/include/sys/_types/_fd_def.h
libff.o: /usr/include/sys/_types/_fd_setsize.h
libff.o: /usr/include/sys/_types/_fd_set.h /usr/include/sys/_types/_fd_clr.h
libff.o: /usr/include/sys/_types/_fd_zero.h
libff.o: /usr/include/sys/_types/_fd_isset.h
libff.o: /usr/include/sys/_types/_fd_copy.h
libff.o: /usr/include/sys/_pthread/_pthread_cond_t.h
libff.o: /usr/include/sys/_pthread/_pthread_condattr_t.h
libff.o: /usr/include/sys/_pthread/_pthread_mutex_t.h
libff.o: /usr/include/sys/_pthread/_pthread_mutexattr_t.h
libff.o: /usr/include/sys/_pthread/_pthread_once_t.h
libff.o: /usr/include/sys/_pthread/_pthread_rwlock_t.h
libff.o: /usr/include/sys/_pthread/_pthread_rwlockattr_t.h
libff.o: /usr/include/sys/_pthread/_pthread_t.h
libff.o: /usr/include/sys/_pthread/_pthread_key_t.h
libff.o: /usr/include/sys/_types/_fsblkcnt_t.h
libff.o: /usr/include/sys/_types/_fsfilcnt_t.h /usr/include/sys/timeb.h
libff.o: /usr/include/sys/times.h memory.h output.h parse.h expressions.h
libff.o: inst_pre.h inst_easy.h inst_hard.h inst_final.h
memory.o: /usr/include/string.h /usr/include/_types.h
memory.o: /usr/include/sys/_types.h /usr/include/sys/cdefs.h
memory.o: /usr/include/sys/_symbol_aliasing.h
memory.o: /usr/include/sys/_posix_availability.h
memory.o: /usr/include/machine/_types.h /usr/include/i386/_types.h
memory.o: /usr/include/sys/_pthread/_pthread_types.h
memory.o: /usr/include/Availability.h /usr/include/AvailabilityInternal.h
memory.o: /usr/include/sys/_types/_size_t.h /usr/include/sys/_types/_null.h
memory.o: /usr/include/sys/_types/_rsize_t.h
memory.o: /usr/include/sys/_types/_errno_t.h
memory.o: /usr/include/sys/_types/_ssize_t.h /usr/include/strings.h
memory.o: /usr/include/secure/_string.h /usr/include/secure/_common.h ff.h
memory.o: /usr/include/stdlib.h /usr/include/sys/wait.h
memory.o: /usr/include/sys/_types/_pid_t.h /usr/include/sys/_types/_id_t.h
memory.o: /usr/include/sys/signal.h /usr/include/sys/appleapiopts.h
memory.o: /usr/include/machine/signal.h /usr/include/i386/signal.h
memory.o: /usr/include/machine/_mcontext.h /usr/include/i386/_mcontext.h
memory.o: /usr/include/mach/i386/_structs.h
memory.o: /usr/include/sys/_pthread/_pthread_attr_t.h
memory.o: /usr/include/sys/_types/_sigaltstack.h
memory.o: /usr/include/sys/_types/_ucontext.h
memory.o: /usr/include/sys/_types/_sigset_t.h
memory.o: /usr/include/sys/_types/_uid_t.h /usr/include/sys/resource.h
memory.o: /usr/include/stdint.h /usr/include/sys/_types/_int8_t.h
memory.o: /usr/include/sys/_types/_int16_t.h
memory.o: /usr/include/sys/_types/_int32_t.h
memory.o: /usr/include/sys/_types/_int64_t.h /usr/include/_types/_uint8_t.h
memory.o: /usr/include/_types/_uint16_t.h /usr/include/_types/_uint32_t.h
memory.o: /usr/include/_types/_uint64_t.h /usr/include/sys/_types/_intptr_t.h
memory.o: /usr/include/sys/_types/_uintptr_t.h
memory.o: /usr/include/_types/_intmax_t.h /usr/include/_types/_uintmax_t.h
memory.o: /usr/include/sys/_types/_timeval.h /usr/include/machine/endian.h
memory.o: /usr/include/i386/endian.h /usr/include/sys/_endian.h
memory.o: /usr/include/libkern/_OSByteOrder.h
memory.o: /usr/include/libkern/i386/_OSByteOrder.h /usr/include/alloca.h
memory.o: /usr/include/sys/_types/_ct_rune_t.h
memory.o: /usr/include/sys/_types/_rune_t.h
memory.o: /usr/include/sys/_types/_wchar_t.h /usr/include/machine/types.h
memory.o: /usr/include/i386/types.h /usr/include/sys/_types/_dev_t.h
memory.o: /usr/include/sys/_types/_mode_t.h /usr/include/stdio.h
memory.o: /usr/include/sys/_types/_va_list.h /usr/include/sys/stdio.h
memory.o: /usr/include/sys/_types/_off_t.h /usr/include/secure/_stdio.h
memory.o: /usr/include/ctype.h /usr/include/runetype.h
memory.o: /usr/include/sys/_types/_wint_t.h /usr/include/sys/types.h
memory.o: /usr/include/sys/_types/_blkcnt_t.h
memory.o: /usr/include/sys/_types/_blksize_t.h
memory.o: /usr/include/sys/_types/_gid_t.h
memory.o: /usr/include/sys/_types/_in_addr_t.h
memory.o: /usr/include/sys/_types/_in_port_t.h
memory.o: /usr/include/sys/_types/_ino_t.h /usr/include/sys/_types/_ino64_t.h
memory.o: /usr/include/sys/_types/_key_t.h /usr/include/sys/_types/_nlink_t.h
memory.o: /usr/include/sys/_types/_clock_t.h
memory.o: /usr/include/sys/_types/_time_t.h
memory.o: /usr/include/sys/_types/_useconds_t.h
memory.o: /usr/include/sys/_types/_suseconds_t.h
memory.o: /usr/include/sys/_types/_fd_def.h
memory.o: /usr/include/sys/_types/_fd_setsize.h
memory.o: /usr/include/sys/_types/_fd_set.h /usr/include/sys/_types/_fd_clr.h
memory.o: /usr/include/sys/_types/_fd_zero.h
memory.o: /usr/include/sys/_types/_fd_isset.h
memory.o: /usr/include/sys/_types/_fd_copy.h
memory.o: /usr/include/sys/_pthread/_pthread_cond_t.h
memory.o: /usr/include/sys/_pthread/_pthread_condattr_t.h
memory.o: /usr/include/sys/_pthread/_pthread_mutex_t.h
memory.o: /usr/include/sys/_pthread/_pthread_mutexattr_t.h
memory.o: /usr/include/sys/_pthread/_pthread_once_t.h
memory.o: /usr/include/sys/_pthread/_pthread_rwlock_t.h
memory.o: /usr/include/sys/_pthread/_pthread_rwlockattr_t.h
memory.o: /usr/include/sys/_pthread/_pthread_t.h
memory.o: /usr/include/sys/_pthread/_pthread_key_t.h
memory.o: /usr/include/sys/_types/_fsblkcnt_t.h
memory.o: /usr/include/sys/_types/_fsfilcnt_t.h /usr/include/sys/timeb.h
memory.o: /usr/include/sys/times.h memory.h inst_pre.h
output.o: ff.h /usr/include/stdlib.h /usr/include/Availability.h
output.o: /usr/include/AvailabilityInternal.h /usr/include/_types.h
output.o: /usr/include/sys/_types.h /usr/include/sys/cdefs.h
output.o: /usr/include/sys/_symbol_aliasing.h
output.o: /usr/include/sys/_posix_availability.h
output.o: /usr/include/machine/_types.h /usr/include/i386/_types.h
output.o: /usr/include/sys/_pthread/_pthread_types.h /usr/include/sys/wait.h
output.o: /usr/include/sys/_types/_pid_t.h /usr/include/sys/_types/_id_t.h
output.o: /usr/include/sys/signal.h /usr/include/sys/appleapiopts.h
output.o: /usr/include/machine/signal.h /usr/include/i386/signal.h
output.o: /usr/include/machine/_mcontext.h /usr/include/i386/_mcontext.h
output.o: /usr/include/mach/i386/_structs.h
output.o: /usr/include/sys/_pthread/_pthread_attr_t.h
output.o: /usr/include/sys/_types/_sigaltstack.h
output.o: /usr/include/sys/_types/_ucontext.h
output.o: /usr/include/sys/_types/_sigset_t.h
output.o: /usr/include/sys/_types/_size_t.h /usr/include/sys/_types/_uid_t.h
output.o: /usr/include/sys/resource.h /usr/include/stdint.h
output.o: /usr/include/sys/_types/_int8_t.h
output.o: /usr/include/sys/_types/_int16_t.h
output.o: /usr/include/sys/_types/_int32_t.h
output.o: /usr/include/sys/_types/_int64_t.h /usr/include/_types/_uint8_t.h
output.o: /usr/include/_types/_uint16_t.h /usr/include/_types/_uint32_t.h
output.o: /usr/include/_types/_uint64_t.h /usr/include/sys/_types/_intptr_t.h
output.o: /usr/include/sys/_types/_uintptr_t.h
output.o: /usr/include/_types/_intmax_t.h /usr/include/_types/_uintmax_t.h
output.o: /usr/include/sys/_types/_timeval.h /usr/include/machine/endian.h
output.o: /usr/include/i386/endian.h /usr/include/sys/_endian.h
output.o: /usr/include/libkern/_OSByteOrder.h
output.o: /usr/include/libkern/i386/_OSByteOrder.h /usr/include/alloca.h
output.o: /usr/include/sys/_types/_ct_rune_t.h
output.o: /usr/include/sys/_types/_rune_t.h
output.o: /usr/include/sys/_types/_wchar_t.h /usr/include/sys/_types/_null.h
output.o: /usr/include/machine/types.h /usr/include/i386/types.h
output.o: /usr/include/sys/_types/_dev_t.h /usr/include/sys/_types/_mode_t.h
output.o: /usr/include/stdio.h /usr/include/sys/_types/_va_list.h
output.o: /usr/include/sys/stdio.h /usr/include/sys/_types/_off_t.h
output.o: /usr/include/sys/_types/_ssize_t.h /usr/include/secure/_stdio.h
output.o: /usr/include/secure/_common.h /usr/include/strings.h
output.o: /usr/include/string.h /usr/include/sys/_types/_rsize_t.h
output.o: /usr/include/sys/_types/_errno_t.h /usr/include/secure/_string.h
output.o: /usr/include/ctype.h /usr/include/runetype.h
output.o: /usr/include/sys/_types/_wint_t.h /usr/include/sys/types.h
output.o: /usr/include/sys/_types/_blkcnt_t.h
output.o: /usr/include/sys/_types/_blksize_t.h
output.o: /usr/include/sys/_types/_gid_t.h
output.o: /usr/include/sys/_types/_in_addr_t.h
output.o: /usr/include/sys/_types/_in_port_t.h
output.o: /usr/include/sys/_types/_ino_t.h /usr/include/sys/_types/_ino64_t.h
output.o: /usr/include/sys/_types/_key_t.h /usr/include/sys/_types/_nlink_t.h
output.o: /usr/include/sys/_types/_clock_t.h
output.o: /usr/include/sys/_types/_time_t.h
output.o: /usr/include/sys/_types/_useconds_t.h
output.o: /usr/include/sys/_types/_suseconds_t.h
output.o: /usr/include/sys/_types/_fd_def.h
output.o: /usr/include/sys/_types/_fd_setsize.h
output.o: /usr/include/sys/_types/_fd_set.h /usr/include/sys/_types/_fd_clr.h
output.o: /usr/include/sys/_types/_fd_zero.h
output.o: /usr/include/sys/_types/_fd_isset.h
output.o: /usr/include/sys/_types/_fd_copy.h
output.o: /usr/include/sys/_pthread/_pthread_cond_t.h
output.o: /usr/include/sys/_pthread/_pthread_condattr_t.h
output.o: /usr/include/sys/_pthread/_pthread_mutex_t.h
output.o: /usr/include/sys/_pthread/_pthread_mutexattr_t.h
output.o: /usr/include/sys/_pthread/_pthread_once_t.h
output.o: /usr/include/sys/_pthread/_pthread_rwlock_t.h
output.o: /usr/include/sys/_pthread/_pthread_rwlockattr_t.h
output.o: /usr/include/sys/_pthread/_pthread_t.h
output.o: /usr/include/sys/_pthread/_pthread_key_t.h
output.o: /usr/include/sys/_types/_fsblkcnt_t.h
output.o: /usr/include/sys/_types/_fsfilcnt_t.h /usr/include/sys/timeb.h
output.o: /usr/include/sys/times.h utility.h output.h
parse.o: ff.h /usr/include/stdlib.h /usr/include/Availability.h
parse.o: /usr/include/AvailabilityInternal.h /usr/include/_types.h
parse.o: /usr/include/sys/_types.h /usr/include/sys/cdefs.h
parse.o: /usr/include/sys/_symbol_aliasing.h
parse.o: /usr/include/sys/_posix_availability.h /usr/include/machine/_types.h
parse.o: /usr/include/i386/_types.h
parse.o: /usr/include/sys/_pthread/_pthread_types.h /usr/include/sys/wait.h
parse.o: /usr/include/sys/_types/_pid_t.h /usr/include/sys/_types/_id_t.h
parse.o: /usr/include/sys/signal.h /usr/include/sys/appleapiopts.h
parse.o: /usr/include/machine/signal.h /usr/include/i386/signal.h
parse.o: /usr/include/machine/_mcontext.h /usr/include/i386/_mcontext.h
parse.o: /usr/include/mach/i386/_structs.h
parse.o: /usr/include/sys/_pthread/_pthread_attr_t.h
parse.o: /usr/include/sys/_types/_sigaltstack.h
parse.o: /usr/include/sys/_types/_ucontext.h
parse.o: /usr/include/sys/_types/_sigset_t.h
parse.o: /usr/include/sys/_types/_size_t.h /usr/include/sys/_types/_uid_t.h
parse.o: /usr/include/sys/resource.h /usr/include/stdint.h
parse.o: /usr/include/sys/_types/_int8_t.h /usr/include/sys/_types/_int16_t.h
parse.o: /usr/include/sys/_types/_int32_t.h
parse.o: /usr/include/sys/_types/_int64_t.h /usr/include/_types/_uint8_t.h
parse.o: /usr/include/_types/_uint16_t.h /usr/include/_types/_uint32_t.h
parse.o: /usr/include/_types/_uint64_t.h /usr/include/sys/_types/_intptr_t.h
parse.o: /usr/include/sys/_types/_uintptr_t.h /usr/include/_types/_intmax_t.h
parse.o: /usr/include/_types/_uintmax_t.h /usr/include/sys/_types/_timeval.h
parse.o: /usr/include/machine/endian.h /usr/include/i386/endian.h
parse.o: /usr/include/sys/_endian.h /usr/include/libkern/_OSByteOrder.h
parse.o: /usr/include/libkern/i386/_OSByteOrder.h /usr/include/alloca.h
parse.o: /usr/include/sys/_types/_ct_rune_t.h
parse.o: /usr/include/sys/_types/_rune_t.h /usr/include/sys/_types/_wchar_t.h
parse.o: /usr/include/sys/_types/_null.h /usr/include/machine/types.h
parse.o: /usr/include/i386/types.h /usr/include/sys/_types/_dev_t.h
parse.o: /usr/include/sys/_types/_mode_t.h /usr/include/stdio.h
parse.o: /usr/include/sys/_types/_va_list.h /usr/include/sys/stdio.h
parse.o: /usr/include/sys/_types/_off_t.h /usr/include/sys/_types/_ssize_t.h
parse.o: /usr/include/secure/_stdio.h /usr/include/secure/_common.h
parse.o: /usr/include/strings.h /usr/include/string.h
parse.o: /usr/include/sys/_types/_rsize_t.h
parse.o: /usr/include/sys/_types/_errno_t.h /usr/include/secure/_string.h
parse.o: /usr/include/ctype.h /usr/include/runetype.h
parse.o: /usr/include/sys/_types/_wint_t.h /usr/include/sys/types.h
parse.o: /usr/include/sys/_types/_blkcnt_t.h
parse.o: /usr/include/sys/_types/_blksize_t.h
parse.o: /usr/include/sys/_types/_gid_t.h
parse.o: /usr/include/sys/_types/_in_addr_t.h
parse.o: /usr/include/sys/_types/_in_port_t.h
parse.o: /usr/include/sys/_types/_ino_t.h /usr/include/sys/_types/_ino64_t.h
parse.o: /usr/include/sys/_types/_key_t.h /usr/include/sys/_types/_nlink_t.h
parse.o: /usr/include/sys/_types/_clock_t.h /usr/include/sys/_types/_time_t.h
parse.o: /usr/include/sys/_types/_useconds_t.h
parse.o: /usr/include/sys/_types/_suseconds_t.h
parse.o: /usr/include/sys/_types/_fd_def.h
parse.o: /usr/include/sys/_types/_fd_setsize.h
parse.o: /usr/include/sys/_types/_fd_set.h /usr/include/sys/_types/_fd_clr.h
parse.o: /usr/include/sys/_types/_fd_zero.h
parse.o: /usr/include/sys/_types/_fd_isset.h
parse.o: /usr/include/sys/_types/_fd_copy.h
parse.o: /usr/include/sys/_pthread/_pthread_cond_t.h
parse.o: /usr/include/sys/_pthread/_pthread_condattr_t.h
parse.o: /usr/include/sys/_pthread/_pthread_mutex_t.h
parse.o: /usr/include/sys/_pthread/_pthread_mutexattr_t.h
parse.o: /usr/include/sys/_pthread/_pthread_once_t.h
parse.o: /usr/include/sys/_pthread/_pthread_rwlock_t.h
parse.o: /usr/include/sys/_pthread/_pthread_rwlockattr_t.h
parse.o: /usr/include/sys/_pthread/_pthread_t.h
parse.o: /usr/include/sys/_pthread/_pthread_key_t.h
parse.o: /usr/include/sys/_types/_fsblkcnt_t.h
parse.o: /usr/include/sys/_types/_fsfilcnt_t.h /usr/include/sys/timeb.h
parse.o: /usr/include/sys/times.h memory.h output.h parse.h
expressions.o: /usr/include/string.h /usr/include/_types.h
expressions.o: /usr/include/sys/_types.h /usr/include/sys/cdefs.h
expressions.o: /usr/include/sys/_symbol_aliasing.h
expressions.o: /usr/include/sys/_posix_availability.h
expressions.o: /usr/include/machine/_types.h /usr/include/i386/_types.h
expressions.o: /usr/include/sys/_pthread/_pthread_types.h
expressions.o: /usr/include/Availability.h
expressions.o: /usr/include/AvailabilityInternal.h
expressions.o: /usr/include/sys/_types/_size_t.h
expressions.o: /usr/include/sys/_types/_null.h
expressions.o: /usr/include/sys/_types/_rsize_t.h
expressions.o: /usr/include/sys/_types/_errno_t.h
expressions.o: /usr/include/sys/_types/_ssize_t.h /usr/include/strings.h
expressions.o: /usr/include/secure/_string.h /usr/include/secure/_common.h
expressions.o: ff.h /usr/include/stdlib.h /usr/include/sys/wait.h
expressions.o: /usr/include/sys/_types/_pid_t.h
expressions.o: /usr/include/sys/_types/_id_t.h /usr/include/sys/signal.h
expressions.o: /usr/include/sys/appleapiopts.h /usr/include/machine/signal.h
expressions.o: /usr/include/i386/signal.h /usr/include/machine/_mcontext.h
expressions.o: /usr/include/i386/_mcontext.h
expressions.o: /usr/include/mach/i386/_structs.h
expressions.o: /usr/include/sys/_pthread/_pthread_attr_t.h
expressions.o: /usr/include/sys/_types/_sigaltstack.h
expressions.o: /usr/include/sys/_types/_ucontext.h
expressions.o: /usr/include/sys/_types/_sigset_t.h
expressions.o: /usr/include/sys/_types/_uid_t.h /usr/include/sys/resource.h
expressions.o: /usr/include/stdint.h /usr/include/sys/_types/_int8_t.h
expressions.o: /usr/include/sys/_types/_int16_t.h
expressions.o: /usr/include/sys/_types/_int32_t.h
expressions.o: /usr/include/sys/_types/_int64_t.h
expressions.o: /usr/include/_types/_uint8_t.h /usr/include/_types/_uint16_t.h
expressions.o: /usr/include/_types/_uint32_t.h
expressions.o: /usr/include/_types/_uint64_t.h
expressions.o: /usr/include/sys/_types/_intptr_t.h
expressions.o: /usr/include/sys/_types/_uintptr_t.h
expressions.o: /usr/include/_types/_intmax_t.h
expressions.o: /usr/include/_types/_uintmax_t.h
expressions.o: /usr/include/sys/_types/_timeval.h
expressions.o: /usr/include/machine/endian.h /usr/include/i386/endian.h
expressions.o: /usr/include/sys/_endian.h /usr/include/libkern/_OSByteOrder.h
expressions.o: /usr/include/libkern/i386/_OSByteOrder.h /usr/include/alloca.h
expressions.o: /usr/include/sys/_types/_ct_rune_t.h
expressions.o: /usr/include/sys/_types/_rune_t.h
expressions.o: /usr/include/sys/_types/_wchar_t.h
expressions.o: /usr/include/machine/types.h /usr/include/i386/types.h
expressions.o: /usr/include/sys/_types/_dev_t.h
expressions.o: /usr/include/sys/_types/_mode_t.h /usr/include/stdio.h
expressions.o: /usr/include/sys/_types/_va_list.h /usr/include/sys/stdio.h
expressions.o: /usr/include/sys/_types/_off_t.h /usr/include/secure/_stdio.h
expressions.o: /usr/include/ctype.h /usr/include/runetype.h
expressions.o: /usr/include/sys/_types/_wint_t.h /usr/include/sys/types.h
expressions.o: /usr/include/sys/_types/_blkcnt_t.h
expressions.o: /usr/include/sys/_types/_blksize_t.h
expressions.o: /usr/include/sys/_types/_gid_t.h
expressions.o: /usr/include/sys/_types/_in_addr_t.h
expressions.o: /usr/include/sys/_types/_in_port_t.h
expressions.o: /usr/include/sys/_types/_ino_t.h
expressions.o: /usr/include/sys/_types/_ino64_t.h
expressions.o: /usr/include/sys/_types/_key_t.h
expressions.o: /usr/include/sys/_types/_nlink_t.h
expressions.o: /usr/include/sys/_types/_clock_t.h
expressions.o: /usr/include/sys/_types/_time_t.h
expressions.o: /usr/include/sys/_types/_useconds_t.h
expressions.o: /usr/include/sys/_types/_suseconds_t.h
expressions.o: /usr/include/sys/_types/_fd_def.h
expressions.o: /usr/include/sys/_types/_fd_setsize.h
expressions.o: /usr/include/sys/_types/_fd_set.h
expressions.o: /usr/include/sys/_types/_fd_clr.h
expressions.o: /usr/include/sys/_types/_fd_zero.h
expressions.o: /usr/include/sys/_types/_fd_isset.h
expressions.o: /usr/include/sys/_types/_fd_copy.h
expressions.o: /usr/include/sys/_pthread/_pthread_cond_t.h
expressions.o: /usr/include/sys/_pthread/_pthread_condattr_t.h
expressions.o: /usr/include/sys/_pthread/_pthread_mutex_t.h
expressions.o: /usr/include/sys/_pthread/_pthread_mutexattr_t.h
expressions.o: /usr/include/sys/_pthread/_pthread_once_t.h
expressions.o: /usr/include/sys/_pthread/_pthread_rwlock_t.h
expressions.o: /usr/include/sys/_pthread/_pthread_rwlockattr_t.h
expressions.o: /usr/include/sys/_pthread/_pthread_t.h
expressions.o: /usr/include/sys/_pthread/_pthread_key_t.h
expressions.o: /usr/include/sys/_types/_fsblkcnt_t.h
expressions.o: /usr/include/sys/_types/_fsfilcnt_t.h /usr/include/sys/timeb.h
expressions.o: /usr/include/sys/times.h output.h memory.h expressions.h
inst_pre.o: /usr/include/string.h /usr/include/_types.h
inst_pre.o: /usr/include/sys/_types.h /usr/include/sys/cdefs.h
inst_pre.o: /usr/include/sys/_symbol_aliasing.h
inst_pre.o: /usr/include/sys/_posix_availability.h
inst_pre.o: /usr/include/machine/_types.h /usr/include/i386/_types.h
inst_pre.o: /usr/include/sys/_pthread/_pthread_types.h
inst_pre.o: /usr/include/Availability.h /usr/include/AvailabilityInternal.h
inst_pre.o: /usr/include/sys/_types/_size_t.h /usr/include/sys/_types/_null.h
inst_pre.o: /usr/include/sys/_types/_rsize_t.h
inst_pre.o: /usr/include/sys/_types/_errno_t.h
inst_pre.o: /usr/include/sys/_types/_ssize_t.h /usr/include/strings.h
inst_pre.o: /usr/include/secure/_string.h /usr/include/secure/_common.h ff.h
inst_pre.o: /usr/include/stdlib.h /usr/include/sys/wait.h
inst_pre.o: /usr/include/sys/_types/_pid_t.h /usr/include/sys/_types/_id_t.h
inst_pre.o: /usr/include/sys/signal.h /usr/include/sys/appleapiopts.h
inst_pre.o: /usr/include/machine/signal.h /usr/include/i386/signal.h
inst_pre.o: /usr/include/machine/_mcontext.h /usr/include/i386/_mcontext.h
inst_pre.o: /usr/include/mach/i386/_structs.h
inst_pre.o: /usr/include/sys/_pthread/_pthread_attr_t.h
inst_pre.o: /usr/include/sys/_types/_sigaltstack.h
inst_pre.o: /usr/include/sys/_types/_ucontext.h
inst_pre.o: /usr/include/sys/_types/_sigset_t.h
inst_pre.o: /usr/include/sys/_types/_uid_t.h /usr/include/sys/resource.h
inst_pre.o: /usr/include/stdint.h /usr/include/sys/_types/_int8_t.h
inst_pre.o: /usr/include/sys/_types/_int16_t.h
inst_pre.o: /usr/include/sys/_types/_int32_t.h
inst_pre.o: /usr/include/sys/_types/_int64_t.h /usr/include/_types/_uint8_t.h
inst_pre.o: /usr/include/_types/_uint16_t.h /usr/include/_types/_uint32_t.h
inst_pre.o: /usr/include/_types/_uint64_t.h
inst_pre.o: /usr/include/sys/_types/_intptr_t.h
inst_pre.o: /usr/include/sys/_types/_uintptr_t.h
inst_pre.o: /usr/include/_types/_intmax_t.h /usr/include/_types/_uintmax_t.h
inst_pre.o: /usr/include/sys/_types/_timeval.h /usr/include/machine/endian.h
inst_pre.o: /usr/include/i386/endian.h /usr/include/sys/_endian.h
inst_pre.o: /usr/include/libkern/_OSByteOrder.h
inst_pre.o: /usr/include/libkern/i386/_OSByteOrder.h /usr/include/alloca.h
inst_pre.o: /usr/include/sys/_types/_ct_rune_t.h
inst_pre.o: /usr/include/sys/_types/_rune_t.h
inst_pre.o: /usr/include/sys/_types/_wchar_t.h /usr/include/machine/types.h
inst_pre.o: /usr/include/i386/types.h /usr/include/sys/_types/_dev_t.h
inst_pre.o: /usr/include/sys/_types/_mode_t.h /usr/include/stdio.h
inst_pre.o: /usr/include/sys/_types/_va_list.h /usr/include/sys/stdio.h
inst_pre.o: /usr/include/sys/_types/_off_t.h /usr/include/secure/_stdio.h
inst_pre.o: /usr/include/ctype.h /usr/include/runetype.h
inst_pre.o: /usr/include/sys/_types/_wint_t.h /usr/include/sys/types.h
inst_pre.o: /usr/include/sys/_types/_blkcnt_t.h
inst_pre.o: /usr/include/sys/_types/_blksize_t.h
inst_pre.o: /usr/include/sys/_types/_gid_t.h
inst_pre.o: /usr/include/sys/_types/_in_addr_t.h
inst_pre.o: /usr/include/sys/_types/_in_port_t.h
inst_pre.o: /usr/include/sys/_types/_ino_t.h
inst_pre.o: /usr/include/sys/_types/_ino64_t.h
inst_pre.o: /usr/include/sys/_types/_key_t.h
inst_pre.o: /usr/include/sys/_types/_nlink_t.h
inst_pre.o: /usr/include/sys/_types/_clock_t.h
inst_pre.o: /usr/include/sys/_types/_time_t.h
inst_pre.o: /usr/include/sys/_types/_useconds_t.h
inst_pre.o: /usr/include/sys/_types/_suseconds_t.h
inst_pre.o: /usr/include/sys/_types/_fd_def.h
inst_pre.o: /usr/include/sys/_types/_fd_setsize.h
inst_pre.o: /usr/include/sys/_types/_fd_set.h
inst_pre.o: /usr/include/sys/_types/_fd_clr.h
inst_pre.o: /usr/include/sys/_types/_fd_zero.h
inst_pre.o: /usr/include/sys/_types/_fd_isset.h
inst_pre.o: /usr/include/sys/_types/_fd_copy.h
inst_pre.o: /usr/include/sys/_pthread/_pthread_cond_t.h
inst_pre.o: /usr/include/sys/_pthread/_pthread_condattr_t.h
inst_pre.o: /usr/include/sys/_pthread/_pthread_mutex_t.h
inst_pre.o: /usr/include/sys/_pthread/_pthread_mutexattr_t.h
inst_pre.o: /usr/include/sys/_pthread/_pthread_once_t.h
inst_pre.o: /usr/include/sys/_pthread/_pthread_rwlock_t.h
inst_pre.o: /usr/include/sys/_pthread/_pthread_rwlockattr_t.h
inst_pre.o: /usr/include/sys/_pthread/_pthread_t.h
inst_pre.o: /usr/include/sys/_pthread/_pthread_key_t.h
inst_pre.o: /usr/include/sys/_types/_fsblkcnt_t.h
inst_pre.o: /usr/include/sys/_types/_fsfilcnt_t.h /usr/include/sys/timeb.h
inst_pre.o: /usr/include/sys/times.h output.h memory.h expressions.h
inst_pre.o: inst_pre.h
inst_easy.o: ff.h /usr/include/stdlib.h /usr/include/Availability.h
inst_easy.o: /usr/include/AvailabilityInternal.h /usr/include/_types.h
inst_easy.o: /usr/include/sys/_types.h /usr/include/sys/cdefs.h
inst_easy.o: /usr/include/sys/_symbol_aliasing.h
inst_easy.o: /usr/include/sys/_posix_availability.h
inst_easy.o: /usr/include/machine/_types.h /usr/include/i386/_types.h
inst_easy.o: /usr/include/sys/_pthread/_pthread_types.h
inst_easy.o: /usr/include/sys/wait.h /usr/include/sys/_types/_pid_t.h
inst_easy.o: /usr/include/sys/_types/_id_t.h /usr/include/sys/signal.h
inst_easy.o: /usr/include/sys/appleapiopts.h /usr/include/machine/signal.h
inst_easy.o: /usr/include/i386/signal.h /usr/include/machine/_mcontext.h
inst_easy.o: /usr/include/i386/_mcontext.h /usr/include/mach/i386/_structs.h
inst_easy.o: /usr/include/sys/_pthread/_pthread_attr_t.h
inst_easy.o: /usr/include/sys/_types/_sigaltstack.h
inst_easy.o: /usr/include/sys/_types/_ucontext.h
inst_easy.o: /usr/include/sys/_types/_sigset_t.h
inst_easy.o: /usr/include/sys/_types/_size_t.h
inst_easy.o: /usr/include/sys/_types/_uid_t.h /usr/include/sys/resource.h
inst_easy.o: /usr/include/stdint.h /usr/include/sys/_types/_int8_t.h
inst_easy.o: /usr/include/sys/_types/_int16_t.h
inst_easy.o: /usr/include/sys/_types/_int32_t.h
inst_easy.o: /usr/include/sys/_types/_int64_t.h
inst_easy.o: /usr/include/_types/_uint8_t.h /usr/include/_types/_uint16_t.h
inst_easy.o: /usr/include/_types/_uint32_t.h /usr/include/_types/_uint64_t.h
inst_easy.o: /usr/include/sys/_types/_intptr_t.h
inst_easy.o: /usr/include/sys/_types/_uintptr_t.h
inst_easy.o: /usr/include/_types/_intmax_t.h /usr/include/_types/_uintmax_t.h
inst_easy.o: /usr/include/sys/_types/_timeval.h /usr/include/machine/endian.h
inst_easy.o: /usr/include/i386/endian.h /usr/include/sys/_endian.h
inst_easy.o: /usr/include/libkern/_OSByteOrder.h
inst_easy.o: /usr/include/libkern/i386/_OSByteOrder.h /usr/include/alloca.h
inst_easy.o: /usr/include/sys/_types/_ct_rune_t.h
inst_easy.o: /usr/include/sys/_types/_rune_t.h
inst_easy.o: /usr/include/sys/_types/_wchar_t.h
inst_easy.o: /usr/include/sys/_types/_null.h /usr/include/machine/types.h
inst_easy.o: /usr/include/i386/types.h /usr/include/sys/_types/_dev_t.h
inst_easy.o: /usr/include/sys/_types/_mode_t.h /usr/include/stdio.h
inst_easy.o: /usr/include/sys/_types/_va_list.h /usr/include/sys/stdio.h
inst_easy.o: /usr/include/sys/_types/_off_t.h
inst_easy.o: /usr/include/sys/_types/_ssize_t.h /usr/include/secure/_stdio.h
inst_easy.o: /usr/include/secure/_common.h /usr/include/strings.h
inst_easy.o: /usr/include/string.h /usr/include/sys/_types/_rsize_t.h
inst_easy.o: /usr/include/sys/_types/_errno_t.h /usr/include/secure/_string.h
inst_easy.o: /usr/include/ctype.h /usr/include/runetype.h
inst_easy.o: /usr/include/sys/_types/_wint_t.h /usr/include/sys/types.h
inst_easy.o: /usr/include/sys/_types/_blkcnt_t.h
inst_easy.o: /usr/include/sys/_types/_blksize_t.h
inst_easy.o: /usr/include/sys/_types/_gid_t.h
inst_easy.o: /usr/include/sys/_types/_in_addr_t.h
inst_easy.o: /usr/include/sys/_types/_in_port_t.h
inst_easy.o: /usr/include/sys/_types/_ino_t.h
inst_easy.o: /usr/include/sys/_types/_ino64_t.h
inst_easy.o: /usr/include/sys/_types/_key_t.h
inst_easy.o: /usr/include/sys/_types/_nlink_t.h
inst_easy.o: /usr/include/sys/_types/_clock_t.h
inst_easy.o: /usr/include/sys/_types/_time_t.h
inst_easy.o: /usr/include/sys/_types/_useconds_t.h
inst_easy.o: /usr/include/sys/_types/_suseconds_t.h
inst_easy.o: /usr/include/sys/_types/_fd_def.h
inst_easy.o: /usr/include/sys/_types/_fd_setsize.h
inst_easy.o: /usr/include/sys/_types/_fd_set.h
inst_easy.o: /usr/include/sys/_types/_fd_clr.h
inst_easy.o: /usr/include/sys/_types/_fd_zero.h
inst_easy.o: /usr/include/sys/_types/_fd_isset.h
inst_easy.o: /usr/include/sys/_types/_fd_copy.h
inst_easy.o: /usr/include/sys/_pthread/_pthread_cond_t.h
inst_easy.o: /usr/include/sys/_pthread/_pthread_condattr_t.h
inst_easy.o: /usr/include/sys/_pthread/_pthread_mutex_t.h
inst_easy.o: /usr/include/sys/_pthread/_pthread_mutexattr_t.h
inst_easy.o: /usr/include/sys/_pthread/_pthread_once_t.h
inst_easy.o: /usr/include/sys/_pthread/_pthread_rwlock_t.h
inst_easy.o: /usr/include/sys/_pthread/_pthread_rwlockattr_t.h
inst_easy.o: /usr/include/sys/_pthread/_pthread_t.h
inst_easy.o: /usr/include/sys/_pthread/_pthread_key_t.h
inst_easy.o: /usr/include/sys/_types/_fsblkcnt_t.h
inst_easy.o: /usr/include/sys/_types/_fsfilcnt_t.h /usr/include/sys/timeb.h
inst_easy.o: /usr/include/sys/times.h output.h memory.h expressions.h
inst_easy.o: inst_pre.h inst_easy.h
inst_hard.o: ff.h /usr/include/stdlib.h /usr/include/Availability.h
inst_hard.o: /usr/include/AvailabilityInternal.h /usr/include/_types.h
inst_hard.o: /usr/include/sys/_types.h /usr/include/sys/cdefs.h
inst_hard.o: /usr/include/sys/_symbol_aliasing.h
inst_hard.o: /usr/include/sys/_posix_availability.h
inst_hard.o: /usr/include/machine/_types.h /usr/include/i386/_types.h
inst_hard.o: /usr/include/sys/_pthread/_pthread_types.h
inst_hard.o: /usr/include/sys/wait.h /usr/include/sys/_types/_pid_t.h
inst_hard.o: /usr/include/sys/_types/_id_t.h /usr/include/sys/signal.h
inst_hard.o: /usr/include/sys/appleapiopts.h /usr/include/machine/signal.h
inst_hard.o: /usr/include/i386/signal.h /usr/include/machine/_mcontext.h
inst_hard.o: /usr/include/i386/_mcontext.h /usr/include/mach/i386/_structs.h
inst_hard.o: /usr/include/sys/_pthread/_pthread_attr_t.h
inst_hard.o: /usr/include/sys/_types/_sigaltstack.h
inst_hard.o: /usr/include/sys/_types/_ucontext.h
inst_hard.o: /usr/include/sys/_types/_sigset_t.h
inst_hard.o: /usr/include/sys/_types/_size_t.h
inst_hard.o: /usr/include/sys/_types/_uid_t.h /usr/include/sys/resource.h
inst_hard.o: /usr/include/stdint.h /usr/include/sys/_types/_int8_t.h
inst_hard.o: /usr/include/sys/_types/_int16_t.h
inst_hard.o: /usr/include/sys/_types/_int32_t.h
inst_hard.o: /usr/include/sys/_types/_int64_t.h
inst_hard.o: /usr/include/_types/_uint8_t.h /usr/include/_types/_uint16_t.h
inst_hard.o: /usr/include/_types/_uint32_t.h /usr/include/_types/_uint64_t.h
inst_hard.o: /usr/include/sys/_types/_intptr_t.h
inst_hard.o: /usr/include/sys/_types/_uintptr_t.h
inst_hard.o: /usr/include/_types/_intmax_t.h /usr/include/_types/_uintmax_t.h
inst_hard.o: /usr/include/sys/_types/_timeval.h /usr/include/machine/endian.h
inst_hard.o: /usr/include/i386/endian.h /usr/include/sys/_endian.h
inst_hard.o: /usr/include/libkern/_OSByteOrder.h
inst_hard.o: /usr/include/libkern/i386/_OSByteOrder.h /usr/include/alloca.h
inst_hard.o: /usr/include/sys/_types/_ct_rune_t.h
inst_hard.o: /usr/include/sys/_types/_rune_t.h
inst_hard.o: /usr/include/sys/_types/_wchar_t.h
inst_hard.o: /usr/include/sys/_types/_null.h /usr/include/machine/types.h
inst_hard.o: /usr/include/i386/types.h /usr/include/sys/_types/_dev_t.h
inst_hard.o: /usr/include/sys/_types/_mode_t.h /usr/include/stdio.h
inst_hard.o: /usr/include/sys/_types/_va_list.h /usr/include/sys/stdio.h
inst_hard.o: /usr/include/sys/_types/_off_t.h
inst_hard.o: /usr/include/sys/_types/_ssize_t.h /usr/include/secure/_stdio.h
inst_hard.o: /usr/include/secure/_common.h /usr/include/strings.h
inst_hard.o: /usr/include/string.h /usr/include/sys/_types/_rsize_t.h
inst_hard.o: /usr/include/sys/_types/_errno_t.h /usr/include/secure/_string.h
inst_hard.o: /usr/include/ctype.h /usr/include/runetype.h
inst_hard.o: /usr/include/sys/_types/_wint_t.h /usr/include/sys/types.h
inst_hard.o: /usr/include/sys/_types/_blkcnt_t.h
inst_hard.o: /usr/include/sys/_types/_blksize_t.h
inst_hard.o: /usr/include/sys/_types/_gid_t.h
inst_hard.o: /usr/include/sys/_types/_in_addr_t.h
inst_hard.o: /usr/include/sys/_types/_in_port_t.h
inst_hard.o: /usr/include/sys/_types/_ino_t.h
inst_hard.o: /usr/include/sys/_types/_ino64_t.h
inst_hard.o: /usr/include/sys/_types/_key_t.h
inst_hard.o: /usr/include/sys/_types/_nlink_t.h
inst_hard.o: /usr/include/sys/_types/_clock_t.h
inst_hard.o: /usr/include/sys/_types/_time_t.h
inst_hard.o: /usr/include/sys/_types/_useconds_t.h
inst_hard.o: /usr/include/sys/_types/_suseconds_t.h
inst_hard.o: /usr/include/sys/_types/_fd_def.h
inst_hard.o: /usr/include/sys/_types/_fd_setsize.h
inst_hard.o: /usr/include/sys/_types/_fd_set.h
inst_hard.o: /usr/include/sys/_types/_fd_clr.h
inst_hard.o: /usr/include/sys/_types/_fd_zero.h
inst_hard.o: /usr/include/sys/_types/_fd_isset.h
inst_hard.o: /usr/include/sys/_types/_fd_copy.h
inst_hard.o: /usr/include/sys/_pthread/_pthread_cond_t.h
inst_hard.o: /usr/include/sys/_pthread/_pthread_condattr_t.h
inst_hard.o: /usr/include/sys/_pthread/_pthread_mutex_t.h
inst_hard.o: /usr/include/sys/_pthread/_pthread_mutexattr_t.h
inst_hard.o: /usr/include/sys/_pthread/_pthread_once_t.h
inst_hard.o: /usr/include/sys/_pthread/_pthread_rwlock_t.h
inst_hard.o: /usr/include/sys/_pthread/_pthread_rwlockattr_t.h
inst_hard.o: /usr/include/sys/_pthread/_pthread_t.h
inst_hard.o: /usr/include/sys/_pthread/_pthread_key_t.h
inst_hard.o: /usr/include/sys/_types/_fsblkcnt_t.h
inst_hard.o: /usr/include/sys/_types/_fsfilcnt_t.h /usr/include/sys/timeb.h
inst_hard.o: /usr/include/sys/times.h output.h memory.h expressions.h
inst_hard.o: inst_pre.h inst_hard.h
inst_final.o: ff.h /usr/include/stdlib.h /usr/include/Availability.h
inst_final.o: /usr/include/AvailabilityInternal.h /usr/include/_types.h
inst_final.o: /usr/include/sys/_types.h /usr/include/sys/cdefs.h
inst_final.o: /usr/include/sys/_symbol_aliasing.h
inst_final.o: /usr/include/sys/_posix_availability.h
inst_final.o: /usr/include/machine/_types.h /usr/include/i386/_types.h
inst_final.o: /usr/include/sys/_pthread/_pthread_types.h
inst_final.o: /usr/include/sys/wait.h /usr/include/sys/_types/_pid_t.h
inst_final.o: /usr/include/sys/_types/_id_t.h /usr/include/sys/signal.h
inst_final.o: /usr/include/sys/appleapiopts.h /usr/include/machine/signal.h
inst_final.o: /usr/include/i386/signal.h /usr/include/machine/_mcontext.h
inst_final.o: /usr/include/i386/_mcontext.h /usr/include/mach/i386/_structs.h
inst_final.o: /usr/include/sys/_pthread/_pthread_attr_t.h
inst_final.o: /usr/include/sys/_types/_sigaltstack.h
inst_final.o: /usr/include/sys/_types/_ucontext.h
inst_final.o: /usr/include/sys/_types/_sigset_t.h
inst_final.o: /usr/include/sys/_types/_size_t.h
inst_final.o: /usr/include/sys/_types/_uid_t.h /usr/include/sys/resource.h
inst_final.o: /usr/include/stdint.h /usr/include/sys/_types/_int8_t.h
inst_final.o: /usr/include/sys/_types/_int16_t.h
inst_final.o: /usr/include/sys/_types/_int32_t.h
inst_final.o: /usr/include/sys/_types/_int64_t.h
inst_final.o: /usr/include/_types/_uint8_t.h /usr/include/_types/_uint16_t.h
inst_final.o: /usr/include/_types/_uint32_t.h /usr/include/_types/_uint64_t.h
inst_final.o: /usr/include/sys/_types/_intptr_t.h
inst_final.o: /usr/include/sys/_types/_uintptr_t.h
inst_final.o: /usr/include/_types/_intmax_t.h
inst_final.o: /usr/include/_types/_uintmax_t.h
inst_final.o: /usr/include/sys/_types/_timeval.h
inst_final.o: /usr/include/machine/endian.h /usr/include/i386/endian.h
inst_final.o: /usr/include/sys/_endian.h /usr/include/libkern/_OSByteOrder.h
inst_final.o: /usr/include/libkern/i386/_OSByteOrder.h /usr/include/alloca.h
inst_final.o: /usr/include/sys/_types/_ct_rune_t.h
inst_final.o: /usr/include/sys/_types/_rune_t.h
inst_final.o: /usr/include/sys/_types/_wchar_t.h
inst_final.o: /usr/include/sys/_types/_null.h /usr/include/machine/types.h
inst_final.o: /usr/include/i386/types.h /usr/include/sys/_types/_dev_t.h
inst_final.o: /usr/include/sys/_types/_mode_t.h /usr/include/stdio.h
inst_final.o: /usr/include/sys/_types/_va_list.h /usr/include/sys/stdio.h
inst_final.o: /usr/include/sys/_types/_off_t.h
inst_final.o: /usr/include/sys/_types/_ssize_t.h /usr/include/secure/_stdio.h
inst_final.o: /usr/include/secure/_common.h /usr/include/strings.h
inst_final.o: /usr/include/string.h /usr/include/sys/_types/_rsize_t.h
inst_final.o: /usr/include/sys/_types/_errno_t.h
inst_final.o: /usr/include/secure/_string.h /usr/include/ctype.h
inst_final.o: /usr/include/runetype.h /usr/include/sys/_types/_wint_t.h
inst_final.o: /usr/include/sys/types.h /usr/include/sys/_types/_blkcnt_t.h
inst_final.o: /usr/include/sys/_types/_blksize_t.h
inst_final.o: /usr/include/sys/_types/_gid_t.h
inst_final.o: /usr/include/sys/_types/_in_addr_t.h
inst_final.o: /usr/include/sys/_types/_in_port_t.h
inst_final.o: /usr/include/sys/_types/_ino_t.h
inst_final.o: /usr/include/sys/_types/_ino64_t.h
inst_final.o: /usr/include/sys/_types/_key_t.h
inst_final.o: /usr/include/sys/_types/_nlink_t.h
inst_final.o: /usr/include/sys/_types/_clock_t.h
inst_final.o: /usr/include/sys/_types/_time_t.h
inst_final.o: /usr/include/sys/_types/_useconds_t.h
inst_final.o: /usr/include/sys/_types/_suseconds_t.h
inst_final.o: /usr/include/sys/_types/_fd_def.h
inst_final.o: /usr/include/sys/_types/_fd_setsize.h
inst_final.o: /usr/include/sys/_types/_fd_set.h
inst_final.o: /usr/include/sys/_types/_fd_clr.h
inst_final.o: /usr/include/sys/_types/_fd_zero.h
inst_final.o: /usr/include/sys/_types/_fd_isset.h
inst_final.o: /usr/include/sys/_types/_fd_copy.h
inst_final.o: /usr/include/sys/_pthread/_pthread_cond_t.h
inst_final.o: /usr/include/sys/_pthread/_pthread_condattr_t.h
inst_final.o: /usr/include/sys/_pthread/_pthread_mutex_t.h
inst_final.o: /usr/include/sys/_pthread/_pthread_mutexattr_t.h
inst_final.o: /usr/include/sys/_pthread/_pthread_once_t.h
inst_final.o: /usr/include/sys/_pthread/_pthread_rwlock_t.h
inst_final.o: /usr/include/sys/_pthread/_pthread_rwlockattr_t.h
inst_final.o: /usr/include/sys/_pthread/_pthread_t.h
inst_final.o: /usr/include/sys/_pthread/_pthread_key_t.h
inst_final.o: /usr/include/sys/_types/_fsblkcnt_t.h
inst_final.o: /usr/include/sys/_types/_fsfilcnt_t.h /usr/include/sys/timeb.h
inst_final.o: /usr/include/sys/times.h output.h memory.h expressions.h
inst_final.o: inst_pre.h inst_final.h
relax.o: ff.h /usr/include/stdlib.h /usr/include/Availability.h
relax.o: /usr/include/AvailabilityInternal.h /usr/include/_types.h
relax.o: /usr/include/sys/_types.h /usr/include/sys/cdefs.h
relax.o: /usr/include/sys/_symbol_aliasing.h
relax.o: /usr/include/sys/_posix_availability.h /usr/include/machine/_types.h
relax.o: /usr/include/i386/_types.h
relax.o: /usr/include/sys/_pthread/_pthread_types.h /usr/include/sys/wait.h
relax.o: /usr/include/sys/_types/_pid_t.h /usr/include/sys/_types/_id_t.h
relax.o: /usr/include/sys/signal.h /usr/include/sys/appleapiopts.h
relax.o: /usr/include/machine/signal.h /usr/include/i386/signal.h
relax.o: /usr/include/machine/_mcontext.h /usr/include/i386/_mcontext.h
relax.o: /usr/include/mach/i386/_structs.h
relax.o: /usr/include/sys/_pthread/_pthread_attr_t.h
relax.o: /usr/include/sys/_types/_sigaltstack.h
relax.o: /usr/include/sys/_types/_ucontext.h
relax.o: /usr/include/sys/_types/_sigset_t.h
relax.o: /usr/include/sys/_types/_size_t.h /usr/include/sys/_types/_uid_t.h
relax.o: /usr/include/sys/resource.h /usr/include/stdint.h
relax.o: /usr/include/sys/_types/_int8_t.h /usr/include/sys/_types/_int16_t.h
relax.o: /usr/include/sys/_types/_int32_t.h
relax.o: /usr/include/sys/_types/_int64_t.h /usr/include/_types/_uint8_t.h
relax.o: /usr/include/_types/_uint16_t.h /usr/include/_types/_uint32_t.h
relax.o: /usr/include/_types/_uint64_t.h /usr/include/sys/_types/_intptr_t.h
relax.o: /usr/include/sys/_types/_uintptr_t.h /usr/include/_types/_intmax_t.h
relax.o: /usr/include/_types/_uintmax_t.h /usr/include/sys/_types/_timeval.h
relax.o: /usr/include/machine/endian.h /usr/include/i386/endian.h
relax.o: /usr/include/sys/_endian.h /usr/include/libkern/_OSByteOrder.h
relax.o: /usr/include/libkern/i386/_OSByteOrder.h /usr/include/alloca.h
relax.o: /usr/include/sys/_types/_ct_rune_t.h
relax.o: /usr/include/sys/_types/_rune_t.h /usr/include/sys/_types/_wchar_t.h
relax.o: /usr/include/sys/_types/_null.h /usr/include/machine/types.h
relax.o: /usr/include/i386/types.h /usr/include/sys/_types/_dev_t.h
relax.o: /usr/include/sys/_types/_mode_t.h /usr/include/stdio.h
relax.o: /usr/include/sys/_types/_va_list.h /usr/include/sys/stdio.h
relax.o: /usr/include/sys/_types/_off_t.h /usr/include/sys/_types/_ssize_t.h
relax.o: /usr/include/secure/_stdio.h /usr/include/secure/_common.h
relax.o: /usr/include/strings.h /usr/include/string.h
relax.o: /usr/include/sys/_types/_rsize_t.h
relax.o: /usr/include/sys/_types/_errno_t.h /usr/include/secure/_string.h
relax.o: /usr/include/ctype.h /usr/include/runetype.h
relax.o: /usr/include/sys/_types/_wint_t.h /usr/include/sys/types.h
relax.o: /usr/include/sys/_types/_blkcnt_t.h
relax.o: /usr/include/sys/_types/_blksize_t.h
relax.o: /usr/include/sys/_types/_gid_t.h
relax.o: /usr/include/sys/_types/_in_addr_t.h
relax.o: /usr/include/sys/_types/_in_port_t.h
relax.o: /usr/include/sys/_types/_ino_t.h /usr/include/sys/_types/_ino64_t.h
relax.o: /usr/include/sys/_types/_key_t.h /usr/include/sys/_types/_nlink_t.h
relax.o: /usr/include/sys/_types/_clock_t.h /usr/include/sys/_types/_time_t.h
relax.o: /usr/include/sys/_types/_useconds_t.h
relax.o: /usr/include/sys/_types/_suseconds_t.h
relax.o: /usr/include/sys/_types/_fd_def.h
relax.o: /usr/include/sys/_types/_fd_setsize.h
relax.o: /usr/include/sys/_types/_fd_set.h /usr/include/sys/_types/_fd_clr.h
relax.o: /usr/include/sys/_types/_fd_zero.h
relax.o: /usr/include/sys/_types/_fd_isset.h
relax.o: /usr/include/sys/_types/_fd_copy.h
relax.o: /usr/include/sys/_pthread/_pthread_cond_t.h
relax.o: /usr/include/sys/_pthread/_pthread_condattr_t.h
relax.o: /usr/include/sys/_pthread/_pthread_mutex_t.h
relax.o: /usr/include/sys/_pthread/_pthread_mutexattr_t.h
relax.o: /usr/include/sys/_pthread/_pthread_once_t.h
relax.o: /usr/include/sys/_pthread/_pthread_rwlock_t.h
relax.o: /usr/include/sys/_pthread/_pthread_rwlockattr_t.h
relax.o: /usr/include/sys/_pthread/_pthread_t.h
relax.o: /usr/include/sys/_pthread/_pthread_key_t.h
relax.o: /usr/include/sys/_types/_fsblkcnt_t.h
relax.o: /usr/include/sys/_types/_fsfilcnt_t.h /usr/include/sys/timeb.h
relax.o: /usr/include/sys/times.h output.h memory.h expressions.h relax.h
relax.o: search.h
search.o: ff.h /usr/include/stdlib.h /usr/include/Availability.h
search.o: /usr/include/AvailabilityInternal.h /usr/include/_types.h
search.o: /usr/include/sys/_types.h /usr/include/sys/cdefs.h
search.o: /usr/include/sys/_symbol_aliasing.h
search.o: /usr/include/sys/_posix_availability.h
search.o: /usr/include/machine/_types.h /usr/include/i386/_types.h
search.o: /usr/include/sys/_pthread/_pthread_types.h /usr/include/sys/wait.h
search.o: /usr/include/sys/_types/_pid_t.h /usr/include/sys/_types/_id_t.h
search.o: /usr/include/sys/signal.h /usr/include/sys/appleapiopts.h
search.o: /usr/include/machine/signal.h /usr/include/i386/signal.h
search.o: /usr/include/machine/_mcontext.h /usr/include/i386/_mcontext.h
search.o: /usr/include/mach/i386/_structs.h
search.o: /usr/include/sys/_pthread/_pthread_attr_t.h
search.o: /usr/include/sys/_types/_sigaltstack.h
search.o: /usr/include/sys/_types/_ucontext.h
search.o: /usr/include/sys/_types/_sigset_t.h
search.o: /usr/include/sys/_types/_size_t.h /usr/include/sys/_types/_uid_t.h
search.o: /usr/include/sys/resource.h /usr/include/stdint.h
search.o: /usr/include/sys/_types/_int8_t.h
search.o: /usr/include/sys/_types/_int16_t.h
search.o: /usr/include/sys/_types/_int32_t.h
search.o: /usr/include/sys/_types/_int64_t.h /usr/include/_types/_uint8_t.h
search.o: /usr/include/_types/_uint16_t.h /usr/include/_types/_uint32_t.h
search.o: /usr/include/_types/_uint64_t.h /usr/include/sys/_types/_intptr_t.h
search.o: /usr/include/sys/_types/_uintptr_t.h
search.o: /usr/include/_types/_intmax_t.h /usr/include/_types/_uintmax_t.h
search.o: /usr/include/sys/_types/_timeval.h /usr/include/machine/endian.h
search.o: /usr/include/i386/endian.h /usr/include/sys/_endian.h
search.o: /usr/include/libkern/_OSByteOrder.h
search.o: /usr/include/libkern/i386/_OSByteOrder.h /usr/include/alloca.h
search.o: /usr/include/sys/_types/_ct_rune_t.h
search.o: /usr/include/sys/_types/_rune_t.h
search.o: /usr/include/sys/_types/_wchar_t.h /usr/include/sys/_types/_null.h
search.o: /usr/include/machine/types.h /usr/include/i386/types.h
search.o: /usr/include/sys/_types/_dev_t.h /usr/include/sys/_types/_mode_t.h
search.o: /usr/include/stdio.h /usr/include/sys/_types/_va_list.h
search.o: /usr/include/sys/stdio.h /usr/include/sys/_types/_off_t.h
search.o: /usr/include/sys/_types/_ssize_t.h /usr/include/secure/_stdio.h
search.o: /usr/include/secure/_common.h /usr/include/strings.h
search.o: /usr/include/string.h /usr/include/sys/_types/_rsize_t.h
search.o: /usr/include/sys/_types/_errno_t.h /usr/include/secure/_string.h
search.o: /usr/include/ctype.h /usr/include/runetype.h
search.o: /usr/include/sys/_types/_wint_t.h /usr/include/sys/types.h
search.o: /usr/include/sys/_types/_blkcnt_t.h
search.o: /usr/include/sys/_types/_blksize_t.h
search.o: /usr/include/sys/_types/_gid_t.h
search.o: /usr/include/sys/_types/_in_addr_t.h
search.o: /usr/include/sys/_types/_in_port_t.h
search.o: /usr/include/sys/_types/_ino_t.h /usr/include/sys/_types/_ino64_t.h
search.o: /usr/include/sys/_types/_key_t.h /usr/include/sys/_types/_nlink_t.h
search.o: /usr/include/sys/_types/_clock_t.h
search.o: /usr/include/sys/_types/_time_t.h
search.o: /usr/include/sys/_types/_useconds_t.h
search.o: /usr/include/sys/_types/_suseconds_t.h
search.o: /usr/include/sys/_types/_fd_def.h
search.o: /usr/include/sys/_types/_fd_setsize.h
search.o: /usr/include/sys/_types/_fd_set.h /usr/include/sys/_types/_fd_clr.h
search.o: /usr/include/sys/_types/_fd_zero.h
search.o: /usr/include/sys/_types/_fd_isset.h
search.o: /usr/include/sys/_types/_fd_copy.h
search.o: /usr/include/sys/_pthread/_pthread_cond_t.h
search.o: /usr/include/sys/_pthread/_pthread_condattr_t.h
search.o: /usr/include/sys/_pthread/_pthread_mutex_t.h
search.o: /usr/include/sys/_pthread/_pthread_mutexattr_t.h
search.o: /usr/include/sys/_pthread/_pthread_once_t.h
search.o: /usr/include/sys/_pthread/_pthread_rwlock_t.h
search.o: /usr/include/sys/_pthread/_pthread_rwlockattr_t.h
search.o: /usr/include/sys/_pthread/_pthread_t.h
search.o: /usr/include/sys/_pthread/_pthread_key_t.h
search.o: /usr/include/sys/_types/_fsblkcnt_t.h
search.o: /usr/include/sys/_types/_fsfilcnt_t.h /usr/include/sys/timeb.h
search.o: /usr/include/sys/times.h output.h memory.h expressions.h relax.h
search.o: search.h
utility.o: /usr/include/string.h /usr/include/_types.h
utility.o: /usr/include/sys/_types.h /usr/include/sys/cdefs.h
utility.o: /usr/include/sys/_symbol_aliasing.h
utility.o: /usr/include/sys/_posix_availability.h
utility.o: /usr/include/machine/_types.h /usr/include/i386/_types.h
utility.o: /usr/include/sys/_pthread/_pthread_types.h
utility.o: /usr/include/Availability.h /usr/include/AvailabilityInternal.h
utility.o: /usr/include/sys/_types/_size_t.h /usr/include/sys/_types/_null.h
utility.o: /usr/include/sys/_types/_rsize_t.h
utility.o: /usr/include/sys/_types/_errno_t.h
utility.o: /usr/include/sys/_types/_ssize_t.h /usr/include/strings.h
utility.o: /usr/include/secure/_string.h /usr/include/secure/_common.h
utility.o: utility.h ff.h /usr/include/stdlib.h /usr/include/sys/wait.h
utility.o: /usr/include/sys/_types/_pid_t.h /usr/include/sys/_types/_id_t.h
utility.o: /usr/include/sys/signal.h /usr/include/sys/appleapiopts.h
utility.o: /usr/include/machine/signal.h /usr/include/i386/signal.h
utility.o: /usr/include/machine/_mcontext.h /usr/include/i386/_mcontext.h
utility.o: /usr/include/mach/i386/_structs.h
utility.o: /usr/include/sys/_pthread/_pthread_attr_t.h
utility.o: /usr/include/sys/_types/_sigaltstack.h
utility.o: /usr/include/sys/_types/_ucontext.h
utility.o: /usr/include/sys/_types/_sigset_t.h
utility.o: /usr/include/sys/_types/_uid_t.h /usr/include/sys/resource.h
utility.o: /usr/include/stdint.h /usr/include/sys/_types/_int8_t.h
utility.o: /usr/include/sys/_types/_int16_t.h
utility.o: /usr/include/sys/_types/_int32_t.h
utility.o: /usr/include/sys/_types/_int64_t.h /usr/include/_types/_uint8_t.h
utility.o: /usr/include/_types/_uint16_t.h /usr/include/_types/_uint32_t.h
utility.o: /usr/include/_types/_uint64_t.h
utility.o: /usr/include/sys/_types/_intptr_t.h
utility.o: /usr/include/sys/_types/_uintptr_t.h
utility.o: /usr/include/_types/_intmax_t.h /usr/include/_types/_uintmax_t.h
utility.o: /usr/include/sys/_types/_timeval.h /usr/include/machine/endian.h
utility.o: /usr/include/i386/endian.h /usr/include/sys/_endian.h
utility.o: /usr/include/libkern/_OSByteOrder.h
utility.o: /usr/include/libkern/i386/_OSByteOrder.h /usr/include/alloca.h
utility.o: /usr/include/sys/_types/_ct_rune_t.h
utility.o: /usr/include/sys/_types/_rune_t.h
utility.o: /usr/include/sys/_types/_wchar_t.h /usr/include/machine/types.h
utility.o: /usr/include/i386/types.h /usr/include/sys/_types/_dev_t.h
utility.o: /usr/include/sys/_types/_mode_t.h /usr/include/stdio.h
utility.o: /usr/include/sys/_types/_va_list.h /usr/include/sys/stdio.h
utility.o: /usr/include/sys/_types/_off_t.h /usr/include/secure/_stdio.h
utility.o: /usr/include/ctype.h /usr/include/runetype.h
utility.o: /usr/include/sys/_types/_wint_t.h /usr/include/sys/types.h
utility.o: /usr/include/sys/_types/_blkcnt_t.h
utility.o: /usr/include/sys/_types/_blksize_t.h
utility.o: /usr/include/sys/_types/_gid_t.h
utility.o: /usr/include/sys/_types/_in_addr_t.h
utility.o: /usr/include/sys/_types/_in_port_t.h
utility.o: /usr/include/sys/_types/_ino_t.h
utility.o: /usr/include/sys/_types/_ino64_t.h
utility.o: /usr/include/sys/_types/_key_t.h
utility.o: /usr/include/sys/_types/_nlink_t.h
utility.o: /usr/include/sys/_types/_clock_t.h
utility.o: /usr/include/sys/_types/_time_t.h
utility.o: /usr/include/sys/_types/_useconds_t.h
utility.o: /usr/include/sys/_types/_suseconds_t.h
utility.o: /usr/include/sys/_types/_fd_def.h
utility.o: /usr/include/sys/_types/_fd_setsize.h
utility.o: /usr/include/sys/_types/_fd_set.h
utility.o: /usr/include/sys/_types/_fd_clr.h
utility.o: /usr/include/sys/_types/_fd_zero.h
utility.o: /usr/include/sys/_types/_fd_isset.h
utility.o: /usr/include/sys/_types/_fd_copy.h
utility.o: /usr/include/sys/_pthread/_pthread_cond_t.h
utility.o: /usr/include/sys/_pthread/_pthread_condattr_t.h
utility.o: /usr/include/sys/_pthread/_pthread_mutex_t.h
utility.o: /usr/include/sys/_pthread/_pthread_mutexattr_t.h
utility.o: /usr/include/sys/_pthread/_pthread_once_t.h
utility.o: /usr/include/sys/_pthread/_pthread_rwlock_t.h
utility.o: /usr/include/sys/_pthread/_pthread_rwlockattr_t.h
utility.o: /usr/include/sys/_pthread/_pthread_t.h
utility.o: /usr/include/sys/_pthread/_pthread_key_t.h
utility.o: /usr/include/sys/_types/_fsblkcnt_t.h
utility.o: /usr/include/sys/_types/_fsfilcnt_t.h /usr/include/sys/timeb.h
utility.o: /usr/include/sys/times.h output.h
test_main.o: libff.h ff.h /usr/include/stdlib.h /usr/include/Availability.h
test_main.o: /usr/include/AvailabilityInternal.h /usr/include/_types.h
test_main.o: /usr/include/sys/_types.h /usr/include/sys/cdefs.h
test_main.o: /usr/include/sys/_symbol_aliasing.h
test_main.o: /usr/include/sys/_posix_availability.h
test_main.o: /usr/include/machine/_types.h /usr/include/i386/_types.h
test_main.o: /usr/include/sys/_pthread/_pthread_types.h
test_main.o: /usr/include/sys/wait.h /usr/include/sys/_types/_pid_t.h
test_main.o: /usr/include/sys/_types/_id_t.h /usr/include/sys/signal.h
test_main.o: /usr/include/sys/appleapiopts.h /usr/include/machine/signal.h
test_main.o: /usr/include/i386/signal.h /usr/include/machine/_mcontext.h
test_main.o: /usr/include/i386/_mcontext.h /usr/include/mach/i386/_structs.h
test_main.o: /usr/include/sys/_pthread/_pthread_attr_t.h
test_main.o: /usr/include/sys/_types/_sigaltstack.h
test_main.o: /usr/include/sys/_types/_ucontext.h
test_main.o: /usr/include/sys/_types/_sigset_t.h
test_main.o: /usr/include/sys/_types/_size_t.h
test_main.o: /usr/include/sys/_types/_uid_t.h /usr/include/sys/resource.h
test_main.o: /usr/include/stdint.h /usr/include/sys/_types/_int8_t.h
test_main.o: /usr/include/sys/_types/_int16_t.h
test_main.o: /usr/include/sys/_types/_int32_t.h
test_main.o: /usr/include/sys/_types/_int64_t.h
test_main.o: /usr/include/_types/_uint8_t.h /usr/include/_types/_uint16_t.h
test_main.o: /usr/include/_types/_uint32_t.h /usr/include/_types/_uint64_t.h
test_main.o: /usr/include/sys/_types/_intptr_t.h
test_main.o: /usr/include/sys/_types/_uintptr_t.h
test_main.o: /usr/include/_types/_intmax_t.h /usr/include/_types/_uintmax_t.h
test_main.o: /usr/include/sys/_types/_timeval.h /usr/include/machine/endian.h
test_main.o: /usr/include/i386/endian.h /usr/include/sys/_endian.h
test_main.o: /usr/include/libkern/_OSByteOrder.h
test_main.o: /usr/include/libkern/i386/_OSByteOrder.h /usr/include/alloca.h
test_main.o: /usr/include/sys/_types/_ct_rune_t.h
test_main.o: /usr/include/sys/_types/_rune_t.h
test_main.o: /usr/include/sys/_types/_wchar_t.h
test_main.o: /usr/include/sys/_types/_null.h /usr/include/machine/types.h
test_main.o: /usr/include/i386/types.h /usr/include/sys/_types/_dev_t.h
test_main.o: /usr/include/sys/_types/_mode_t.h /usr/include/stdio.h
test_main.o: /usr/include/sys/_types/_va_list.h /usr/include/sys/stdio.h
test_main.o: /usr/include/sys/_types/_off_t.h
test_main.o: /usr/include/sys/_types/_ssize_t.h /usr/include/secure/_stdio.h
test_main.o: /usr/include/secure/_common.h /usr/include/strings.h
test_main.o: /usr/include/string.h /usr/include/sys/_types/_rsize_t.h
test_main.o: /usr/include/sys/_types/_errno_t.h /usr/include/secure/_string.h
test_main.o: /usr/include/ctype.h /usr/include/runetype.h
test_main.o: /usr/include/sys/_types/_wint_t.h /usr/include/sys/types.h
test_main.o: /usr/include/sys/_types/_blkcnt_t.h
test_main.o: /usr/include/sys/_types/_blksize_t.h
test_main.o: /usr/include/sys/_types/_gid_t.h
test_main.o: /usr/include/sys/_types/_in_addr_t.h
test_main.o: /usr/include/sys/_types/_in_port_t.h
test_main.o: /usr/include/sys/_types/_ino_t.h
test_main.o: /usr/include/sys/_types/_ino64_t.h
test_main.o: /usr/include/sys/_types/_key_t.h
test_main.o: /usr/include/sys/_types/_nlink_t.h
test_main.o: /usr/include/sys/_types/_clock_t.h
test_main.o: /usr/include/sys/_types/_time_t.h
test_main.o: /usr/include/sys/_types/_useconds_t.h
test_main.o: /usr/include/sys/_types/_suseconds_t.h
test_main.o: /usr/include/sys/_types/_fd_def.h
test_main.o: /usr/include/sys/_types/_fd_setsize.h
test_main.o: /usr/include/sys/_types/_fd_set.h
test_main.o: /usr/include/sys/_types/_fd_clr.h
test_main.o: /usr/include/sys/_types/_fd_zero.h
test_main.o: /usr/include/sys/_types/_fd_isset.h
test_main.o: /usr/include/sys/_types/_fd_copy.h
test_main.o: /usr/include/sys/_pthread/_pthread_cond_t.h
test_main.o: /usr/include/sys/_pthread/_pthread_condattr_t.h
test_main.o: /usr/include/sys/_pthread/_pthread_mutex_t.h
test_main.o: /usr/include/sys/_pthread/_pthread_mutexattr_t.h
test_main.o: /usr/include/sys/_pthread/_pthread_once_t.h
test_main.o: /usr/include/sys/_pthread/_pthread_rwlock_t.h
test_main.o: /usr/include/sys/_pthread/_pthread_rwlockattr_t.h
test_main.o: /usr/include/sys/_pthread/_pthread_t.h
test_main.o: /usr/include/sys/_pthread/_pthread_key_t.h
test_main.o: /usr/include/sys/_types/_fsblkcnt_t.h
test_main.o: /usr/include/sys/_types/_fsfilcnt_t.h /usr/include/sys/timeb.h
test_main.o: /usr/include/sys/times.h output.h
