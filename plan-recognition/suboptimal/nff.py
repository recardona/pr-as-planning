#!/usr/bin/python
import sys, os, tarfile, glob
import nff_options
import benchmark

def to_human( signal ) :
	if signal == 0 :
		return "OK"
	else :
		return "Failed"	


def main() :
	opt = nff_options.Program_Options( sys.argv[1:] )
	opt.print_options()

	dom_name_clean = os.path.basename(opt.domain).replace( '.pddl', '' )
	inst_name_clean = os.path.basename(opt.instance).replace( '.pddl', '' )

	name = '%s_%s'%(dom_name_clean, inst_name_clean)

	log = benchmark.Log( '%s.log'%name )
	exec_name = 'c3'

	
	cmd = './%s -d %s -i %s'%(exec_name, opt.domain, opt.instance )
	#if opt.use_h1 : cmd += ' -1'
	#if opt.use_hcl : cmd += ' -2'
	#if opt.pw_each_layer : cmd += ' -P'
	#if opt.do_bnb : cmd += ' -B'
	#if opt.constrain_h1 : cmd += ' -C'
	#if opt.joint_persistency : cmd += ' -J'
	#if opt.keep_based_ranking : cmd += ' -K'
	#if opt.reachable : cmd += ' -R'

	#cmd += ' -b %d'%opt.branch_opt
	
	signal, time = benchmark.run( cmd, opt.max_time, opt.max_memory, log )

	res_info = [ dom_name_clean, inst_name_clean, str(signal), str(time) ]

	if os.path.exists( 'execution.stats' ) :
		instream = open( 'execution.stats' )
		for line in instream :
			line = line.strip()
			toks = line.split('=')
			res_info.append( toks[1] )
		instream.close()

	outstream = open( '%s.result'%name, 'w' )
	
	print >> outstream, ",".join(res_info)
	
	outstream.close()

if __name__ == '__main__' :
	main()
