#!/usr/bin/python
import sys, os, csv
from options import Program_Options
import benchmark

def custom_partition( s, sep ) :
	i = 0
	while i < len(s) :
		if s[i] == sep : break
		i = i + 1
	if i == len(s) : return (None,None,None)
	if i == 0 : return ( None, s[i], s[i+1:] )
	return ( s[:i-1], s[i], s[i+1:] )

class PR_Command :

	def __init__( self, domain, problem, simple=True, bfs=False, max_time = 14400, max_mem = 2048 ) :
		self.domain = domain
		self.problem = problem
		self.simple = simple
		self.bfs = bfs
		self.noext_problem = os.path.basename(self.problem).replace( '.pddl', '' )
		self.max_time = max_time
		self.max_mem = max_mem
		self.num_accounted_obs = 'n/a' 

	def execute( self ) :
		if self.simple :
			cmd_string = './subopt_PR -d %s -i %s -I'%( self.domain, self.problem)
		else :
			if self.bfs :
				cmd_string = './subopt_PR -d %s -i %s -B'%( self.domain, self.problem)
			else :
				cmd_string = './subopt_PR -d %s -i %s'%( self.domain, self.problem)
		self.log = benchmark.Log( '%s.log'%self.noext_problem )
		self.signal, self.time = benchmark.run( cmd_string, self.max_time, self.max_mem, self.log )
		self.gather_data()

	def gather_data( self ) :
		if self.signal == 0 and os.path.exists( 'hyp-rank' ) :
			instream = open( 'hyp-rank' )
			for line in instream :
				line = line.strip()
				if not '--' in line:
					self.num_obs_accounted = int(line)
			instream.close()

	def write_result( self, filename ) :
		res = csv.writer(  open( '%s'%filename, 'w' ) )
		res.writerow( [ os.path.basename(self.domain), os.path.basename(self.problem), self.signal, self.time, self.num_accounted_obs ] )


class Translate_Command :
	
	def __init__( self, domain, problem, obs, max_time = 1800, max_mem = 2048 ) :
		self.domain = domain
		self.problem = problem
		self.obs_stream = obs
		self.max_time = max_time
		self.max_mem = max_mem

	def execute( self ) :
		cmd_string = './pr2plan -d %s -i %s -o %s'%(self.domain, self.problem, self.obs_stream)
		self.log = benchmark.Log( '%s_%s_%s_transcription.log'%(self.domain, self.problem, self.obs_stream) )
		self.signal, self.time = benchmark.run( cmd_string, self.max_time, self.max_mem, self.log )

class Hypothesis :
	
	def __init__( self ) :
		self.atoms = []
		self.Delta = 0.0
		self.plan = []
		self.test_failed = False
		self.trans_time = 0.0
		self.plan_time = 0.0
		self.total_time = 0.0
		self.is_true = True

	def test( self, index, do_simple, do_bfs ) :
		# generate the problem with G=H
		hyp_problem = 'hyp_%d_problem.pddl'%index
		self.generate_pddl_for_hyp_plan( hyp_problem )
		# derive problem with G_Obs
		trans_cmd = Translate_Command( 'domain.pddl', hyp_problem, 'obs.dat' )
		trans_cmd.execute()
		os.system( 'mv -f pr-domain.pddl pr-domain-hyp-%d.pddl'%index )
		os.system( 'mv -f pr-problem.pddl pr-problem-hyp-%d.pddl'%index )
		plan_for_H_cmd = PR_Command( 'pr-domain-hyp-%d.pddl'%index, 'pr-problem-hyp-%d.pddl'%index, do_simple, do_bfs )
		plan_for_H_cmd.execute()
		self.trans_time = trans_cmd.time
		self.plan_time = plan_for_H_cmd.time
		self.total_time = trans_cmd.time + plan_for_H_cmd.time 
		plan_for_H_cmd.write_result( 'hyp_%d_planning_H.csv'%index)
		if plan_for_H_cmd.signal == 0  :
			self.score = float( plan_for_H_cmd.num_obs_accounted)
			self.load_plan( 'pr-problem-hyp-%d.soln'%index )
		else :
			self.test_failed = True

	def load_plan( self, plan_name ) :
		instream = open( plan_name )
		self.plan = []
		for line in instream :
			line = line.strip()
			if line[0] == ';' : continue
			#_, _, stuff = line.partition(':')
			#op, _, _ = stuff.partition('[')
			_, _, stuff = custom_partition( line, ':' )
			op, _, _ = custom_partition( stuff, '[' )
			self.plan.append( op.strip().upper() )	
		instream.close()


	def generate_pddl_for_hyp_plan( self, out_name ) :
		instream = open( 'template.pddl' )
		outstream = open( out_name, 'w' )

		for line in instream :
			line = line.strip()
			if '<HYPOTHESIS>' not in line :
				print >> outstream, line
			else :
				for atom in self.atoms :
					print >> outstream, atom
		
		outstream.close()
		instream.close()

	def check_if_actual( self ) :
		real_hyp_atoms = []
		instream = open( 'real_hyp.dat' )
		for line in instream :
			real_hyp_atoms = [ tok.strip() for tok in line.split(',') ]
		instream.close()

		for atom in real_hyp_atoms :
			if not atom in self.atoms :
				self.is_true = False
				break

def load_hypotheses() :

	hyps = []
	
	instream = open( 'hyps.dat' )
	
	for line in instream :
		line = line.strip()
		H = Hypothesis()
		H.atoms = [  tok.strip() for tok in line.split(',') ]
		H.check_if_actual()
		hyps.append( H )

	instream.close()

	return hyps
	
def write_report( experiment, hyps ) :
	
	outstream = open( 'report.txt', 'w' )
	
	print >> outstream, "Experiment=%s"%experiment
	print >> outstream, "Num_Hyp=%d"%len(hyps)
	for hyp in hyps :
		print >> outstream, "Hyp_Atoms=%s"%",".join( hyp.atoms )
		if hyp.test_failed :
			print >> outstream, "Hyp_Score=unknown"
			print >> outstream, "Hyp_Plan_Len=unknown"
		else :
			print >> outstream, "Hyp_Score=%f"%hyp.score
			print >> outstream, "Hyp_Plan_Len=%d"%len(hyp.plan)
		print >> outstream, "Hyp_Trans_Time=%f"%hyp.trans_time
		print >> outstream, "Hyp_Plan_Time=%f"%hyp.plan_time
		print >> outstream, "Hyp_Test_Time=%f"%hyp.total_time
		print >> outstream, "Hyp_Is_True=%s"%hyp.is_true

	outstream.close()

def main() :
	print sys.argv
	options = Program_Options( sys.argv[1:] )

	hyps = load_hypotheses()
	
	for i in range( 0, len(hyps) ) :
		hyps[i].test(i, options.simple_pr, options.bfs)

	write_report(options.exp_file, hyps)
	# pack logs, csvs and report.txt
	cmd = 'tar jcvf results.tar.bz2 *.log *.csv *.pddl *.soln report.txt'
	os.system( cmd )	

if __name__ == '__main__' :
	main()
