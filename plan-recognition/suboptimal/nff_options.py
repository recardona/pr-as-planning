import getopt, os, sys

def usage() :
	print >> sys.stderr, "Parameters:"
	print >> sys.stderr, "-d  --domain <file>              Planning Domain"
	print >> sys.stderr, "-i  --instance <file>            Planning Instance"
	print >> sys.stderr, "-Z  --zipped-problem <file>      Zipped Planning domain and instance"
	print >> sys.stderr, "-h  --help                       Get Help"
	print >> sys.stderr, "-t  --max-time <time>            Maximum allowed execution time (defaults to 1800 secs)"
	print >> sys.stderr, "-m  --max-memory <time>          Maximum allowed memory consumption (defaults to 1Gb)"
	print >> sys.stderr, "-1                               Use h^1 instead of h^2"
	print >> sys.stderr, "-2                               Use h_CL instead of h^2"
	print >> sys.stderr, "-P                               Compute Persists-With(p) for each layer in the planning graph"
	print >> sys.stderr, "-B                               Perform Branch & Bound search"
	print >> sys.stderr, "-C                               Constrain h^1 with active causal links"
	print >> sys.stderr, "-J                               Use Joint Persistency in the Label propagation"
	print >> sys.stderr, "-K                               Use causal link ranking based on Keep(p) potential causal links disturbed"
	print >> sys.stderr
	print >> sys.stderr, "Branching options:"
	print >> sys.stderr, "-a  --branch-C1                  Branch on C^1(s)"
	print >> sys.stderr, "-b  --branch-C2                  Branch on C^2(s)"
	print >> sys.stderr, "-c  --branch-C1-R                Branch on C^1(s) Reachable"
	print >> sys.stderr, "-e  --branch-C2-R                Branch on C^2(s) Reachable"

class Program_Options :

	def __init__( self, args ) :
		try:
			opts, args = getopt.getopt(	args,
							"d:i:ht:m:12PBCabceJKZ:",
							["domain=",
							"instance=",
							"help",
							"max-time=",
							"max-memory=",
							"use-h1",
							"use-hcl",
							"compute-pw-at-each-layer",
							"branch-and-bound",
							"constrain-h1",
							"branch-C1",
							"branch-C1-R",
							"branch-C2",
							"joint-persistency",
							"rank-by-keeps",
							"zipped-problem="] )
		except getopt.GetoptError :
			print >> sys.stderr, "Missing or incorrect parameters specified!"
			usage()
			sys.exit(1)

		self.domain = None
		self.instance = None
		self.max_time = 1800
		self.max_memory = 1024
		self.use_h1 = False
		self.use_hcl = False
		self.pw_each_layer = False
		self.do_bnb = False
		self.constrain_h1 = False
		self.joint_persistency = False
		self.keep_based_ranking = False
		self.branch_opt = 0
		self.reachable = False
		self.zipped_problem = None

		for opcode, oparg in opts :
			if opcode in ( '-h', '--help' ) :
				print >> sys.stderr, "Help invoked!"
				usage()
				sys.exit(0)
			if opcode in ('-d', '--domain' ) :
				self.domain = oparg
				if not os.path.exists( self.domain ) :
					print >> sys.stderr, "File", self.domain, "does not exist"
					print >> sys.stderr, "Aborting"
					sys.exit(1)

			if opcode in ('-i', '--instance' ) :
				self.instance = oparg
				if not os.path.exists( self.instance ) :
					print >> sys.stderr, "File", self.instance, "does not exist"
					print >> sys.stderr, "Aborting"
					sys.exit(1)
			if opcode in ('-Z', '--zipped-file' ) :
				self.zipped_problem = oparg
				if not os.path.exists( self.zipped_problem ) or not '.zip' in self.zipped_problem :
					print >> sys.stderr, "File", self.zipped_problem, "does not exist or hasn't zip extension"
					print >> sys.stderr, "Aborting"
					sys.exit(1)


			if opcode in ('-t', '--max-time' ) :
				try :
					self.max_time = int(oparg)
					if self.max_time <= 0 :
						print >> sys.stderr, "Maximum time must be greater than zero"
						sys.exit(1)
				except ValueError :
					print >> sys.stderr, "Time must be an integer"
					sys.exit(1)
			if opcode in ('-m', '--max-memory' ) :
				try :
					self.max_memory = int(oparg)
					if self.max_memory <= 0 :
						print >> sys.stderr, "Maximum memory must be greater than zero"
						sys.exit(1)
				except ValueError :
					print >> sys.stderr, "Memory amount must be an integer"
					sys.exit(1)
			if opcode in ('-1', '--use-h1' ) :
				self.use_h1 = True
			if opcode in ('-2', '--use-hcl' ) :
				self.use_hcl = True
			if opcode in ('-P', '--compute-pw-at-each-layer' ) :
				self.pw_each_layer = True
			if opcode in ('-B', '--branch-and-bound' ) :
				self.do_bnb = True
			if opcode in ('-C', '--constrain-h1' ) :
				self.constrain_h1 = True
			if opcode in ('-J', '--joint-persistency' ) :
				self.joint_persistency = True
			if opcode in ('-K', '--rank-by-keeps' ) :
				self.keep_based_ranking = True
			if opcode in ('-a', '--branch-C1' ) :
				self.branch_opt = 0
			if opcode in ('-b', '--branch-C2' ) :
				self.branch_opt = 1
			if opcode in ('-c', '--branch-C1-R' ) :
				self.reachable = True
			if opcode in ('-e', '--branch-C2-R' ) :
				self.reachable = True				
				self.branch_opt = 1
		if self.zipped_problem is not None :
			os.system( 'unzip -o %s'%self.zipped_problem )
			if not os.path.exists( 'MANIFEST' ) :
				print >> sys.stderr, "No MANIFEST file found in local directory"
				usage()
				sys.exit(1)
			manifest_fo = open( 'MANIFEST' )
			for line in manifest_fo :
				line = line.strip()
				name, value = line.split('=')
				if name == 'domain_file' :
					if not os.path.exists( value ) :
						print >> sys.stderr, "Something wrong found in MANIFEST: domain_file %s not found in local directory"%value
						usage()
						sys.exit(1)
					self.domain = value
				if name == 'instance_file' :
					if not os.path.exists( value ) :
						print >> sys.stderr, "Something wrong found in MANIFEST: instance_file %s not found in local directory"%value
						usage()
						sys.exit(1)
					self.instance = value
								
		if self.instance is None :
			print >> sys.stderr, "You need to specify an experiment descriptor as input"
			usage()
			sys.exit(1)

	def print_options( self ) :
		def print_yes() : print >> sys.stdout, "Yes"
		def print_no() : print >> sys.stdout, "No"
		
		print >> sys.stdout, "Options set"
		print >> sys.stdout, "==========="
		print >> sys.stdout, "Domain File:", self.domain
		print >> sys.stdout, "Instance File:", self.instance
		print >> sys.stdout, "Max. Time Allowed", self.max_time
		print >> sys.stdout, "Max. Memory Allowed", self.max_memory
