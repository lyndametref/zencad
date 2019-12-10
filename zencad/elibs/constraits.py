import pyservoce
from zencad.libs.screw import screw

class constrait:
	def __init__(self):
		self.connections = []

	def constrait_screws(self):
		raise NotImplementedError
		
	def attach_positive_connection(self, body, radius):
		c = constrait_connection_positive(self, body, radius)
		self.connections.append(c)

	def attach_negative_connection(self, body, radius):
		c = constrait_connection_negative(self, body, radius)
		self.connections.append(c)

	def update_globals(self):
		for c in self.connections:
			c.update_globals()

class rotator_constrait(constrait):
	def __init__(self, ax):
		super().__init__()
		self.axis = pyservoce.vector3(ax)
		self.__dbg = 0

	def rank(self):
		return 5

	def constrait_screws(self):
		ang = self.axis

		if ang.early(pyservoce.vector3(1,0,0), 0.00001):
			r = pyservoce.vector3(0,1,0)
		else:
			r = pyservoce.vector3(1,0,0)

		f = ang.cross(r).normalize()
		s = ang.cross(f)

		scrs = [
			screw(lin=(1,0,0), ang=(0,0,0)),
			screw(lin=(0,1,0), ang=(0,0,0)),
			screw(lin=(0,0,1), ang=(0,0,0)),
			screw(lin=(0,0,0), ang=f),
			screw(lin=(0,0,0), ang=s),
		]

		return scrs

class constrait_connection:
	def __init__(self, constrait, body, radius):
		if not isinstance(radius, pyservoce.vector3):
			raise Exception("not vector")

		self.constrait = constrait
		self.body = body
		self.radius = radius

	def get_reaction_force_global(self):
		reactions = self.constrait.reactions
		sreactions = [ (self.body_carried_constrait_screws()[i] * reactions[i][0]) for i in range(self.rank()) ]

		return sreactions

	def body_carried_constrait_screws(self):
		scrs = self.constrait_screws()
		arm = self.body.global_pose(self.radius)

		tscrs = [ s.force_carry(-arm) for s in scrs ]

		return tscrs

	def update_globals(self):
		pass

	def rank(self):
		return self.constrait.rank()

class constrait_connection_positive(constrait_connection):
	def __init__(self, constrait, body, radius):
		super().__init__(constrait, body, radius)	

	def constrait_screws(self):
		return self.constrait.constrait_screws()

class constrait_connection_negative(constrait_connection):
	def __init__(self, constrait, body, radius):
		super().__init__(constrait, body, radius)	

	def constrait_screws(self):
		return [ 
			-scr for scr in self.constrait.constrait_screws()
		]
