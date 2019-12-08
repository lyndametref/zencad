import numpy

# Based on:
# Моделирование динамики системы твердых и упругих тел в программном комплексе EULER В.Г. Бойков, А.А. Юдаков 

class reaction_solver:
	def __init__(self, rigid_bodies, constraits):
		self.rigid_bodies = rigid_bodies
		self.constraits = constraits
		self.init()

	def init(self):		
		self.enumerate_elements()

	def enumerate_elements(self):
		self.numerate_rigid_bodies()
		self.numerate_constraits()

	def update_constraits_globals(self):
		for c in self.constraits:
			c.update_globals()

	def update_rbody_globals(self):
		for b in self.rigid_bodies:
			b.update_globals()

	def update_globals(self):
		self.update_constraits_globals()
		self.update_rbody_globals()

	def numerate_rigid_bodies(self):
		for i, r in enumerate(self.rigid_bodies):
			r.dynno = i 
		
	def numerate_constraits(self):
		constrait_idx = 0
		for i, c in enumerate(self.constraits):
			c.dynno = i 
			c.constrait_idx = constrait_idx
			constrait_idx += c.rank()

	def collect_constraits(self):
		arr = []
		pass

	def mass_matrix(self):
		NR = len(self.rigid_bodies)
		N = NR * 6

		M = numpy.zeros((N, N))		
		for idx, rbody in enumerate(self.rigid_bodies):
			l = rbody.global_mass_matrix
			for i in range(6):
				for j in range(6):
					M[idx*6+i, idx*6+j] = l[i,j]

		return M

	def constraits_count(self):
		accum = 0
		for c in self.constraits:
			accum += c.rank()
		return accum

	def constrait_matrix(self):
		NR = len(self.rigid_bodies)
		N = NR * 6
		NC = self.constraits_count()

		G = numpy.zeros((NC, N), dtype=numpy.float64)
		h = numpy.zeros((NC, 1), dtype=numpy.float64)

		for idx, rbody in enumerate(self.rigid_bodies):
			for connection in rbody.constrait_connections:
				links = connection.constrait_screws()
				conidx = connection.constrait.constrait_idx

				#print(links)
				#exit(0)

				for i in range(connection.rank()):
					scr = links[i]

					G[conidx + i, idx*6+0] = scr.lin.x  
					G[conidx + i, idx*6+1] = scr.lin.y
					G[conidx + i, idx*6+2] = scr.lin.z
					G[conidx + i, idx*6+3] = scr.ang.x
					G[conidx + i, idx*6+4] = scr.ang.y
					G[conidx + i, idx*6+5] = scr.ang.z

		return G, h

	def active_forces(self):
		NR = len(self.rigid_bodies)
		N = NR * 6

		S = numpy.zeros((N, 1))

		return S

	def inertia_forces(self):
		NR = len(self.rigid_bodies)
		N = NR * 6

		K = numpy.zeros((N, 1))
		for idx, rbody in enumerate(self.rigid_bodies):
			scr = rbody.inertia_force()

			K[idx*6+0,0] = scr.lin.x  
			K[idx*6+1,0] = scr.lin.y
			K[idx*6+2,0] = scr.lin.z
			K[idx*6+3,0] = scr.ang.x
			K[idx*6+4,0] = scr.ang.y
			K[idx*6+5,0] = scr.ang.z


		return K

	def solve(self):
#		NR = len(self.rigid_bodies)
#		NF = 0
#		N = len(self.rigid_bodies) * 6
#		NC = len(self.constraits)

		M = self.mass_matrix()
		S = self.active_forces()
		K = self.inertia_forces()
		G, h = self.constrait_matrix()

		
		Minv = numpy.linalg.inv(M)
		A = numpy.matmul(G, numpy.matmul(Minv, G.transpose()))
		b = - numpy.matmul(G, numpy.matmul(Minv, (S + K))) - h

		return numpy.matmul(numpy.linalg.inv(A), b)

	def apply_reactions_for_constraits(self, reactions):
		for c in self.constraits:
			strt = c.constrait_idx
			rank = c.rank()

			c.reactions = [ reactions[i+strt] for i in range(rank) ]

