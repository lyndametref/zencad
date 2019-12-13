from zencad.libs.rigid_body import rigid_body
from zencad.libs.constraits import constrait, constrait_connection
from zencad.libs.screw import screw
import numpy

class matrix_solver:
	def __init__(self, rigid_bodies, constraits, workspace_scale=1):
		self.rigid_bodies = rigid_bodies
		self.constraits = constraits
		self.workspace_scale = workspace_scale

		self.numerate_rigid_bodies()
		self.numerate_constraits()

	def update_views(self):
		for s in self.rigid_bodies:
			s.update_views()

	def numerate_rigid_bodies(self):
		for i, r in enumerate(self.rigid_bodies):
			r.dynno = i 
		
	def numerate_constraits(self):
		constrait_idx = 0
		for i, c in enumerate(self.constraits):
			c.dynno = i 
			c.constrait_idx = constrait_idx
			constrait_idx += c.rank()

	def mass_matrix(self):
		NR = len(self.rigid_bodies)
		N = NR * 6

		M = numpy.zeros((N, N))		
		for idx, rbody in enumerate(self.rigid_bodies):
			l = rbody.reference_mass_matrix
			for i in range(6):
				for j in range(6):
					if (i>=3 and j<3) or (i<3 and j>=3):
						# Умножаем элементы матрицы масс, зависимые от радиуса на масштабный коэффициент.
						M[idx*6+i, idx*6+j] = l[i,j] * self.workspace_scale
					else:
						M[idx*6+i, idx*6+j] = l[i,j]


		return M

	def update_constraits_globals(self):
		pass

	def update_rbody_globals(self):
		for s in self.rigid_bodies:
			s.update_globals()

	def update_globals(self):
		self.update_constraits_globals()
		self.update_rbody_globals()

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

		for constrait in self.constraits:
			for connection in constrait.connections:
				#links = connection.body_carried_constrait_screws()
				conidx = constrait.constrait_idx
				idx = connection.body.dynno
				m = connection.constrait_matrix()

				for i in range(connection.rank()):
					for j in range(6):
						G[conidx+i, idx*6+j] = m[i, j]
				
		for constrait in self.constraits:
			#links = constrait.constrait_screws()
			conidx = constrait.constrait_idx

			m = connection.compensate_vector()

			for i in range(connection.rank()):
				h[conidx+i] = m[i]

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
			#scr = rbody.inertia_force_in_body_frame()
			scr = rbody.inertia_force()

			K[idx*6+0,0] = scr.lin.x  
			K[idx*6+1,0] = scr.lin.y
			K[idx*6+2,0] = scr.lin.z
			K[idx*6+3,0] = scr.ang.x
			K[idx*6+4,0] = scr.ang.y
			K[idx*6+5,0] = scr.ang.z

		return K

	def solve(self):
		NR = len(self.rigid_bodies)
		NC = len(self.constraits)

		self.update_globals()

		M = self.mass_matrix()
		S = self.active_forces()
		K = self.inertia_forces()
		G, h = self.constrait_matrix()
		





		#Minv = numpy.linalg.inv(M)
		#A = numpy.matmul(G, numpy.matmul(Minv, G.transpose()))
		#b = - numpy.matmul(G, numpy.matmul(Minv, (S + K))) - h
		#
		#self.reactions = numpy.linalg.solve(A,b)
		#self.accelerations = numpy.matmul(Minv, (S + K)) + numpy.matmul(Minv, numpy.matmul(G.transpose(), self.reactions))
#
		#print(self.accelerations)
#
		#return self.accelerations, self.reactions

		


		SK = S+K
		
		A = numpy.zeros((M.shape[0] + G.shape[0], M.shape[1] + G.shape[0]))
		for k in range(NR):
			for i in range(6):
				for j in range(6):
					A[k*6+i,k*6+j] = M[i,j]
		
		for i in range(G.shape[0]):
			for j in range(G.shape[1]):
				A[NR*6+i,j] = G[i,j]
				A[j,NR*6+i] = -G[i,j]
		
		B = numpy.zeros((M.shape[0] + G.shape[0]))
		
		for i in range(SK.shape[0]):
			B[i] = SK[i]
		
		for i in range(h.shape[0]):
			B[i+SK.shape[0]] = -h[i]
		
		res = numpy.matmul(numpy.linalg.inv(A), B)
		
		self.accelerations = numpy.zeros((NR*6))
		self.reactions = numpy.zeros((G.shape[0]))
		
		for i in range(SK.shape[0]):
			self.accelerations[i] = res[i]
		
		for i in range(h.shape[0]):
			self.reactions[i] = res[i + NR*6]




		return self.accelerations, self.reactions

	def apply_acceleration_to_rigid_bodies(self):
		for r in self.rigid_bodies:
			r.acceleration = screw(
				lin = ( 
					self.accelerations[r.dynno*6+0],
					self.accelerations[r.dynno*6+1],
					self.accelerations[r.dynno*6+2]),
				ang = (
					self.accelerations[r.dynno*6+3],
					self.accelerations[r.dynno*6+4],
					self.accelerations[r.dynno*6+5])
			)

	def rbodies_integrate(self, delta):
		for r in self.rigid_bodies:
			#diff = (r.speed * delta).to_trans()
			diff = (r.speed * delta).inverse_rotate_by(r.pose).to_trans()
			r.pose = r.pose * diff
			r.speed = r.speed + r.acceleration * delta 

	def apply(self, delta):
		self.apply_acceleration_to_rigid_bodies()
		self.rbodies_integrate(delta)
		self.update_views()

	def apply_reactions_for_constraits(self, reactions):
		for c in self.constraits:
			strt = c.constrait_idx
			rank = c.rank()

			c.reactions = [ reactions[i+strt] for i in range(rank) ]