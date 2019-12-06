from zencad.libs.screw import screw

class lagrange_multiplier:
	no = 0

	@classmethod
	def getno(cls):
		ret = cls.no
		cls.no += 1
		return ret

	def __init__(self):
		self.no = self.getno()

	def __repr__(self):
		return "l" + str(self.no)

#class base_lagrange_screw:
#	def __init__(self, ang, lin, lmultiplier=None):
#		if lmultiplier is None:
#			lmultiplier = lagrange_multiplier()

#		self.screw = screw(ang=ang, lin=lin)
#		self.lmultiplier = lmultiplier

#	def same(oth):
#		return self.lmultiplier is oth.lmultiplier

class lagrange_polynom:
	def __init__(self, dct):
		self.dct = dct

	def __add__(self, oth):
		dct = self.dct.copy()
		for h in oth.dct.keys():
			if h in dct:
				dct[h] = dct[h] + oth.dct[h]
			else:
				dct[h] = oth.dct[h]

	def __sub__(self, oth):
		dct = self.dct.copy()
		for h in oth.dct.keys():
			if h in dct:
				dct[h] = dct[h] - oth.dct[h]
			else:
				dct[h] = oth.dct[h]

	def foreach(self, lmb):
		return lagrange_polynom({
			l : lmb(v) for l, v in self.dct.items() 
		})

	def __repr__(self):
		return repr(self.dct)

def make_new_lagrange_polynom(lst):
	return lagrange_polynom({
		lagrange_multiplier() : l for l in lst
	})

def free(obj):
	return lmultiplier_complex_screw({
		lagrange_multiplier() : l for l in lst
	})

class lagrange_solver:
	def __init__(self):
		pass