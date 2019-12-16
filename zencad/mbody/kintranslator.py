#!/usr/bin/env

from zencad.libs.screw import screw


class kintranslator:
	"""Решает задачу построения динамической модели 
	по кинематическому дерева.
	"""

	FREE_SPACE_MODE = 0
	CONSTRAIT_MODE = 1

	def __init__(self, baseunit):
		self.baseunit = baseunit

	def build(self, mode=FREE_SPACE_MODE):
		self.kinframes = self.collect_all_kinframes()
		self.collect_all_kinframe_inertia()

		if mode == self.FREE_SPACE_MODE:
			pass

		elif mode == self.CONSTRAIT_MODE:
			pass

	def collect_all_kinframes(self):
		pass

	def collect_all_kinframe_inertia(self):
		for k in self.kinframes:
			k.collect_inertia()

	def make_constraits(self):
		ret = []
		for k in self.kinframes:
			k.init_constrait()
			ret.append(k.constrait)

	def make_bodies(self):
		ret = []
		for k in self.kinframes:
			k.init_body()
			ret.append(k.constrait)

	def produce_bodies_spdacc(self):
		pass

	def reduce_bodies_spdacc(self):
		pass