#!/usr/bin/env python3 

from zencad import *
import zencad.mbody.kinematic as kin
import zencad.assemble as assemble

u0 = assemble.unit(name="u0")
u1 = assemble.unit(name="u1")
u2 = assemble.unit(name="u2")
u3 = assemble.unit(name="u3")

u0.link(u1)
u0.link(u2)
u2.link(u3)

kin.attach_kinematic_chains(u0)

print(u3)
print(u3.kinematic_chain.chain)