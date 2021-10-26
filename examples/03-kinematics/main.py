import sympy as sp
from sympy import sin, cos
from sympy.polys.domains.integerring import ZZ
from sympy.simplify.trigsimp import trigsimp

def SymTransDH(link:list):
    a, p, d, t = link
    return sp.Matrix([[cos(t),       -sin(t),         0,        a       ],
                      [sin(t)*cos(p), cos(t)*cos(p), -sin(p),  -d*sin(p)],
                      [sin(t)*sin(p), cos(t)*sin(p),  cos(p),   d*cos(p)],
                      [0,             0,              0,        1       ]])

a1 = sp.Symbol('a1')
a2 = sp.Symbol('a2')
a3 = sp.Symbol('a3')
d4 = sp.Symbol('d4')
d6 = sp.Symbol('d6')
t1 = sp.Symbol('t1')
t2 = sp.Symbol('t2')
t3 = sp.Symbol('t3')
t4 = sp.Symbol('t4')
t5 = sp.Symbol('t5')
t6 = sp.Symbol('t6')
pi = sp.pi


link1 = [ 0,   0,     0,    t1     ]
link2 = [ a1, -pi/2,  0,    t2-pi/2]
link3 = [ a2,  0,     0,    t3     ]
link4 = [ a3, -pi/2,  d4,   t4     ]
link5 = [  0,  pi/2,  0,    t5     ]
link6 = [  0, -pi/2,  d6,   t6     ]

T1 = SymTransDH(link1)
T2 = SymTransDH(link2)
T3 = SymTransDH(link3)
T4 = SymTransDH(link4)
T5 = SymTransDH(link5)
T6 = SymTransDH(link6)

T = T1*T2*T3*T4*T5*T6

x = sp.Symbol('x')
y = sp.Symbol('y')
z = sp.Symbol('z')
a = sp.Symbol('a')
b = sp.Symbol('b')
c = sp.Symbol('c')

f1 = T[0,3]-x
f2 = T[1,3]-y
f3 = T[2,3]-z
f4 = T[2,0]+sin(b)
f5 = T[0,0]-cos(a)*cos(b)
f6 = T[2,1]-cos(b)*sin(c)
result = sp.solve([f1,f2,f3,f4,f5,f6],[t1,t2,t3,t4,t5,t6])

print(result)

