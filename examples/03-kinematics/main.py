import sympy as sp

def SymTransDH(link:list):
    a, alpha, d, theta = link

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


print(sp.sin(t2))