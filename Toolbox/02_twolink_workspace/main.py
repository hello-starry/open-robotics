# -*- coding: utf-8 -*-

import numpy as np
import math
import matplotlib.pyplot as plt

plt.rcParams['xtick.direction'] = 'in'
plt.rcParams['ytick.direction'] = 'in'
plt.rcParams['axes.edgecolor'] = '#aaaaaa'
plt.rcParams['axes.linewidth'] = '1'

PI = 3.1415926535
DELTA_THETA = 0.1*PI/180
LINE_COLOR = {'solid':  '#b85450',
              'dotted': '#999999',
              'thick':  '#6c8ebf'}

LINE_WIDTH = {'solid':  5,
              'dotted': 1,
              'thick':  5}

LINE_STYLE = {'solid':  '-',
              'dotted': '--',
              'thick':  '-'}

class ScaraWorkspace:
    def __init__(self, L1 = 400, L2 = 400):
        self.L1 = L1
        self.L2 = L2
        self.TL1 = -130 *PI/180.0
        self.TU1 =  130 *PI/180.0
        self.TL2 = -148 *PI/180.0
        self.TU2 =  148 *PI/180.0
        
        self.RB  = -400 # right_bound
        self.LB = 400
        self.FB = 700 # front bound
        self.BB = 200
        
        self.DMAX = 400 # 大圆直径
        self.DMIN = 40 # 小圆直径
        
    def plot(self, config = 0):
        assert (0==config or 1==config), "config只能为0（右手）或1（左手）"
         
        fig = plt.figure(figsize=(6, 6), facecolor='#fafaff')
        ax = fig.add_subplot(1, 1, 1)
        ax.set_facecolor('#fafaff')
        ax.tick_params(axis='x',colors='#aaaaaa')
        ax.tick_params(axis='y',colors='#aaaaaa')

        lim = (self.L1 + self.L2)*1.1
        ax.set_xlim([-lim, lim])
        ax.set_ylim([-lim, lim])
        ax.set_xlabel('x (mm)', color="#aaaaaa")
        ax.set_ylabel('y (mm)', color="#aaaaaa")
           
        self.plot_workspace(ax, config, arc_type='thick')
        self.plot_guide(ax, config, arc_type='dotted')
        self.plot_link(ax, config, arc_type='solid')
        
    def plot_workspace(self, ax, config=0, arc_type='thick'):
        # 绘制可达区域
        if config:
            T = self.TL2
        else:
            T = self.TU2
        
        L1, L2, TL1, TU1, TL2, TU2 = self.L1, self.L2, self.TL1, self.TU1, self.TL2, self.TU2
        
        x0, y0 = 0, 0
        r = math.sqrt(L1**2 + L2**2 + 2*L1*L2*math.cos(T))
        alpha = math.acos((r**2+L1**2-L2**2) / (2*r*L1))
        if config:
            TS = TL1-alpha
            TE = TU1-alpha
        else:
            TS = TL1+alpha
            TE = TU1+alpha
        self.plot_arc(ax, x0, y0, r, TS, TE, arc_type)

        
        x0, y0 = 0, 0
        r = L1+L2
        TS, TE = TL1, TU1
        self.plot_arc(ax, x0, y0, r, TS, TE, arc_type)

        
        x0, y0 = L1*math.cos(TU1), L1*math.sin(TU1)
        r = L2
        if config:
            TS = TU1+TL2
            TE = TU1
        else:
            TS = TU1
            TE = TU1+TU2
        self.plot_arc(ax, x0, y0, r, TS, TE, arc_type)

        
        x0, y0 = L1*math.cos(TL1), L1*math.sin(TL1)
        r = L2
        if config:
            TS = TL1+TL2
            TE = TL1
        else:
            TS = TL1
            TE = TL1+TU2
        self.plot_arc(ax, x0, y0, r, TS, TE, arc_type)   

    def plot_guide(self, ax, config=0, arc_type='dotted'):
        # 绘制辅助线
        L1, L2, TL1, TU1 = self.L1, self.L2, self.TL1, self.TU1

        if config:
            T = self.TL2
        else:
            T = self.TU2
        
        x0, y0 = 0, 0
        r = math.sqrt(L1**2 + L2**2 + 2*L1*L2*math.cos(T))
        self.plot_arc(ax, x0, y0, r, 0,  2*PI, arc_type)
        
        x0, y0 = 0, 0
        r = L1+L2
        self.plot_arc(ax, x0, y0, r, 0,  2*PI, arc_type)
        
        x0, y0 = L1*math.cos(TU1), L1*math.sin(TU1)
        r = L2
        self.plot_arc(ax, x0, y0, r, 0,  2*PI, arc_type)
        
        x0, y0 = L1*math.cos(TL1), L1*math.sin(TL1)
        r = L2
        self.plot_arc(ax, x0, y0, r, 0,  2*PI, arc_type)
        
    def plot_link(self,  ax, config=0, arc_type='solid'):
        # 绘制连杆
        if config:
            T = self.TL2
        else:
            T = self.TU2
        L1, L2, TL1, TU1 = self.L1, self.L2, self.TL1, self.TU1
        xA1 = L1 * math.cos(TL1)
        yA1 = L1 * math.sin(TL1)
        xB1 = xA1 + L2 * math.cos(TL1 + T)
        yB1 = yA1 + L2 * math.sin(TL1 + T)
        xA2 = L1 * math.cos(TU1)
        yA2 = L1 * math.sin(TU1)
        xB2 = xA2 + L2 * math.cos(TU1 + T)
        yB2 = yA2 + L2 * math.sin(TU1 + T)
        ax.plot([0, xA1, xB1], [0, yA1, yB1], color = LINE_COLOR[arc_type], 
                                              linewidth = LINE_WIDTH[arc_type],
                                              linestyle = LINE_STYLE[arc_type])
        ax.plot([0, xA2, xB2], [0, yA2, yB2], color = LINE_COLOR[arc_type], 
                                              linewidth = LINE_WIDTH[arc_type],
                                              linestyle = LINE_STYLE[arc_type])
        
    
    def plot_arc(self, ax, center_x, center_y, radius, theta1, theta2, arc_type = 'thick'):
        theta = np.arange(theta1, theta2, DELTA_THETA)
        plot_x = center_x+radius*np.cos(theta)
        plot_y = center_y+radius*np.sin(theta)
        ax.plot(plot_x, plot_y, 
                color = LINE_COLOR[arc_type], 
                linewidth = LINE_WIDTH[arc_type],
                linestyle = LINE_STYLE[arc_type])

        
        
if __name__ == "__main__":
    SW = ScaraWorkspace()
    SW.plot(0)