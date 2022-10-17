import os
import tkinter as tk
from tkinter import *
from tkinter import ttk
from tkinter.font import Font
from matplotlib import axes

import numpy as np

import GUI_tools as gui
from os_interface import OsInterface
import SensorInterface
import Sampler

from matplotlib import pyplot as plt
from matplotlib import cm
from matplotlib.figure import Figure
from matplotlib.axes import Axes
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

from scipy.optimize import curve_fit
from scipy.stats import multivariate_normal

from RadarPlot import ComplexRadar

PLOT_REFRESH_DELAY = 100 # ms
PLOT_INDEX_STR = "Samples"

PLOT_TYPE_GRAPH = "Graph"
PLOT_TYPE_BAR = "Bars"
PLOT_TYPE_F2221 = "Fingers (F2221)"
PLOT_TYPE_F2221_II = "Fingers II (F2221)"
PLOT_TYPE_F2221_III = "Fingers III (F2221)"
PLOT_TYPE_RADAR = "Radar"
PLOT_TYPES = [PLOT_TYPE_GRAPH,PLOT_TYPE_BAR,PLOT_TYPE_F2221,PLOT_TYPE_F2221_II,PLOT_TYPE_F2221_III,PLOT_TYPE_RADAR]

CAL_TYPE_POLY2 = "Poly, deg 2"
CAL_TYPE_POLY3 = "Poly, deg 3"
CAL_TYPE_LINSPLINE = "Linear Spline"
CAL_TYPE_LINSPLINE_K = "Linear Spline, offset"
CAL_TYPE_LINSPLINE_ACT = "Linear Spline, activation"
CAL_TYPE_POLYSPLINE = "Poly Spline"
CAL_TYPE_POLYSPLINE_K = "Poly Spline, offset"
CAL_TYPE_POLYSPLINE_ACT = "Poly Spline, activation"
CAL_TYPE_POLYSPLINE2 = "Poly Spline (non-smooth)"
CAL_TYPES = [CAL_TYPE_POLY2,CAL_TYPE_POLY3,CAL_TYPE_LINSPLINE,CAL_TYPE_LINSPLINE_K, CAL_TYPE_LINSPLINE_ACT,CAL_TYPE_POLYSPLINE,CAL_TYPE_POLYSPLINE_K,CAL_TYPE_POLYSPLINE_ACT,CAL_TYPE_POLYSPLINE2]


COLOR_WHEEL = plt.rcParams['axes.prop_cycle'].by_key()['color']

TS:int = 2; TM:int = 1; FP_MAR:int = 1
F2221_T_LOC = np.zeros((7,2), dtype=int)
for r in range(3):
    for c in range(2):
        F2221_T_LOC[2*r+c, 0] = r*(TS+TM)
        F2221_T_LOC[2*r+c, 1] = c*2
F2221_T_LOC[6,:] = [3*(TS+TM), 1]

F2221_TAX_LOC = np.array([[ -2.75, 2.75,
                            -2.75, 2.75,
                            -2.75, 2.75,
                                 0]
               
                            ,[30,30,
                              20,20,
                              10,10,
                                0]])

class GraphFrame(ttk.Frame):

    def __init__(self, win, sampler:Sampler.Sampler, os_handle:OsInterface):
        super().__init__(win)
        self.win = win
        self.sampler = sampler
        self.os_handle = os_handle

        self._show_header_footer = True

        self.x_index = -1
        self.y_indices = np.array([0,1])

        self.sample_index = -1 # last index
        self.plot_type = PLOT_TYPE_GRAPH

        self.plotting_on = False # GUI button state
        self._is_plotting=False
        self._stop_plotting=False

        self.fig:plt.Figure = None
        self.graphs = []
        self.contours = []
        self.axes = []

        self.initUI()
        self.sampler.subscribe(self.sampler_callback)
        self.needs_ui_update = False
        self.checkNeedsUpdate()

    def sampler_callback(self):
        self.needs_ui_update=True

    def checkNeedsUpdate(self):
        if(self.needs_ui_update): 
            if(self.cmb_x.get() not in self.sampler.sampling_header()):
                self.cmb_x.current(0)
            self.combo_y_will_show(None) # update list
            if(self.plotting_on):  self.reinitialize_plot()
            self.updateUI()
            self.needs_ui_update = False

        self.after(100,self.checkNeedsUpdate)

    def getSelectedXY(self) -> tuple: #[bool,np.ndarray,np.ndarray]:

        data = self.sampler.sampling_data()
        if(data.shape[0]==0 or data.shape[1]==0):
            return False,None,None

        # self.sld_plot_index["_to"] = data.shape[0]
        # print(data[self.sample_index,:])

        if(self.x_index == -1):
            X = np.arange(data.shape[0])
        else:
            X = data[:,self.x_index]

        Y = np.atleast_2d(data[:,self.y_indices])
        return True,X,Y

    def reinitialize_plot(self):
        self._stop_plotting=True
        self.init_plot()

    def stop_plotting(self):
        self.plotting_on = False
        self._stop_plotting = True

    def start_plotting(self):
        self.plotting_on = True
        self.init_plot()

    def clear_plot(self):
        self.graphs = []
        self.fig.clear()
        self.ax = self.fig.add_subplot(111)
        self.canvas.draw()
        self._is_plotting=False
        self._stop_plotting=True

    def init_plot(self, start_plotting=True):
        print("init plot")
        N = self.sampler.sampling_columns()
        # self.y_indices = self.ent_y.getArray() -1 # convert to 0-indexing
        self.y_indices = np.array(self.cmb_y.getSelectedIndices())
        self.y_indices = self.y_indices[self.y_indices<N]
        N = self.y_indices.size

        print(f"GraphGUI - init_plot - cmbY: {self.cmb_y.getSelectedItems()}")

        self.graphs = []
        self.fig.clear()
        self.ax = self.fig.add_subplot(111)
        # self.ax.cla()

        if(N==0):
            print("No signals to log!")
            self.canvas.draw()
            return

        if(self.plot_type==PLOT_TYPE_BAR): self.init_barplot(N)
        elif(self.plot_type==PLOT_TYPE_GRAPH): self.init_graphplot(N)
        elif(self.plot_type==PLOT_TYPE_F2221): self.init_F2221plot(N)
        elif(self.plot_type==PLOT_TYPE_F2221_II): self.init_F2221plot2(N)
        elif(self.plot_type==PLOT_TYPE_F2221_III): self.init_F2221plot3(N)
        elif(self.plot_type==PLOT_TYPE_RADAR): self.init_radarplot(N)
        else: raise Exception("Unknown plot type")

        if(start_plotting):
            self._is_plotting=True
            self._stop_plotting=False
            self.update_plot()

    def update_limits(self,_):
        if(self.ent_x_min.isValid()): self.ax.set_xlim(left   = self.ent_x_min.getValue())
        if(self.ent_x_max.isValid()): self.ax.set_xlim(right  = self.ent_x_max.getValue())
        if(self.ent_y_min.isValid()): self.ax.set_ylim(bottom = self.ent_y_min.getValue())
        if(self.ent_y_max.isValid()): self.ax.set_ylim(top    = self.ent_y_max.getValue())
        self.update_plot()

    def update_plot(self):
        if(self._stop_plotting):
            self._is_plotting=False
            return

        ret,X,Y = self.getSelectedXY()
        # if(not ret):
        #     self._is_plotting=False
        #     return

        if(ret):
            # print("update plot")
            if(  self.plot_type==PLOT_TYPE_BAR):   self.update_barplot(X, Y)
            elif(self.plot_type==PLOT_TYPE_GRAPH): self.update_graphplot(X, Y)
            elif(self.plot_type==PLOT_TYPE_F2221): self.update_F2221plot(X, Y)
            elif(self.plot_type==PLOT_TYPE_F2221_II): self.update_F2221plot2(X, Y)
            elif(self.plot_type==PLOT_TYPE_F2221_III): self.update_F2221plot3(X, Y)
            elif(self.plot_type==PLOT_TYPE_RADAR): self.update_radarplot(X, Y)
            else: raise Exception("Unknown plot type")
            self.canvas.draw()

        if(self.sampler.sampling_stopped() or self._is_plotting==False):
            self._is_plotting=False
        else:
            self.after(PLOT_REFRESH_DELAY, self.update_plot)

    def init_graphplot(self, N):
        if(N>=5): self.var_twinx.set(0) # disallow more than 5 multi axes

        if(self.var_twinx.get()==1):
            self.fig.subplots_adjust(right=0.9-max(0,(N-2))*0.2)
            ax_offset = 1
        else:
            self.fig.subplots_adjust(right=0.9)

        a = self.ax
        self.axes = [self.ax]
        for i in range(N):
            self.graphs.append( a.plot([], [], 'o-', ms=5, lw=1, color=COLOR_WHEEL[i%len(COLOR_WHEEL)])[0] )
            if(self.var_twinx.get()==1 and i<N-1):
                a = self.ax.twinx()
                a.spines['right'].set_color(COLOR_WHEEL[(i+1)%len(COLOR_WHEEL)])
                a.spines['left'].set_color(COLOR_WHEEL[0])
                a.spines['right'].set_position(("axes", ax_offset))
                ax_offset+=0.2
                self.axes.append(a)

        self.ax.grid(True)
        h = np.array(self.sampler.sampling_header())
        if(self.var_twinx.get()==1):
            for i,ax in enumerate(self.axes):
                ax.set_ylabel(h[self.y_indices[i]])

        # self.ax.legend(h[self.y_indices])
        self.fig.legend(h[self.y_indices], ncol=4, loc='upper center', bbox_to_anchor=(0.5,0.95), fancybox=True, shadow=True)
        # self.fig.legend(h[self.y_indices], bbox_to_anchor=(1,1), bbox_transform=self.axes[0].transAxes)
        if(self.x_index!=-1):
            self.ax.set_xlabel(h[self.x_index])
        else:
            self.ax.set_xlabel(PLOT_INDEX_STR)

    def update_graphplot(self, X, Y):
        for i in range(len(self.graphs)):
            g = self.graphs[i]
            g.set_data(X, Y[:,i])

        M = 0.05
        for i,ax in enumerate(self.axes):
            x_dif = X.max() - X.min()
            if(not self.ent_x_min.isValid()): ax.set_xlim(left   = X.min()-M*x_dif)
            if(not self.ent_x_max.isValid()): ax.set_xlim(right  = X.max()+M*x_dif)
            if(self.var_twinx.get()==1): y = Y[:,i]
            else: y = Y
            yMax = y[y<np.Inf].max()
            yMin = y[y>-np.Inf].min()
            y_dif = yMax-yMin
            if(not self.ent_y_min.isValid()): ax.set_ylim(bottom = yMin-M*y_dif)
            if(not self.ent_y_max.isValid()): ax.set_ylim(top    = yMax+M*y_dif)


    def init_barplot(self, N):
        X = np.arange(N)
        Y = np.zeros((N))
        self.graphs = self.ax.bar(X,X)
        self.ax.yaxis.grid(True)

    def update_barplot(self, X, Y):
        for i, b in enumerate(self.graphs):
            b.set_height(Y[self.sample_index,i])

        M = 0.05
        y_dif = Y.max() - Y.min()
        if(not self.ent_x_min.isValid()): self.ax.set_xlim(left   = -0.6)
        if(not self.ent_x_max.isValid()): self.ax.set_xlim(right  = Y.shape[1]-1+0.6)
        if(not self.ent_y_min.isValid()): self.ax.set_ylim(bottom = 0)
        if(not self.ent_y_max.isValid()): self.ax.set_ylim(top    = Y.max()+M*y_dif)

    def init_F2221plot(self, N):
        if(N == 7):     self.fn=1
        elif(N == 2*7): self.fn=2
        else:           self.fn=0; return

        V_MAX = 20
        self.im_arr = np.ones((11+2*FP_MAR, self.fn*(2*TS+FP_MAR)+FP_MAR))*V_MAX
        self.im = self.ax.imshow(self.im_arr, interpolation='none', cmap="hot", vmin=0, vmax=V_MAX)

    def update_F2221plot(self, X, Y):
        for f in range(self.fn):
            for i,v in enumerate(Y[0, f*7:(f+1)*7]):
                xl = F2221_T_LOC[i,1] + (FP_MAR+2*TS)*f + FP_MAR
                yl = F2221_T_LOC[i,0] + FP_MAR
                self.im_arr[yl:(yl+TS), xl:(xl+TS)] = v

        self.im.set_array(self.im_arr)
        self.canvas.draw()

    def init_F2221plot2(self, N):
        if(N == 7):     self.fn=1
        elif(N == 2*7): self.fn=2
        else:           self.fn=0; return

        for f in range(self.fn):
            self.graphs.append( self.ax.scatter(F2221_TAX_LOC[0,:]+f*15,F2221_TAX_LOC[1,:],s=1,color='black') )
            self.contours.append( None )

        self.ax.axis('equal')

    def update_F2221plot2(self, X, Y):
        if(self.fn == 0): return 

        for fi in range(self.fn):
            y = Y[0,fi*7:(fi+1)*7]
            self.graphs[fi].set_sizes(np.maximum(y+1, 1)*500)

            if(self.contours[fi] != None):
                for coll in self.contours[fi].collections:
                    coll.remove()
                self.contours[fi] = None

            if(y.sum() > 0):
                mu = np.sum(F2221_TAX_LOC * y, axis=1) / y.sum()
                C = np.cov(F2221_TAX_LOC, aweights=np.maximum(y,0.01)) # TODO 0.01 is an abitrary value
                rv = multivariate_normal(mu, C)

                xx, yy = np.mgrid[-10:10:1.0, -5:35:1.0]
                pos = np.dstack((xx, yy))
                # self.contours[fi] = self.ax.contour(xx+fi*15, yy, y.sum()*rv.pdf(pos), levels=[0.05,0.06,0.07])
                zz =  y.sum()*rv.pdf(pos)
                zz_max = zz.max()
                self.contours[fi] = self.ax.contour(xx+fi*15, yy, zz, levels=[0.7*zz_max,0.8*zz_max,0.9*zz_max])

        self.canvas.draw()

    def init_F2221plot3(self, N):
        if(N == 7):     self.fn=1
        elif(N == 2*7): self.fn=2
        else:           self.fn=0; return

        self.fig.clear()
        self.ax_3d = self.fig.add_subplot(111, projection='3d')

        D = 25
        self.graphs = []
        for f in range(self.fn):
            self.graphs.append(self.ax_3d.bar3d(F2221_TAX_LOC[0,:]+f*D, F2221_TAX_LOC[1,:], 0, 1, 1, 0, shade=True))
        
        #self.ax_3d.set_title('Finger 1')
        self.ax_3d.set_axis_off()
        scaling = np.array([getattr(self.ax_3d, 'get_{}lim'.format(dim))() for dim in 'xyz'])
        self.ax_3d.auto_scale_xyz(*[[np.min(scaling), np.max(scaling)]]*3)
        self.ax_3d.view_init(30, 45+90)

    def update_F2221plot3(self, X, Y):
        if(self.fn == 0): return 

        [g.remove() for g in self.graphs]
        MAX_FORCE = 20
        D = 25
        self.graphs = []
        for fi in range(self.fn):
            y = Y[0,fi*7:(fi+1)*7]
            #bar_colors = cm.jet(y/MAX_FORCE)
            bar_colors = np.zeros((len(y),3))
            bar_colors[:] = (0,0,0) # black
            bar_colors[y>0] = (0.6,0,0) # blue

            self.graphs.append(self.ax_3d.bar3d(F2221_TAX_LOC[0,:]+fi*D, F2221_TAX_LOC[1,:], np.zeros_like(y), 1, 1, y, color=bar_colors, shade=True))

        scaling = np.array([getattr(self.ax_3d, 'get_{}lim'.format(dim))() for dim in 'xyz'])
        self.ax_3d.auto_scale_xyz(*[[np.min(scaling), np.max(scaling)]]*3)
        self.ax_3d.view_init(30, 45+90)
        self.canvas.draw()

    def init_radarplot(self, N):
        label_names = ("Width", "Hardness", "Contact Area", "Curvature")
        ranges = [(0, 100), (0, 25), (0, 1.6), (0, 20)]

        self.fig.clear()
        radar = ComplexRadar(self.fig, label_names, ranges)

        # radar.plot((63, 7, 200, 3), 'g')
        # radar.plot((67, 20, 500, 5), 'g')
        # radar.plot(data)
        # radar.fill(data, alpha=0.2)

        self.graphs = radar

    def update_radarplot(self, X, Y):
        self.graphs.clear()

        x = Y[:,0]
        Y = Y[:,1:]
        print(x)
        types = np.unique(x)
        
        #colors = ['g','b','y','r','p']
        val_str = ""
        for i,t in enumerate(types[types>0]):
            y = Y[x==t, :]
            print(f"{i}: {t} /\n")
            print(y)
            self.graphs.plot(np.min(y, axis=0), f'C{i}')
            self.graphs.plot(np.max(y, axis=0), f'C{i}')
            self.graphs.plot(np.mean(y, axis=0), f'--C{i}')

            val_str += f"{t}: min: {np.min(y, axis=0)}, max: {np.max(y, axis=0)}, mean: {np.mean(y, axis=0)}" + "\n"
        
        print(val_str)
            

        for y in Y[x==0, :]:
            self.graphs.plot(y, 'k')


        #val_str = f"min: {np.min(Y[x==0, :], axis=0)}, max: {np.max(Y[x==0, :], axis=0)}, mean: {np.mean(Y[x==0, :], axis=0)}"         

        # print(np.mean(Y, axis=0))
        # self.graphs.plot(np.min(Y, axis=0), 'g')
        # self.graphs.plot(np.max(Y, axis=0), 'g')
        # self.graphs.plot(np.mean(Y, axis=0), '--k')
        #self.graphs.fill(y, alpha=0.2)

        self.txt_coefsDisplay.configure(state=NORMAL)
        self.txt_coefsDisplay.delete(1.0, "end")
        self.txt_coefsDisplay.insert(1.0, val_str)
        # self.txt_coefsDisplay.insert(1.0, ' '.join([f"{x:.4e}" for x in cal_params]))
        self.txt_coefsDisplay.configure(state=DISABLED)


    def combo_plot_selected(self,_):
        self.plot_type = self.cmb_plot_type.get()
        self.plotting_on = True
        self.reinitialize_plot()
        self.updateUI()

    def combo_x_selected(self,_):
        selected = self.cmb_x.get()
        if(selected==PLOT_INDEX_STR):
            self.x_index = -1
        else:
            self.x_index = self.sampler.sampling_header().index(selected)
        
        self.plotting_on = True
        self.reinitialize_plot()
        self.updateUI()

    def combo_x_clicked(self,_):
        self.cmb_x['values'] = [PLOT_INDEX_STR] + self.sampler.sampling_header()

    def combo_y_will_show(self,_):
        if(self.cmb_y.getItems() != self.sampler.sampling_header()):
            self.cmb_y.setItems(self.sampler.sampling_header())

    def combo_y_selected(self,_):
        print("y selected")
        print(self.cmb_y.getSelectedIndices())
        self.plotting_on=True
        self.reinitialize_plot()
        self.updateUI()

    def multi_axes_clicked(self):
        if(self.var_twinx.get()==0):
            self.stop_plotting()
            self.clear_plot()
            self.start_plotting()
        else:
            self.reinitialize_plot()

        self.updateUI()

    def toggle_on_clicked(self):
        if(self.btn_on.isOn()):
            self.start_plotting()
        else:
            self.stop_plotting()
        
    def clear_clicked(self):
        self.stop_plotting()
        self.clear_plot()
        self.updateUI()

    def export_clicked(self):
        fname = self.os_handle.save_png_dialog()
        print("export figure '"+fname+"'")
        self.fig.savefig(fname)

    def openplot_clicked(self):
        #TODO
        plt.plot([1,2,3],[1,2,3])
        plt.show(block=False)
        # self.init_graphplot( *pass figure* )
        # fig.show(block=false)
        
    def lin_interp_XY(self, X:np.ndarray, Y:np.ndarray):
        Xlin = np.linspace(X.min(), X.max(), len(X))
        Ylin = np.zeros_like(Xlin)

        for i in range(0,len(Xlin)):
            i1 = np.argmin(X < Xlin[i])-1
            if(i1<0):
                Ylin[i] = Y.min()
                continue
            i2 = np.argmax(X > Xlin[i])
            s = (Xlin[i]-X[i1])/(X[i2]-X[i1])
            Ylin[i] = Y[i1] + s*(Y[i2]-Y[i1])

        return Xlin, Ylin

    def calib_clicked(self):
        if(self.plot_type != PLOT_TYPE_GRAPH): return

        ret,X,Y = self.getSelectedXY() # X: Cond, Y: Force
        if(ret==False): return
        if(Y.shape[1]!=1):
            print("Y must be a single data column")

        X,Y = self.lin_interp_XY(X,Y.ravel())
        # X = X*1e-6

        if(self.cmb_cal_type.get() == CAL_TYPE_POLY2):
            fit_func = lambda x,a,b,c: a*x**2+b*x+c
            fit_coefs,_ = curve_fit(fit_func, X, Y, bounds=([0, 0, 0],[np.inf, np.inf, np.inf]))
            cal_params = fit_coefs

        elif(self.cmb_cal_type.get() == CAL_TYPE_POLY3):
            fit_func = lambda x,a,b,c,d: a*x**3+b*x**2+c*x+d
            fit_coefs,_ = curve_fit(fit_func, X, Y, bounds=([0, 0, 0, 0],[np.inf, np.inf, np.inf, np.inf]))
            cal_params = fit_coefs

        elif(self.cmb_cal_type.get() == CAL_TYPE_LINSPLINE):
            fit_func = lambda x,a,b,c: np.minimum(x,a)*b + np.maximum(x-a,0)*c
            fit_coefs,_ = curve_fit(fit_func, X, Y, bounds=([0, 0, 0],[np.inf, np.inf, np.inf]))
            cal_params = fit_coefs

        elif(self.cmb_cal_type.get() == CAL_TYPE_LINSPLINE_K):
            fit_func = lambda x,a,b,c, k: np.minimum(x,a)*b + np.maximum(x-a,0)*c + k 
            fit_coefs,_ = curve_fit(fit_func, X, Y, bounds=([0, 0, 0, -np.inf],[np.inf, np.inf, np.inf, np.inf]))
            cal_params = fit_coefs

        elif(self.cmb_cal_type.get() == CAL_TYPE_LINSPLINE_ACT):
            act_i = np.argmax(Y>Y[0])
            act_F = Y[act_i]
            act_G = X[act_i]
            print(f"Activation force: {act_F}")
            print(f"Activation cond: {act_G}")
            fit_func = lambda x,a,b,c: np.minimum(x-act_G,a)*b + np.maximum(x-act_G-a,0)*c + act_F
            fit_coefs,_ = curve_fit(fit_func, X, Y, bounds=([0, 0, 0],[np.inf, np.inf, np.inf]))
            cal_params = np.append(fit_coefs, [act_G,act_F])

        elif(self.cmb_cal_type.get() == CAL_TYPE_POLYSPLINE):
            fit_func = lambda x,a,b,c: np.minimum(x,a)*b + np.maximum(x-a,0)*c
            lcoefs,_ = curve_fit(fit_func, X, Y, bounds=([0, 0, 0],[np.inf, np.inf, np.inf]))

            def polyspline_deg2_N2_smooth(x,a,b,c,e):
                x1 = np.minimum(x,a)
                x2 = np.maximum(x-a,0)
                d = 2*a*c + b
                return b*x1 + c*x1**2 + d*x2 + e*x2**2

            fit_func = polyspline_deg2_N2_smooth
            # fit_coefs,_ = curve_fit(fit_func, X, Y, bounds=([3e3,0,0,0],[20e3,1e-3,1e-6,1e-4]), p0 = [lcoefs[0],lcoefs[1],0,0])
            la,lb,ld = lcoefs
            fit_coefs,_ = curve_fit(fit_func, X, Y, bounds=([0.6*la,-np.inf,0,0],[1.4*la,np.inf,np.inf,np.inf]), p0 = [la,lb,(ld-lb)/(2*la),0])
            cal_params = fit_coefs

        elif(self.cmb_cal_type.get() == CAL_TYPE_POLYSPLINE_K):
            fit_func = lambda x,a,b,c, k: np.minimum(x,a)*b + np.maximum(x-a,0)*c + k 
            lcoefs,_ = curve_fit(fit_func, X, Y, bounds=([0, 0, 0, -np.inf],[np.inf, np.inf, np.inf, np.inf]))

            def polyspline_deg2_N2_smooth(x,a,b,c,e,k):
                x1 = np.minimum(x,a)
                x2 = np.maximum(x-a,0)
                d = 2*a*c + b
                return b*x1 + c*x1**2 + d*x2 + e*x2**2 + k

            fit_func = polyspline_deg2_N2_smooth
            # fit_coefs,_ = curve_fit(fit_func, X, Y, bounds=([3e3,0,0,0],[20e3,1e-3,1e-6,1e-4]), p0 = [lcoefs[0],lcoefs[1],0,0])
            la,lb,ld,lk = lcoefs
            fit_coefs,_ = curve_fit(fit_func, X, Y, bounds=([0.6*la,-np.inf,0,0,-np.inf],[1.4*la,np.inf,np.inf,np.inf,np.inf]), p0 = [la,lb,(ld-lb)/(2*la),0,lk])
            cal_params = fit_coefs

        elif(self.cmb_cal_type.get() == CAL_TYPE_POLYSPLINE_ACT):
            act_i = np.argmax(Y>Y[0])
            act_F = Y[act_i]
            # act_G = X[act_i]
            print(f"Activation force: {act_F}")
            # print(f"Activation cond: {act_G}")
            # fit_func = lambda x,a,b,c: np.minimum(x-act_G,a)*b + np.maximum(x-act_G-a,0)*c + act_F
            fit_func = lambda x,a,b,c: np.minimum(x,a)*b + np.maximum(x-a,0)*c + act_F
            lcoefs,_ = curve_fit(fit_func, X, Y, bounds=([0, 0, 0],[np.inf, np.inf, np.inf]))

            def polyspline_deg2_N2_smooth(x,a,b,c,e):
                # x1 = np.minimum(x-act_G,a)
                # x2 = np.maximum(x-act_G-a,0)
                x1 = np.minimum(x,a)
                x2 = np.maximum(x-a,0)
                d = 2*a*c + b
                return b*x1 + c*x1**2 + d*x2 + e*x2**2 + act_F

            fit_func = polyspline_deg2_N2_smooth
            # fit_coefs,_ = curve_fit(fit_func, X, Y, bounds=([3e3,0,0,0],[20e3,1e-3,1e-6,1e-4]), p0 = [lcoefs[0],lcoefs[1],0,0])
            la,lb,ld = lcoefs
            fit_coefs,_ = curve_fit(fit_func, X, Y, bounds=([0.6*la,-np.inf,-np.inf,-np.inf],[1.4*la,np.inf,np.inf,np.inf]), p0 = [la,lb,(ld-lb)/(2*la),0])
            cal_params = np.append(fit_coefs, [act_F])

        elif(self.cmb_cal_type.get() == CAL_TYPE_POLYSPLINE2):
            def polyspline_deg2_N2(x,a,b,c,d,e):
                x1 = np.minimum(x,a)
                x2 = np.maximum(x-a,0)
                return b*x1 + c*x1**2 + d*x2 + e*x2**2

            fit_func = polyspline_deg2_N2
            fit_coefs,_ = curve_fit(fit_func, X, Y, bounds=([3e3,0,0,0,0],[20e3,1e-3,1e-6,1e-2,1e-4]))
            cal_params = fit_coefs

        else:
            return

        print(cal_params)

        # X *= 1e6
        self.ax.plot(X,Y,'.r')
        self.ax.plot(X,fit_func(X, *fit_coefs),'--k')
        self.canvas.draw()

        self.txt_coefsDisplay.configure(state=NORMAL)
        self.txt_coefsDisplay.delete(1.0, "end")
        # self.txt_coefsDisplay.insert(1.0, ' '.join(["%.5f" % x for x in fit_coefs * 1e6**np.arange(len(fit_coefs))[::-1]]))
        # self.txt_coefsDisplay.insert(1.0, ' '.join(["%.5f" % x for x in fit_coefs]))
        self.txt_coefsDisplay.insert(1.0, ' '.join([f"{x:.4e}" for x in cal_params]))
        self.txt_coefsDisplay.configure(state=DISABLED)


    def focusCoefDisplay(self, event):
        self.txt_coefsDisplay.config(state='normal')
        self.txt_coefsDisplay.focus()
        self.txt_coefsDisplay.tag_add(SEL, "1.0", "end-1c")
        self.txt_coefsDisplay.mark_set(INSERT, "1.0")
        self.txt_coefsDisplay.see(INSERT)
        self.txt_coefsDisplay.config(state='disabled')
        return "break"

    def updateUI(self):
        gui.set_enabled(self.ent_y_min, self.var_twinx.get()==0)
        gui.set_enabled(self.ent_y_max, self.var_twinx.get()==0)
        gui.set_enabled(self.chk_multiaxes, self.plot_type==PLOT_TYPE_GRAPH)
        if(self.plot_type==PLOT_TYPE_GRAPH):
            self.cmb_x['state'] = "readonly"
        else:
            self.cmb_x['state'] = "disabled"
        self.btn_on.setState(self.plotting_on)
    
    def initUI(self):
        frame = self
        frame.columnconfigure(0, weight=1)

        frame.rowconfigure(0, weight=0)
        frame.rowconfigure(1, weight=1)
        frame.rowconfigure(2, weight=0)

        frame_header = ttk.Frame(frame)
        self.frame_header = frame_header
        frame_header.grid(row=0,column=0, padx=5, pady=5, sticky=N+S+E+W)

        frame_body = ttk.Frame(frame)
        frame_body.grid(row=1,column=0, padx=5, pady=2, sticky=N+S+E+W)

        frame_foot = ttk.Frame(frame)
        self.frame_footer = frame_foot
        frame_foot.grid(row=2,column=0, padx=5, pady=5, sticky=N+S+E+W)

        ## HEADER ##
        frame_header.columnconfigure(0, weight=1)

        C = 1; R = 1
        tk.Label(frame_header,text="X:").grid(row=R,column=C,sticky=E,padx=(5,5))
        self.cmb_x = ttk.Combobox(frame_header,width=18,values=[PLOT_INDEX_STR],state="readonly")
        self.cmb_x.current(0)
        self.cmb_x.bind('<<ComboboxSelected>>', self.combo_x_selected) 
        self.cmb_x.bind("<Button-1>",           self.combo_x_clicked)
        self.cmb_x .grid(row=R,column=C+1, sticky=EW)
        frame_header.columnconfigure(C+1, weight=10)

        R += 1
        tk.Label(frame_header,text="Y:").grid(row=R,column=C,sticky=E,padx=(5,5))
        # self.ent_y = gui.ListEntry(frame_header, width=20, allow_zero=False, on_edit=self.reinitialize_plot)
        # self.ent_y.grid(row=R,column=2)
        self.cmb_y = gui.MultiItemDropdown(frame_header, width=18, on_open=self.combo_y_will_show, on_select=self.combo_y_selected)
        self.cmb_y.grid(row=R, column=C+1, sticky=EW)

        R += 1
        tk.Label(frame_header,text="Plot:").grid(row=R,column=C,sticky=E,padx=(5,5))
        self.cmb_plot_type = ttk.Combobox(frame_header, width=18, values=PLOT_TYPES, state="readonly")
        self.cmb_plot_type.bind('<<ComboboxSelected>>', self.combo_plot_selected) 
        self.cmb_plot_type.current(self.cmb_plot_type["values"].index(self.plot_type))
        self.cmb_plot_type.grid(row=R,column=C+1, sticky=EW)

        # Limits
        C = 3; R = 1
        frame_header.columnconfigure(C, weight=3)
        frame_header.columnconfigure(C+1, weight=0)
        frame_header.columnconfigure(C+2, weight=0)
        tk.Label(frame_header,text="x limits:").grid(row=R,column=C,sticky=E,padx=(5,0))
        self.ent_x_min = gui.NumberEntry(frame_header, width=4, on_edit=self.update_limits)
        self.ent_x_min.grid(row=R,column=C+1,sticky=EW)
        self.ent_x_max = gui.NumberEntry(frame_header, width=4, on_edit=self.update_limits)
        self.ent_x_max.grid(row=R,column=C+2,sticky=W)
        R += 1
        tk.Label(frame_header,text="y limits:").grid(row=R,column=C,sticky=E,padx=(5,0))
        self.ent_y_min = gui.NumberEntry(frame_header, width=4, on_edit=self.update_limits)
        self.ent_y_min.grid(row=R,column=C+1,sticky=EW)
        self.ent_y_max = gui.NumberEntry(frame_header, width=4, on_edit=self.update_limits)
        self.ent_y_max.grid(row=R,column=C+2,sticky=W)

        # C = 3; R = 3
        # self.var_plot_index = tk.DoubleVar()
        # self.sld_plot_index = ttk.Scale(frame_header, from_=0, to_=1, orient="horizontal", length=140, variable=self.var_plot_index)
        # self.sld_plot_index.grid(row=R, column=C, columnspan=4, sticky=EW, padx=20)
        # frame_header.columnconfigure(C+3, weight=3)
        # frame_header.columnconfigure(C+4, weight=10)

        # C = 6; R = 2
        R += 1
        self.var_twinx = tk.IntVar()
        self.chk_multiaxes = Checkbutton(frame_header, text="Multi axes", variable=self.var_twinx, command=self.multi_axes_clicked)
        self.chk_multiaxes.grid(row=R, column=C, columnspan=3)        

        frame_header.columnconfigure(C+3, weight=10)

        # Buttons
        r=1; c=7
        w = 10
        frame_header.columnconfigure(c, weight=0)
        self.btn_on = gui.ToggleButton(frame_header, width=w, text_on="Update plot", text_off="Update plot", command=self.toggle_on_clicked)
        self.btn_on.grid(row=r, column=c)
        r+=1
        frame_header.columnconfigure(c, weight=0)
        self.btn_clear = Button(frame_header, text="Clear", width=w, command=self.clear_clicked)
        self.btn_clear.grid(row=r, column=c)
        r+=1
        frame_header.columnconfigure(c, weight=0)
        self.btn_export = Button(frame_header, text="Save plot", width=w, command=self.export_clicked)
        self.btn_export.grid(row=r, column=c)

        ## BODY ##
        frame_body.columnconfigure(0, weight=1)
        frame_body.rowconfigure(0, weight=1)

        self.fig = Figure(figsize=(6,6))
        self.ax:Axes = self.fig.add_subplot(111)
        self.canvas = FigureCanvasTkAgg(self.fig, master=frame_body)
        # self.canvas.get_tk_widget().pack(padx=30, pady=30)
        self.canvas.get_tk_widget().grid(column=0, row=0, sticky=tk.NSEW, padx=5, pady=5)
        self.canvas.draw()

        ## FOOTER ##
        frame_foot.rowconfigure(0, weight=0)

        # r=0; c=0
        # frame_foot.columnconfigure(c, weight=0)
        # self.btn_on = gui.ToggleButton(frame_foot, text_on="Update plot", text_off="Update plot", command=self.toggle_on_clicked)
        # # self.btn_on = Button(frame_foot,text="Off", width=5, relief="raised", command=self.toggle_on_clicked)
        # self.btn_on.grid(row=r, column=c)
        # c+=1
        # frame_foot.columnconfigure(c, weight=0)
        # self.btn_clear = Button(frame_foot, text="Clear", command=self.clear_clicked)
        # self.btn_clear.grid(row=r, column=c)
        # c+=1
        # frame_foot.columnconfigure(c, weight=0)
        # self.btn_export = Button(frame_foot, text="Save plot", command=self.export_clicked)
        # self.btn_export.grid(row=r, column=c, padx=(10,0))

        c+=1
        frame_foot.columnconfigure(c, weight=3)
        self.txt_coefsDisplay = Text(frame_foot, height=1, width=40, relief="flat", borderwidth=0, background=self.winfo_toplevel().cget('bg'), state=DISABLED)
        self.txt_coefsDisplay.configure(inactiveselectbackground=self.txt_coefsDisplay.cget("selectbackground"))
        self.txt_coefsDisplay.grid(row=r, column=c, sticky=EW, padx=(30,20))
        self.txt_coefsDisplay.bind('<Button-1>', self.focusCoefDisplay)

        c+=1
        frame_foot.columnconfigure(c, weight=1)
        self.cmb_cal_type = ttk.Combobox(frame_foot, width=12, values=CAL_TYPES, state="readonly")
        self.cmb_cal_type.grid(row=r, column=c, sticky=E, padx=5)

        c+=1
        frame_foot.columnconfigure(c, weight=0)
        self.btn_calib = Button(frame_foot, text="Cal", command=self.calib_clicked)
        self.btn_calib.grid(row=r, column=c, sticky=E)

    def save_UI_state(self, db):
        db["graph_gui"] = {

            "ent_x_min":self.ent_x_min.get(),
            "ent_x_max":self.ent_x_max.get(),
            "ent_y_min":self.ent_y_min.get(),
            "ent_y_max":self.ent_y_max.get(),

            "cmb_plot_type":self.cmb_plot_type.get(),
            "cmb_cal_type":self.cmb_cal_type.get(),
            "cmb_y.items":self.cmb_y.getItems(),
            "cmb_y.selected":self.cmb_y.getSelectedIndices(),
        }
        return db

    def load_UI_state(self, db):
        d = db["graph_gui"]
        if(d==None): return

        gui.setText(self.ent_x_min, d["ent_x_min"])
        gui.setText(self.ent_x_max, d["ent_x_max"])
        gui.setText(self.ent_y_min, d["ent_y_min"])
        gui.setText(self.ent_y_max, d["ent_y_max"])
        
        self.cmb_plot_type.set(d["cmb_plot_type"])
        self.combo_plot_selected(None)
        # self.cmb_plot_type.current(self.cmb_plot_type['values'].index(d["cmb_plot_type"]))
        self.cmb_cal_type.set(d["cmb_cal_type"])

        # self.combo_y_will_show(None) # update item list
        print(d["cmb_y.items"])
        print(d["cmb_y.selected"])
        self.cmb_y.setItems(d["cmb_y.items"])
        self.cmb_y.setSelectedIndices(d["cmb_y.selected"])
        print(f"GraphGUI - loadUI - cmbY: {self.cmb_y.getSelectedItems()}")

    def toggle_header_footer(self):
        self.show_header_footer(not self._show_header_footer)

    def show_header_footer(self, b):
        self._show_header_footer = b
        if(b):
            self.frame_header.grid(row=0,column=0, padx=5, pady=5, sticky=N+S+E+W)
            self.frame_footer.grid(row=2,column=0, padx=5, pady=5, sticky=N+S+E+W)
        else:
            self.frame_header.grid_forget()
            self.frame_footer.grid_forget()

if __name__ == "__main__":
    window = tk.Tk()
    window.title("Graph")
    window.style = ttk.Style()
    window.style.theme_use("alt")
    sensor = SensorInterface.SensorInterface()
    sampler = Sampler.Sampler(sensor)
    sf = GraphFrame(window, sampler, OsInterface(window))
    sf.pack(fill=BOTH, expand=True)

    window.mainloop()
    sensor.shutdown()
    sampler.shutdown()
