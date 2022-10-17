# https://datascience.stackexchange.com/questions/6084/how-do-i-create-a-complex-radar-chart

import numpy as np
import matplotlib.pyplot as plt
#import seaborn as sns # improves plot aesthetics
#sns.set_theme()

def _invert(x, limits):
    """inverts a value x on a scale from
    limits[0] to limits[1]"""
    return limits[1] - (x - limits[0])

def _scale_data(data, ranges):
    """scales data[1:] to ranges[0],
    inverts if the scale is reversed"""
    # for d, (y1, y2) in zip(data[1:], ranges[1:]):
    for d, (y1, y2) in zip(data, ranges):
        # assert (y1 <= d <= y2) or (y2 <= d <= y1)
        if not ((y1 <= d <= y2) or (y2 <= d <= y1)):
            print(f"{y1} <= {d} <= {y2} || {y2} <= {d} <= {y1} fails")
            return

    x1, x2 = ranges[0]
    d = data[0]

    if x1 > x2:
        d = _invert(d, (x1, x2))
        x1, x2 = x2, x1

    sdata = [d]

    for d, (y1, y2) in zip(data[1:], ranges[1:]):
        if y1 > y2:
            d = _invert(d, (y1, y2))
            y1, y2 = y2, y1

        sdata.append((d-y1) / (y2-y1) * (x2 - x1) + x1)

    return sdata

def set_rgrids(self, radii, labels=None, angle=None, fmt=None,
               **kwargs):
    """
    Set the radial locations and labels of the *r* grids.
    The labels will appear at radial distances *radii* at the
    given *angle* in degrees.
    *labels*, if not None, is a ``len(radii)`` list of strings of the
    labels to use at each radius.
    If *labels* is None, the built-in formatter will be used.
    Return value is a list of tuples (*line*, *label*), where
    *line* is :class:`~matplotlib.lines.Line2D` instances and the
    *label* is :class:`~matplotlib.text.Text` instances.
    kwargs are optional text properties for the labels:
    %(Text)s
    ACCEPTS: sequence of floats
    """
    # Make sure we take into account unitized data
    radii = self.convert_xunits(radii)
    radii = np.asarray(radii)
    rmin = radii.min()
    # if rmin <= 0:
    #     raise ValueError('radial grids must be strictly positive')

    self.set_yticks(radii)
    if labels is not None:
        self.set_yticklabels(labels)
    elif fmt is not None:
        self.yaxis.set_major_formatter(FormatStrFormatter(fmt))
    if angle is None:
        angle = self.get_rlabel_position()
    self.set_rlabel_position(angle)
    for t in self.yaxis.get_ticklabels():
        t.update(kwargs)
    return self.yaxis.get_gridlines(), self.yaxis.get_ticklabels()

class ComplexRadar():
    def __init__(self, fig, variables, ranges, n_ordinate_levels=6):

        angles = np.arange(0, 360, 360./len(variables))

        M = 0.1 # margin
        axes = [fig.add_axes([M,M,1-2*M,1-2*M], polar=True, label = "axes{}".format(i)) for i in range(len(variables))]
        
        l, text = axes[0].set_thetagrids(angles, labels=variables)

        # Rotate labels
        labels = []
        for label, angle in zip(axes[0].get_xticklabels(), angles):
            x,y = label.get_position()
            lab = axes[0].text(x,y, label.get_text(), transform=label.get_transform(),
                        ha=label.get_ha(), va=label.get_va())
            lab.set_rotation(angle-90)
            labels.append(lab)
        axes[0].set_xticklabels([])

        for ax in axes[1:]:
            ax.patch.set_visible(False)
            ax.grid("off")
            ax.xaxis.set_visible(False)

        for i, ax in enumerate(axes):
            grid = np.linspace(*ranges[i], num=n_ordinate_levels)
            gridlabel = ["{}".format(round(x,2)) for x in grid]
            if ranges[i][0] > ranges[i][1]:
                grid = grid[::-1] # hack to invert grid
                          # gridlabels aren't reversed
            gridlabel[0] = "" # clean up origin
            #ax.set_rgrids(grid, labels=gridlabel, angle=angles[i])
            set_rgrids(ax, grid, labels=gridlabel, angle=angles[i])
            ax.spines["polar"].set_visible(False)
            ax.set_ylim(*ranges[i])
        # variables for plotting
        self.angle = np.deg2rad(np.r_[angles, angles[0]])
        self.ranges = ranges
        self.ax = axes[0]

    def plot(self, data, *args, **kw):
        sdata = _scale_data(data, self.ranges)
        self.ax.plot(self.angle, np.r_[sdata, sdata[0]], *args, **kw)

    def fill(self, data, *args, **kw):
        sdata = _scale_data(data, self.ranges)
        self.ax.fill(self.angle, np.r_[sdata, sdata[0]], *args, **kw)

    def clear(self):
        #self.ax.clear()
        for line in self.ax.get_lines():
            line.remove()

if __name__ == "__main__":
    # example data
    variables = ("Width", "Hardness", "Contact Area", "Curvature", "Smoothness")
    ranges = [(0, 100), (0, 20), (0, 1600), (0, 10), (0, 10)]
    data = (65, 14, 400, 4, 5)
    # plotting
    fig1 = plt.figure(figsize=(6, 6))
    radar = ComplexRadar(fig1, variables, ranges)
    radar.plot((63, 7, 200, 3, 2), 'g')
    radar.plot((67, 20, 500, 5, 6), 'g')
    radar.plot(data)
    radar.fill(data, alpha=0.2)
    plt.show()