import matplotlib.pyplot as plt
import matplotlib as mpl
import pylab
from numpy import pi, cos, sin
import math

def Graph_01 (x, y, name_x, name_y, name_graph, label, filename):
    fig = plt.figure()
    plt.plot(x,y,label=label, linewidth=2)
    plt.xlabel(name_x, fontsize = 13)
    plt.ylabel(name_y, fontsize = 13)
    plt.title(name_graph, fontsize = 13)
    plt.legend()
    plt.grid(b=None, which='major', axis='both', color = 'black', alpha = 0.4)
    plt.rcParams['pdf.fonttype'] = 42
    plt.rcParams['font.family'] = 'Calibri'
    plt.show()
    fig.savefig(filename + '.pdf')

def Graph_02(x1, y1, x2, y2, name_x, name_y, name_graph, label1, label2, filename):
    fig = plt.figure()
    plt.plot(x1, y1, color = 'r', label = label1, linewidth=2)
    plt.plot(x2, y2, label = label2, color='b', alpha=0.5, linewidth=2)
    plt.xlabel(name_x, fontsize = 13)
    plt.ylabel(name_y, fontsize = 13)
    plt.title(name_graph, fontsize = 13)
    plt.legend()
    plt.grid(b=None, which='major', axis='both', color = 'black', alpha = 0.4)
    plt.show()
    fig.savefig(filename + '.pdf')

def Graph_04(x1, y1, x2, y2, x3, y3, x4, y4, name_x, name_y, name_graph, label1, label2, label3, label4, filename):
    fig = plt.figure()
    plt.plot(x1, y1, color = 'r', label = label1, linewidth=2)
    plt.plot(x2, y2, label = label2, color='g', alpha=0.5, linewidth=2)
    plt.plot(x3, y3, label = label3, color='m', alpha=0.5, linewidth=2)
    plt.plot(x4, y4, label = label4, color='c', alpha=0.5, linewidth=2)
    plt.xlabel(name_x, fontsize = 13)
    plt.ylabel(name_y, fontsize = 13)
    plt.title(name_graph, fontsize = 13)
    plt.legend()
    plt.grid(b=None, which='major', axis='both', color = 'black', alpha = 0.4)
    plt.show()
    fig.savefig(filename + '.pdf')
