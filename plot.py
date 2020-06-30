import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

BBox1 = ((-75.71293, -75.70849,45.37282, 45.37517))
BBox2 = ((-75.58720,-75.58290,45.39984,45.40199))
BBox3 = ((-75.60109,-75.59885,45.39639,45.39516))

j = 0
def plot(dataset):

    map = plt.imread('C:/Users/baoti/4th/zhixian2.PNG')
    fig, ax = plt.subplots(figsize = (8,7))
    for i in dataset:
        print (i[0],i[1])
        ax.scatter( i[1],i[0], zorder=2, alpha= 0.8, marker = 'o', c='r', s=10)
    ax.set_title('different result due to heading difference')
    ax.set_xlim(BBox1[0],BBox1[1])
    ax.set_ylim(BBox1[2],BBox1[3])
    ax.imshow(map, zorder=0, extent = BBox1, aspect= 'equal')
    plt.show()

def plotTurn(dataset):
    map = plt.imread('C:/Users/baoti/4th/CaptureTurn.PNG')
    fig, ax = plt.subplots(figsize=(8, 7))
    for i in dataset:
        print(i[1])
        ax.scatter(i[0], i[1], zorder=1, alpha=0.8, marker='.', c='k', s=100)
    ax.set_title('Plotting Predicted Data on Real Map')
    ax.set_xlim(BBox2[0], BBox2[1])
    ax.set_ylim(BBox2[2], BBox2[3])
    ax.imshow(map, zorder=0, extent=BBox2, aspect='equal')
    plt.show()

def plotTurn2(dataset):
    map = plt.imread('C:/Users/baoti/4th/CaptureTurn2.PNG')
    fig, ax = plt.subplots(figsize=(8, 7))
    for i in dataset:
        #print(i[0])
        ax.scatter(i[1], i[0], zorder=1, alpha=0.8, marker='.', c='k', s=100)
    ax.set_title('Plotting Predicted Data on Real Map')
    ax.set_xlim(BBox3[0], BBox3[1])
    ax.set_ylim(BBox3[2], BBox3[3])
    ax.imshow(map, zorder=0, extent=BBox3, aspect='equal')
    plt.show()
