# -*- coding: utf-8 -*-
"""
Created on Sun Dec  1 18:43:57 2019

@author: marti
"""

import pandas as pd
import plotly.graph_objects as go
from plotly.offline import plot


df = pd.read_csv('points.csv', header=None, names=['x', 'y', 'z'])
fig = go.Figure(data=[go.Scatter3d(x=df['x'], y=df['y'], z=df['z'],
                                   mode='markers', marker=dict(size=2))])


plot(fig, filename='pointcloud.html', auto_open=False)