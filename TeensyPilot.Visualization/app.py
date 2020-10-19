# -*- coding: utf-8 -*-

# Run this app with `python app.py` and
# visit http://127.0.0.1:8050/ in your web browser.

import dash
import dash_core_components as dcc
import dash_html_components as html
import plotly.express as px
import plotly.graph_objects as go
import pandas as pd

external_stylesheets = ['https://codepen.io/chriddyp/pen/bWLwgP.css']

app = dash.Dash(__name__, external_stylesheets=external_stylesheets)

# assume you have a "long-form" data frame
# see https://plotly.com/python/px-arguments/ for more options

df = pd.read_csv("TeensyPilot.Visualization/data.csv")

fig1 = px.line(df, x="t", y="fr", color_discrete_sequence=px.colors.qualitative.Pastel, width=1900, height=1000)
fig1.add_trace(px.line(df, x="t", y="fl", color_discrete_sequence=px.colors.qualitative.Pastel1).data[0])
fig1.add_trace(px.line(df, x="t", y="bl", color_discrete_sequence=px.colors.qualitative.Prism).data[0])
fig1.add_trace(px.line(df, x="t", y="br", color_discrete_sequence=px.colors.qualitative.Set1).data[0])
fig1.add_trace(px.line(df, x="t", y="pitch", color_discrete_sequence=px.colors.qualitative.Set2).data[0])
fig1.add_trace(px.line(df, x="t", y="roll", color_discrete_sequence=px.colors.qualitative.Set3).data[0])
fig1.add_trace(px.line(df, x="t", y="yaw", color_discrete_sequence=px.colors.qualitative.Vivid).data[0])
fig1.add_trace(px.line(df, x="t", y="pp", color_discrete_sequence=px.colors.qualitative.Antique).data[0])
fig1.add_trace(px.line(df, x="t", y="pi", color_discrete_sequence=px.colors.qualitative.Bold).data[0])
fig1.add_trace(px.line(df, x="t", y="pd", color_discrete_sequence=px.colors.qualitative.D3).data[0])
fig1.add_trace(px.line(df, x="t", y="rp", color_discrete_sequence=px.colors.qualitative.Dark2).data[0])
fig1.add_trace(px.line(df, x="t", y="ri", color_discrete_sequence=px.colors.qualitative.Pastel2).data[0])
fig1.add_trace(px.line(df, x="t", y="rd", color_discrete_sequence=px.colors.qualitative.T10).data[0])
fig1.add_trace(px.line(df, x="t", y="ch3", color_discrete_sequence=px.colors.qualitative.Pastel).data[0])

app.layout = html.Div(children=[

    dcc.Graph(
        id='example-graph',
        figure=fig1
    )
], style={ "height" : "100%",'width': "100%"})

if __name__ == '__main__':
    app.run_server(debug=True)