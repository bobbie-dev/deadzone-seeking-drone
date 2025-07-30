import plotly.graph_objects as go
import plotly.express as px
from plotly.subplots import make_subplots
import pandas as pd
import numpy as np
from datetime import datetime, timedelta
import random

# Generate fake cellular measurement data
def generate_fake_cellular_data(num_points=200):
    """Generate realistic fake cellular network measurement data"""
    
    # Base location (San Francisco area)
    base_lat = 37.7749
    base_lon = -122.4194
    
    data = []
    start_time = datetime.now() - timedelta(hours=2)
    
    for i in range(num_points):
        # Simulate movement in a area
        lat_offset = random.uniform(-0.05, 0.05)
        lon_offset = random.uniform(-0.05, 0.05)
        
        # Simulate altitude variation (0-500m)
        altitude = random.uniform(0, 500)
        
        # Simulate cellular signal parameters with realistic ranges
        rssi = random.uniform(-110, -50)  # dBm
        rsrp = random.uniform(-120, -70)  # dBm  
        rsrq = random.uniform(-20, -5)    # dB
        sinr = random.uniform(-5, 30)     # dB
        
        # Simulate cell tower info
        cell_id = random.choice([12345, 23456, 34567, 45678, 56789])
        pci = random.randint(0, 503)  # Physical Cell ID range
        ber = random.uniform(0, 0.1)  # Bit Error Rate
        
        timestamp = start_time + timedelta(seconds=i*36)  # Every 36 seconds
        
        data.append({
            'packet_counter': i,
            'lat_int': base_lat + lat_offset,
            'lon_int': base_lon + lon_offset,
            'alt_int': altitude,
            'rssi': rssi,
            'rsrp': rsrp,
            'rsrq': rsrq,
            'sinr': sinr,
            'cell_id': cell_id,
            'pci': pci,
            'ber': ber,
            'timestamp': timestamp
        })
    
    return pd.DataFrame(data)

# Generate the fake data
df = generate_fake_cellular_data()

# 1. Time Series Plot of Signal Quality Metrics
fig_timeseries = make_subplots(
    rows=4, cols=1,
    subplot_titles=['RSSI (dBm)', 'RSRP (dBm)', 'RSRQ (dB)', 'SINR (dB)'],
    vertical_spacing=0.08,
    shared_xaxes=True
)

# Add traces for each metric
fig_timeseries.add_trace(
    go.Scatter(x=df['timestamp'], y=df['rssi'], name='RSSI', line=dict(color='red')),
    row=1, col=1
)
fig_timeseries.add_trace(
    go.Scatter(x=df['timestamp'], y=df['rsrp'], name='RSRP', line=dict(color='blue')),
    row=2, col=1
)
fig_timeseries.add_trace(
    go.Scatter(x=df['timestamp'], y=df['rsrq'], name='RSRQ', line=dict(color='green')),
    row=3, col=1
)
fig_timeseries.add_trace(
    go.Scatter(x=df['timestamp'], y=df['sinr'], name='SINR', line=dict(color='orange')),
    row=4, col=1
)

fig_timeseries.update_layout(
    title='Cellular Signal Quality Over Time',
    height=800,
    showlegend=False
)
fig_timeseries.show()

# 2. RSSI Signal Strength Map
fig_rssi_map = px.scatter_mapbox(
    df, 
    lat='lat_int', 
    lon='lon_int',
    color='rssi',
    size=abs(df['rssi']/10),  # Size based on signal strength
    hover_data=['rsrp', 'rsrq', 'sinr', 'cell_id', 'pci', 'alt_int'],
    color_continuous_scale='RdYlGn_r',  # Red for weak, Green for strong
    title='RSSI Signal Strength Map',
    mapbox_style='open-street-map',
    height=700,
    zoom=11,
    center=dict(lat=df['lat_int'].mean(), lon=df['lon_int'].mean())
)
fig_rssi_map.update_traces(
    hovertemplate='<b>Signal Quality</b><br>' +
                  'RSSI: %{color:.1f} dBm<br>' +
                  'RSRP: %{customdata[0]:.1f} dBm<br>' +
                  'RSRQ: %{customdata[1]:.1f} dB<br>' +
                  'SINR: %{customdata[2]:.1f} dB<br>' +
                  'Cell ID: %{customdata[3]}<br>' +
                  'PCI: %{customdata[4]}<br>' +
                  'Altitude: %{customdata[5]:.0f}m<br>' +
                  '<extra></extra>'
)
fig_rssi_map.show()

# 3. Cell Tower Coverage Map
fig_cell_map = px.scatter_mapbox(
    df, 
    lat='lat_int', 
    lon='lon_int',
    color='cell_id',
    size='sinr',
    hover_data=['rssi', 'rsrp', 'rsrq', 'pci', 'timestamp'],
    title='Cell Tower Coverage Map',
    mapbox_style='open-street-map',
    height=700,
    zoom=11,
    center=dict(lat=df['lat_int'].mean(), lon=df['lon_int'].mean())
)
fig_cell_map.update_traces(
    hovertemplate='<b>Cell Tower Info</b><br>' +
                  'Cell ID: %{color}<br>' +
                  'SINR: %{marker.size:.1f} dB<br>' +
                  'RSSI: %{customdata[0]:.1f} dBm<br>' +
                  'RSRP: %{customdata[1]:.1f} dBm<br>' +
                  'RSRQ: %{customdata[2]:.1f} dB<br>' +
                  'PCI: %{customdata[3]}<br>' +
                  '<extra></extra>'
)
fig_cell_map.show()

# 4. Signal Quality Heatmap (using density mapbox)
fig_heatmap = px.density_mapbox(
    df, 
    lat='lat_int', 
    lon='lon_int',
    z='rssi',
    radius=15,
    title='Signal Strength Density Heatmap',
    mapbox_style='open-street-map',
    height=700,
    zoom=11,
    center=dict(lat=df['lat_int'].mean(), lon=df['lon_int'].mean())
)
fig_heatmap.show()

# 5. Path/Route Map with Signal Quality
# Sort by timestamp to show the path taken
df_sorted = df.sort_values('timestamp')

fig_path = go.Figure()

# Add the path line
fig_path.add_trace(go.Scattermapbox(
    mode="lines+markers",
    lon=df_sorted['lon_int'],
    lat=df_sorted['lat_int'],
    marker=dict(
        size=8,
        color=df_sorted['rssi'],
        colorscale='RdYlGn_r',
        showscale=True,
        colorbar=dict(title="RSSI (dBm)")
    ),
    line=dict(width=2, color='blue'),
    text=[f'Time: {ts}<br>RSSI: {rssi:.1f} dBm<br>Cell: {cid}' 
          for ts, rssi, cid in zip(df_sorted['timestamp'], df_sorted['rssi'], df_sorted['cell_id'])],
    hovertemplate='<b>Measurement Point</b><br>' +
                  '%{text}<br>' +
                  'Location: (%{lat:.4f}, %{lon:.4f})<br>' +
                  '<extra></extra>',
    name='Signal Path'
))

fig_path.update_layout(
    mapbox=dict(
        style='open-street-map',
        center=dict(lat=df['lat_int'].mean(), lon=df['lon_int'].mean()),
        zoom=11
    ),
    title='Signal Quality Along Path',
    height=700
)
fig_path.show()

# 6. Multi-layer Map with All Metrics
fig_multi = make_subplots(
    rows=2, cols=2,
    subplot_titles=['RSSI Map', 'RSRP Map', 'RSRQ Map', 'SINR Map'],
    specs=[[{"type": "mapbox"}, {"type": "mapbox"}],
           [{"type": "mapbox"}, {"type": "mapbox"}]],
    vertical_spacing=0.1
)

# RSSI
fig_multi.add_trace(
    go.Scattermapbox(
        lat=df['lat_int'], lon=df['lon_int'],
        mode='markers',
        marker=dict(size=6, color=df['rssi'], colorscale='RdYlGn_r', showscale=True),
        text=df['rssi'].round(1),
        name='RSSI'
    ), row=1, col=1
)

# RSRP  
fig_multi.add_trace(
    go.Scattermapbox(
        lat=df['lat_int'], lon=df['lon_int'],
        mode='markers',
        marker=dict(size=6, color=df['rsrp'], colorscale='RdYlGn_r'),
        text=df['rsrp'].round(1),
        name='RSRP'
    ), row=1, col=2
)

# RSRQ
fig_multi.add_trace(
    go.Scattermapbox(
        lat=df['lat_int'], lon=df['lon_int'],
        mode='markers',
        marker=dict(size=6, color=df['rsrq'], colorscale='RdYlGn_r'),
        text=df['rsrq'].round(1),
        name='RSRQ'
    ), row=2, col=1
)

# SINR
fig_multi.add_trace(
    go.Scattermapbox(
        lat=df['lat_int'], lon=df['lon_int'],
        mode='markers',
        marker=dict(size=6, color=df['sinr'], colorscale='RdYlGn_r'),
        text=df['sinr'].round(1),
        name='SINR'
    ), row=2, col=2
)

# Update mapbox settings for all subplots
center_lat, center_lon = df['lat_int'].mean(), df['lon_int'].mean()
for i in range(1, 3):
    for j in range(1, 3):
        fig_multi.update_layout({
            f'mapbox{i*2+j-2 if i*j > 1 else ""}': dict(
                style='open-street-map',
                center=dict(lat=center_lat, lon=center_lon),
                zoom=10
            )
        })

fig_multi.update_layout(
    title='All Signal Metrics on Maps',
    height=800,
    showlegend=False
)
fig_multi.show()

# 4. Cell Tower Analysis
cell_stats = df.groupby('cell_id').agg({
    'rssi': ['mean', 'count'],
    'rsrp': 'mean',
    'sinr': 'mean'
}).round(2)

# Flatten column names
cell_stats.columns = ['_'.join(col).strip() for col in cell_stats.columns]
cell_stats = cell_stats.reset_index()

fig_cells = px.bar(
    cell_stats, 
    x='cell_id', 
    y='rssi_mean',
    color='rssi_count',
    title='Average Signal Strength by Cell Tower',
    labels={'rssi_mean': 'Average RSSI (dBm)', 'rssi_count': 'Measurements'},
    text='rssi_mean'
)
fig_cells.update_traces(texttemplate='%{text:.1f}', textposition='outside')
fig_cells.show()

# 5. Signal Quality Correlation Matrix
import plotly.figure_factory as ff

signal_metrics = df[['rssi', 'rsrp', 'rsrq', 'sinr', 'ber']].corr()

fig_corr = ff.create_annotated_heatmap(
    z=signal_metrics.values,
    x=list(signal_metrics.columns),
    y=list(signal_metrics.index),
    annotation_text=signal_metrics.round(2).values,
    colorscale='RdBu',
    showscale=True
)
fig_corr.update_layout(title='Signal Metrics Correlation Matrix')
fig_corr.show()

# 6. Signal Quality Distribution
fig_dist = make_subplots(
    rows=2, cols=2,
    subplot_titles=['RSSI Distribution', 'RSRP Distribution', 
                   'RSRQ Distribution', 'SINR Distribution']
)

fig_dist.add_trace(go.Histogram(x=df['rssi'], name='RSSI'), row=1, col=1)
fig_dist.add_trace(go.Histogram(x=df['rsrp'], name='RSRP'), row=1, col=2)
fig_dist.add_trace(go.Histogram(x=df['rsrq'], name='RSRQ'), row=2, col=1)
fig_dist.add_trace(go.Histogram(x=df['sinr'], name='SINR'), row=2, col=2)

fig_dist.update_layout(
    title='Signal Quality Metrics Distribution',
    showlegend=False,
    height=600
)
fig_dist.show()

# Print sample data
print("Sample of generated data:")
print(df.head(10))
print(f"\nTotal data points: {len(df)}")
print(f"Time range: {df['timestamp'].min()} to {df['timestamp'].max()}")
print(f"Unique cell IDs: {df['cell_id'].nunique()}")