"""One figure with velocity, relative depth and thrust for all three trials."""
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

from .far_plots import chart

TITLES = {'free_sink':'Free sinking', 'hover_balance':'Open-loop balance', 'bidirectional':'Bidirectional heave'}


def panels(rows, metrics):
    stamps = [metrics['start_time_s']] + [r['t_s'] for r in rows]
    for field,unit,initial,label in [('velocity_up','m_s',metrics['start_velocity_up_m_s'],'Upward velocity (m/s)'),
                                    ('depth_change','m',0.,'Depth change, down positive (m)')]:
        lines = []
        for source,name,color in [('gazebo','Gazebo odometry','#007477'),('theory','1D theory','#ad4a21')]:
            values = [initial]+[r[f'{field}_{source}_{unit}'] for r in rows]
            lines.append((name,color,list(zip(stamps,values))))
        yield label,lines
    lines = []
    for field,name,color in [('published_world_up_N','Published, world up','#345987'),
                             ('requested_body_z_N','Request, body z','#a04c1c')]:
        lines.append((name,color,[(r[key],r[field]) for r in rows for key in ('command_start_s','command_end_s')]))
    yield 'Vertical thrust command (N)',lines


def plot(out, traces):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        plot_svg(out,traces)
        return
    fig, axes = plt.subplots(3,3,sharex=True,figsize=(12.5,7.8),constrained_layout=True)
    for col,(case,(rows,m)) in enumerate(traces.items()):
        axes[0,col].set_title(TITLES[case])
        for row,(label,lines) in enumerate(panels(rows,m)):
            ax = axes[row,col]
            for i,(name,color,points) in enumerate(lines):
                ax.plot(*zip(*points),label=name,color=color,ls='--' if i else '-',lw=1.6)
            if col==0:
                ax.set_ylabel(label)
            ax.legend(frameon=False,fontsize=8,loc='best')
            ax.grid(alpha=.2)
            ax.ticklabel_format(useOffset=False,axis='y')
            for a,b in zip(rows,rows[1:]):
                if a['phase'] != b['phase']:
                    ax.axvline(b['command_start_s'],color='.6',ls=':',lw=.8)
        axes[-1,col].set(xlabel='Simulation time (s)',xlim=(min(0,m['start_time_s']),rows[-1]['command_end_s']))
    fig.savefig(out/'heave_response.svg')
    fig.savefig(out/'heave_response.png',dpi=180)
    plt.close(fig)


def plot_svg(out,traces):
    """Portable vector fallback without Matplotlib. Scale small depth/speed signals to mm."""
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp)/'panel.svg'
        root = ET.Element('svg',xmlns='http://www.w3.org/2000/svg')
        for col,(case,(rows,m)) in enumerate(traces.items()):
            boundaries = [b['command_start_s'] for a,b in zip(rows,rows[1:]) if a['phase']!=b['phase']]
            for row,(label,lines) in enumerate(panels(rows,m)):
                factor = 1000 if row<2 else 1
                series = [(name,color,'stroke-dasharray="6 4"' if i else '',[(x,factor*y) for x,y in points])
                          for i,(name,color,points) in enumerate(lines)]
                chart(path,TITLES[case],'Simulation time (s)',label.replace('(m','(mm'),series,boundaries=boundaries)
                panel = ET.parse(path).getroot()
                width,height = int(panel.get('width')),int(panel.get('height'))
                panel.set('x',str(col*width))
                panel.set('y',str(row*height))
                root.append(panel)
        root.set('width',str(3*width))
        root.set('height',str(3*height))
        ET.ElementTree(root).write(out/'heave_response.svg',encoding='utf-8',xml_declaration=True)
