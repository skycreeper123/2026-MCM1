from pathlib import Path
import json,math
import numpy as np
from PIL import Image,ImageDraw,ImageFont

HERE=Path(__file__).parent;DATA=HERE.parents[2]/'B/q2/validation_results_v3'
maps=json.loads((DATA/'visualization_data.json').read_text())['maps']
records=json.loads((DATA/'paired_results.json').read_text())['records']
# 2000 px at 16 cm; a 40 px font corresponds to 9.1 pt in the Word page.
FONT='C:/Windows/Fonts/simsun.ttc'
def font(n=40):return ImageFont.truetype(FONT,n)
def text(draw,xy,s,size=40,anchor='mm',fill='#111111'):
    draw.text(xy,str(s),font=font(size),fill=fill,anchor=anchor)
def line(draw,xy,fill='#687780',width=3):draw.line(xy,fill=fill,width=width)
def star(draw,x,y,r=16):
    p=[(x+(r if k%2==0 else r*.42)*math.cos(-math.pi/2+k*math.pi/5),y+(r if k%2==0 else r*.42)*math.sin(-math.pi/2+k*math.pi/5)) for k in range(10)]
    draw.polygon(p,fill='white',outline='#203646',width=3)
colors=np.array([[68,1,84],[59,82,139],[33,145,140],[94,201,98],[253,231,37]],float)
def cmap(t):
    a=np.clip(np.asarray(t),0,1)*4;k=np.minimum(a.astype(int),3);w=a-k
    return (colors[k]*(1-w[...,None])+colors[k+1]*w[...,None]).astype(np.uint8)
def heatmap():
    im=Image.new('RGB',(2000,1000),'white');draw=ImageDraw.Draw(im)
    for panel,(name,title) in enumerate([('wide_fixed','大范围案例'),('narrow_fixed','边界案例')]):
        m=maps[name];f=m['field'];points=np.array(f['points']);values=np.array(f['radius_upper_m']);tris=np.array(f['triangles'])
        first=np.array(list(m['first']['position'].values()));v=np.array(m['result']['initial_region']['vertices'])
        pool=np.vstack([points,first]);lo=pool.min(0);hi=pool.max(0);span=hi-lo
        lo-=span*.07;hi+=span*.07
        x0=95+panel*1010;y0=95;w=740;h=735
        scale=min(w/(hi[0]-lo[0]),h/(hi[1]-lo[1]));center=(hi+lo)/2
        def project(p):
            p=np.asarray(p);return np.stack([x0+w/2+(p[...,0]-center[0])*scale,y0+h/2-(p[...,1]-center[1])*scale],-1)
        pp=project(points);arr=np.asarray(im).copy();zmin=float(values.min());zmax=float(values.max())
        for t in tris:
            p=pp[t];xmin=max(x0,int(np.floor(p[:,0].min())));xmax=min(x0+w,int(np.ceil(p[:,0].max())))
            ymin=max(y0,int(np.floor(p[:,1].min())));ymax=min(y0+h,int(np.ceil(p[:,1].max())))
            if xmax<=xmin or ymax<=ymin:continue
            xx,yy=np.meshgrid(np.arange(xmin,xmax)+.5,np.arange(ymin,ymax)+.5)
            den=(p[1,1]-p[2,1])*(p[0,0]-p[2,0])+(p[2,0]-p[1,0])*(p[0,1]-p[2,1])
            if abs(den)<1e-12:continue
            a=((p[1,1]-p[2,1])*(xx-p[2,0])+(p[2,0]-p[1,0])*(yy-p[2,1]))/den
            b=((p[2,1]-p[0,1])*(xx-p[2,0])+(p[0,0]-p[2,0])*(yy-p[2,1]))/den;c=1-a-b
            mask=(a>=-1e-8)&(b>=-1e-8)&(c>=-1e-8)
            z=a*values[t[0]]+b*values[t[1]]+c*values[t[2]]
            rgb=cmap((np.log(np.maximum(z,zmin))-np.log(zmin))/(np.log(zmax)-np.log(zmin)))
            sub=arr[ymin:ymax,xmin:xmax];sub[mask]=rgb[mask]
        im=Image.fromarray(arr);draw=ImageDraw.Draw(im)
        text(draw,(x0+w/2,43),title,44)
        line(draw,[(x0,y0),(x0,y0+h),(x0+w,y0+h)],'#333333',3)
        for xv in ([-1000,-500,0] if panel==0 else [1000,1500,2000,2500]):
            x=project([xv,center[1]])[0]
            if x0<=x<=x0+w:
                line(draw,[(x,y0+h),(x,y0+h+10)],'#333333');text(draw,(x,y0+h+37),xv,36)
        for yv in ([-600,0,600] if panel==0 else [-1400,-700,0,700]):
            y=project([center[0],yv])[1]
            if y0<=y<=y0+h:
                line(draw,[(x0-8,y),(x0,y)],'#333333');text(draw,(x0-12,y),yv,36,'rm')
        text(draw,(x0+w/2,y0+h+84),'x / m',38);text(draw,(x0+6,y0-24),'y / m',38,'lm')
        path=project(np.vstack([v,v[0]]));line(draw,[tuple(p) for p in path],'white',5)
        for comp in f['components']:
            bp=project(comp['boundary']);line(draw,[tuple(p) for p in bp],'#E34E65',6)
            if comp['recommended_point'] is not None:star(draw,*project(comp['recommended_point']))
        x,y=project(first);draw.rectangle((x-9,y-9,x+9,y+9),fill='#203646')
        cbx=x0+w+35;cby=210;cbh=465
        for k in range(cbh):
            col=tuple(cmap(1-k/(cbh-1)));draw.line((cbx,cby+k,cbx+28,cby+k),fill=col,width=1)
        draw.rectangle((cbx,cby,cbx+28,cby+cbh),outline='#333333',width=2)
        text(draw,(cbx+20,cby-42),'U / m',36)
        for z in ([60,100,200,400] if panel==0 else [20,40,80,160]):
            y=cby+(1-(math.log(z)-math.log(zmin))/(math.log(zmax)-math.log(zmin)))*cbh
            if cby<=y<=cby+cbh:text(draw,(cbx+40,y),z,36,'lm')
    text(draw,(1000,973),'红线：数值5%近优边界    ☆：已评分推荐点    □：首测点    白线：初始外包',38)
    im.save(HERE/'q2_region_paper.png',dpi=(317.5,317.5))

def comparisons():
    r=[x for x in records if not x['failure'] and x['case']['stratum']=='random']
    old=np.array([x['evaluation']['old']['common_precision_bound']['worst_updated_cover_radius_m'] for x in r])
    new=np.array([x['evaluation']['new']['common_precision_bound']['worst_updated_cover_radius_m'] for x in r])
    im=Image.new('RGB',(2000,700),'white');d=ImageDraw.Draw(im)
    text(d,(535,40),'200个配对案例',44);text(d,(1500,40),'半径上界改善分布',44)
    x0,y0,w,h=130,98,770,455
    def xy(x,y):return x0+x/65*w,y0+h-y/65*h
    line(d,[(x0,y0),(x0,y0+h),(x0+w,y0+h)],'#333333')
    for t in [0,20,40,60]:
        x,y=xy(t,t);text(d,(x,y0+h+34),t,36);text(d,(x0-18,y),t,36,'rm')
    for k in range(0,65,3):line(d,[xy(k,k),xy(min(k+1.6,65),min(k+1.6,65))],'#9AA8AE',3)
    for a,b in zip(old,new):
        x,y=xy(a,b);d.ellipse((x-5,y-5,x+5,y+5),fill='#176B87')
    text(d,(x0+10,y0-24),'本文 U / m',38,'lm');text(d,(x0+w/2,617),'基线 U / m',40)
    counts,bins=np.histogram(old-new,bins=24)
    x0,y0,w,h=1150,98,760,455;xmin=-.8;xmax=4.;ymax=max(30,int(counts.max()+2))
    def hx(x):return x0+(x-xmin)/(xmax-xmin)*w
    def hy(y):return y0+h-y/ymax*h
    line(d,[(x0,y0),(x0,y0+h),(x0+w,y0+h)],'#333333')
    for a,b,n in zip(bins[:-1],bins[1:],counts):d.rectangle((hx(a)+1,hy(n),hx(b)-1,hy(0)),fill='#176B87')
    line(d,[(hx(0),y0),(hx(0),y0+h)],'#C44E52',3)
    for t in [0,1,2,3,4]:text(d,(hx(t),y0+h+34),t,36)
    for t in [0,10,20,30]:text(d,(x0-18,hy(t)),t,36,'rm')
    text(d,(x0+10,y0-24),'案例数',38,'lm');text(d,(x0+w/2,617),'改善量  基线 U - 本文 U / m',40)
    text(d,(1000,676),'左图虚线下方表示改善；两种策略均以256区间上限和0.05 m目标容差复评。',38)
    im.save(HERE/'q2_comparison_paper.png',dpi=(317.5,317.5))
heatmap();comparisons();print('Created readable publication charts from verified numerical data.')
