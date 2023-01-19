import pandas as pd
import numpy as np
import re
import matplotlib.pyplot as plt
import math
from math import cos,sin,radians,atan2,pi
from mpl_toolkits.mplot3d import Axes3D
from robodk.robolink import *
from robodk.robomath import * 

def degtorad(name):
    name = np.array(name)
    name = np.deg2rad(name)
    name = name.flatten().tolist()
    return name

def radtodeg(name):
    name = np.array(name)
    name = np.rad2deg(name)
    name = np.round(name,3)
    name = name.flatten().tolist()
    return name

def replacedot(lst):
    new=[]
    for i in lst:
        if i !=None:
            if i[0] =='.':
                i = '0'+i
            new.append(i)
        else:
            new.append(i)
    lst = new
    return(lst)

def extractgcode(name):
    gcode=name.split('\n')
    gcode = gcode[1:]
    # print(gcode)
    X=[]
    Y=[]
    Z=[]
    A=[]
    B=[]
    C=[]
    G=[]
    n=0
    N=[]
    count=0
    value={}
    for line in gcode:
        ## X
        x = re.findall('[X]-?[\w]*.\w*', line)
        # x = re.findall('[X]-?\d\d?.\d*', line)
        if x:
            X.append(x[0][1:])
        else:
            X.append(None)
        ## Y
        y = re.findall('[Y]-?[\w]*.\w*', line)
        if y:
            Y.append(y[0][1:])
        else:
            Y.append(None)
        ## Z
        z = re.findall('[Z]-?[\w]*.\w*', line)
        if z:
            Z.append(z[0][1:])
        else:
            Z.append(None)
        ## A
        a = re.findall('[A]-?[\w]*.\w*', line)
        if a:
            A.append(a[0][1:])
        else:
            A.append(None)
        ## B
        b = re.findall('[B]-?[\w]*.\w*', line)
        if b:
            B.append(b[0][1:])
        else:
            B.append(None)
        ## C
        c = re.findall('[C]-?[\w]*.\w*', line)
        if c:
            C.append(c[0][1:])
        else:
            C.append(None)
        
        ## G
        g = re.findall('[G]\w*', line)
        if g:
            if len(g)>1:
                G.append(g)
            else:
                G.append(g[0])
        else:
            G.append(None)
        count+=1
        n+=1
        N.append(n)
    Y =replacedot(Y)
    value['x_val'] = X
    value['y_val'] = Y
    value['z_val'] = Z
    value['a_val'] = A
    value['b_val'] = B
    value['c_val'] = C
    # value['g'] = G
    df = pd.DataFrame(value)
    # df = pd.DataFrame(value,index=N)
    if len(gcode) == count:
        # print(df)
        return df ,G
    else:
        print(len(gcode))
        print(count)

def cleandata(df):
    df.fillna(method = 'ffill',inplace = True)
    df.fillna(0,inplace = True)
    df=df.astype(float)
    return df

def variaxis(name):
    ls=[]
    for i in name.columns:
        li =name[i].tolist()
        ls.append(li)

    x = ls[0]
    y = ls[1]
    z = ls[2]
    a = ls[3]
    b = ls[4]
    c = ls[5]

    a = degtorad(a)
    b = degtorad(b)
    c = degtorad(c)

    length = len(a)
    val ={}
    x_new = []
    y_new = []
    z_new = []
    a_new = []
    b_new = []
    c_new = []
    for i in range(0,length):
        # new_a =  atan2(cos(c[i])*sin(a[i]),cos(a[i]))
        # new_b = atan2(sin(a[i])*sin(c[i])*-1*sin(new_a),cos(c[i])*sin(a[i]))
        # new_b = new_b - pi
        # new_c = atan2(cos(a[i])*sin(c[i]),cos(c[i]))
        # robot = rotx(a[i])*rotz(c[i])
        robot = transl(x[i],y[i],z[i])*rotx(a[i])*rotz(c[i])
        robot = robot.inv()
        fanuc = Pose_2_Fanuc(robot)
        # print(fanuc)
        new_x = fanuc[0]
        new_y = fanuc[1]
        new_z = fanuc[2]
        new_a = fanuc[3]
        new_b = fanuc[4]
        new_c = fanuc[5]
        x_new.append(new_x)
        y_new.append(new_y)
        z_new.append(new_z)
        a_new.append(new_a)
        b_new.append(new_b)
        c_new.append(new_c)
    
    # a_new = radtodeg(a_new)
    # b_new = radtodeg(b_new)
    # c_new = radtodeg(c_new)

    val['x_new'] = x_new
    val['y_new'] = y_new
    val['z_new'] = z_new
    val['a_new'] = a_new
    val['b_new'] = b_new
    val['c_new'] = c_new

    d = pd.DataFrame(val)

    return d

def integrexaxis(name):
    ls=[]
    for i in name.columns:
        li =name[i].tolist()
        ls.append(li)

    x = ls[0]
    y = ls[1]
    z = ls[2]
    a = ls[3]
    b = ls[4]
    c = ls[5]

    a = degtorad(a)
    b = degtorad(b)
    c = degtorad(c)

    length = len(a)
    val ={}
    x_new = []
    y_new = []
    z_new = []
    a_new = []
    b_new = []
    c_new = []
    for i in range(0,length):

        robot = rotz(c[i])*transl(x[i],y[i],z[i])*roty(b[i])
        fanuc = Pose_2_Fanuc(robot)
        # print(fanuc)
        new_x = fanuc[0]
        new_y = fanuc[1]
        new_z = fanuc[2]
        new_a = fanuc[3]
        new_b = fanuc[4]
        new_c = fanuc[5]

        x_new.append(new_x)
        y_new.append(new_y)
        z_new.append(new_z)
        a_new.append(new_a)
        b_new.append(new_b)
        c_new.append(new_c)

    val['x_new'] = x_new
    val['y_new'] = y_new
    val['z_new'] = z_new
    val['a_new'] = a_new
    val['b_new'] = b_new
    val['c_new'] = c_new

    d = pd.DataFrame(val)

    return d




def invxyz(name):
    ls = []
    for column in name.columns:
        li = name[column].tolist()
        ls.append(li)
    x = ls[0]
    y = ls[1]
    z = ls[2]
    a = ls[3]
    b = ls[4]
    c = ls[5]

    a = degtorad(a)
    b = degtorad(b)
    c = degtorad(c)

    length = len(x)
    val ={}
    x_new = []
    y_new = []
    z_new = []
    a_new = []
    b_new = []
    c_new = []

    for i in range(0,length):

        # Fanuc code
        robot = transl(x[i],y[i],z[i])*rotz(c[i])*roty(b[i])*rotx(a[i])
        robot = robot.inv()
        fanuc = Pose_2_Fanuc(robot)
        # print(fanuc)
        new_x = fanuc[0]
        new_y = fanuc[1]
        new_z = fanuc[2]
        new_a = fanuc[3]
        new_b = fanuc[4]
        new_c = fanuc[5]

        new_x = round(new_x,3)
        new_y = round(new_y,3)
        new_z = round(new_z,3)
        new_a = round(new_a,3)
        new_b = round(new_b,3)
        new_c = round(new_c,3)

        x_new.append(new_x)
        y_new.append(new_y)
        z_new.append(new_z)
        a_new.append(new_a)
        b_new.append(new_b)
        c_new.append(new_c)

    # a_new = radtodeg(a_new)
    # b_new = radtodeg(b_new)
    # c_new = radtodeg(c_new)
    val['x_new'] = x_new
    val['y_new'] = y_new
    val['z_new'] = z_new
    val['a_new'] = a_new
    val['b_new'] = b_new
    val['c_new'] = c_new

    d = pd.DataFrame(val)

    # d_copy =df.drop(columns=['x_val','y_val','z_val'])
    # # d_copy.reset_index(drop = True,inplace = True)
    # df = pd.concat([d,d_copy],axis=1)
    return d

def final_code(name):
    df_str = name.astype(str).copy()
    res=[]
    
    for column in df_str.columns:
        li = df_str[column].tolist()
        res.append(li)
        
    x_new = res[0]
    y_new = res[1]
    z_new = res[2]
    a_new = res[3]
    b_new = res[4]
    c_new = res[5]

    val_new ={}
    val_new['x_new_val'] = x_new
    val_new['y_new_val'] = y_new
    val_new['z_new_val'] = z_new
    val_new['a_new_val'] = a_new
    val_new['b_new_val'] = b_new
    val_new['c_new_val'] = c_new

    key = list(val_new.keys())
    key[1]
    length = len(val_new[key[1]])

    for i in range(0,length):
        xx = 'X'+val_new['x_new_val'][i]
        yy = 'Y'+val_new['y_new_val'][i]
        zz = 'Z'+val_new['z_new_val'][i]
        aa ='A'+val_new['a_new_val'][i]
        bb = 'B'+val_new['b_new_val'][i]
        cc = 'C'+val_new['c_new_val'][i]
        final = xx+' '+yy+' '+zz+' '+aa+' '+bb+' '+cc
        print(final)

def firstsector(data,stepover,ggg,filename):
    # df_ls = data.astype(str).copy()
    # ls =[]
    # for column in df_ls.columns:
    #     li = df_ls[column].tolist()
    #     ls.append(li)
    if type(stepover)==int:
        stepover = [i+1 for i in range(0,stepover)]
    else:
        stepover = stepover

    length = data.shape[0]
    total = length+len(stepover)
    # ggg=stepover+ggg
    n=1
    co=[]
    for i in ggg:
        if i=='G00':
            co.append(n)
        elif type(i)==list:
            if 'G00' in i:
                co.append(n)
        n+=1
    # print(co)
    count =1
    # use =[]
    speed = ' 100mm/sec'
    stype = ' CNT1 ;'
    last = stepover[-1]
    for i in range(0,total):
        num = i+1
        line ='   '+str(num)+':'
        if num not in stepover:
            if count in co:
                line = line + 'J'+' '+'P['+str(count)+']'+speed+stype
                print(line)
            else:
                line = line + 'L'+' '+'P['+str(count)+']'+speed+stype
                print(line)
                # use.append(count)
            count +=1
            
        else:
            print(line)

def secondsector(df):
    df_ls = df.astype(str).copy()
    ls =[]
    for column in df_ls.columns:
        li = df_ls[column].tolist()
        ls.append(li)

    x_ls = ls[0]
    y_ls = ls[1]
    z_ls = ls[2]
    a_ls = ls[3]
    b_ls = ls[4]
    c_ls = ls[5]

    val_ls ={}
    val_ls['x_ls'] = x_ls
    val_ls['y_ls'] = y_ls
    val_ls['z_ls'] = z_ls
    val_ls['a_ls'] = a_ls
    val_ls['b_ls'] = b_ls
    val_ls['c_ls'] = c_ls

    key = list(val_ls.keys())
    key[1]
    length = len(val_ls[key[1]])
    print('/POS')

    for i in range(0,length):
        x = val_ls['x_ls'][i]
        y = val_ls['y_ls'][i]
        z = val_ls['z_ls'][i]
        w = val_ls['a_ls'][i]
        p = val_ls['b_ls'][i]
        r = val_ls['c_ls'][i]
        num =i+1
        print(f'P[{num}]'+'{')
        print('   GP1:')
        print('''	UF : 1, UT : 1,		CONFIG : 'N U T, 0, 0, 0',''')
        print(f'	X =   {x}  mm,	Y =     {y}  mm,	Z =   {z}  mm,')
        print(f'	W =   {w} deg,	P =     {p} deg,	R =  {r} deg')
        print('};')
    print('/END')

def totextfile(data,stepover,ggg,filename):
    if type(stepover)==int:
        stepover = [i+1 for i in range(0,stepover)]
    else:
        stepover = stepover

    length = data.shape[0]
    total = length+len(stepover)
    # ggg=stepover+ggg
    n=1
    co=[]
    for i in ggg:
        if i=='G00':
            co.append(n)
        elif type(i)==list:
            if 'G00' in i:
                co.append(n)
        n+=1
    # filen = '/content/'+filename+'.txt'
    f_in = open(f'/Users/pichakornwangcharoenwongse/{filename}.txt','w')
    f_in.write(f'''/PROG  {filename}
/ATTR
OWNER		= MNEDITOR;
COMMENT		= "RoboDK sequence";
PROG_SIZE	= 0;
CREATE		= DATE 08-02-22  TIME 23:41:40;
MODIFIED	= DATE 08-02-22  TIME 23:41:40;
FILE_NAME	= {filename};
VERSION	= 0;
LINE_COUNT	= 55;
MEMORY_SIZE	= 0;
PROTECT		= READ_WRITE;
TCD:  STACK_SIZE	= 0,
      TASK_PRIORITY	= 50,
TIME_SLICE	= 0,
BUSY_LAMP_OFF	= 0,
ABORT_REQUEST	= 0,
PAUSE_REQUEST	= 0;
DEFAULT_GROUP	= 1,*,*,*,*;
CONTROL_CODE	= 00000000 00000000;
/MN
   1:  ! UTOOL_NUM=1;
   2:  ! UFRAME_NUM=1;''')

    count =1
    speed = ' 100mm/sec'
    spcnt = ' 100%'
    stype = ' CNT20 ;'
    last = stepover[-1]
    for i in range(0,total):
        num = i+1
        line ='   '+str(num)+':'
        if num not in stepover:
            if count in co:
                line = line + 'L'+' '+'P['+str(count)+']'+speed+stype
                # line = line + 'J'+' '+'P['+str(count)+']'+spcnt+stype
                f_in.write(f'\n{line}')
            else:
                line = line + 'L'+' '+'P['+str(count)+']'+speed+stype
                f_in.write(f'\n{line}')
            count +=1
            
        else:
            if num>2:
                f_in.write(f'\n{line}')
        #     print(line)
    df_ls = data.astype(str).copy()
    ls =[]
    for column in df_ls.columns:
        li = df_ls[column].tolist()
        ls.append(li)

    x_ls = ls[0]
    y_ls = ls[1]
    z_ls = ls[2]
    a_ls = ls[3]
    b_ls = ls[4]
    c_ls = ls[5]

    val_ls ={}
    val_ls['x_ls'] = x_ls
    val_ls['y_ls'] = y_ls
    val_ls['z_ls'] = z_ls
    val_ls['a_ls'] = a_ls
    val_ls['b_ls'] = b_ls
    val_ls['c_ls'] = c_ls

    key = list(val_ls.keys())
    key[1]
    length = len(val_ls[key[1]])
    f_in.write('\n/POS')

    for i in range(0,length):
        x = val_ls['x_ls'][i]
        y = val_ls['y_ls'][i]
        z = val_ls['z_ls'][i]
        w = val_ls['a_ls'][i]
        p = val_ls['b_ls'][i]
        r = val_ls['c_ls'][i]
        num =i+1
        f_in.write(f'\nP[{num}]'+'{')
        f_in.write('\n   GP1:')
        f_in.write('''\n	UF : 1, UT : 1,		CONFIG : 'N U T, 0, 0, 0',\n''')
        f_in.write(f'	X =   {x}  mm,	Y =     {y}  mm,	Z =   {z}  mm,\n')
        f_in.write(f'	W =   {w} deg,	P =     {p} deg,	R =  {r} deg\n')
        f_in.write('};')
    f_in.write('\n/END')

    f_in.close()
def shifttoolaxis(q,w,r,theta4,theta5,theta6,name):
    ls = []
    for column in name.columns:
        li = name[column].tolist()
        ls.append(li)
    x = ls[0]
    y = ls[1]
    z = ls[2]
    a = ls[3]
    b = ls[4]
    c = ls[5]

    a = degtorad(a)
    b = degtorad(b)
    c = degtorad(c)

    length = len(x)
    val ={}
    x_new = []
    y_new = []
    z_new = []
    a_new = []
    b_new = []
    c_new = []
    theta4 = theta4*pi/180
    theta5 = theta5*pi/180
    theta6 = theta6*pi/180
    shiftax = transl(q,w,r)*rotz(theta6)*roty(theta5)*rotx(theta4)
    shiftax = shiftax.inv()

    for i in range(0,length):
        robot = transl(x[i],y[i],z[i])*rotz(c[i])*roty(b[i])*rotx(a[i])
        allshift = robot*shiftax
        fanuc = Pose_2_Fanuc(allshift)

        new_x = fanuc[0]
        new_y = fanuc[1]
        new_z = fanuc[2]
        new_a = fanuc[3]
        new_b = fanuc[4]
        new_c = fanuc[5]
        
        new_x = round(new_x,3)
        new_y = round(new_y,3)
        new_z = round(new_z,3)
        new_a = round(new_a,3)
        new_b = round(new_b,3)
        new_c = round(new_c,3)
        x_new.append(new_x)
        y_new.append(new_y)
        z_new.append(new_z)
        a_new.append(new_a)
        b_new.append(new_b)
        c_new.append(new_c)
    val['x_new'] = x_new
    val['y_new'] = y_new
    val['z_new'] = z_new
    val['a_new'] = a_new
    val['b_new'] = b_new
    val['c_new'] = c_new

    d = pd.DataFrame(val)
    return d