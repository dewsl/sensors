# -*- coding: utf-8 -*-
"""
Created on Fri Mar 08 08:51:10 2019

@author: Dynaslope
"""

# -*- coding: utf-8 -*-
"""
Created on Thu Aug 21 14:27:07 2014

@author: kennex
"""
#from tkinter import *
import tkinter
import serial
import numpy as np
import sys
import re
import time
import os
import pandas as pd
from datetime import datetime

global M,empty_matrix,S_MATRIX,P_MATRIX, df
empty_matrix = np.empty((1,2), dtype=np.object)
M = np.empty((1,2), dtype=np.object) # initialize to 1 row of matrices # dapat maappend mamaya
S_MATRIX = np.empty((1,2), dtype=np.object) # calibration parameters
P_MATRIX = np.empty((1,2), dtype=np.object) # contains flags for data error, or failure from magnitude verification
global node_id
node_id = [0]
global com_port_variable
global com_port
global status
status = ''
global jig_name

def main():
    global com_port,jig_name,S_MATRIX,node_id,P_MATRIX
    jig_name = 'DINUGUAN'
    print("Available com ports:")
    com_available = serial_ports()
    print (com_available)
    com_port = input("Enter com_port number (ex. 5, 10): ")
    
    start = datetime.now()
    
    go_home()
#    serial_check_selftest('##','s')
    
    calibrate_batt()  #calibrate first the batt voltage of sensor
    
    DATA = serial_reader('##','d')
#    print ("kasjhdhakhd")
    P_MATRIX = SUMMARY_MATRIX(DATA,node_id)
    print (P_MATRIX)

#    initial_guess = np.matrix('0.99 0.01 0.01 0.01 0.99 0.01 0.01 0.01 0.99 1 1 1')
    initial_guess = np.matrix('0.9999 0.0001 0.0001 0.0001 0.9999 0.0001 0.0001 0.0001 0.9999 0.0001 0.0001 0.0001')
#    initial_guess = np.matrix('1.00 0.00 0.00 0.00 1.00 0.00 0.00 0.00 1.00 0 0 0')
    S_MATRIX = store_params(M,initial_guess)
    print (S_MATRIX)
    
    calibfactors(S_MATRIX,node_id)
    
    VMM = serial_reader_df('##','v')
    evaluate(VMM)
    change_debug_mode()
    print ('runtime =', str(datetime.now() - start))

class simpleapp_tk(tkinter.Tk):

    def __init__(self,parent):
        tkinter.Tk.__init__(self,parent)
        self.parent = parent
        self.initialize()
        
    def close_window(self):
        self.destroy()  # destroying the main window
        sys.exit()

    def initialize(self):
        global com_port_variable
        self.grid()
        
        self.entryVariable = tkinter.StringVar()
        self.entry = tkinter.Entry(self,textvariable=self.entryVariable)
        self.entry.grid(column=0,row=0,sticky='EW')
        self.entry.bind("<Return>",self.OnPressEnter)
        self.entryVariable.set(u"DINUGUAN")        
        
        button = tkinter.Button(self,text=u"Start Calibration",command=self.OnButtonClick)
        button.grid(column=1,row=0)
        
        button = tkinter.Button(self,text=u"Stop",command=self.close_window)
        button.grid(column=2,row=0)
        
        OPTIONS = list(serial_ports())        
        
        self.com_port_variable = tkinter.StringVar()
        self.com_port_variable.set('Choose COM PORT here.')
        drop_down = tkinter.OptionMenu(self,self. com_port_variable,*OPTIONS)
        drop_down.grid(column=0,row=2,sticky='W')
        
        
        self.labelVariable = tkinter.StringVar()
        label = tkinter.Label(self,textvariable=self.labelVariable,anchor="w",fg="white",bg="blue")
        label.grid(column=0,row=1,columnspan=3,sticky='EW')
        self.labelVariable.set(u" Enter Jig Name above")
        
        self.grid_columnconfigure(0,weight=1)
        self.resizable(True,False) #resize (horizontal,vertical)

        
    def OnButtonClick(self):

        global com_port,jig_name,S_MATRIX,node_id,P_MATRIX
        jig_name = self.entryVariable.get()
        com_port = self.com_port_variable.get()
        
        com_port = re.sub(r"\D","", com_port)
        status = 'com_port: ' + com_port
        self.labelVariable.set(status)
        status = status + '\n' + 'DATA Gathered'
        self.labelVariable.set(status)
        start = datetime.now()
        
        serial_check_selftest('##','s')
        
#        calibrate_batt()  #calibrate first the batt voltage of sensor
#        
#        DATA = serial_reader('##','d')
#        
#        P_MATRIX = SUMMARY_MATRIX(DATA,node_id)
#        print P_MATRIX
#
#
#        initial_guess = np.matrix('0.99 0.01 0.01 0.01 0.99 0.01 0.01 0.01 0.99 1 1 1')
#        S_MATRIX = store_params(M,initial_guess)
#        print S_MATRIX
#        
#        calibfactors(S_MATRIX,node_id)
#        
#        VMM = serial_reader_df('##','v')
#        evaluate(VMM)
        print ('runtime =', str(datetime.now() - start))
#        print VMM
#        change_debug_mode()
#        sys.exit()

#####################################        
#        Send_to_DUE(S_MATRIX,node_id)       #store data to node
#
#        clear_M()
#        VMM = serial_reader('##','e')
#        
#        node_id_m = np.matrix(node_id)
#        node_id_m = node_id_m.T
#        print node_id_m
#               
#        node_id_m = np.matrix(node_id)
#        node_id_m = node_id_m.T
#        VMM = np.concatenate((VMM,node_id_m),axis=1)
#        #print VMM
#        
#        verify_magnitude(VMM)
#        
#        status = status + '\n' + 'Checked'
#        self.labelVariable.set(status)
#        
#        s0 = P_MATRIX.shape[0]
#        s1 = P_MATRIX.shape[1]
#        uid = P_MATRIX[:,2]
#        #print uid
#        for i in range(0,s0):
#            for j in range(0,s1-1):
#                fail = int(P_MATRIX[i,j])
#                if fail == 0:
#                    fid = format(int(uid[i]),'x')
#                    printstr = 'Node ' + fid + ' ( ' + str(uid[i])+ ' )' +' accel: ' + str(j+1) + ' PASSED!'                    
#                    #printstr = 'Node ' + str(int(uid[i])) + ' accel: ' + str(j+1) + ' PASSED!'
#                    print printstr
#                else:
#                    fid = format(int(uid[i]),'x')
#                    printstr = 'Node ' + fid + ' ( ' + str(uid[i])+ ' )' + ' accel: ' + str(j+1) + ' FAILED!' 
#                    #printstr = 'Node ' + str(int(uid[i])) + ' accel: ' + str(j+1) + ' FAILED!'
#                    print printstr
#        LOG_VAL(VMM,P_MATRIX,uid,node_id_m) #write data to csv

####################################################
        
#        status = status + '\n' + 'Y Matrix Created'
#        self.labelVariable.set(status)
        
#        Y = Create_Y_Matrix(DATA)
        
#        status = status + '\n' + 'Min Max Created'
#        self.labelVariable.set(status)
        
#        MM = ComputeMinMax(DATA,Y)
        
#        status = status + '\n' + 'MIN MAX PASSED to PIC'
#        self.labelVariable.set(status)
        #print 'MM='
        #print MM
#        Write_Save(MM)
        #print S_MATRIX ######################################
#        clear_M() # node_id is reset to contain only 0s
#        status = status + '\n' + 'Validation Data Gathering'
#        self.labelVariable.set(status)
#        VMM = serial_reader('##','d')

        #print node_id ######################################
#       node_id_m = np.matrix(node_id)
#       node_id_m = node_id_m.T
#       VMM = np.concatenate((VMM,node_id_m),axis=1)
        #print VMM
#        status = status + '\n' + 'Checked'
#        self.labelVariable.set(status)
 ######################################
#        verify(VMM)
#        s0 = S_MATRIX.shape[0]
#        s1 = S_MATRIX.shape[1]
#        uid = S_MATRIX[:,2]
#        for i in range(0,s0):
#            for j in range(0,s1-1):
#                fail = int(S_MATRIX[i,j])
#                if fail == 0:
#                    fid = format(int(uid[i]),'x')
#                    printstr = 'Node ' + fid + ' accel: ' + str(j+1) + ' PASSED!'                    
                    #printstr = 'Node ' + str(int(uid[i])) + ' accel: ' + str(j+1) + ' PASSED!'
#                    print printstr
#                else:
#                    fid = format(int(uid[i]),'x')
#                    printstr = 'Node ' + fid + ' accel: ' + str(j+1) + ' FAILED!' 
#                    #printstr = 'Node ' + str(int(uid[i])) + ' accel: ' + str(j+1) + ' FAILED!'
#                    print printstr
#        Count_Val_Rec(VMM)
#        LOG_VAL(VMM,S_MATRIX,uid,node_id_m) #write data to csv

        
    def OnPressEnter(self,event):
        self.labelVariable.set(self.entryVariable.get() + " Enter Pressed!")

#############################################################################
## Gauss Newton Code 


def residuals(adxl_data, beta):
    length = len(adxl_data)
    res = np.ones((length,1))*(13158**2)    #if iis3dhhc 13158 # if iis328dq 16384
    for i in range (0,length):
        x = adxl_data[i,0] - beta[0,9]
        y = adxl_data[i,1] - beta[0,10]
        z = adxl_data[i,2] - beta[0,11]
        for j in range (0,3):
            a = (j*3)
            b = (j*3)+1
            c = (j*3)+2
            cal = (((x*beta[0,a]) + (y*beta[0,b]) + (z*beta[0,c]))**2)
            res[i] = res[i] - cal
    return res
    
#r = residuals(data,initial_guess)

def jacobian(adxl_data,beta):
    #print 'jacobian called'
    length = len(adxl_data)
    jac = np.zeros((length,12))
    array_bt = np.zeros((1,9))
    for i in range (0,length):
        x = -adxl_data[i,0] + beta[0,9]
        y = -adxl_data[i,1] + beta[0,10]
        z = -adxl_data[i,2] + beta[0,11]
        for j in range(0,3):
            a = (j*3)+0
            b = (j*3)+1
            c = (j*3)+2
            big_term = (beta[0,a]*x)+(beta[0,b]*y)+(beta[0,c]*z)
            jac[i,a] = -2*x*big_term
            jac[i,b] = -2*y*big_term
            jac[i,c] = -2*z*big_term
            array_bt[0,j+1] = big_term
        jac[i,9] =   -2*((beta[0,0]*array_bt[0,1] + beta[0,3]*array_bt[0,2] + beta[0,6]*array_bt[0,3]))
        jac[i,10] =   -2*((beta[0,1]*array_bt[0,1] + beta[0,4]*array_bt[0,2] + beta[0,7]*array_bt[0,3]))
        jac[i,11] =   -2*((beta[0,2]*array_bt[0,1] + beta[0,5]*array_bt[0,2] + beta[0,8]*array_bt[0,3]))
    return jac
  
  
def gn_step(adxl_data,beta):
    r = residuals(adxl_data,beta)
    J = jacobian(adxl_data,beta)
    Jt = np.transpose(J)
    JS = np.dot(Jt,J)
    Jtr = np.dot(Jt,r)
    delta = np.linalg.lstsq(JS,Jtr)[0]
    ret = np.subtract(beta,np.transpose(delta))
    return ret

    
def gauss_newton(adxl_data, beta):
    A = adxl_data
    change = 100
    step = 0
    diverge_beta = np.matrix('0 0 0 0 0 0 0 0 0 0 0 0')
    while (change > 0.0000001) and (step < 10):
        oldbeta = beta
        beta = gn_step(A,beta)
        change = 0
        for i in range(0,12):
            change = change + abs((beta[0,i]-oldbeta[0,i])/oldbeta[0,i])
        #print change
        step = step + 1
    printstr = 'number of steps taken to converge:' + str(step)
    print (printstr)
    if (step < 10) & (step > 2):
        return beta
    else:
        return diverge_beta
 ############################################################################
def store_params(MATRIX,initial_guess):
    length = len(MATRIX)
    result = np.empty((length,2), dtype=np.object)
    for i in range(0,2):
        for j in range(0,length):
            try:
                result[j,i] = np.matrix.reshape(gauss_newton(MATRIX[j,i],initial_guess), (4,3)) #resolution 
            except TypeError:
                print ("1 accel lang, ok lang yan")
    return result
    
def Send_to_DUE(MATRIX,uid):
    global com_port
    length = len(MATRIX)
    
    filename = '.\\' + str(jig_name) + '_PARAMETERS.csv'
    
    ser = OpenSerial(com_port)
    ser.close()
    ser.open()
    time.sleep(1.5)
    ser.write('c'.encode())
    time.sleep(1.5)
    rows = 0
    columns = 0
    uid = np.matrix(uid)
    uid = uid.T
    
    try:
        f = open(filename,'a')
    except:
        print(' Cannot open file with filename: %s', filename)
        
    for columns in range(0,length):
        id_to_send = uid[columns,0]
        for rows in range(0,2):
            #id_to_send = int(MATRIX[columns,2])
            print ('id_to send =' ,id_to_send)
            ser.write(str(id_to_send).encode())
            ser.write('#'.encode())
            which_axel = str(rows + 1);
            ser.write(which_axel.encode())
            ser.write('#'.encode())
            
            f.write(str(id_to_send))#np.matrix(data_set)
            f.write(',')
            f.write(str(which_axel))
            f.write(',')
            
            dummy = MATRIX[columns,rows]
            for j in range(0,4): #4
                for i in range(0,3): #3
                    if j < 3: # 4th row 0,1,2,3
                        dummy[j,i] = dummy[j,i]*100
                        print (dummy[j,i])
                    ser.write(str(dummy[j,i]).encode())
                    ser.write('#'.encode())
                    
                    f.write(str(dummy[j,i]))
                    f.write(',')
                    
                    time.sleep(0.5) # kulang yung 0.15 
            ser.write('<'.encode())
            f.write('\n') #??
            while ser.read(2).decode("utf-8") != '&&':
                data = ser.readline().decode("utf-8");
                print (data)
            print ('&& received')
    f.close()
    ser.close()

#############################################################################
def serial_ports():
    if sys.platform.startswith('win'):
        ports = ['COM' + str(i + 1) for i in range(256)]
    else:
        raise EnvironmentError('Unsupported platform')
    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result

def clear_M():
    global M
    M = np.empty((1,2), dtype=np.object)
    global node_id
    node_id = [0]


    
        
def Store_to_Matrix(data,MATRIX): # data is data read from serial
    global node_id
    mheight = len(MATRIX)
    data = data.replace('\r\n','')
    axel = int(data.split(',')[1])
    data_set = data.split(',')[2]
    uid = abs(int(data.split(',')[0]))
    
    if axel == 1 and uid not in node_id:
        index_row = 0
        node_id.append(uid)
        try:
            node_id.remove(0)
        except: # wala nang 0
            pass
        index_col = node_id.index(uid)
        if index_col > (mheight-1):
            MATRIX = np.concatenate((MATRIX,empty_matrix), axis = 0)
        if MATRIX[index_col,index_row] == None:
            MATRIX[index_col,index_row] = np.matrix(data_set)
        elif MATRIX[index_col,index_row] == None:
            MATRIX[index_col,index_row] = np.matrix(data_set)
            
    elif axel == 2 and uid not in node_id:
        index_row = 1
        node_id.append(uid)
        try:
            node_id.remove(0)
        except: # wala nang 0
            pass
        index_col = node_id.index(uid)
        if index_col > (mheight-1):
            MATRIX = np.concatenate((MATRIX,empty_matrix), axis = 0)
        if MATRIX[index_col,index_row] == None:
            MATRIX[index_col,index_row] = np.matrix(data_set)
        elif MATRIX[index_col,index_row] == None:
            MATRIX[index_col,index_row] = np.matrix(data_set)
        index_col = index_col + 1

    elif axel == 2 and uid in node_id:
        index_row = 1
        index_col = node_id.index(uid)
#        print("accel 2")
        try:
            if MATRIX[index_col,index_row] == None:
                MATRIX[index_col,index_row] = np.matrix(data_set)
            else:
                MATRIX[index_col,index_row] = np.concatenate((np.matrix(MATRIX[index_col,index_row]),np.matrix(data_set)),axis = 0)
        except ValueError:
            MATRIX[index_col,index_row] = np.concatenate((np.matrix(MATRIX[index_col,index_row]),np.matrix(data_set)),axis = 0)
#        print("ok")
    elif axel == 1 and uid in node_id:
        index_row = 0
        index_col = node_id.index(uid)
#        print("accel 1")
        try:
            if MATRIX[index_col,index_row] == None:
                MATRIX[index_col,index_row] = np.matrix(data_set)
            else:
                MATRIX[index_col,index_row] = np.concatenate((np.matrix(MATRIX[index_col,index_row]),np.matrix(data_set)),axis = 0)
        except ValueError:
            MATRIX[index_col,index_row] = np.concatenate((np.matrix(MATRIX[index_col,index_row]),np.matrix(data_set)),axis = 0)
    return MATRIX
    
def Create_Y_Matrix(MATRIX):
    global S_MATRIX
    s0 = MATRIX.shape[0]
    s1 = MATRIX.shape[1]
    for i in range(0,s0):
        for j in range(0,s1):
            w = MATRIX[i,j]
            num_rows = w.shape[0]
            num_columns = w.shape[1]
            printstr = 'num_rows: ' + str(num_rows) + ' num_columns: ' + str(num_columns)
            #print printstr
            y = np.zeros(shape=(num_rows,num_columns))
            ## create Y matrix
            for a in range(0, num_columns):
                for b in range(0, num_rows):
                    if w[b,a] > 921:
                        y[b][a] = 1
                    elif w[b,a] <-921:
                        y[b][a] = -1
                    else:
                        y[b][a] = 0
            checky = np.matrix(y)
            yheight = len(checky)
            if yheight == 6:
                checky = np.sum(checky,axis = 0)
                checky = np.sum(checky,axis = 1)
                checky = int(checky)
                if checky == 0 :
                    #print 'return Y matrix here'
                    return y
                else:
                    fid = format(node_id[i],'x')
                    printstr = 'failed node: ' + fid + ' accel: ' + str(j+1) + ' failure mode: data out of bounds'
                    #printstr = 'failed node: ' + str(node_id[i]) + ' accel: ' + str(j+1) + ' failure mode: data out of bounds'
                    print (printstr)
                    S_MATRIX[i,j] = 1
            else:
                fid = format(node_id[i],'x')
                printstr = 'failed node: ' + fid + ' accel: ' + str(j+1) + ' failure mode: missing data'
                #printstr = 'failed node: ' + str(node_id[i]) + ' accel: ' + str(j+1) + ' failure mode: missing data'
                print (printstr)
                S_MATRIX[i,j] = 1

def verify_magnitude(MATRIX):
    global S_MATRIX,P_MATRIX
    s0 = MATRIX.shape[0]
    #print s0
    s1 = MATRIX.shape[1]
    #print s1
    ss0 = MATRIX[0,0].shape[0]
    p0 = P_MATRIX.shape[0]
    
    for i in range(0,s0):
        for j in range(0,s1-1):
            check = np.sqrt(np.sum(np.multiply(MATRIX[i,j],MATRIX[i,j]),axis = 1))
            print (check)
            result = (check > 1014.0) & (check < 1034)
            result = int(np.sum(result,axis = 0))
            if result == ss0: #tama ba?
                fid = format(node_id[i],'x')
                #printstr = 'node: ' + str(node_id[i]) + ' accel: ' + str(j+1) + ' passed Orthogonality'
                printstr = 'node: ' + fid + '( '+ str(node_id[i]) + ' )' +' accel: ' + str(j+1) + ' passed MAGNITUDE CHECK'
                print (printstr)
            else:
                fid = format(node_id[i],'x')
                #printstr = 'node: ' + str(node_id[i]) + ' accel: ' + str(j+1) + ' FAILED Orthogonality'
                printstr = 'node: ' + fid +'( '+ str(node_id[i]) + ' )' + ' accel: ' + str(j+1) + ' FAILED MAGNITUDE CHECK'                
                print (printstr)
#                current_id = node_id[i]
                for si in range (0,p0):
                    if P_MATRIX[si,2] == node_id[i]:
                        P_MATRIX[si,j] = 1
                        print (P_MATRIX)

def SUMMARY_MATRIX(MATRIX,uid):
    s0 = MATRIX.shape[0]
    s1 = MATRIX.shape[1]
    uid = np.matrix(uid)
    uid = uid.T
    #print 's0: ', s0 
    #print 's1: ', s1
    summary = np.matrix(np.zeros(shape=(s0,s1)))
    summary = np.concatenate((summary,uid),axis=1)
    #print summary
    return summary

def ComputeMinMax(MATRIX,Y):
    mheight = len(MATRIX)
    #print 'mheight: ' , mheight
    for columns in range(0,(mheight)):
        for rows in range(0,2):
            max_values = np.matrix(MATRIX[columns,rows])
            #print max_values
            #print Y
            max_values = np.multiply(max_values,Y)
            range_values = ((np.sum(max_values, axis = 0)/2)*10)
            offset = np.multiply(max_values,Y)
            offset = ((np.sum(offset, axis = 0)/2))
            MATRIX[columns,rows] = np.concatenate((range_values,offset),axis = 0)
            
    return MATRIX

###########################
##SERIAL FUNCTIONS
##
##############
 
def OpenSerial(comport):
    #global com_port
    try:
        ser = serial.Serial(
                port = "COM"+str(comport),\
                baudrate = 9600,\
                parity = serial.PARITY_NONE,\
                stopbits = serial.STOPBITS_ONE,\
                bytesize = serial.EIGHTBITS,\
                timeout = None)
                        
    except serial.SerialException:
            print ("ERROR: Could Not Open COM %r!" % (int(comport)))
            return
    while not ser.isOpen:
            try:
                    ser = serial.Serial(
                            port = "COM"+str(comport),\
                            baudrate = 9600,\
                            parity = serial.PARITY_NONE,\
                            stopbits = serial.STOPBITS_ONE,\
                            bytesize = serial.EIGHTBITS,\
                            timeout = None)
            except serial.SerialException:
                    print ("ERROR: Could Not Open COM %r!" % (int(comport)))
                    input (' Press anything to continue')
    return ser

def CloseSerial(comport) :
    try:
        ser = serial.Serial(
                port = "COM"+str(comport),\
                baudrate = 9600,\
                parity = serial.PARITY_NONE,\
                stopbits = serial.STOPBITS_ONE,\
                bytesize = serial.EIGHTBITS,\
                timeout = None)
                        
    except serial.SerialException:
            print ("ERROR: Could Not Open COM %r!" % (int(comport)))
            return
    ser.close()



def serial_reader(terminator,command):
    global M,com_port
    ser = OpenSerial(com_port)
    ser.close()
    ser.open()
    time.sleep(2)
    ser.write("p".encode())
    ser.write(command.encode()) # write command to arduino
    time.sleep(1)
    while ser.read(2).decode("utf-8") != terminator:
        # data format is "--unique_id,axel_number,x y z\n"ones = np.ones(num_rows)
        data = ser.readline().decode("utf-8")
        print (data)
        data = data.replace('\r\n','')
        M = Store_to_Matrix(data,M)
    ser.close()
    MATRIX = M
    #clear_M()
    return MATRIX

def go_home():
    global com_port
    ser = OpenSerial(com_port)
    ser.close()
    ser.open()
    time.sleep(2)
    ser.write("p".encode())
    ser.write("j".encode()) # write command to arduino
    while ser.read(2).decode("utf-8") != "&&":
        # data format is "--unique_id,axel_number,x y z\n"ones = np.ones(num_rows)
        data = ser.readline().decode("utf-8")
        
        data = data.replace('\r','')
        print (data)
    print ("done")
    ser.close()
    #clear_M()
    
def serial_reader_df(terminator,command):
    global M,com_port, df
    df = pd.DataFrame(columns = ["node_id","accel","x","y","z","vref","position"])
    i=0
    x=0
    ser = OpenSerial(com_port)
    ser.close()
    ser.open()
    time.sleep(2)
    ser.write("p".encode())
    ser.write(command.encode()) # write command to arduino
    while ser.read(2).decode("utf-8") != terminator:
        # data format is "--unique_id,axel_number,x y z\n"ones = np.ones(num_rows)
        data = ser.readline().decode("utf-8")
        print (data)
        if data.find('+')>=0:
            print ('next')
            x += 1
            continue
        else:
            data = data.replace('\r\n','')
            data += " {}".format(x)
            data_parsed = re.split(",| ",data)
            df.loc[i] = list(map(float, data_parsed))
            i += 1
            
#        M = Store_to_Matrix(data,M)
#    print df
    ser.close()
#    MATRIX = M
    #clear_M()
    return df

def serial_check_selftest(terminator,command):
    global M,com_port, df
    df = pd.DataFrame(columns = ["node_id","msg_id","vref","temp","checker"])
    i=0
    ser = OpenSerial(com_port)
    ser.close()
    ser.open()
    time.sleep(2)
    ser.write("p".encode())
    ser.write(command.encode()) # write command to arduino
    while ser.read(2).decode("utf-8") != terminator:
        # data format is "--unique_id,axel_number,x y z\n"ones = np.ones(num_rows)
        data = ser.readline().decode("utf-8")
        print (data)
        
        data = data.replacement('\r\n','')
        data_parsed = re.split(",| ",data)
        df.loc[i] = map(float, data_parsed)
        i += 1
            
    ser.close()
    
    df["eval"]="FAILED"
    df["eval"][df.checker==63]="PASSED"
    df = df[df.node_id!=65535]
    if not os.path.isfile("SELFTEST EVALUATION.csv"):
        df.to_csv("SELFTEST EVALUATION.csv", index = False)
    else:
        df.to_csv("SELFTEST EVALUATION.csv",mode='a', index = False, header = False)


def evaluate(df):
        
    df["vref_diff"]=abs(df.vref-3.3)
    df["xy"]=np.degrees(np.arctan(df.x/df.y))
    df["xz"]=np.degrees(np.arctan(df.x/df.z))
    df["yz"]=np.degrees(np.arctan(df.y/df.z))
    
    df["xy_diff"] = np.nan
    df["xz_diff"] = np.nan
    df["yz_diff"] = np.nan
    for i in df.position.groupby(df.position).first():
#        print i
        df.xz_diff[df.position == i] = abs (df.xz[df.position == i] - df.xz[(df.node_id == 65535)&(df.accel == 2)&(df.position == i)].values[0])
        df.xy_diff[df.position == i] = abs (df.xy[df.position == i] - df.xy[(df.node_id == 65535)&(df.accel == 2)&(df.position == i)].values[0])
        df.yz_diff[df.position == i] = abs (df.yz[df.position == i] - df.yz[(df.node_id == 65535)&(df.accel == 2)&(df.position == i)].values[0])
    df.xy_diff[df.xy_diff>170] = 180-df.xy_diff
    df.xz_diff[df.xz_diff>170] = 180-df.xz_diff
    df.yz_diff[df.yz_diff>170] = 180-df.yz_diff
              
    for i in df.position.groupby(df.position).first():
        if i in (0,1):
            df.yz_diff[df.position == i] = 0
        elif i in (2,3):
            df.xz_diff[df.position == i] = 0
        elif i in (4,5):
            df.xy_diff[df.position == i] = 0
    df["eval_xyz"] = 0
    df["eval_xyz"][(df.xy_diff<1.5)&(df.xz_diff<1.5)&(df.yz_diff<1.5)] = 1
    df["eval_vref"] = 0
    df["eval_vref"][(df.vref_diff<0.3)]=1
    df["mag"] = np.sqrt(df.x**2+df.y**2+df.z**2)
    df["eval_mag"] = 0
#    df.eval_mag[(df.mag>1014) & (df.mag<1034)] = 1
    df.eval_mag[(df.mag>13058) & (df.mag<13258)] = 1
    
    if not os.path.isfile("DF_EVALUATION.csv"):
        df.to_csv("DF_EVALUATION.csv", index = False)
    else:
        df.to_csv("DF_EVALUATION.csv", mode='a',header = False, index = False)
    
    df_eval = pd.DataFrame(columns = ["node_id","eval_vref","eval_mag","eval_xyz"])
    
    if not os.path.isfile("EVALUATION.csv"):
        df_eval.to_csv("EVALUATION.csv", index = False)
        
    j=0
    for i in df.node_id[df.node_id!=65535].groupby(df.node_id).first(): 
        if df[(df.node_id == i) & (df.eval_vref==0)].size > 0:
            status_vref = "FAILED"
        else:
            status_vref = "PASSED"
            
#        print i,"vref", status_vref
        
        if df[(df.node_id == i) & (df.eval_mag==0)].size > 0:
            status_mag = "FAILED"
        else:
            status_mag = "PASSED"
        
#        print i,"mag", status_mag
        
        if df[(df.node_id == i) & (df.eval_xyz==0)].size > 0:
            status_xyz = "FAILED"
        else:
            status_xyz = "PASSED"
            
#        print i,"xyz", status_xyz
        
        df_eval.loc[j] = [int(i), status_vref, status_mag, status_xyz]
        j+=1
    print (df_eval)  
    df_eval.to_csv("EVALUATION.csv",mode='a', index = False, header = False)

#        print "eval_mag"    
        
        
        
        


####################################################################


def Count_Val_Rec(MATRIX): #check if VMM's matrices are 6 by 3
    s0 = MATRIX.shape[0]
    #s1 = MATRIX.shape[1]
#    print sheight
    for i in range (0,s0):
        for j in range(0,1):
            current_matrix = MATRIX[i,j]
            current_id = MATRIX[i,2]
            fid = format(int(current_id),'x')
            if len(current_matrix) == 6:
                #current_id = MATRIX[i,2]
                printstr = 'NODE ' + fid + ': complete'
                #printstr = 'NODE' + str(int(current_id)) + ': complete'
                print (printstr)
            elif len(current_matrix) < 6:
                #current_id = MATRIX[i,2]
                printstr = 'NODE ' + fid + ': INCOMPLETE VALIDATION DATA'
                #printstr = 'NODE' + str(int(current_id)) + ': INCOMPLETE VALIDATION DATA'
                print (printstr)
            elif len(current_matrix) > 6:
                #current_id = MATRIX[i,2]
                printstr = 'NODE ' + fid + ': TOO MUCH DATA'
                #printstr = 'NODE' + str(int(current_id)) + ': TOO MUCH DATA'
                print (printstr)
            
def LOG_VAL(MATRIX, SUMMARY, id1,id2):
    sheight = len(SUMMARY)
    global jig_name
    id1_list = list(np.array(id1).reshape(-1))  # list of nodes after data gathering
    id2_list = list(np.array(id2).reshape(-1))  # list of nodes after validation
    SUMMARY_lookup =np.sum(SUMMARY, axis = 1)
 
    filename = '.\\' + str(jig_name) + '_VALIDATION.csv'
    try:
        f = open(filename,'a')
    except:
        print(' Cannot open file with filename: %s', filename)
        
    for i in range(0,sheight):
        current_id = id2_list[i] # current id being checked
        index_current_id = id1_list.index(current_id) # index of current id from id1_list
        #printstr = 'current_id: ' + str(current_id) + ' index_current_id: ' + str(index_current_id) + ' SUMMARY_lookup[index_current_id]: '+ str(SUMMARY_lookup[index_current_id])
        #print printstr
        if SUMMARY_lookup[index_current_id] == current_id:
            #print 'write the matrix here'
            for j in range(0,2):
                TO_WRITE = MATRIX[i,j]
                s0 = TO_WRITE.shape[0]
                s1 = TO_WRITE.shape[1]
                uid_to_write = int(MATRIX[i,2])
                f.write(str(uid_to_write))
                f.write(',')
                f.write(str(int(j+1)))
                f.write(',')
                for x in range(0, s0):
                    for y in range (0, s1):
                        f.write(str(int(TO_WRITE[x,y])))
                        f.write(',')
                f.write('\n')
                #print 'uid_to_write: ' , uid_to_write
        else:
            print (' may MALI')
            
    f.close()
    
def calibfactors(MATRIX, uid):
    global com_port
    length = len(MATRIX)
    
    filename = '.\\' + str(jig_name) + '_PARAMETERS.csv'
    
    ser = OpenSerial(com_port)
    ser.close()
    ser.open()
    time.sleep(1.5)
#    str_to_send = 'c'
#    ser.write('c')
#    time.sleep(1.5)
    rows = 0
    columns = 0
    uid = np.matrix(uid)
    uid = uid.T
    
    try:
        f = open(filename,'a')
    except:
        print(' Cannot open file with filename: %s', filename)
        
    for columns in range(0,length):
        id_to_send = uid[columns,0]
        for rows in range(0,2):
            #id_to_send = int(MATRIX[columns,2])
            try:
                print ('id_to send =' ,id_to_send)
    #            ser.write(str(id_to_send))
    #            ser.write('#')
                which_axel = str(rows + 1);
    #            ser.write(which_axel)
    #            ser.write('#')
                
                str_to_send = 'c{}#{}#'.format(id_to_send,which_axel)
                str_to_save = '{},{},'.format(id_to_send,which_axel)
    #            f.write(str(id_to_send))#np.matrix(data_set)
    #            f.write(',')
    #            f.write(str(which_axel))
    #            f.write(',')
                
                dummy = MATRIX[columns,rows]
                for j in range(0,4): #4
                    for i in range(0,3): #3
                        if j < 3: # 4th row 0,1,2,3
                            dummy[j,i] = dummy[j,i]*10000
                        print (dummy[j,i])
                            
                        str_to_send += '{}#'.format(int(dummy[j,i]))
                        str_to_save += '{},'.format(int(dummy[j,i]))
    #                    f.write(str(dummy[j,i]))
    #                    f.write(',')
                        
    #                    time.sleep(0.10) # kulang yung 0.15 
                str_to_send += '<'
                ser.write(str_to_send.encode())
                print (str_to_send)
                str_to_save += '\n'
    #            f.write('\n') #??
                f.write(str_to_save)
                while ser.read(2).decode("utf-8") != '&&':
                    data = ser.readline().decode("utf-8");
                    print (data)
                print ('&& received')
            except TypeError:
                print  ("1 accel lang, ok lang yan")
    f.close()
    ser.close()

def calibrate_batt():
    global com_port
    
    ser = OpenSerial(com_port)
    ser.close()
    ser.open()
    time.sleep(1.5)   
    ser.write('b'.encode())
    
    while ser.read(2).decode("utf-8") != '&&':
        data = ser.readline().decode("utf-8")
        print (data)
    ser.close()
    
def change_debug_mode():
    global com_port
    
    ser = OpenSerial(com_port)
    ser.close()
    ser.open()
    time.sleep(1.5)   
    ser.write('z'.encode())
    
    while ser.read(2).decode("utf-8") != '&&':
        data = ser.readline().decode("utf-8")
        print (data)
    ser.close()
    

def Write_Save(MATRIX): #pass MM here
    mheight = len(MATRIX)
    global jig_name,node_id
    uid = np.matrix(node_id)
    uid = uid.T
    MATRIX = np.concatenate((MATRIX,uid),axis=1)
    filename = '.\\' + str(jig_name) + '_Log.csv'
    ser = OpenSerial(com_port)
    ser.close()
    ser.open()
    time.sleep(1.5)
    ser.write('g'.encode())
    time.sleep(.5)
    rows = 0
    columns = 0
    try:
        f = open(filename,'a')
    except:
        print(' Cannot open file with filename: %s', filename)
    for columns in range(0,mheight):
        for rows in range(0,2):
            id_to_send = int(MATRIX[columns,2])
            print ('id_to send =' ,id_to_send)
            #write to serial
            ser.write(str(id_to_send).encode())
            ser.write('#'.encode())
            which_axel = str(rows + 1);
            ser.write(which_axel.encode())
            ser.write('#'.encode())
            #write to file
            f.write(str(id_to_send))#np.matrix(data_set)
            f.write(',')
            f.write(str(which_axel))
            f.write(',')
            dummy = MATRIX[columns,rows]
            for j in range(0,2): #4
                for i in range(0,3): #3
                    ser.write(str(dummy[j,i]).encode())
                    ser.write('#'.encode())
                    f.write(str(dummy[j,i]))
                    f.write(',')
            ser.write('<'.encode())
            f.write('\n') #??
            while ser.read(2).decode("utf-8") != '&&':
                data = ser.readline().decode("utf-8")
                #pass
            #print '&& received'
    f.close()
    ser.close()
        

#def main():
#    global M

    
if __name__ == '__main__':
#    app = simpleapp_tk(None)
#    app.title('CALIBRATION')
#    app.mainloop()
    main()
    