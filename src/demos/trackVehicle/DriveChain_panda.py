# -*- coding: utf-8 -*-
"""
Created on Friday, 2/13/15

@author: Justin Madsen, 2015
"""

import pandas as pd
import matplotlib.pyplot as plt
import matplotlib
import pylab as py


class DriveChain_panda:
    '''
    @class: loads, manages and plots output from any number of output files from 
            Chrono Track Vehicle Toolkit
    '''
    def __init__(self,filename_list, handle_list):
        '''
        Input:
            filename_list:  .csv path + filename list of all the file names
            handle_list:    handle identifiers for each file, for legend and getting a handle to different dataframes
        Appends:
            _filename_list  full name of csv output files to read from
            _DF_list        contains data frames for each loaded output data file
            _handle_list    string, to access a DF handle
        '''
        # first file is the steady state magic formula output
        if( len(filename_list) != len(handle_list)):
            print 'must have same length input arrays'
            return
        
        self._filename_list = []   
        self._DF_list = []      
        self._handle_list = []
        self._dict = {} # for associating the string handle to the DF
        self._nFiles = len(filename_list)
        
        # append members with input lists, create the dict for lookup
        for i in range(0, self._nFiles):
            self._filename_list.append(filename_list[i])
            DF_curr = pd.read_csv(filename_list[i], header=0, sep=',')   # index_col=0,
            self._DF_list.append(DF_curr)
            self._handle_list.append(handle_list[i])
            # associate the handle string with the index of the DF in the list
            self._dict[handle_list[i]] = i
    
    
    # to manage lots of different input files and their associated data frames, 
    #   Access handles to the _DF_list members by specifying the corresponding 
    #   as a dictionary
    def _getDFhandle(self, str_handle):
        return self._DF_list[self._dict[str_handle]]
    
    # want to use the specified tmin, tmax time bounds to set the x axis
    # can do lots of silly things to mess this up
    # return a valid list [xmin,xmax] for use in plot.set_xlim()
    # assume tmin > 0, tmax > tmin, and both are < t_end; else return [0,t_end]
    def _get_timeMinMax(self, t_end_data, tmin = -1, tmax = -1):
        # t_end = DF['time'][-1]  # last timestep in the output file
        out_minmax = [0, t_end_data] # return this if both tmin and tmax are invalid
        
        # default value for tmin is -1, so check to see if it was even given as input
        if(tmin > 0 and tmax > tmin and tmin < t_end_data and tmax < t_end_data):
            out_minmax = [tmin,tmax]
        # else:
            # lg.WARN("invalid tmin or tmax passed to _get_timeMinMax() ")
                    
        return out_minmax
    
    # plot gear body info, with an optional time interval
    def plot_gear(self, tmin = -1 , tmax = -1):
        # plot pos, vel, omega (pitch,yaw,roll)
        fig, axarr = plt.subplots(3,sharex=True)
        
        # pull the data from the DF for the gear
        inDat = ['time','x','y','z','Vx','Vy','Vz','Wx','Wy','Wz']
        # create a dataframe for the desired data, by getting ahandle to the loaded DF for the gear
        DF = pd.DataFrame(self._getDFhandle('Gear'), columns = inDat)
        
        # create the 3 subplots
        DF.plot(ax = axarr[0], linewidth=1.5, x='time', y=['x','y','z'])
        DF.plot(ax = axarr[1], linewidth=1.5, x='time', y=['Vx','Vy','Vz'])
        DF.plot(ax = axarr[2], linewidth=1.5, x='time', y=['Wx','Wy','Wz'])
        
        # if specified, set the xlim of the plot
        if(tmin > 0):
            xminmax = self._get_timeMinMax(DF['time'].iget(-1), tmin, tmax)
            axarr[0].set_xlim(xminmax[0],xminmax[1])

        # label axes
        axarr[0].set_xlabel('time [s]')
        axarr[0].set_ylabel('position [m]')
        axarr[1].set_ylabel('velocity [m/s]')
        axarr[2].set_ylabel('rot. vel. [rad/sec]')
        
        # set the legend
        axarr[0].legend(loc='upper right')
        axarr[1].legend(loc='upper right')
        axarr[2].legend(loc='upper right')
        axarr[0].set_title('Gear body')
        
        
    # plot idler body info, tensioner force,  with a optional time interval
    def plot_idler(self, tmin = -1, tmax = -1):
        # plot pos, vel, omega
        figA, axarrA = plt.subplots(3, sharex=True)
        
        #and F_tensioner
        FigB, axarrB = plt.subplots(1)
        
        # data headers to pull from
        inDat = ['time','x','y','z','Vx','Vy','Vz','Wx','Wy','Wz','F_tensioner','F_k','F_c']      
        
        # data frame from the headers
        DFid = pd.DataFrame(self._getDFhandle('idler'), columns = inDat)
        
        # crate 3 subplots for the body data
        DFid.plot(ax = axarrA[0], linewidth=1.5, x='time', y=['x','y','z'])
        DFid.plot(ax = axarrA[1], linewidth=1.5, x='time', y=['Vx','Vy','Vz'])
        DFid.plot(ax = axarrA[2], linewidth=1.5, x='time', y=['Wx','Wy','Wz'])
        
        #if specified, set the xlim of the two plots. Set any ylim that change a lot when tspan is not the whole sim time.
        tminidx = 0
        tmaxidx = -1
        if(tmin > 0):
            xminmax = self._get_timeMinMax(DFid['time'].iget(-1), tmin, tmax)
            axarrA[0].set_xlim(xminmax[0], xminmax[1])
            axarrB.set_xlim(xminmax[0], xminmax[1])
            # Gt the DF row index for tmin, tmax
            # first index in this list is where we start
            tminidxlist = DFid[ DFid['time'] > tmin].index.tolist()
            tminidx = tminidxlist[0]
            # last index in this list is the end point
            tmaxidxlist = DFid[ DFid['time'] < tmax].index.tolist()
            tmaxidx = tmaxidxlist[-1]   
            
            # reset any ylim that look much better when specified time span is plotted
            axarrA[1].set_ylim(DFid['Vx'][tminidx:tmaxidx].min(), DFid['Vx'][tminidx:tmaxidx].max())
            maxF = DFid['F_tensioner'][tminidx:tmaxidx].max()
            axarrB.set_ylim(-maxF,maxF)
            
        # first plot label axes 
        axarrA[0].set_xlabel('time [s]')
        axarrA[0].set_ylabel('position [m]')
        axarrA[1].set_ylabel('velocity [m/s]')
        axarrA[2].set_ylabel('rot vel [rad/sec]')
        # first plot, legend
        axarrA[0].legend()
        axarrA[1].legend()
        axarrA[2].legend()
        axarrA[0].set_title('idler body')
        
        # create the tensioner force plot       
        DFid.plot(ax = axarrB, linewidth=1.5, x='time', y=['F_tensioner','F_k','F_c'])
        
        # second plot label axes
        axarrB.set_xlabel('time [s]')
        axarrB.set_ylabel('tensioner Force [N]')
        # second plot legend
        axarrB.legend()
        axarrB.set_title('tensioner')
        
    # plot powertrain info, with a optional time interval
    def plot_ptrain(self, tmin = -1, tmax = -1):
        # plot mot speed, mot torque, and output shaft torque
        fig, axarr = plt.subplots(3, sharex=True)
        
        #data headers
        inDat = ['time','throttle','motSpeed','motT','outT']
        # data frame from headers
        DFpt = pd.DataFrame(self._getDFhandle('ptrain'), columns = inDat)
        
        # create a subplot for speed, and torues
        DFpt.plot(ax = axarr[0], linewidth=1.5, x='time', y=['motSpeed'])
        axRHS = axarr[0].twinx()
        axRHS.plot(DFpt['time'], DFpt['throttle'],'r--', linewidth = 1.5, label = 'throttle')
        DFpt.plot(ax = axarr[1], linewidth=1.5, x='time', y=['motT'])
        DFpt.plot(ax = axarr[2], linewidth=1.5, x='time', y=['outT'])
        
        #if specified, set the xlim of the two plots
        #if specified, set the xlim of the two plots. Set any ylim that change a lot when tspan is not the whole sim time.
        tminidx = 0
        tmaxidx = -1
        if(tmin > 0):
            xminmax = self._get_timeMinMax(DFpt['time'].iget(-1), tmin, tmax)
            axarr[0].set_xlim(xminmax[0], xminmax[1])
            # Gt the DF row index for tmin, tmax
            # first index in this list is where we start
            tminidxlist = DFpt[ DFpt['time'] > tmin].index.tolist()
            tminidx = tminidxlist[0]
            # last index in this list is the end point
            tmaxidxlist = DFpt[ DFpt['time'] < tmax].index.tolist()
            tmaxidx = tmaxidxlist[-1]   
            
            axarr[0].set_ylim(DFpt['motSpeed'][tminidx:tmaxidx].min(), DFpt['motSpeed'][tminidx:tmaxidx].max())
            axarr[1].set_ylim(DFpt['motT'][tminidx:tmaxidx].min(), DFpt['motT'][tminidx:tmaxidx].max())
            axarr[2].set_ylim(DFpt['outT'][tminidx:tmaxidx].min(), DFpt['outT'][tminidx:tmaxidx].max())
        
        # labels, legend
        axarr[0].set_xlabel('time [s]')
        axarr[0].set_ylabel('motor speed [RPM]')
        axRHS.set_ylabel('throttle [-]')
        axarr[1].set_ylabel('engine torque [N-m]')
        axarr[2].set_ylabel('output torque [N-m]')
        axarr[0].legend()
        axRHS.legend(loc ='lower right')
        axarr[1].legend()
        axarr[0].set_title('powertrain')
        
    # plot shoe 0 body info, and pin 0 force/torque, with an optional time interval
    def plot_shoe(self, tmin = -1, tmax = -1):
        # a plot for the shoe body,
        figA, axarrA = plt.subplots(4,sharex=True)
        #  pin rxn forces
        figB, axarrB = plt.subplots(3,sharex=True)
        # pin rxn torques
        figC, axarrC = plt.subplots(3,sharex=True)
        
        # data headers
        inDat = ['time','x','y','z','Vx','Vy','Vz','Ax','Ay','Az','Wx','Wy','Wz','Fx','Fy','Fz','Tx','Ty','Tz']
        # get the data frame from the shoe
        DFs = pd.DataFrame(self._getDFhandle('shoe0'), columns=inDat)
        
        # create 4 subplots for the shoe body
        DFs.plot(ax = axarrA[0], linewidth=1.5, x='time', y=['x','y','z'])
        DFs.plot(ax = axarrA[1], linewidth=1.5, x='time', y=['Vx','Vy','Vz'])
        DFs.plot(ax = axarrA[2], linewidth=1.5, x='time', y=['Wx','Wy','Wz'])
        DFs.plot(ax = axarrA[3], linewidth=1.5, x='time', y=['Ax','Ay','Az'])                
        
        #if specified, set the xlim of the two plots. Set any ylim that change a lot when tspan is not the whole sim time.
        tminidx = 0
        tmaxidx = -1
        if(tmin > 0):
            xminmax = self._get_timeMinMax(DFs['time'].iget(-1), tmin, tmax)
            axarrA[0].set_xlim(xminmax[0], xminmax[1])
            axarrB[0].set_xlim(xminmax[0], xminmax[1])
            axarrC[0].set_xlim(xminmax[0], xminmax[1])
            # Gt the DF row index for tmin, tmax
             # first index in this list is where we start
            tminidxlist = DFs[ DFs['time'] > tmin].index.tolist()
            tminidx = tminidxlist[0]
            # last index in this list is the end point
            tmaxidxlist = DFs[ DFs['time'] < tmax].index.tolist()
            tmaxidx = tmaxidxlist[-1]   
            
            # set any limits that are much better when tspan is changed
            axarrA[2].set_ylim(DFs['Wz'][tminidx:tmaxidx].min(), DFs['Wz'][tminidx:tmaxidx].max() )
            axarrA[3].set_ylim(DFs['Ay'][tminidx:tmaxidx].min(), DFs['Ay'][tminidx:tmaxidx].max() )
            axarrB[0].set_ylim(DFs['Fx'][tminidx:tmaxidx].min(), DFs['Fx'][tminidx:tmaxidx].max() )
            axarrB[1].set_ylim(DFs['Fy'][tminidx:tmaxidx].min(), DFs['Fy'][tminidx:tmaxidx].max() )
            axarrC[0].set_ylim(DFs['Tx'][tminidx:tmaxidx].min(), DFs['Tx'][tminidx:tmaxidx].max() )
            axarrC[1].set_ylim(DFs['Ty'][tminidx:tmaxidx].min(), DFs['Ty'][tminidx:tmaxidx].max() )
        
        # first plot label axes 
        axarrA[0].set_xlabel('time [s]')
        axarrA[0].set_ylabel('position [m]')
        axarrA[1].set_ylabel('velocity [m/s]')
        axarrA[2].set_ylabel('rot vel [rad/sec]')
        axarrA[3].set_ylabel('accel [m/s2]')
        # first plot, legend
        axarrA[0].legend()
        axarrA[1].legend()
        axarrA[2].legend()
        axarrA[3].legend()
        axarrA[0].set_title('shoe 0 body')
        
        # create the second plot, pin reaction forces
        DFs.plot(ax = axarrB[0], linewidth=1.5, x='time', y=['Fx'])
        DFs.plot(ax = axarrB[1], linewidth=1.5, x='time', y=['Fy'])
        DFs.plot(ax = axarrB[2], linewidth=1.5, x='time', y=['Fz'])
        
        # second plot label axes, legend
        axarrB[0].set_xlabel('time [s]')
        axarrB[0].set_ylabel('Fx [N]')
        axarrB[1].set_ylabel('Fy [N]')
        axarrB[2].set_ylabel('Fz [N]')
        # first plot, legend
        axarrB[0].legend()
        axarrB[1].legend()
        axarrB[2].legend()
        axarrB[0].set_title('inter-shoe revolute reaction forces')
        
        # create third plot, pin reaction torques
        DFs.plot(ax = axarrC[0], linewidth=1.5, x='time', y=['Tx'])
        DFs.plot(ax = axarrC[1], linewidth=1.5, x='time', y=['Ty'])
        DFs.plot(ax = axarrC[2], linewidth=1.5, x='time', y=['Tz'])
        
        # second plot label axes, legend
        axarrC[0].set_xlabel('time [s]')
        axarrC[0].set_ylabel('Tx [N-m]')
        axarrC[1].set_ylabel('Ty [N-m]')
        axarrC[2].set_ylabel('Tz [N-m]')
        # first plot, legend
        axarrC[0].legend()
        axarrC[1].legend()
        axarrC[2].legend()
        axarrC[0].set_title('inter-shoe0 revolute reaction torques')
    
    # plot gear Constraint Violations, with optional time interval
    def plot_gearCV(self, tmin = -1, tmax = -1):
        # plot gear revolute constraint violations
        fig, axarr = plt.subplots(2,sharex=True)
        
        #data headers
        inDat = ['time','x','y','z','rx','ry']
        # data frame from headers
        DFgCV = pd.DataFrame(self._getDFhandle('gearCV'), columns = inDat)
        
        # create a subplot for speed, and torues
        DFgCV.plot(ax = axarr[0], linewidth=1.5, x='time', y=['x','y','z'])
        DFgCV.plot(ax = axarr[1], linewidth=1.5, x='time', y=['rx','ry'])
        
        #if specified, set the xlim of the two plots
        if(tmin > 0):
            xminmax = self._get_timeMinMax(DFgCV['time'].iget(-1), tmin, tmax)
            axarr[0].set_xlim(xminmax[0], xminmax[1])
            
        # labels, legend
        axarr[0].set_xlabel('time [s]')
        axarr[0].set_ylabel('pos. coord violation ')
        axarr[1].set_ylabel('rot. coord violation ')
        axarr[0].legend()
        axarr[1].legend()
        axarr[0].set_title('Gear rev. Constraint Violtion')
        
    # plot idler Constraint Violations , with the specified time interval 
    def plot_idlerCV(self, tmin = -1, tmax = -1):
        # plot two pos. and two rot. CVs
        fig, axarr = plt.subplots(2,sharex=True)
        
        #data headers
        inDat = ['time','y','z','rx','ry']
        # data frame from headers
        DFidCV = pd.DataFrame(self._getDFhandle('idlerCV'), columns = inDat)
        
        # create a subplot for speed, and torues
        DFidCV.plot(ax = axarr[0], linewidth=1.5, x='time', y=['y','z'])
        DFidCV.plot(ax = axarr[1], linewidth=1.5, x='time', y=['rx','ry'])
        
        #if specified, set the xlim of the two plots
        if(tmin > 0):
            xminmax = self._get_timeMinMax(DFidCV['time'].iget(-1), tmin, tmax)
            axarr[0].set_xlim(xminmax[0], xminmax[1])
        
        # labels, legend
        axarr[0].set_xlabel('time [s]')
        axarr[0].set_ylabel('pos. coord violation ')
        axarr[1].set_ylabel('rot. coord violation ')
        axarr[0].legend()
        axarr[1].legend()
        axarr[0].set_title('Idler Constraint Violtion')
        
    # plot roller Constraint Violations
    def plot_rollerCV(self, tmin = -1, tmax = -1):
        # plot roller revolute constraint violations
        fig, axarr = plt.subplots(2,sharex=True)
        
        #data headers
        inDat = ['time','x','y','z','rx','ry']
        # data frame from headers
        DFrCV = pd.DataFrame(self._getDFhandle('rollerCV'), columns = inDat)
        
        # create a subplot for speed, and torues
        DFrCV.plot(ax = axarr[0], linewidth=1.5, x='time', y=['x','y','z'])
        DFrCV.plot(ax = axarr[1], linewidth=1.5, x='time', y=['rx','ry'])
        
        #if specified, set the xlim of the two plots
        if(tmin > 0):
            xminmax = self._get_timeMinMax(DFrCV['time'].iget(-1), tmin, tmax)
            axarr[0].set_xlim(xminmax[0], xminmax[1])        
        
        # labels, legend
        axarr[0].set_xlabel('time [s]')
        axarr[0].set_ylabel('pos. coord violation ')
        axarr[1].set_ylabel('rot. coord violation ')
        axarr[0].legend()
        axarr[1].legend()
        axarr[0].set_title('Roller rev. Constraint Violtion')
    
    # plot contact info for the gear, with a optional time interval
    def plot_gearContactInfo(self, tmin = -1, tmax = -1):
        # plot normal contact force info, # contacts, FnMax, FnAvg, FnStdDev
        figA, axarrA = plt.subplots(4, sharex=True)
        
        # plot friction force info, # contacts, FtMax, FtAvg, FtStdDev
        FigB, axarrB = plt.subplots(4,sharex=True)
        
        # data headers to pull from
        inDat = ['time','Ncontacts','FnMax','FnAvg','FnSig','FtMax','FtAvg','FtSig']      
        
        # data frame from the headers
        DFgci = pd.DataFrame(self._getDFhandle('gearContact'), columns = inDat)
        

        # for all plots: take the per step data and find 1) avg., 2) stdev over the whole sim time
        #if specified, set the xlim of the two plots. Set any ylim that change a lot when tspan is not the whole sim time.
        tminidx = 0
        tmaxidx = -1
        if(tmin > 0):
            xminmax = self._get_timeMinMax(DFgci['time'].iget(-1), tmin, tmax)
            axarrA[0].set_xlim(xminmax[0], xminmax[1])
            axarrB[0].set_xlim(xminmax[0], xminmax[1])
            # Gt the DF row index for tmin, tmax
             # first index in this list is where we start
            tminidxlist = DFgci[ DFgci['time'] > tmin].index.tolist()
            tminidx = tminidxlist[0]
            # last index in this list is the end point
            tmaxidxlist = DFgci[ DFgci['time'] < tmax].index.tolist()
            tmaxidx = tmaxidxlist[-1]           
            
            # change any y_lim that need it
            axarrA[2].set_ylim(DFgci['FnAvg'][tminidx:tmaxidx].min(), DFgci['FnAvg'][tminidx:tmaxidx].max() )
            
            # set the avg. data
            #) A ---- # of contacts on the gear body
            n_contacts_avg = DFgci['Ncontacts'][tminidx:tmaxidx].mean()
            n_contacts_sig = py.sqrt(DFgci['Ncontacts'][tminidx:tmaxidx].var())
            # add these two as cosntant columns to the dataframe
            DFgci['NcontactsAvg'] = n_contacts_avg
            DFgci['NcontactsSig'] = n_contacts_sig
    
            # --- B) max Normal Force 
            FnMaxAvg = DFgci['FnMax'][tminidx:tmaxidx].mean()
            FnMaxSig = py.sqrt(DFgci['FnMax'][tminidx:tmaxidx].var() )
            # add as constant columns to the DF
            DFgci['FnMaxAvg'] = FnMaxAvg
            DFgci['FnMaxSig'] = FnMaxSig
            
            # --- C) average normal force
            FnAvgAvg = DFgci['FnAvg'][tminidx:tmaxidx].mean()
            FnAvgSig = py.sqrt(DFgci['FnAvg'][tminidx:tmaxidx].var() )
            # add to DF
            DFgci['FnAvgAvg'] = FnAvgAvg
            DFgci['FnAvgSig'] = FnAvgSig  
            
            # --- D) stdev normal force
            FnSigAvg = DFgci['FnSig'][tminidx:tmaxidx].mean()
            FnSigSig = py.sqrt(DFgci['FnSig'][tminidx:tmaxidx].var() )
            # add to DF
            DFgci['FnSigAvg'] = FnSigAvg
            DFgci['FnSigSig'] = FnSigSig
        else:
            #) A ---- # of contacts on the gear body
            n_contacts_avg = DFgci['Ncontacts'].mean()
            n_contacts_sig = py.sqrt(DFgci['Ncontacts'].var())
            # add these two as cosntant columns to the dataframe
            DFgci['NcontactsAvg'] = n_contacts_avg
            DFgci['NcontactsSig'] = n_contacts_sig
    
    
            # --- B) max Normal Force 
            FnMaxAvg = DFgci['FnMax'].mean()
            FnMaxSig = py.sqrt(DFgci['FnMax'].var() )
            # add as constant columns to the DF
            DFgci['FnMaxAvg'] = FnMaxAvg
            DFgci['FnMaxSig'] = FnMaxSig
            
            # --- C) average normal force
            FnAvgAvg = DFgci['FnAvg'].mean()
            FnAvgSig = py.sqrt(DFgci['FnAvg'].var() )
            # add to DF
            DFgci['FnAvgAvg'] = FnAvgAvg
            DFgci['FnAvgSig'] = FnAvgSig  
            
            # --- D) stdev normal force
            FnSigAvg = DFgci['FnSig'].mean()
            FnSigSig = py.sqrt(DFgci['FnSig'].var() )
            # add to DF
            DFgci['FnSigAvg'] = FnSigAvg
            DFgci['FnSigSig'] = FnSigSig
        
        # crate 4 subplots for the normal force data
        DFgci.plot(ax = axarrA[0], linewidth=1.5, x='time', y=['Ncontacts','NcontactsAvg','NcontactsSig'])
        DFgci.plot(ax = axarrA[1], linewidth=1.5, x='time', y=['FnMax','FnMaxAvg','FnMaxSig'])
        DFgci.plot(ax = axarrA[2], linewidth=1.5, x='time', y=['FnAvg','FnAvgAvg','FnAvgSig'])
        DFgci.plot(ax = axarrA[3], linewidth=1.5, x='time', y=['FnSig','FnSigAvg','FnSigSig'])
        
            
        # first plot label axes 
        axarrA[0].set_xlabel('time [s]')
        axarrA[0].set_ylabel('# contacts')
        axarrA[1].set_ylabel('max Force [N]')
        axarrA[2].set_ylabel('avg. Force[N')
        axarrA[3].set_ylabel('std. dev Force [N]')
        # first plot, legend
        axarrA[0].legend()
        axarrA[1].legend()
        axarrA[2].legend()
        axarrA[3].legend()
        axarrA[0].set_title('Gear body normal contact force')
        
        # for all plots: take the per step data and find 1) avg., 2) stdev over the whole sim time
        if(tmin > 0):
            # --- B) max Normal Force 
            FtMaxAvg = DFgci['FtMax'][tminidx:tmaxidx].mean()
            FtMaxSig = py.sqrt(DFgci['FtMax'][tminidx:tmaxidx].var() )
            # add as constant columns to the DF
            DFgci['FtMaxAvg'] = FtMaxAvg
            DFgci['FtMaxSig'] = FtMaxSig
            
            # --- C) average normal force
            FtAvgAvg = DFgci['FtAvg'][tminidx:tmaxidx].mean()
            FtAvgSig = py.sqrt(DFgci['FtAvg'][tminidx:tmaxidx].var() )
            # add to DF
            DFgci['FtAvgAvg'] = FtAvgAvg
            DFgci['FtAvgSig'] = FtAvgSig       
            
            # --- D) stdev normal force
            FtSigAvg = DFgci['FtSig'][tminidx:tmaxidx].mean()
            FtSigSig = py.sqrt(DFgci['FtSig'][tminidx:tmaxidx].var() )
            # add to DF
            DFgci['FtSigAvg'] = FtSigAvg
            DFgci['FtSigSig'] = FtSigSig    
        else:
            # --- B) max Normal Force 
            FtMaxAvg = DFgci['FtMax'].mean()
            FtMaxSig = py.sqrt(DFgci['FtMax'].var() )
            # add as constant columns to the DF
            DFgci['FtMaxAvg'] = FtMaxAvg
            DFgci['FtMaxSig'] = FtMaxSig
            
            # --- C) average normal force
            FtAvgAvg = DFgci['FtAvg'].mean()
            FtAvgSig = py.sqrt(DFgci['FtAvg'].var() )
            # add to DF
            DFgci['FtAvgAvg'] = FtAvgAvg
            DFgci['FtAvgSig'] = FtAvgSig       
            
            # --- D) stdev normal force
            FtSigAvg = DFgci['FtSig'].mean()
            FtSigSig = py.sqrt(DFgci['FtSig'].var() )
            # add to DF
            DFgci['FtSigAvg'] = FtSigAvg
            DFgci['FtSigSig'] = FtSigSig        
        
        
        # create second plot, 4 subplots for the friction contact force data
        DFgci.plot(ax = axarrB[0], linewidth=1.5, x='time', y=['Ncontacts','NcontactsAvg','NcontactsSig'])
        DFgci.plot(ax = axarrB[1], linewidth=1.5, x='time', y=['FtMax','FtMaxAvg','FtMaxSig'])
        DFgci.plot(ax = axarrB[2], linewidth=1.5, x='time', y=['FtAvg','FtAvgAvg','FtAvgSig'])
        DFgci.plot(ax = axarrB[3], linewidth=1.5, x='time', y=['FtSig','FtSigAvg','FtSigSig'])
        
        # second plot label axes
        axarrB[0].set_xlabel('time [s]')
        axarrB[0].set_ylabel('# contacts')
        axarrB[1].set_ylabel('max Force [N]')
        axarrB[2].set_ylabel('avg. Force[N]')
        axarrB[3].set_ylabel('std. dev Force [N]')
        # first plot, legend
        axarrB[0].legend()
        axarrB[1].legend()
        axarrB[2].legend()
        axarrB[3].legend()
        axarrB[0].set_title('Gear body friction contact force')  
        
        
    # plot contact info for the shoe-gear, with a optional time interval
    def plot_shoeGearContactInfo(self, tmin = -1, tmax = -1):
        # plot normal contact force info, # contacts, FnMax, FnAvg, FnStdDev
        figA, axarrA = plt.subplots(4, sharex=True)
        
        # plot friction force info, # contacts, FtMax, FtAvg, FtStdDev
        figB, axarrB = plt.subplots(4,sharex=True)
        
        # plot # of contacts, time in contact, max time in contact
        figC, axarrC = plt.subplots(2,sharex=True)
        
        # plot contact point relative position, velcoity, pitch radius/velocity
        figD, axarrD = plt.subplots(3,sharex=True)        
        
        # data headers to pull from
        inDat = ['time','Ncontacts','t_persist','t_persist_max','FnMax','FnAvg','FnSig','FtMax','FtAvg','FtSig','xRel','yRel','zRel','VxRel','VyRel','VzRel']      
        
        # data frame from the headers
        DFsgci = pd.DataFrame(self._getDFhandle('shoeGearContact'), columns = inDat)
        
        # for all plots: take the per step data and find 1) avg., 2) stdev over the whole sim time
        # If the time interval is specified, find the row which corresponds to t_begin and t_end.
        # Useful info for a specified time interval: Avg. # of contacts
        tminidx = 0
        tmaxidx = -1
        # find statistics based on a specified (tmin,tmax) time span
        if(tmin > 0):
            # first index in this list is where we start
            tminidxlist = DFsgci[ DFsgci['time'] > tmin].index.tolist()
            tminidx = tminidxlist[0]
            # last index in this list is the end point
            tmaxidxlist = DFsgci[ DFsgci['time'] < tmax].index.tolist()
            tmaxidx = tmaxidxlist[-1]
            
            #) A ---- # of contacts on the gear body
            n_contacts_avg = DFsgci['Ncontacts'][tminidx:tmaxidx].mean()
            n_contacts_sig = py.sqrt(DFsgci['Ncontacts'][tminidx:tmaxidx].var())
            # add these two as cosntant columns to the dataframe
            DFsgci['NcontactsAvg'] = n_contacts_avg
            DFsgci['NcontactsSig'] = n_contacts_sig
    
            # --- B) max Normal Force 
            FnMaxAvg = DFsgci['FnMax'][tminidx:tmaxidx].mean()
            FnMaxSig = py.sqrt(DFsgci['FnMax'][tminidx:tmaxidx].var() )
            # add as constant columns to the DF
            DFsgci['FnMaxAvg'] = FnMaxAvg
            DFsgci['FnMaxSig'] = FnMaxSig
            
            # --- C) average normal force
            FnAvgAvg = DFsgci['FnAvg'][tminidx:tmaxidx].mean()
            FnAvgSig = py.sqrt(DFsgci['FnAvg'][tminidx:tmaxidx].var() )
            # add to DF
            DFsgci['FnAvgAvg'] = FnAvgAvg
            DFsgci['FnAvgSig'] = FnAvgSig  
            
            # --- D) stdev normal force
            FnSigAvg = DFsgci['FnSig'][tminidx:tmaxidx].mean()
            FnSigSig = py.sqrt(DFsgci['FnSig'][tminidx:tmaxidx].var() )
            # add to DF
            DFsgci['FnSigAvg'] = FnSigAvg
            DFsgci['FnSigSig'] = FnSigSig
            
        else:
            #) A ---- # of contacts on the gear body
            n_contacts_avg = DFsgci['Ncontacts'].mean()
            n_contacts_sig = py.sqrt(DFsgci['Ncontacts'].var())
            # add these two as cosntant columns to the dataframe
            DFsgci['NcontactsAvg'] = n_contacts_avg
            DFsgci['NcontactsSig'] = n_contacts_sig
    
            # --- B) max Normal Force 
            FnMaxAvg = DFsgci['FnMax'].mean()
            FnMaxSig = py.sqrt(DFsgci['FnMax'].var() )
            # add as constant columns to the DF
            DFsgci['FnMaxAvg'] = FnMaxAvg
            DFsgci['FnMaxSig'] = FnMaxSig
            
            # --- C) average normal force
            FnAvgAvg = DFsgci['FnAvg'].mean()
            FnAvgSig = py.sqrt(DFsgci['FnAvg'].var() )
            # add to DF
            DFsgci['FnAvgAvg'] = FnAvgAvg
            DFsgci['FnAvgSig'] = FnAvgSig  
            
            # --- D) stdev normal force
            FnSigAvg = DFsgci['FnSig'].mean()
            FnSigSig = py.sqrt(DFsgci['FnSig'].var() )
            # add to DF
            DFsgci['FnSigAvg'] = FnSigAvg
            DFsgci['FnSigSig'] = FnSigSig
        
        # ---- first plot: 4 subplots for the normal force data
        DFsgci.plot(ax = axarrA[0], linewidth=1.5, x='time', y=['Ncontacts','NcontactsAvg','NcontactsSig'])
        DFsgci.plot(ax = axarrA[1], linewidth=1.5, x='time', y=['FnMax','FnMaxAvg','FnMaxSig'])
        DFsgci.plot(ax = axarrA[2], linewidth=1.5, x='time', y=['FnAvg','FnAvgAvg','FnAvgSig'])
        DFsgci.plot(ax = axarrA[3], linewidth=1.5, x='time', y=['FnSig','FnSigAvg','FnSigSig'])
        
        #if specified, set the xlim of the 3 plots
        if(tmin > 0):
            xminmax = self._get_timeMinMax(DFsgci['time'].iget(-1), tmin, tmax)
            axarrA[0].set_xlim(xminmax[0], xminmax[1])
            axarrB[0].set_xlim(xminmax[0], xminmax[1])
            axarrC[0].set_xlim(xminmax[0], xminmax[1])
            
        # first plot label axes 
        axarrA[0].set_xlabel('time [s]')
        axarrA[0].set_ylabel('# contacts')
        axarrA[1].set_ylabel('max Force [N]')
        axarrA[2].set_ylabel('avg. Force[N}')
        axarrA[3].set_ylabel('std. dev Force [N]')
        # first plot, legend
        axarrA[0].legend()
        axarrA[1].legend()
        axarrA[2].legend()
        axarrA[3].legend()
        axarrA[0].set_title('shoe0-Gear normal contact force')

        # for all plots: take the per step data and find 1) avg., 2) stdev over the input time span     
        if(tmin > 0):
            # --- B) max Normal Force 
            FtMaxAvg = DFsgci['FtMax'][tminidx:tmaxidx].mean()
            FtMaxSig = py.sqrt(DFsgci['FtMax'][tminidx:tmaxidx].var() )
            # add as constant columns to the DF
            DFsgci['FtMaxAvg'] = FtMaxAvg
            DFsgci['FtMaxSig'] = FtMaxSig
            
            # --- C) average normal force
            FtAvgAvg = DFsgci['FtAvg'][tminidx:tmaxidx].mean()
            FtAvgSig = py.sqrt(DFsgci['FtAvg'][tminidx:tmaxidx].var() )
            # add to DF
            DFsgci['FtAvgAvg'] = FtAvgAvg
            DFsgci['FtAvgSig'] = FtAvgSig       
            
            # --- D) stdev normal force
            FtSigAvg = DFsgci['FtSig'][tminidx:tmaxidx].mean()
            FtSigSig = py.sqrt(DFsgci['FtSig'][tminidx:tmaxidx].var() )
            # add to DF
            DFsgci['FtSigAvg'] = FtSigAvg
            DFsgci['FtSigSig'] = FtSigSig        
        else:
            # for all plots: take the per step data and find 1) avg., 2) stdev over the whole sim time
            # --- B) max Normal Force 
            FtMaxAvg = DFsgci['FtMax'].mean()
            FtMaxSig = py.sqrt(DFsgci['FtMax'].var() )
            # add as constant columns to the DF
            DFsgci['FtMaxAvg'] = FtMaxAvg
            DFsgci['FtMaxSig'] = FtMaxSig
            
            # --- C) average normal force
            FtAvgAvg = DFsgci['FtAvg'].mean()
            FtAvgSig = py.sqrt(DFsgci['FtAvg'].var() )
            # add to DF
            DFsgci['FtAvgAvg'] = FtAvgAvg
            DFsgci['FtAvgSig'] = FtAvgSig       
            
            # --- D) stdev normal force
            FtSigAvg = DFsgci['FtSig'].mean()
            FtSigSig = py.sqrt(DFsgci['FtSig'].var() )
            # add to DF
            DFsgci['FtSigAvg'] = FtSigAvg
            DFsgci['FtSigSig'] = FtSigSig        
        
        
        # -----  create second plot, 4 subplots for the friction contact force data
        DFsgci.plot(ax = axarrB[0], linewidth=1.5, x='time', y=['Ncontacts','NcontactsAvg','NcontactsSig'])
        DFsgci.plot(ax = axarrB[1], linewidth=1.5, x='time', y=['FtMax','FtMaxAvg','FtMaxSig'])
        DFsgci.plot(ax = axarrB[2], linewidth=1.5, x='time', y=['FtAvg','FtAvgAvg','FtAvgSig'])
        DFsgci.plot(ax = axarrB[3], linewidth=1.5, x='time', y=['FtSig','FtSigAvg','FtSigSig'])
        
        # second plot label axes
        axarrB[0].set_xlabel('time [s]')
        axarrB[0].set_ylabel('# contacts')
        axarrB[1].set_ylabel('max Force [N]')
        axarrB[2].set_ylabel('avg. Force[N]')
        axarrB[3].set_ylabel('std. dev Force [N]')
        # first plot, legend
        axarrB[0].legend()
        axarrB[1].legend()
        axarrB[2].legend()
        axarrB[3].legend()
        axarrB[0].set_title('shoe0-Gear friction contact force')     
        
        # -----  create third plot, # of contacts, time-persist
        DFsgci.plot(ax = axarrC[0], linewidth=1.5, x='time', y=['Ncontacts'])
        DFsgci.plot(ax = axarrC[1], linewidth=1.5, x='time', y=['t_persist'])
        
        # plot label axes, legend
        axarrC[0].set_xlabel('time [s]')
        axarrC[0].set_ylabel('# contacts [-]')
        axarrC[1].set_ylabel('time [s]')
        #  legend
        axarrC[0].legend()
        axarrC[1].legend()
        axarrC[0].set_title('shoe0-gear contact persistence')
        
        # the pitch radius of the contact point will be in the x-y dims, w.r.t. gear c-sys
        pitch_rad = (DFsgci['xRel']**2 + DFsgci['yRel']**2)**0.5
        pitch_vel = (DFsgci['VxRel']**2 + DFsgci['VyRel']**2)**0.5
        DFsgci['pitchRad'] = pitch_rad
        DFsgci['pitchVel'] = pitch_vel
        
         # -----  fourth plot, follow a single gear-shoe contact point, its pos and vel relative to the gear c-sys
        DFsgci.plot(ax = axarrD[0], linewidth=1.5, x='time', y=['xRel','yRel','zRel'])
        DFsgci.plot(ax = axarrD[1], linewidth=1.5, x='time', y=['VxRel','VyRel','VzRel'])
        DFsgci.plot(ax = axarrD[2], linewidth=1.5, x='time', y=['pitchRad'])
        axRHS = axarrD[2].twinx()
        axRHS.plot(DFsgci['time'], DFsgci['pitchVel'],'r--', linewidth = 1.5, label = 'pitch vel.')
        
        #if specified, set the xlim of the 3 plots
        if(tmin > 0):
            axarrD[0].set_xlim(xminmax[0], xminmax[1])        
        
        # plot label axes, legend
        axarrD[0].set_xlabel('time [s]')
        axarrD[0].set_ylabel('relative position [m]')
        axarrD[1].set_ylabel('realtive vel. [m/s]')
        axarrD[2].set_ylabel('relative position [m]')
        axRHS.set_ylabel('relative vel. [m/s]')
        #  legend
        axarrD[0].legend()
        axarrD[1].legend()
        axarrD[2].legend()
        axRHS.legend(loc='center right')
        axarrD[0].set_title('shoe0-gear, track a contact point, position, vel. relative to gear c-sys')
      
if __name__ == '__main__':
    
    # logger
    import logging as lg
    lg.basicConfig(fileName = 'logFile.log', level=lg.WARN, format='%(message)s')
    # default font size
    font = {'size' : 14}
    matplotlib.rc('font', **font)       
    
    #  **********************************************************************    
    #  ===============   USER INPUT   =======================================
    # laptop data dir
    # data_dir = 'E:/Chrono-T_Build/bin/Release/'
    # desktop data dir
    data_dir = 'D:/Chrono_github_Build/bin/outdata_driveChain/'
    
    # list of data files to plot
    gearSubsys = 'test_driveChain_gear.csv'
    idlerSubsys = 'test_driveChain_idler.csv'
    ptrainSubsys = 'test_driveChain_ptrain.csv'
    shoe0 = 'test_driveChain_shoe0.csv'
    gearCV = 'test_driveChain_GearCV.csv'
    idlerCV = 'test_driveChain_idler0CV.csv'
    rollerCV = 'test_driveChain_roller0CV.csv'
    gearContact = 'test_driveChain_gearContact.csv'
    shoeGearContact = 'test_driveChain_shoe0GearContact.csv'
    
    data_files = [data_dir + gearSubsys, data_dir + idlerSubsys, data_dir + ptrainSubsys, data_dir + shoe0, data_dir + gearCV, data_dir + idlerCV, data_dir + rollerCV, data_dir + gearContact, data_dir+shoeGearContact]
    handle_list = ['Gear','idler','ptrain','shoe0','gearCV','idlerCV','rollerCV','gearContact','shoeGearContact']
    
 
    # construct with file list and list of legend
    Chain = DriveChain_panda(data_files,handle_list)
    
    tmin = -1
    tmax = 2.54
    
    # 1) plot the gear body info
    Chain.plot_gear()
    
    # 2) plot idler body info, tensioner force
    Chain.plot_idler(tmin,tmax)

    # 3) plot powertrain info
    Chain.plot_ptrain()    
    
    # 4) plot shoe 0 body info, and pin 0 force/torque
    Chain.plot_shoe(tmin,tmax)
    
    # 5) plot gear Constraint Violations
    Chain.plot_gearCV(tmin,tmax)
    
    # 6) plot idler Constraint Violations
    Chain.plot_idlerCV(tmin,tmax)
    
    # 7) plot roller Constraint Violations
    Chain.plot_rollerCV(tmin,tmax)
    
    # 8) from the contact report callback function, gear contact info
    Chain.plot_gearContactInfo(tmin,tmax)
    
    # 9)  from shoe-gear report callback function, contact info
    Chain.plot_shoeGearContactInfo(tmin,tmax)

py.show()