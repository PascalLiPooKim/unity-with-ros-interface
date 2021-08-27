# -*- coding: utf-8 -*-
"""
Created on Sat Jul  3 23:37:51 2021

@author: paswe
"""

#%%
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from collections import Counter
import scipy.stats as stats
from scipy.optimize import curve_fit
#import seaborn as sns
from scipy.stats import norm

#%%
#sns.set()
plt.style.use("seaborn")
params = {
   'axes.labelsize': 25,
   'font.size': 25,
   'font.family': 'sans-serif',
   'font.serif': 'Arial',
   'legend.fontsize': 15,
   'xtick.labelsize': 25,
   'ytick.labelsize': 25, 
   'figure.figsize': [25, 15],
   'axes.titlesize': 30,
   } 

plt.rcParams.update({"lines.markeredgewidth" : 3,
                     "errorbar.capsize" : 10})
plt.rcParams.update(params)

#%%
"""
PRELIMINARY EXPERIMENT DATA
"""
x = np.random.randint(10, 30, size=(5, 3))
#print(x)
#n_bins = 10
num = np.array([1, 2, 3, 4, 5])

path_3FAS = r"C:\Users\paswe\OneDrive\Desktop\3 Fixed Audio Sources Preliminary Test V2.csv"
path_5FAS = r"D:\GitHub\MSc Robotics and Computation\unity-with-ros-interface\GoogleFormsData\5 Fixed Audio Sources Preliminary Test V2.csv"
path_3DAS = r"D:\GitHub\MSc Robotics and Computation\unity-with-ros-interface\GoogleFormsData\3 Dynamic Audio Sources Preliminary Test V2.csv"


#df_3FAS = pd.read_csv(path_3FAS)
#print(type(df_3FAS["Total score"]))
#print(float(df_3FAS["Total score"][1]))
#df_5FAS = pd.read_csv(path_5FAS)
#df_3DAS = pd.read_csv(path_3DAS)

total_score_3FAS = [14, 15, 8, 13, 13, 12, 15, 13]
total_score_5FAS = [14, 15, 12, 13, 13, 14, 15, 13]
total_score_3DAS = [14, 13, 9, 8, 14, 13, 15, 14]

candidates_num = np.linspace(1, 8, num=8, dtype=int)


prelim_df = pd.DataFrame({'total_score_3FAS':[14, 15, 8, 13, 13, 12, 15, 13], 
                          'total_score_5FAS' :[14, 15, 12, 13, 13, 14, 15, 13], 
                          'total_score_3DAS':[14, 13, 9, 8, 14, 13, 15, 14]})


print(stats.ttest_ind(total_score_3FAS, total_score_5FAS), 
      stats.ttest_ind(total_score_3FAS, total_score_3DAS), 
      stats.ttest_ind(total_score_3DAS, total_score_5FAS))

#%%
"""
PRELIMINARY EXPERIMENT AVERAGE AND STANDARD DEVIATION PLOT
"""

plt.bar(0, np.mean(total_score_3FAS), yerr = np.std(total_score_3FAS), align='center', alpha=0.5, ecolor = 'red')
plt.bar(1, np.mean(total_score_5FAS), yerr = np.std(total_score_5FAS), align='center', alpha=0.5, ecolor = 'red')
plt.bar(2, np.mean(total_score_3DAS), yerr = np.std(total_score_3DAS), align='center', alpha=0.5, ecolor = 'red')
plt.xticks(np.arange(3), ['3 Fixed Audio Sources', '5 Fixed Audio Sources', '3 Dynamic Audio Sources'])
plt.ylabel("Average score")
plt.title("Average Point of each System in the Preliminary Experiment")

plt.text(0, np.mean(total_score_3FAS)/2, "Mean: {} \n\n Std: {:.2f}".format(np.mean(total_score_3FAS), 
                    np.std(total_score_3FAS)), fontsize=20, horizontalalignment='center', verticalalignment='center', color = 'white')

plt.text(1, np.mean(total_score_5FAS)/2, "Mean: {} \n\n Std: {:.2f}".format(np.mean(total_score_5FAS), 
                    np.std(total_score_5FAS)), fontsize=20, horizontalalignment='center', verticalalignment='center', color = 'white')

plt.text(2, np.mean(total_score_3DAS)/2, "Mean: {} \n\n Std: {:.2f}".format(np.mean(total_score_3DAS), 
                    np.std(total_score_3DAS)), fontsize=20, horizontalalignment='center', verticalalignment='center', color = 'white')



#%%
"""
PRELIMINARY EXPERIMENT TOTAL SCORE VS CANDIDATE NUMBER PLOT
"""

plt.figure()
#colors = ['red', 'tan', 'lime']
metrics =['3 Fixed Audio Sources', '5 Fixed Audio Sources', '3 Dynamic Audio Sources']
#plt.hist(x, n_bins, density=True, histtype='bar', color=colors, label=metrics)
plt.bar(candidates_num-0.2, total_score_3FAS, width=0.2, color='b', align='center' , alpha=0.8)
plt.bar(candidates_num, total_score_5FAS, width=0.2, color='g', align='center', alpha=0.8)
plt.bar(candidates_num+0.2, total_score_3DAS, width=0.2, color='r', align='center', alpha=0.8)
plt.legend(prop={'size': 10})
plt.ylabel("Total score")
plt.xlabel("Candidate Number")
plt.legend(metrics, loc='best')
#plt.grid(axis='y')
plt.title('Total Score for the 8 Candidates in the Preliminary Test for \n3 Different Auditory Systems')


#!plt.text(2, 6, r'an equation: $E=mc^2$', fontsize=15)
#plt.show()

#%%
"""
PRELIMINARY EXPERIMENT NUMBER OF RESPONDENTS VS SCORE PLOT
"""


count_3FAS = Counter(total_score_3FAS)
#print(count_3FAS)
count_5FAS = Counter(total_score_5FAS)
count_3DAS = Counter(total_score_3DAS)

scores_3FAS = [i for i in count_3FAS]
scores_5FAS = [i for i in count_5FAS]
scores_3DAS = [i for i in count_3DAS]

def func(x, a, b, c):
    # a Gaussian distribution
    return  a * 1/(c*np.sqrt(2*np.pi))* np.exp(-(x - b)**2.0 / (2 * c**2))

from matplotlib.ticker import MaxNLocator

ax = plt.figure().gca()
ax.yaxis.set_major_locator(MaxNLocator(integer=True))


#plt.figure()
#colors = ['red', 'tan', 'lime']
metrics =['3 Fixed Audio Sources: Avg = {0}, Std = {1}'.format(np.mean(total_score_3FAS), np.round(np.std(total_score_3FAS), 3)), 
          '5 Fixed Audio Sources: Avg = {0}, Std = {1}'.format(np.mean(total_score_5FAS), np.round(np.std(total_score_5FAS), 3)), 
          '3 Dynamic Audio Sources: Avg = {0}, Std = {1}'.format(np.mean(total_score_3DAS), np.round(np.std(total_score_3DAS), 3))]
#plt.hist(x, n_bins, density=True, histtype='bar', color=colors, label=metrics)
plt.bar(np.array(scores_3FAS)-0.2, [int(count_3FAS[i]) for i in count_3FAS], width=0.2, color='b', align='center')


#val_3FAS = np.linspace(np.mean(total_score_3FAS) - np.std(total_score_3FAS), np.mean(total_score_3FAS) + np.std(total_score_3FAS), 100)
#plt.plot(val_3FAS, 100*stats.norm.pdf(val_3FAS), np.mean(total_score_3FAS), np.std(total_score_3FAS))
print(scores_3FAS, [count_3FAS[i] for i in count_3FAS])
#guess = (10.0, np.mean(total_score_3FAS), 2)
popt, pcov = curve_fit(func, np.array(scores_3FAS)-0.2, [count_3FAS[i] for i in count_3FAS])
print(popt, pcov)
 
x = np.linspace(min(scores_3FAS)-1, max(scores_3FAS)+1, 100)
y = func(x, *popt)

#plt.plot(x + 0.2/2, y, c='g')




plt.bar(scores_5FAS, [int(count_5FAS[i]) for i in count_5FAS], width=0.2, color='g', align='center')
plt.bar(np.array(scores_3DAS)+0.2, [int(count_3DAS[i]) for i in count_3DAS], width=0.2, color='r', align='center')
plt.legend(prop={'size': 10})
plt.ylabel("Number of respondents")
plt.xlabel("Score")
plt.legend(metrics)
#plt.grid(axis='y')
plt.title('Number of Candidates having acheieved a certain Score for each Auditory System')




from scipy.interpolate import interp1d
from scipy import interpolate
from scipy.stats import skewnorm
f = interp1d(np.array(scores_3FAS)-0.2, [int(count_3FAS[i]) for i in count_3FAS])
#f = interpolate.splrep(np.array(scores_3FAS)-0.2, [int(count_3FAS[i]) for i in count_3FAS], s =0)
#plt.plot(np.array(scores_3FAS)-0.2, [int(count_3FAS[i]) for i in count_3FAS], 'kx')
new_scores_3FAS = np.linspace(min(np.array(scores_3FAS)-0.2)-1, max(np.array(scores_3FAS)-0.2)+1, num = 100, endpoint=True)
#plt.plot(new_scores_3FAS, f(new_scores_3FAS), '--')
plt.plot(new_scores_3FAS, func(new_scores_3FAS, 1, np.mean(total_score_3FAS), np.std(total_score_3FAS))*len(total_score_3FAS)*1, 'b')

#plt.plot(np.array(scores_5FAS), [int(count_5FAS[i]) for i in count_5FAS], 'kx')
new_scores_5FAS = np.linspace(min(np.array(scores_5FAS))-1, max(np.array(scores_5FAS))+1, num = 100, endpoint=True)
#plt.plot(new_scores_3FAS, f(new_scores_3FAS), '--')
plt.plot(new_scores_5FAS, func(new_scores_5FAS, 1, np.mean(total_score_5FAS), np.std(total_score_5FAS))*len(total_score_5FAS)*1, 'g')

#plt.plot(np.array(scores_3DAS)+0.2, [int(count_3DAS[i]) for i in count_3DAS], 'kx')
new_scores_3DAS = np.linspace(min(np.array(scores_3DAS)+0.2)-1, max(np.array(scores_3DAS)+0.2)+1, num = 100, endpoint=True)
#plt.plot(new_scores_3FAS, f(new_scores_3FAS), '--')
plt.plot(new_scores_3DAS, func(new_scores_3DAS, 1, np.mean(total_score_3DAS), 
                               np.std(total_score_3DAS))*len(total_score_3DAS)*1, 'r')

#plt.plot(np.array(scores_5FAS), [int(count_5FAS[i]) for i in count_5FAS], 'kx')
##new_scores_5FAS = np.linspace(min(np.array(scores_5FAS)), max(np.array(scores_5FAS)), num = 100, endpoint=True)
#plt.plot(np.array(scores_5FAS), [int(count_5FAS[i]) for i in count_5FAS], '--')
#
#plt.plot(np.array(scores_3FAS)-0.2, [int(count_3FAS[i]) for i in count_3FAS], 'kx')
#new_scores_3FAS = np.linspace(min(np.array(scores_3FAS)-0.2), max(np.array(scores_3FAS)-0.2), num = 100, endpoint=True)
#plt.plot(new_scores_3FAS, f(new_scores_3FAS), '--')
#plt.plot(new_scores_3FAS, interpolate.splev(new_scores_3FAS, f))
plt.show()

#x = total_score_3FAS
#y = total_score_5FAS
#z = total_score_3DAS
#bins = np.linspace(0, 15, 16)
#
##plt.hist([x, y, z], bins, label=['x', 'y', 'z'])
#plt.bar([x, y, z], 0.2, label=['x', 'y', 'z'])
#plt.legend(loc='upper right')
#plt.show()
#data_3FAS = []
#data_5FAS = []
#for i in range(len(scores_3FAS)): 
#    data_3FAS += [scores_3FAS[i]]*[count_3FAS[j] for j in count_3FAS][i] 
#    
#for i in range(len(scores_5FAS)): 
#    data_5FAS += [scores_5FAS[i]]*[count_5FAS[j] for j in count_5FAS][i] 
#sns.set()
#plt.figure()
#sns.barplot(scores_3FAS,[count_3FAS[i] for i in count_3FAS])
#sns.barplot(scores_5FAS, [count_5FAS[i] for i in count_5FAS])
#plt.show()
#sns.distplot(data_5FAS, kde=False)
#sns.distplot(data_5FAS, kde=False)
#plt.show()

#plt.hist(np.array(total_score_3FAS), alpha = 0.5)
#plt.hist(np.array(total_score_5FAS), alpha = 0.5)
#plt.hist(np.array(total_score_3DAS), alpha = 0.5)
#plt.show()
#%%
"""
DO NOT RUN THIS CELL
"""


def skew_norm_pdf(x,e=0,w=1,a=0):
    # adapated from:
    # http://stackoverflow.com/questions/5884768/skew-normal-distribution-in-scipy
    t = (x-e) / w
    return 2.0 * w * stats.norm.pdf(t) * stats.norm.cdf(a*t)

ax = plt.figure().gca()
ax.yaxis.set_major_locator(MaxNLocator(integer=True))


#plt.figure()
#colors = ['red', 'tan', 'lime']
metrics =['3 Fixed Audio Sources: Avg = {0}, Std = {1}'.format(np.mean(total_score_3FAS), np.round(np.std(total_score_3FAS), 3)), 
          '5 Fixed Audio Sources: Avg = {0}, Std = {1}'.format(np.mean(total_score_5FAS), np.round(np.std(total_score_5FAS), 3)), 
          '3 Dynamic Audio Sources: Avg = {0}, Std = {1}'.format(np.mean(total_score_3DAS), np.round(np.std(total_score_3DAS), 3))]
#plt.hist(x, n_bins, density=True, histtype='bar', color=colors, label=metrics)
plt.bar(np.array(scores_3FAS)-0.2, [int(count_3FAS[i]) for i in count_3FAS], width=0.2, color='b', align='center')


#val_3FAS = np.linspace(np.mean(total_score_3FAS) - np.std(total_score_3FAS), np.mean(total_score_3FAS) + np.std(total_score_3FAS), 100)
#plt.plot(val_3FAS, 100*stats.norm.pdf(val_3FAS), np.mean(total_score_3FAS), np.std(total_score_3FAS))
print(scores_3FAS, [count_3FAS[i] for i in count_3FAS])
#guess = (10.0, np.mean(total_score_3FAS), 2)
popt, pcov = curve_fit(func, np.array(scores_3FAS)-0.2, [count_3FAS[i] for i in count_3FAS])
print(popt, pcov)
 
x = np.linspace(min(scores_3FAS)-1, max(scores_3FAS)+1, 100)
y = func(x, *popt)

#plt.plot(x + 0.2/2, y, c='g')




plt.bar(scores_5FAS, [int(count_5FAS[i]) for i in count_5FAS], width=0.2, color='g', align='center')
plt.bar(np.array(scores_3DAS)+0.2, [int(count_3DAS[i]) for i in count_3DAS], width=0.2, color='r', align='center')
plt.legend(prop={'size': 10})
plt.ylabel("Number of respondents")
plt.xlabel("Score")
plt.legend(metrics)
plt.grid(axis='y')
plt.title('Total score for the 8 candidates in the preliminary \n test for 3 different auditory systems')


import scipy

from scipy.interpolate import interp1d
from scipy import interpolate
from scipy.stats import skewnorm
f = interp1d(np.array(scores_3FAS)-0.2, [int(count_3FAS[i]) for i in count_3FAS])
#f = interpolate.splrep(np.array(scores_3FAS)-0.2, [int(count_3FAS[i]) for i in count_3FAS], s =0)
#plt.plot(np.array(scores_3FAS)-0.2, [int(count_3FAS[i]) for i in count_3FAS], 'kx')
new_scores_3FAS = np.linspace(min(np.array(scores_3FAS)-0.2)-1, max(np.array(scores_3FAS)-0.2)+1, num = 100, endpoint=True)
#plt.plot(new_scores_3FAS, f(new_scores_3FAS), '--')
skew = scipy.stats.skew(new_scores_3FAS)
plt.plot(new_scores_3FAS, skew_norm_pdf(new_scores_3FAS, np.mean(total_score_3FAS), np.std(total_score_3FAS), skew)*len(total_score_3FAS)*0.5, 'b')

#plt.plot(np.array(scores_5FAS), [int(count_5FAS[i]) for i in count_5FAS], 'kx')
new_scores_5FAS = np.linspace(min(np.array(scores_5FAS))-1, max(np.array(scores_5FAS))+1, num = 100, endpoint=True)
##plt.plot(new_scores_3FAS, f(new_scores_3FAS), '--')
skew = scipy.stats.skew(new_scores_3FAS)
plt.plot(new_scores_5FAS, skew_norm_pdf(new_scores_5FAS, np.mean(total_score_5FAS), np.std(total_score_5FAS), skew)*len(total_score_5FAS)*0.5, 'g')
#
##plt.plot(np.array(scores_3DAS)+0.2, [int(count_3DAS[i]) for i in count_3DAS], 'kx')
skew = scipy.stats.skew(new_scores_3FAS)
new_scores_3DAS = np.linspace(min(np.array(scores_3DAS)+0.2)-1, max(np.array(scores_3DAS)+0.2)+1, num = 100, endpoint=True)
##plt.plot(new_scores_3FAS, f(new_scores_3FAS), '--')
plt.plot(new_scores_3DAS, skew_norm_pdf(new_scores_5FAS, np.mean(total_score_5FAS), np.std(total_score_5FAS), skew)*len(total_score_3DAS)*0.5, 'r')

#plt.plot(np.array(scores_5FAS), [int(count_5FAS[i]) for i in count_5FAS], 'kx')
##new_scores_5FAS = np.linspace(min(np.array(scores_5FAS)), max(np.array(scores_5FAS)), num = 100, endpoint=True)
#plt.plot(np.array(scores_5FAS), [int(count_5FAS[i]) for i in count_5FAS], '--')
#
#plt.plot(np.array(scores_3FAS)-0.2, [int(count_3FAS[i]) for i in count_3FAS], 'kx')
#new_scores_3FAS = np.linspace(min(np.array(scores_3FAS)-0.2), max(np.array(scores_3FAS)-0.2), num = 100, endpoint=True)
#plt.plot(new_scores_3FAS, f(new_scores_3FAS), '--')
#plt.plot(new_scores_3FAS, interpolate.splev(new_scores_3FAS, f))
plt.show()

#%%
"""
SAGAT EXPERIMENT DATA
"""

SAGAT_total_scores_VA = [11, 8, 9, 10]
SAGAT_color_VA = []
SAGAT_total_scores_V = [9, 5, 4 , 8]
candidate_num = np.linspace(1, 6, num=6, dtype=int)

np.mean(SAGAT_total_scores_VA), np.mean(SAGAT_total_scores_V)

import scipy as sp
print(sp.stats.ttest_ind(SAGAT_total_scores_VA, SAGAT_total_scores_V))

#%%
"""
SAGAT EXPERIMENT MEAN AND STD PLOT
"""
plt.figure()

metrics =['Vision', 'Audio & Vision']
#plt.hist(x, n_bins, density=True, histtype='bar', color=colors, label=metrics)
#plt.bar(candidate_num-0.2, x[:, 0], width=0.4, color='b', align='center')
#plt.bar(candidate_num+0.2, x[:, 1], width=0.4, color='g', align='center')
plt.bar(0, np.mean(SAGAT_total_scores_VA), width=0.5, yerr = np.std(SAGAT_total_scores_VA), align='center', alpha=0.5, ecolor = 'red')
plt.bar(1, np.mean(SAGAT_total_scores_V), width=0.5, yerr = np.std(SAGAT_total_scores_V), align='center', alpha=0.5, ecolor = 'red')

plt.xticks(np.arange(2), ['Vision & Audio ({} Candidates)'.format(len(SAGAT_total_scores_VA)), 'Vision ({} Candidates)'.format(len(SAGAT_total_scores_V))])
plt.yticks(np.arange(12))

plt.ylabel("Average Score")
plt.xlabel("Feedback Enabled")
#plt.legend(metrics)
#plt.grid(axis='y')
plt.title('Average score of SAGAT under 2 test cases')


plt.text(0, 1, "{}% Daily".format(0.25), fontsize=20, horizontalalignment='center', verticalalignment='center')
plt.text(0, 3, "{}% Weekly".format(0.25), fontsize=20, horizontalalignment='center', verticalalignment='center')
plt.text(0, 5, "{}% Monthly".format(0.25), fontsize=20, horizontalalignment='center', verticalalignment='center')
plt.text(0, 7, "{}% Once a year".format(0.25), fontsize=20, horizontalalignment='center', verticalalignment='center')
plt.text(1, 1, "{}% Daily".format(0.25), fontsize=20, horizontalalignment='center', verticalalignment='center')
plt.text(1, 2, "{}% Monthly".format(0.25), fontsize=20, horizontalalignment='center', verticalalignment='center')
plt.text(1, 3, "{}% Weekly".format(0.25), fontsize=20, horizontalalignment='center', verticalalignment='center')
plt.text(1, 4, "{}% Once a year".format(0.25), fontsize=20, horizontalalignment='center', verticalalignment='center')

plt.show()

#%%
"""
SAGAT EXPERIMENT SCORE VS CANDIDATE NUMBER PLOT
"""
ax = plt.figure()

metrics =['Audio & Vision', 'Vision']
plt.bar([1, 2, 4, 5], SAGAT_total_scores_VA, width=0.5, color='g', align='center', alpha=0.5)
plt.bar([3, 6, 7, 8], SAGAT_total_scores_V, width=0.5, color='r' , align='center', alpha=0.5)
plt.ylabel("SAGAT Total Score")
plt.xlabel("Candidate Number")
plt.legend(metrics)
#[extra, bar_0_10, bar_10_100], ("My explanatory text", "0-10", "10-100")
#plt.grid(axis='y')
plt.yticks(np.arange(12))
plt.title('Total Score of the {} Participants'.format(len(SAGAT_total_scores_V) + len(SAGAT_total_scores_VA)))

plt.text(5, 5, r"$\varnothing$", fontsize=40, horizontalalignment='center', verticalalignment='center')
plt.text(7, 2, r"$\varnothing$", fontsize=40, horizontalalignment='center', verticalalignment='center')
#plt.legend([r"$\varnothing$"], ("Robotics & Teleoperation Expertise"))
#plt.text()
plt.show()

#%%
"""
DO NOT RUN THIS CELL
"""

plt.figure()
#colors = ['red', 'tan', 'lime']
metrics =['Audio', 'Vision', 'Audio + Vision']
#plt.hist(x, n_bins, density=True, histtype='bar', color=colors, label=metrics)
plt.bar(num-0.2, x[:, 0], width=0.2, color='b', align='center')
plt.bar(num, x[:, 1], width=0.2, color='g', align='center')
plt.bar(num+0.2, x[:, 2], width=0.2, color='r', align='center')
plt.legend(prop={'size': 10})
plt.ylabel("Completion Time(s)")
plt.xlabel("Candidate Number")
plt.legend(metrics)
plt.grid(axis='y')
plt.title('Navigation Completion Time Histogram')

plt.show()

#%%
#mental_dem_VA = [1, 4, 4]
#physical_dem_VA = [1, 2, 3]
"""
COURSE 3 NASA TLX DATA + PLOT
"""



NASATLX_path = r"C:\Users\paswe\OneDrive\Desktop\NASA TLX Course 3.csv"
df = pd.read_csv(NASATLX_path, engine='python')
#print(df.dtypes)
print(df["Mental Demand: How mentally demanding was the task?.1"])
NASA_TLX_VA = df[["Mental Demand: How mentally demanding was the task?", 
                  "Physical Demand: How physically demanding was the task?", 
                  "Temporal Demand: How hurried or rushed was the pace of the task?", 
                  "Performance: How successful were you in accomplishing what you were asked to do?", 
                  "Effort: How hard did you have to work to accomplish your level of performance?",
                  "Frustration: How insecure, discouraged, irritated, stressed, and annoyed were you?"]]

print(NASA_TLX_VA.values)

NASA_TLX_V = df[["Mental Demand: How mentally demanding was the task?.1", 
                  "Physical Demand: How physically demanding was the task?.1", 
                  "Temporal Demand: How hurried or rushed was the pace of the task?.1", 
                  "Performance: How successful were you in accomplishing what you were asked to do?.1", 
                  "Effort: How hard did you have to work to accomplish your level of performance?.1",
                  "Frustration: How insecure, discouraged, irritated, stressed, and annoyed were you?.1"]]

print(NASA_TLX_V.values)

#fig, ax = plt.subplots()
yerr=np.array([[np.std(NASA_TLX_VA.values[:, i]), np.std(NASA_TLX_V.values[:, i])] for i in range(6)])
print(yerr)
#yerr = 
new_df = pd.DataFrame({"Mean of Vision & Audio": NASA_TLX_VA.values.mean(axis=0), "Mean of Vision": NASA_TLX_V.values.mean(axis=0), "Errors_VA":yerr[:, 0], "Errors_V":yerr[:, 1]})
new_df_plot = new_df[["Mean of Vision & Audio", "Mean of Vision"]].plot(kind="bar", rot=0, yerr = [yerr[:, 0], yerr[:, 1]], alpha = 0.5, capsize = 10, ecolor = "red")
new_df_plot.set_xticklabels(["Mental\nDemand", "Physical\nDemand", "Temporal\nDemand", "Performance\nLevel", "Effort\nLevel", "Frustration\nLevel"])
new_df_plot.set_yerr = new_df.Errors_VA.to_frame("Mean of Vision & Audio")
new_df_plot.set_ylabel("Avegare Point")
new_df_plot.set_title("NASA Task Load Index")
#ax.set_xticklabels(["Mental_demand"])
#print(sp.stats.ttest_ind(NASA_TLX_VA, NASA_TLX_V))

#plt.figure()
#plt.bar(new_df["Mean of Vision & Audio"])

#%%
"""
COURSE 2 NASA TLX DATA + PLOT
"""

NASATLX_path = r"C:\Users\paswe\OneDrive\Desktop\NASA TLX Course 2.csv"
df = pd.read_csv(NASATLX_path, engine='python')
#print(df.dtypes)
print(df["Mental Demand: How mentally demanding was the task?.1"])
NASA_TLX_VA = df[["Mental Demand: How mentally demanding was the task?", 
                  "Physical Demand: How physically demanding was the task?", 
                  "Temporal Demand: How hurried or rushed was the pace of the task?", 
                  "Performance: How successful were you in accomplishing what you were asked to do?", 
                  "Effort: How hard did you have to work to accomplish your level of performance?",
                  "Frustration: How insecure, discouraged, irritated, stressed, and annoyed were you?"]]

print(NASA_TLX_VA.values)

NASA_TLX_V = df[["Mental Demand: How mentally demanding was the task?.1", 
                  "Physical Demand: How physically demanding was the task?.1", 
                  "Temporal Demand: How hurried or rushed was the pace of the task?.1", 
                  "Performance: How successful were you in accomplishing what you were asked to do?.1", 
                  "Effort: How hard did you have to work to accomplish your level of performance?.1",
                  "Frustration: How insecure, discouraged, irritated, stressed, and annoyed were you?.1"]]

print(NASA_TLX_V.values)

#fig, ax = plt.subplots()
yerr=np.array([[np.std(NASA_TLX_VA.values[:, i]), np.std(NASA_TLX_V.values[:, i])] for i in range(6)])
print(yerr)
#yerr = 
new_df = pd.DataFrame({"Mean of Vision & Audio": NASA_TLX_VA.values.mean(axis=0), "Mean of Vision": NASA_TLX_V.values.mean(axis=0), "Errors_VA":yerr[:, 0], "Errors_V":yerr[:, 1]})
new_df_plot = new_df[["Mean of Vision & Audio", "Mean of Vision"]].plot(kind="bar", rot=0, yerr = [yerr[:, 0], yerr[:, 1]], alpha = 0.5, capsize = 10, ecolor = "red")
new_df_plot.set_xticklabels(["Mental\nDemand", "Physical\nDemand", "Temporal\nDemand", "Performance\nLevel", "Effort\nLevel", "Frustration\nLevel"])
new_df_plot.set_yerr = new_df.Errors_VA.to_frame("Mean of Vision & Audio")
new_df_plot.set_ylabel("Avegare Point")
new_df_plot.set_title("NASA Task Load Index")




#%%
"""
COURSE 3 DATA FROM UNITY
"""

workload_CT_VA = [253, 264, 222]
workload_CT_V = [237, 243, 243]

workload_KH_VA = [132, 110, 78]
workload_KH_V = [129, 103, 124]

workload_C_VA = [0, 1, 2]
workload_C_V = [0, 1, 2]

print(stats.ttest_ind(workload_CT_VA, workload_CT_V), 
      stats.ttest_ind(workload_KH_VA, workload_KH_V), 
      stats.ttest_ind(workload_C_VA, workload_C_V))

#%%

plt.subplot(1, 3, 1)
plt.bar(0 , np.mean(workload_CT_VA), align='center', alpha=0.5)
plt.errorbar(0 , np.mean(workload_CT_VA), yerr =np.std(workload_CT_VA), capsize = 8, elinewidth = 3, markeredgewidth=3, ecolor='red')
plt.bar(1 , np.mean(workload_CT_V), align='center', alpha=0.5)
plt.errorbar(1 , np.mean(workload_CT_V), yerr =np.std(workload_CT_V), capsize = 8, elinewidth = 3, markeredgewidth=3, ecolor='red')
plt.xticks(np.arange(2), ['Vision & Audio', 'Vision'])
plt.title(r'$\mu$ of Completion Time')

plt.subplot(1, 3, 2)
plt.bar(0 , np.mean(workload_KH_VA), align='center', alpha=0.5)
plt.errorbar(0 , np.mean(workload_KH_VA), yerr =np.std(workload_KH_VA), capsize = 8, elinewidth = 3, markeredgewidth=3, ecolor='red')
plt.bar(1 , np.mean(workload_KH_V), align='center', alpha=0.5)
plt.errorbar(1 , np.mean(workload_KH_V), yerr =np.std(workload_KH_V), capsize = 8, elinewidth = 3, markeredgewidth=3, ecolor='red')
plt.xticks(np.arange(2), ['Vision & Audio', 'Vision'])
plt.title(r'$\mu$ of # Keystrokes')

plt.subplot(1, 3, 3)
plt.bar(0 , np.mean(workload_C_VA),  align='center', alpha=0.5)
plt.errorbar(0 , np.mean(workload_C_VA), yerr =np.std(workload_C_VA), capsize = 8, elinewidth = 3, markeredgewidth=3, ecolor='red')
plt.bar(1 , np.mean(workload_C_V),  align='center', alpha=0.5)
plt.errorbar(1 , np.mean(workload_C_V), yerr =np.std(workload_C_V), capsize = 8, elinewidth = 3, markeredgewidth=3, ecolor='red')
plt.xticks(np.arange(2), ['Vision & Audio', 'Vision'])
plt.title(r'$\mu$ of # Collisions')
plt.show()


#%%

"""
COURSE 2 DATA FROM UNITY
"""

workload_CT_VA = [179, 125, 101]
workload_CT_V = [106, 104, 121]

workload_KH_VA = [85, 78, 35]
workload_KH_V = [61, 50, 67]

workload_C_VA = [2, 1, 2]
workload_C_V = [1, 1, 4]

print(stats.ttest_ind(workload_CT_VA, workload_CT_V), 
      stats.ttest_ind(workload_KH_VA, workload_KH_V), 
      stats.ttest_ind(workload_C_VA, workload_C_V))


#%%

plt.subplot(1, 3, 1)
plt.bar(0 , np.mean(workload_CT_VA), align='center', alpha=0.5)
plt.errorbar(0 , np.mean(workload_CT_VA), yerr =np.std(workload_CT_VA), capsize = 8, elinewidth = 3, markeredgewidth=3, ecolor='red')
plt.bar(1 , np.mean(workload_CT_V), align='center', alpha=0.5)
plt.errorbar(1 , np.mean(workload_CT_V), yerr =np.std(workload_CT_V), capsize = 8, elinewidth = 3, markeredgewidth=3, ecolor='red')
plt.xticks(np.arange(2), ['Vision & Audio', 'Vision'])
plt.title(r'$\mu$ of Completion Time')

plt.subplot(1, 3, 2)
plt.bar(0 , np.mean(workload_KH_VA), align='center', alpha=0.5)
plt.errorbar(0 , np.mean(workload_KH_VA), yerr =np.std(workload_KH_VA), capsize = 8, elinewidth = 3, markeredgewidth=3, ecolor='red')
plt.bar(1 , np.mean(workload_KH_V), align='center', alpha=0.5)
plt.errorbar(1 , np.mean(workload_KH_V), yerr =np.std(workload_KH_V), capsize = 8, elinewidth = 3, markeredgewidth=3, ecolor='red')
plt.xticks(np.arange(2), ['Vision & Audio', 'Vision'])
plt.title(r'$\mu$ of # Keystrokes')

plt.subplot(1, 3, 3)
plt.bar(0 , np.mean(workload_C_VA),  align='center', alpha=0.5)
plt.errorbar(0 , np.mean(workload_C_VA), yerr =np.std(workload_C_VA), capsize = 8, elinewidth = 3, markeredgewidth=3, ecolor='red')
plt.bar(1 , np.mean(workload_C_V),  align='center', alpha=0.5)
plt.errorbar(1 , np.mean(workload_C_V), yerr =np.std(workload_C_V), capsize = 8, elinewidth = 3, markeredgewidth=3, ecolor='red')
plt.xticks(np.arange(2), ['Vision & Audio', 'Vision'])
plt.title(r'$\mu$ of # Collisions')
plt.show()



#%%
x = np.random.randint(30, 60, size=(5, 3))
print(x)
num = np.array([1, 2, 3, 4, 5])


#%%
plt.figure()
metrics =['Audio', 'Vision', 'Audio + Vision']
plt.bar(num-0.2, x[:, 0], width=0.2, color='b', align='center')
plt.bar(num, x[:, 1], width=0.2, color='g', align='center')
plt.bar(num+0.2, x[:, 2], width=0.2, color='r', align='center')
plt.legend(prop={'size': 10})
plt.ylabel("Number of keystrokes")
plt.xlabel("Candidate Number")
plt.legend(metrics)
plt.grid(axis='y')
plt.title('Keystrokes Count Histogram')
plt.show()

#%%
x = np.random.uniform(7.5, 10, (5, 3))
print(x)
cat = ["Audio", "Vision", "Audio + Vision"]


#%%
plt.figure()
#metrics =['Audio', 'Vision', 'Audio + Vision']
plt.bar(cat[0], x[:, 0], width=0.2, color='b', align='center')
plt.bar(cat[1], x[:, 1], width=0.2, color='g', align='center')
plt.bar(cat[2], x[:, 2], width=0.2, color='r', align='center')
#plt.legend(prop={'size': 10})
plt.ylabel("Average workload rating")
#plt.xlabel("Candidate Number")
#plt.legend(metrics)
plt.grid(axis='y')
plt.title('Average workload rating Histogram')
plt.show()