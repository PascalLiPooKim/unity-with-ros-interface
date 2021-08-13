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
import seaborn as sns
from scipy.stats import norm

#%%
params = {
   'axes.labelsize': 25,
   'font.size': 25,
   'font.family': 'sans-serif',
   'font.serif': 'Arial',
   'legend.fontsize': 15,
   'xtick.labelsize': 25,
   'ytick.labelsize': 25, 
   'figure.figsize': [25, 15],
   'axes.titlesize': 30
   } 
plt.rcParams.update(params)

#%%
x = np.random.randint(10, 30, size=(5, 3))
print(x)
#n_bins = 10
num = np.array([1, 2, 3, 4, 5])

#path_3FAS = r"D:\GitHub\MSc Robotics and Computation\unity-with-ros-interface\GoogleFormsData\3 Fixed Audio Sources Preliminary Test V2.csv"
#path_5FAS = r"D:\GitHub\MSc Robotics and Computation\unity-with-ros-interface\GoogleFormsData\5 Fixed Audio Sources Preliminary Test V2.csv"
#path_3DAS = r"D:\GitHub\MSc Robotics and Computation\unity-with-ros-interface\GoogleFormsData\3 Dynamic Audio Sources Preliminary Test V2.csv"
#
#
#df_3FAS = pd.read_csv(path_3FAS, engine="python")
#df_5FAS = pd.read_csv(path_5FAS)
#df_3DAS = pd.read_csv(path_3DAS)

total_score_3FAS = [14, 15, 8, 13, 13, 12, 15, 13]
total_score_5FAS = [14, 15, 12, 13, 13, 14, 15, 13]
total_score_3DAS = [14, 13, 9, 8, 14, 13, 15, 14]

candidates_num = np.linspace(1, 8, num=8, dtype=int)

#%%

plt.figure()
#colors = ['red', 'tan', 'lime']
metrics =['3 Fixed Audio Sources', '5 Fixed Audio Sources', '3 Dynamic Audio Sources']
#plt.hist(x, n_bins, density=True, histtype='bar', color=colors, label=metrics)
plt.bar(candidates_num-0.2, total_score_3FAS, width=0.2, color='b', align='center')
plt.bar(candidates_num, total_score_5FAS, width=0.2, color='g', align='center')
plt.bar(candidates_num+0.2, total_score_3DAS, width=0.2, color='r', align='center')
plt.legend(prop={'size': 10})
plt.ylabel("Total score")
plt.xlabel("Candidate Number")
plt.legend(metrics)
plt.grid(axis='y')
plt.title('Total score for the 8 candidates in the preliminary test for 3 different auditory systems')


#!plt.text(2, 6, r'an equation: $E=mc^2$', fontsize=15)
plt.show()

#%%
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
plt.grid(axis='y')
plt.title('Number of candidates having acheieved a certain score for each auditory system')




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
plt.plot(new_scores_3DAS, func(new_scores_3DAS, 1, np.mean(total_score_3DAS), np.std(total_score_3DAS))*len(total_score_3DAS)*1, 'r')

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