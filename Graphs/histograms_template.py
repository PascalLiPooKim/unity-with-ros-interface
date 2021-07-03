# -*- coding: utf-8 -*-
"""
Created on Sat Jul  3 23:37:51 2021

@author: paswe
"""

#%%
import numpy as np
import matplotlib.pyplot as plt


#%%
x = np.random.randint(10, 30, size=(5, 3))
print(x)
#n_bins = 10
num = np.array([1, 2, 3, 4, 5])

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