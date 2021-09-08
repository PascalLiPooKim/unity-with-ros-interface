% Results of the SAGAT Experiment
SAGAT_total_scores_VA = [11, 8, 9, 10]; % Vision + Audio
SAGAT_total_scores_V = [9, 5, 4 , 8]; % Vision


SAGAT_data = [SAGAT_total_scores_VA' SAGAT_total_scores_V'];

% Create ANOVA plot
anova1(SAGAT_data)
ylabel("Score")
title("One-Way ANOVA plot of SAGAT score")
xticklabels(["Vision & Audio", "Vision only"])

%%

% Results of Workload Experiment with optimal vision

% Completion Time
workload_CT_VA = [253, 264, 222, 234, 290]; % Vision + Audio
workload_CT_V = [237, 243, 243, 235, 265]; % Vision


% Get the mean and standard deviation of the completion time
disp([mean(workload_CT_VA), std(workload_CT_VA)])
disp([mean(workload_CT_V), std(workload_CT_V)])

% Number of keyboard hits
workload_KH_VA = [132, 110, 78, 87, 119];
workload_KH_V = [129, 103, 124, 80, 134];

% Get the mean and standard deviation of the number of keystrokes
disp([mean(workload_KH_VA), std(workload_KH_VA)])
disp([mean(workload_KH_V), std(workload_KH_V)])

% Number of collisions
workload_C_VA = [0, 1, 2, 0, 1];
workload_C_V = [0, 1, 2, 0, 3];

%%
NASA_CT = [workload_CT_VA' workload_CT_V'];
anova1(NASA_CT)
ylabel("Completion Time")
title("One-Way ANOVA plot of Completion Time for Workload Experiment")
xticklabels(["Vision & Audio", "Vision only"])


%%
NASA_KH = [workload_KH_VA' workload_KH_V'];
anova1(NASA_KH)
ylabel("Number of keystrokes")
title("One-Way ANOVA plot of Keyboard Hits for Workload Experiment")
xticklabels(["Vision & Audio", "Vision only"])

%%
NASA_C = [workload_C_VA' workload_C_V'];
anova1(NASA_C)
ylabel("Number of collisions")
title("One-Way ANOVA plot of Collision Occurences for Workload Experiment")
xticklabels(["Vision & Audio", "Vision only"])


%%
% Results of the Degraded Vision Experiment
workload_CT_VA = [179, 125, 101, 96, 186];
workload_CT_V = [106, 104, 121, 102, 115];

% Mean and std of completion time
disp([mean(workload_CT_VA), std(workload_CT_VA)])
disp([mean(workload_CT_V), std(workload_CT_V)])


workload_KH_VA = [85, 78, 35, 70, 84];
workload_KH_V = [61, 50, 67, 57, 41];

% Mean and std of number of keyboard hits
disp([mean(workload_KH_VA), std(workload_KH_VA)])
disp([mean(workload_KH_V), std(workload_KH_V)])

workload_C_VA = [2, 1, 2, 0, 1];
workload_C_V = [1, 1, 4, 1, 2];


%%
NASA_CT = [workload_CT_VA' workload_CT_V'];
anova1(NASA_CT)
ylabel("Completion Time")
title("One-Way ANOVA plot of Completion Time for Degraded Vision Experiment")
xticklabels(["Vision & Audio", "Vision only"])


%%
NASA_KH = [workload_KH_VA' workload_KH_V'];
anova1(NASA_KH)
ylabel("Number of keystrokes")
title("One-Way ANOVA plot of Keyboard Hits for Degraded Vision Experiment")
xticklabels(["Vision & Audio", "Vision only"])

%%
NASA_C = [workload_C_VA' workload_C_V'];
anova1(NASA_C)
ylabel("Number of collisions")
title("One-Way ANOVA plot of Collision Times for Degraded Vision Experiment")
xticklabels(["Vision & Audio", "Vision only"])
