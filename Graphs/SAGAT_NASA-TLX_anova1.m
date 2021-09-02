SAGAT_total_scores_VA = [11, 8, 9, 10];
SAGAT_total_scores_V = [9, 5, 4 , 8];


SAGAT_data = [SAGAT_total_scores_VA' SAGAT_total_scores_V'];

anova1(SAGAT_data)
ylabel("Score")
title("One-Way ANOVA plot of SAGAT score")
xticklabels(["Vision & Audio", "Vision only"])

%%
% Workload Experiment
workload_CT_VA = [253, 264, 222, 234, 290];
workload_CT_V = [237, 243, 243, 235, 265];

workload_KH_VA = [132, 110, 78, 87, 119];
workload_KH_V = [129, 103, 124, 80, 134];

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
% Degraded Vision Experiment
workload_CT_VA = [179, 125, 101, 96, 186];
workload_CT_V = [106, 104, 121, 102, 115];



workload_KH_VA = [85, 78, 35, 70, 84];
workload_KH_V = [61, 50, 67, 57, 41];

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
