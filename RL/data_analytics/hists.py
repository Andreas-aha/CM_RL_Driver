import pandas as pd
import os
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import re
import math

DIRECTORY = "RL/rl_data/first_trial_1/rl_uaq_store"
min_ep_dist = 50
last_ep_num = -min_ep_dist

appended_data = []
hists = {}

rows = 2
cols = 3
fig, axs = plt.subplots(rows, cols, sharex=True, sharey=True)

file_list = os.listdir(DIRECTORY)[::-1]
i = 0
for file in file_list:

    matched = re.match("ep_(?P<ep_num>[0-9]+)_[0-9]+.json", file)
    JSON_FILE = os.path.join(DIRECTORY, file)
    size = os.stat(JSON_FILE).st_size
    is_match = bool(matched) and (size > 1e5)

    if is_match and i < rows*cols:
        ep_num = int(matched.group('ep_num'))
        if ep_num not in hists and min_ep_dist < abs(ep_num - last_ep_num):
            last_ep_num = ep_num
            data = pd.read_json(JSON_FILE)
            data['filename'] = file
            r = rows-1 - math.floor(i/cols)
            c = cols-1 - i - r*cols
            plt.sca(axs[r, c])
            plt.minorticks_on()
            data['RL_Agent.SteerAng'].plot.hist(bins=11, alpha=0.8, density=True, grid=True)
            plt.grid(b=True, which='major', color='#666666', linestyle='-', alpha=0.4)
            plt.grid(b=True, which='minor', color='#999999', linestyle='-', alpha=0.2)
            axs[r,c].set_title("Episode " + str(ep_num))
            i += 1

            print(JSON_FILE)

plt.setp(axs[-1, 1], xlabel='Steering wheel angle [rad]')
plt.setp(axs[:, 0], ylabel=' ')
plt.setp(axs[1, 0], ylabel='Probability density')
plt.xticks([-10, -5, 0, 5, 10])
plt.yticks([0.1, 0.2, 0.3])
fig.suptitle("Probalitiy densities of steering commands")
fig.tight_layout()
plt.show()

































