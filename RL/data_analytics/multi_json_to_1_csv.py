import pandas as pd 
import seaborn as sns
import re
import os


DIRECTORY = "/home/andreas-z97x-ud3h/CM_Projects/CM_RL_Driver/RL/rl_data/first_trial_1/rl_uaq_store"
file_list = os.listdir(DIRECTORY)

df = pd.DataFrame()

for file in file_list:

    matched = re.match("ep_(?P<ep_num>[0-9]+).json", file)
    JSON_FILE = os.path.join(DIRECTORY, file)
    size = os.stat(JSON_FILE).st_size
    is_match = bool(matched) and (size > 1e5)

    if is_match:
        ep_num = int(matched.group('ep_num'))
        tmp_df = pd.read_json(JSON_FILE)
        tmp_df['ep_num'] = ep_num
        df = df.append(tmp_df)
        print("Finished " + str(tmp_df.shape) +"\t"+ file)


sns.histplot(data=df, x="ep_num", y="RL_Agent.Steer", bins=30, discrete=(True, False))

