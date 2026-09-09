# open both csv

import pandas as pd
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = next(p for p in HERE.parents if (p/".git").exists())

df_vid = pd.read_csv('vid_log.csv')
df_pos = pd.read_csv('position_log.csv')

for _, row in df_vid.iterrows():
    times = row["timelapse"].split("-")
    print(times, row["link"])

for _, row in df_pos.iterrows():
    print(row["time_cmd"], row["time_settled"], row["channel"], row["value"])

a = row["time_settled"][1] - row["time_cmd"][1]
b = times[2] - times[1]

print a, b
