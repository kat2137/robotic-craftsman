import pandas as pd
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = next(p for p in HERE.parents if (p/".git").exists())
DATA = HERE / "data"

fps = 30

df_vid = pd.read_csv(DATA/'vid_log.csv', sep="\t")
df_pos = pd.read_csv(DATA/'position_log.csv')
df_pos = df_pos[df_pos["time_cmd"] < 1788436182]
END = 1788435596
df_pos["to_video"] = df_pos["time_cmd"] - END
print(df_vid.columns.to_list())

for _, row in df_vid.iterrows():
    times = row["timelapse"].split("-")
    seconds =[]
    for stamps in times:
        m, s = stamps.split(":")
        new_stamp = float(m)*60 + float(s)
        seconds.append(new_stamp) 
    print(seconds, row["link"])

time_elapsed = []
for _, row in df_pos.iterrows():
    print(row["element"], row["time_cmd"], row["time_settled"], row["channel"], row["position"])
    time_elapsed.append(row["time_settled"] - row["time_cmd"])
    
a = time_elapsed[0]
b = seconds[1] - seconds[0]
print(a, b) 
fingers = df_pos[df_pos["element"] == "finger"]
print (fingers, df_pos["channel"], df_pos["position"], df_pos["to_video"].to_string())

df_pos["frame"] = (df_pos["to_video"] * fps).round().astype(int)
print(df_pos["frame"].to_string())