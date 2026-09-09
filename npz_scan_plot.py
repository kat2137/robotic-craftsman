
import sys
import numpy as np
import matplotlib.pyplot as plt

path = sys.argv[1]
data = np.load(path)
headers = data.files
joint_shape = data["joints"].shape

valid = ~np.isnan(data["joints"][:, 8, 0])  # valid frames where hand is detected
no_hand = np.isnan(data["joints"]).all(axis=(1,2)).sum()  # number of frames with no hand detected
t = np.arange(len(valid))

tip = data["joints"][:, 8, :] 
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14,6), sharex=True)
ax1.plot(t, valid.astype(int), label="Hand detected", drawstyle="steps-post")
for val, label in enumerate("xyz"):
    ax2.plot(t, tip[:, val], label=label)
ax2.legend(); ax2.set_ylabel("Tip position 8 (m)"); ax2.set_xlabel("frame")
plt.tight_layout(); plt.show()