import numpy as np
import matplotlib.pyplot as plt
import seaborn

profile = []
with open('total_error_track.txt', 'r') as myfile:
  for row in myfile:
    profile.append(float(row.strip('\n')))

print(profile)
plt.plot(profile)
plt.ylabel('Total movement error')
plt.xlabel('Trial')
plt.show()
