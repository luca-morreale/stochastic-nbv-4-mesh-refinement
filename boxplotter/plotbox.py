import pandas as pd
import seaborn
import matplotlib.pyplot as plt
import numpy as np

import json
import sys

filename = sys.argv[1]

scores = []
rounds = []

with open(filename) as cin:    
    data = json.load(cin)

    points = data['cams']
    for point in points:

        scores.append(point['score'])
        rounds.append(point['round'])

# current = 0
# for i in xrange(rounds[-1]):
#     last_ind = len(rounds) - 1 - rounds[::-1].index(i)
    
#     plot_data = scores[current:last_ind]
#     current = last_ind+1

#     plt.boxplot(plot_data)


# plt.show()


df = pd.DataFrame({'data': scores, 'rounds': rounds})

df.boxplot(by='rounds')
plt.show()


first_ind = len(rounds) - rounds[::-1].index(0)
df = pd.DataFrame({'data': scores[first_ind:-1], 'rounds': rounds[first_ind:-1]})

df.boxplot(by='rounds')
plt.show()

