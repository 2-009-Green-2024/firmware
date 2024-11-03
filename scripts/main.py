import matplotlib.pyplot as plt

inputBlock = []
outputBlock = []

# concatenate sequential blockwise output of the test into two lists
with open("test_input.txt", 'r', encoding='utf-8') as infile:
    for line in infile:
        if line.startswith("inputBlock"):
            inputBlock += [int(x) for x in line[14:-4].split(',')]
        if line.startswith("outputBlock"):
            outputBlock += [int(x) for x in line[15:-4].split(',')]

# plot test output
plt.plot(inputBlock)
plt.plot(outputBlock)

# plot helper lines
max_value = 32767
plt.axhline(y=max_value, xmin=0, xmax=len(outputBlock), color='red', zorder=2, linewidth=0.5)
plt.axhline(y=-max_value, xmin=0, xmax=len(outputBlock), color='red', zorder=2, linewidth=0.5)
plt.axhline(y=0, xmin=0, xmax=len(outputBlock), color='black', zorder=2, linewidth=0.5)
periods = 2
for x in range(periods + 1):
    plt.axvline(x=1024 * x, ymin=-max_value, ymax=max_value, color='green', zorder=3, linewidth=0.5)
for x in range(8 * periods):
    plt.axvline(x=128 * x, ymin=-max_value, ymax=max_value, color='blue', zorder=2, linewidth=0.5)

plt.show()
