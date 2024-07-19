
# The path to the computed metrics of Dunes of 1 m plot size.
file_paths = [
    r'D:\Upscaling\Output\Plotting\1m\Woodland_1m\metrics_density\metrics_density_original.txt',
    r'D:\Upscaling\Output\Plotting\1m\Woodland_1m\metrics_density\metrics_density_20.txt',
    r'D:\Upscaling\Output\Plotting\1m\Woodland_1m\metrics_density\metrics_density_15.txt',
    r'D:\Upscaling\Output\Plotting\1m\Woodland_1m\metrics_density\metrics_density_10.txt',
    r'D:\Upscaling\Output\Plotting\1m\Woodland_1m\metrics_density\metrics_density_5.txt',
    r'D:\Upscaling\Output\Plotting\1m\Woodland_1m\metrics_density\metrics_density_2.txt',
    r'D:\Upscaling\Output\Plotting\1m\Woodland_1m\metrics_density\metrics_density_1.txt',
]


#save_directory = '/path/to/your/folder'

# Initialize a list to store data from all files
all_files_data = []

# Loop over each file
for file_path in file_paths:
    # Initialize lists for each column for the current file
    columns = [[] for _ in range(25)]

    # Open and read the file
    with open(file_path, 'r') as file:
        for line in file:
            data = line.strip().split()
            for i in range(25):
                columns[i].append(data[i])

    # Add the columns data to the overall data list
    all_files_data.append(columns)


def transpose(matrix):
    """ Transpose a 2D list (matrix) """
    return list(map(list, zip(*matrix)))



cleaned_data = []

for file_data in all_files_data:
    # Transpose to work with rows
    transposed_data = transpose(file_data)

    # Filter out rows that contain '-nan(ind)'
    filtered_data = [row for row in transposed_data if '-nan(ind)' not in row]

    # Transpose back and add to cleaned data
    cleaned_data.append(transpose(filtered_data))

# Now, cleaned_data contains data from all files with '-nan(ind)' rows removed
all_files_data = cleaned_data



import matplotlib.pyplot as plt
import os

# The directory should be modified 
save_directory = r'D:\Upscale_Plotting\MetricsDensity'
file_name = 'Woodland_1m.png'

if not os.path.exists(save_directory):
    os.makedirs(save_directory)

# Save the figure
save_path = os.path.join(save_directory, file_name)


# Assuming all_files_data is a list of lists of lists,
# where the outer list represents files, the middle list represents columns,
# and the inner list contains the column data

# Convert data to float or the appropriate type if necessary
numeric_data = [[[float(value) for value in col] for col in file] for file in all_files_data]

# Number of columns and files
num_columns = 25
num_files = 7

# Create a figure and a set of subplots
fig, axes = plt.subplots(5, 5, figsize=(15, 15))
axes = axes.flatten()

tick_labels = ['ori.', '20', '15', '10', '5', '2', '1']

# Set the overall figure title
fig.suptitle('LiDAR metrics of woodland (1m) by 7 scales of density', fontsize=16)


# Penetration ratio

H_max = [file[0] for file in numeric_data]
#axes[0].boxplot(H_max, showfliers=True)
#axes[0].boxplot(H_max)
axes[0].boxplot(H_max,
                patch_artist=True,
                whis=[5, 95],
                #whis=1.5,
                showfliers=True,
                medianprops=dict(color="red"),
                boxprops=dict(color="black", facecolor="skyblue"),
                whiskerprops=dict(color="green"),
                capprops=dict(color="gray"),
                flierprops=dict(marker='.', markerfacecolor='black', markersize=5)
                )

axes[0].set_title('H_max')
axes[0].set_xlabel('Point density (pts/m2)')
axes[0].set_xticklabels(tick_labels)
axes[0].set_ylabel('Meter')

# Max vegetation height
H_mean = [file[1] for file in numeric_data]
#axes[1].boxplot(H_mean, showfliers=False)
axes[1].boxplot(H_mean,
                patch_artist=True,
                whis=[5, 95],
                #whis=1.5,
                showfliers=True,
                medianprops=dict(color="red"),
                boxprops=dict(color="black", facecolor="skyblue"),
                whiskerprops=dict(color="green"),
                capprops=dict(color="gray"),
                flierprops=dict(marker='.', markerfacecolor='black', markersize=5))

axes[1].set_title('H_mean')
axes[1].set_xlabel('Point density (pts/m2)')
axes[1].set_xticklabels(tick_labels)
axes[1].set_ylabel('Meter')


# Median vegetation height
H_median = [file[2] for file in numeric_data]
#axes[2].boxplot(H_median, showfliers=False)
axes[2].boxplot(H_median,
                patch_artist=True,
                whis=[5, 95],
                #whis=1.5,
                showfliers=True,
                medianprops=dict(color="red"),
                boxprops=dict(color="black", facecolor="skyblue"),
                whiskerprops=dict(color="green"),
                capprops=dict(color="gray"),
                flierprops=dict(marker='.', markerfacecolor='black', markersize=5))

axes[2].set_title('H_median')
axes[2].set_xlabel('Point density (pts/m2)')
axes[2].set_xticklabels(tick_labels)
axes[2].set_ylabel('Meter')

# Shannon index of vegetation
Hp_25 = [file[3] for file in numeric_data]
#axes[3].boxplot(Hp_25, showfliers=False)
axes[3].boxplot(Hp_25,
                patch_artist=True,
                whis=[5, 95],
                #whis=1.5,
                showfliers=True,
                medianprops=dict(color="red"),
                boxprops=dict(color="black", facecolor="skyblue"),
                whiskerprops=dict(color="green"),
                capprops=dict(color="gray"),
                flierprops=dict(marker='.', markerfacecolor='black', markersize=5))
axes[3].set_title('Hp_25')
axes[3].set_xlabel('Point density (pts/m2)')
axes[3].set_xticklabels(tick_labels)
axes[3].set_ylabel('Meter')

# Shannon index of vegetation
Hp_50 = [file[4] for file in numeric_data]
#axes[4].boxplot(Hp_50, showfliers=False)
axes[4].boxplot(Hp_50,
                patch_artist=True,
                whis=[5, 95],
                #whis=1.5,
                showfliers=True,
                medianprops=dict(color="red"),
                boxprops=dict(color="black", facecolor="skyblue"),
                whiskerprops=dict(color="green"),
                capprops=dict(color="gray"),
                flierprops=dict(marker='.', markerfacecolor='black', markersize=5)
                )
axes[4].set_title('Hp_50')
axes[4].set_xlabel('Point density (pts/m2)')
axes[4].set_xticklabels(tick_labels)
axes[4].set_ylabel('Meter')

# Shannon index of vegetation
Hp_75 = [file[5] for file in numeric_data]
#axes[5].boxplot(Hp_75, showfliers=False)
axes[5].boxplot(Hp_75, patch_artist=True,
                whis=[5, 95],
                #whis=1.5,
                showfliers=True,
                medianprops=dict(color="red"),
                boxprops=dict(color="black", facecolor="skyblue"),
                whiskerprops=dict(color="green"),
                capprops=dict(color="gray"),
                flierprops=dict(marker='.', markerfacecolor='black', markersize=5))

axes[5].set_title('Hp_75')
axes[5].set_xlabel('Point density (pts/m2)')
axes[5].set_xticklabels(tick_labels)
axes[5].set_ylabel('Meter')

Hp_95 = [file[6] for file in numeric_data]
#axes[6].boxplot(Hp_95, showfliers=False)
axes[6].boxplot(Hp_95, patch_artist=True,
                whis=[5, 95],
                #whis=1.5,
                showfliers=True,
                medianprops=dict(color="red"),
                boxprops=dict(color="black", facecolor="skyblue"),
                whiskerprops=dict(color="green"),
                capprops=dict(color="gray"),
                flierprops=dict(marker='.', markerfacecolor='black', markersize=5))
axes[6].set_title('Hp_95')
axes[6].set_xlabel('Point density (pts/m2)')
axes[6].set_xticklabels(tick_labels)
axes[6].set_ylabel('Meter')

PPR = [file[7] for file in numeric_data]
#axes[7].boxplot(PPR, showfliers=False)
axes[7].boxplot(PPR,
                patch_artist=True,
                whis=[5, 95],
                #whis=1.5,
                showfliers=True,
                medianprops=dict(color="red"),
                boxprops=dict(color="black", facecolor="lightgreen"),
                whiskerprops=dict(color="green"),
                capprops=dict(color="gray"),
                flierprops=dict(marker='.', markerfacecolor='black', markersize=5)
                )
axes[7].set_title('PPR')
axes[7].set_xlabel('Point density (pts/m2)')
axes[7].set_xticklabels(tick_labels)
axes[7].set_ylabel('Ratio')

Density_above_mean = [file[8] for file in numeric_data]
#axes[8].boxplot(Density_above_mean, showfliers=False)
axes[8].boxplot(Density_above_mean,
                patch_artist=True,
                #whis=1.5,
                whis=[5, 95],
                showfliers=True,
                medianprops=dict(color="red"),
                boxprops=dict(color="black", facecolor="lightgreen"),
                whiskerprops=dict(color="green"),
                capprops=dict(color="gray"),
                flierprops=dict(marker='.', markerfacecolor='black', markersize=5)
                )
axes[8].set_title('Density above mean z')
axes[8].set_xlabel('Point density (pts/m2)')
axes[8].set_xticklabels(tick_labels)
axes[8].set_ylabel('Number of points')


BR_1 = [file[9] for file in numeric_data]
#axes[9].boxplot(BR_1, showfliers=False)
axes[9].boxplot(BR_1,
                patch_artist=True,
                #whis=1.5,
                whis=[5, 95],
                showfliers=True,
                medianprops=dict(color="red"),
                boxprops=dict(color="black", facecolor="lightgreen"),
                whiskerprops=dict(color="green"),
                capprops=dict(color="gray"),
                flierprops=dict(marker='.', markerfacecolor='black', markersize=5)
                )
axes[9].set_title('BR_1')
axes[9].set_xlabel('Point density (pts/m2)')
axes[9].set_xticklabels(tick_labels)
axes[9].set_ylabel('Ratio')

BR_1_2 = [file[10] for file in numeric_data]
#axes[10].boxplot(BR_1_2, showfliers=False)
axes[10].boxplot(BR_1_2,
                 patch_artist=True,
                 # whis=1.5,
                 whis=[5, 95],
                 showfliers=True,
                 medianprops=dict(color="red"),
                 boxprops=dict(color="black", facecolor="lightgreen"),
                 whiskerprops=dict(color="green"),
                 capprops=dict(color="gray"),
                 flierprops=dict(marker='.', markerfacecolor='black', markersize=5)
                 )
axes[10].set_title('BR_1_2')
axes[10].set_xlabel('Point density (pts/m2)')
axes[10].set_xticklabels(tick_labels)
axes[10].set_ylabel('Ratio')

BR_2_3 = [file[11] for file in numeric_data]
#axes[11].boxplot(BR_2_3, showfliers=False)
axes[11].boxplot(BR_2_3,
                 patch_artist=True,
                 # whis=1.5,
                 whis=[5, 95],
                 showfliers=True,
                 medianprops=dict(color="red"),
                 boxprops=dict(color="black", facecolor="lightgreen"),
                 whiskerprops=dict(color="green"),
                 capprops=dict(color="gray"),
                 flierprops=dict(marker='.', markerfacecolor='black', markersize=5)
                 )
axes[11].set_title('BR_2_3')
axes[11].set_xlabel('Point density (pts/m2)')
axes[11].set_xticklabels(tick_labels)
axes[11].set_ylabel('Ratio')

BR_3 = [file[12] for file in numeric_data]
#axes[12].boxplot(BR_3, showfliers=False)
axes[12].boxplot(BR_3, patch_artist=True,
                 # whis=1.5,
                 whis=[5, 95],
                 showfliers=True,
                 medianprops=dict(color="red"),
                 boxprops=dict(color="black", facecolor="lightgreen"),
                 whiskerprops=dict(color="green"),
                 capprops=dict(color="gray"),
                 flierprops=dict(marker='.', markerfacecolor='black', markersize=5)
                 )
axes[12].set_title('BR_3')
axes[12].set_xlabel('Point density (pts/m2)')
axes[12].set_xticklabels(tick_labels)
axes[12].set_ylabel('Ratio')

BR_3_4 = [file[13] for file in numeric_data]
#axes[13].boxplot(BR_3_4, showfliers=False)
axes[13].boxplot(BR_3_4,
                 patch_artist=True,
                 # whis=1.5,
                 whis=[5, 95],
                 showfliers=True,
                 medianprops=dict(color="red"),
                 boxprops=dict(color="black", facecolor="lightgreen"),
                 whiskerprops=dict(color="green"),
                 capprops=dict(color="gray"),
                 flierprops=dict(marker='.', markerfacecolor='black', markersize=5)
                 )
axes[13].set_title('BR_3_4')
axes[13].set_xlabel('Point density (pts/m2)')
axes[13].set_xticklabels(tick_labels)
axes[13].set_ylabel('Ratio')

BR_4_5 = [file[14] for file in numeric_data]
#axes[14].boxplot(BR_4_5, showfliers=False)
axes[14].boxplot(BR_4_5,
                 patch_artist=True,
                 # whis=1.5,
                 whis=[5, 95],
                 showfliers=True,
                 medianprops=dict(color="red"),
                 boxprops=dict(color="black", facecolor="lightgreen"),
                 whiskerprops=dict(color="green"),
                 capprops=dict(color="gray"),
                 flierprops=dict(marker='.', markerfacecolor='black', markersize=5)
                 )
axes[14].set_title('BR_4_5')
axes[14].set_xlabel('Point density (pts/m2)')
axes[14].set_xticklabels(tick_labels)
axes[14].set_ylabel('Ratio')

BR_5 = [file[15] for file in numeric_data]
#axes[15].boxplot(BR_5, showfliers=False)
axes[15].boxplot(BR_5,
                 patch_artist=True,
                 # whis=1.5,
                 whis=[5, 95],
                 showfliers=True,
                 medianprops=dict(color="red"),
                 boxprops=dict(color="black", facecolor="lightgreen"),
                 whiskerprops=dict(color="green"),
                 capprops=dict(color="gray"),
                 flierprops=dict(marker='.', markerfacecolor='black', markersize=5)
                 )
axes[15].set_title('BR_5')
axes[15].set_xlabel('Point density (pts/m2)')
axes[15].set_xticklabels(tick_labels)
axes[15].set_ylabel('Ratio')

BR_5_20 = [file[16] for file in numeric_data]
#axes[16].boxplot(BR_5_20, showfliers=False)
axes[16].boxplot(BR_5_20,
                 patch_artist=True,
                 # whis=1.5,
                 whis=[5, 95],
                 showfliers=True,
                 medianprops=dict(color="red"),
                 boxprops=dict(color="black", facecolor="lightgreen"),
                 whiskerprops=dict(color="green"),
                 capprops=dict(color="gray"),
                 flierprops=dict(marker='.', markerfacecolor='black', markersize=5)
                 )
axes[16].set_title('BR_5_20')
axes[16].set_xlabel('Point density (pts/m2)')
axes[16].set_xticklabels(tick_labels)
axes[16].set_ylabel('Ratio')

BR_20 = [file[17] for file in numeric_data]
#axes[17].boxplot(BR_20, showfliers=False)
axes[17].boxplot(BR_20,
                 patch_artist=True,
                 # whis=1.5,
                whis=[5, 95],
                showfliers=True,
                medianprops=dict(color="red"),
                boxprops=dict(color="black", facecolor="lightgreen"),
                whiskerprops=dict(color="green"),
                capprops=dict(color="gray"),
                flierprops=dict(marker='.', markerfacecolor='black', markersize=5)
                 )
axes[17].set_title('BR_20')
axes[17].set_xlabel('Point density (pts/m2)')
axes[17].set_xticklabels(tick_labels)
axes[17].set_ylabel('Ratio')

coeff_var_z = [file[18] for file in numeric_data]
#axes[18].boxplot(coeff_var_z, showfliers=False)
axes[18].boxplot(coeff_var_z,
                 patch_artist=True,
                 # whis=1.5,
                 whis=[5, 95],
                 showfliers=True,
                 medianprops=dict(color="red"),
                 boxprops=dict(color="black", facecolor="orange"),
                 whiskerprops=dict(color="green"),
                 capprops=dict(color="gray"),
                 flierprops=dict(marker='.', markerfacecolor='black', markersize=5)
                 )
axes[18].set_title('Coeff. var. z')
axes[18].set_xlabel('Point density (pts/m2)')
axes[18].set_xticklabels(tick_labels)
axes[18].set_ylabel('Values')

entropy_z = [file[19] for file in numeric_data]
#axes[19].boxplot(entropy_z, showfliers=False)
axes[19].boxplot(entropy_z, patch_artist=True,
                 # whis=1.5,
                 whis=[5, 95],
                 showfliers=True,
                 medianprops=dict(color="red"),
                 boxprops=dict(color="black", facecolor="orange"),
                 whiskerprops=dict(color="green"),
                 capprops=dict(color="gray"),
                 flierprops=dict(marker='.', markerfacecolor='black', markersize=5))
axes[19].set_title('Shannon index')
axes[19].set_xlabel('Point density (pts/m2)')
axes[19].set_xticklabels(tick_labels)
axes[19].set_ylabel('Nat')

H_kurt = [file[20] for file in numeric_data]
#axes[20].boxplot(H_kurt, showfliers=False)
axes[20].boxplot(H_kurt,
                 patch_artist=True,
                 # whis=1.5,
                 whis=[5, 95],
                 showfliers=True,
                 medianprops=dict(color="red"),
                 boxprops=dict(color="black", facecolor="orange"),
                 whiskerprops=dict(color="green"),
                 capprops=dict(color="gray"),
                 flierprops=dict(marker='.', markerfacecolor='black', markersize=5)
                 )
axes[20].set_title('Kurtosis of H_veg.')
axes[20].set_xlabel('Point density (pts/m2)')
axes[20].set_xticklabels(tick_labels)
axes[20].set_ylabel('Values')

sigma_z = [file[21] for file in numeric_data]
#axes[21].boxplot(sigma_z, showfliers=False)
axes[21].boxplot(sigma_z,  patch_artist=True,
                 # whis=1.5,
                 whis=[5, 95],
                 showfliers=True,
                 medianprops=dict(color="red"),
                 boxprops=dict(color="black", facecolor="orange"),
                 whiskerprops=dict(color="green"),
                 capprops=dict(color="gray"),
                 flierprops=dict(marker='.', markerfacecolor='black', markersize=5)
                 )
axes[21].set_title('Roughness of veg. height')
axes[21].set_xlabel('Point density (pts/m2)')
axes[21].set_xticklabels(tick_labels)
axes[21].set_ylabel('Meter')

H_skew = [file[22] for file in numeric_data]
#axes[22].boxplot(H_skew, showfliers=False)
axes[22].boxplot(H_skew,
                 patch_artist=True,
                 # whis=1.5,
                 whis=[5, 95],
                 showfliers=True,
                 medianprops=dict(color="red"),
                 boxprops=dict(color="black", facecolor="orange"),
                 whiskerprops=dict(color="green"),
                 capprops=dict(color="gray"),
                 flierprops=dict(marker='.', markerfacecolor='black', markersize=5)
                 )
axes[22].set_title('Skewness of veg. height')
axes[22].set_xlabel('Point density (pts/m2)')
axes[22].set_xticklabels(tick_labels)
axes[22].set_ylabel('Values')

H_std = [file[23] for file in numeric_data]
#axes[23].boxplot(H_std, showfliers=False)
axes[23].boxplot(H_std,
                 patch_artist=True,
                 # whis=1.5,
                 whis=[5, 95],
                 showfliers=True,
                 medianprops=dict(color="red"),
                 boxprops=dict(color="black", facecolor="orange"),
                 whiskerprops=dict(color="green"),
                 capprops=dict(color="gray"),
                 flierprops=dict(marker='.', markerfacecolor='black', markersize=5)
                 )
axes[23].set_title('Std.dev. of veg. height')
axes[23].set_xlabel('Point density (pts/m2)')
axes[23].set_xticklabels(tick_labels)
axes[23].set_ylabel('Meter')

H_var = [file[24] for file in numeric_data]
#axes[24].boxplot(H_var, showfliers=False)
axes[24].boxplot(H_var,
                 patch_artist=True,
                 # whis=1.5,
                 whis=[5, 95],
                 showfliers=True,
                 medianprops=dict(color="red"),
                 boxprops=dict(color="black", facecolor="orange"),
                 whiskerprops=dict(color="green"),
                 capprops=dict(color="gray"),
                 flierprops=dict(marker='.', markerfacecolor='black', markersize=5)
                 )
axes[24].set_title('Variance of veg. height')
axes[24].set_xlabel('Point density (pts/m2)')
axes[24].set_xticklabels(tick_labels)
axes[24].set_ylabel('Meter^2')


plt.tight_layout()
plt.subplots_adjust(top=0.92)
plt.savefig(save_path)
plt.show()

# Plot each column's data from each file in a group
for i in range(num_columns):
    # Extract the i-th column data from each file
    column_data = [file[i] for file in numeric_data]

    # Plot boxplot for the i-th column with data from all files
    axes[i].boxplot(column_data, showfliers=False)
    axes[i].set_title(f'Column {i + 1}')
    axes[i].set_xlabel('File Number')
    axes[i].set_ylabel('Values')

# Adjust layout to prevent overlap
plt.tight_layout()

# Show the plot
plt.show()