import pandas as pd
import matplotlib.pyplot as plt

# Reload and clean the CSV to be safe
df_raw = pd.read_csv('./data.csv')
header = df_raw.iloc[0]
df = df_raw[1:].copy()
df.columns = header
df.columns = [c.strip() for c in df.columns]

# Convert numeric columns
num_cols = [c for c in df.columns if c not in ["Scene", "Accel Structure"]]
df[num_cols] = df[num_cols].apply(pd.to_numeric, errors="coerce")

# Derived metrics
df["Tri per primary"] = df["Tri Tests (Primary)"] / df["Primary Rays"]
df["Tri per shadow"]  = df["Tri Tests (Shadow)"]  / df["Shadow Rays"]
df["BBox per primary"] = df["BBox Tests (Primary)"] / df["Primary Rays"]
df["BBox per shadow"]  = df["BBox Tests (Shadow)"]  / df["Shadow Rays"]

def grouped_bar(metric_col, filename, ylabel, title, data=None):
    d = df if data is None else data
    pivot = d.pivot(index="Scene", columns="Accel Structure", values=metric_col)
    ax = pivot.plot(kind="bar")
    ax.set_xlabel("Scene")
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    plt.tight_layout()
    plt.savefig(f"./{filename}", dpi=300)
    plt.close()

# a) Render time all scenes
grouped_bar(
    metric_col="Render Time (s)",
    filename="render_time_all_scenes.png",
    ylabel="Render time (s)",
    title="Render time per acceleration structure for all scenes"
)

# b) Triangles per primary ray all scenes
grouped_bar(
    metric_col="Tri per primary",
    filename="tri_per_primary_all_scenes.png",
    ylabel="Triangle tests per primary ray",
    title="Triangle tests per primary ray"
)

# c) Triangles per shadow ray all scenes
grouped_bar(
    metric_col="Tri per shadow",
    filename="tri_per_shadow_all_scenes.png",
    ylabel="Triangle tests per shadow ray",
    title="Triangle tests per shadow ray"
)

# d) BBox per primary ray, BVH vs OCTREE only
df_bo = df[df["Accel Structure"].isin(["BVH", "OCTREE"])]
grouped_bar(
    metric_col="BBox per primary",
    filename="bbox_per_primary_all_scenes.png",
    ylabel="BBox tests per primary ray",
    title="BBox tests per primary ray",
    data=df_bo
)
