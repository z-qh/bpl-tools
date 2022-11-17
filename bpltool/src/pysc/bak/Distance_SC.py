import numpy as np


def distance_sc(sc1, sc2):
    num_sectors = sc1.shape[1]
    # repeate to move 1 columns
    sim_for_each_cols = np.zeros(num_sectors)
    for i in range(num_sectors):
        # Shift
        one_step = 1  # const
        sc1 = np.roll(sc1, one_step, axis=1)  # columne shift
        # compare
        sum_of_cos_sim = 0
        num_col_engaged = 0
        for j in range(num_sectors):
            col_j_1 = sc1[:, j]
            col_j_2 = sc2[:, j]
            if (~np.any(col_j_1) or ~np.any(col_j_2)):
                continue
            # calc sim
            cos_similarity = np.dot(col_j_1, col_j_2) / (np.linalg.norm(col_j_1) * np.linalg.norm(col_j_2))
            sum_of_cos_sim = sum_of_cos_sim + cos_similarity
            num_col_engaged = num_col_engaged + 1
        # devided by num_col_engaged: So, even if there are many columns that are excluded from the calculation, we
        # can get high scores if other columns are well fit.
        sim_for_each_cols[i] = sum_of_cos_sim / num_col_engaged
    sim = max(sim_for_each_cols)
    dist = 1 - sim
    return dist


def Similarity(sc1, sc2):
    num_sectors = sc1.shape[1]
    # repeate to move 1 columns
    sim_for_each_cols = np.zeros(num_sectors)
    for i in range(num_sectors):
        # Shift
        one_step = 1  # const
        sc1 = np.roll(sc1, one_step, axis=1)  # columne shift
        # compare
        sum_of_cos_sim = 0
        num_col_engaged = 0
        for j in range(num_sectors):
            col_j_1 = sc1[:, j]
            col_j_2 = sc2[:, j]
            if (~np.any(col_j_1) or ~np.any(col_j_2)):
                continue
            # calc sim
            cos_similarity = np.dot(col_j_1, col_j_2) / (np.linalg.norm(col_j_1) * np.linalg.norm(col_j_2))
            sum_of_cos_sim = sum_of_cos_sim + cos_similarity
            num_col_engaged = num_col_engaged + 1
        # devided by num_col_engaged: So, even if there are many columns that are excluded from the calculation, we
        # can get high scores if other columns are well fit.
        sim_for_each_cols[i] = sum_of_cos_sim / num_col_engaged
    sim = max(sim_for_each_cols)
    return sim
