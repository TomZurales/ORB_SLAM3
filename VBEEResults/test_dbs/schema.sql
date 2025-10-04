CREATE TABLE
    'Params' (
        'name' TEXT NOT NULL,
        'use_vbee' INTEGER NOT NULL DEFAULT 1,
        'weight_ransac' INTEGER NOT NULL DEFAULT 1,
        'k' INTEGER NOT NULL DEFAULT 7,
        'n' INTEGER NOT NULL DEFAULT 18,
        'a_th' INTEGER NOT NULL DEFAULT 0.52,
        'f_th' REAL NOT NULL DEFAULT 0.1,
        'init_p_e' REAL NOT NULL DEFAULT 0.9,
        'damp_coeff' REAL NOT NULL DEFAULT 0.12,
        'init_obs' REAL NOT NULL DEFAULT 0.5,
        'obs_damp_coeff' REAL NOT NULL DEFAULT 0.3
    );

CREATE TABLE
    'Trajectories' (
        'Order' INTEGER PRIMARY KEY AUTOINCREMENT,
        'Id' INTEGER NOT NULL
    );

CREATE TABLE
    'VBEE' (
        'mpID' INTEGER NOT NULL,
        'timestamp' REAL NOT NULL,
        'traj' INTEGER NOT NULL,
        'p_e' REAL NOT NULL,
        'est' REAL NOT NULL,
        'obs' REAL NOT NULL,
        'seen' INTEGER NOT NULL
    );

CREATE TABLE
    'FramePoses' (
        'timestamp' REAL NOT NULL,
        'traj' INTEGER NOT NULL,
        'x' REAL NOT NULL,
        'y' REAL NOT NULL,
        'z' REAL NOT NULL,
        'r_x' REAL NOT NULL,
        'r_y' REAL NOT NULL,
        'r_z' REAL NOT NULL,
        'r_w' REAL NOT NULL
    );

CREATE TABLE
    'KeyframePoses' (
        'timestamp' REAL NOT NULL,
        'traj' INTEGER NOT NULL,
        'x' REAL NOT NULL,
        'y' REAL NOT NULL,
        'z' REAL NOT NULL,
        'r_x' REAL NOT NULL,
        'r_y' REAL NOT NULL,
        'r_z' REAL NOT NULL,
        'r_w' REAL NOT NULL
    );

CREATE TABLE
    'RANSACStats' (
        'timestamp' REAL NOT NULL,
        'type' TEXT NOT NULL,
        'iterations' INTEGER NOT NULL,
        'inliers' INTEGER NOT NULL,
        'time' REAL NOT NULL,
        'success' INTEGER NOT NULL,
        'refined' INTEGER NOT NULL DEFAULT 0
    );

CREATE TABLE
    'TrackTimes' (
        'Id' INTEGER PRIMARY KEY AUTOINCREMENT,
        'time' REAL NOT NULL,
        'traj' INTEGER NOT NULL
    );

CREATE TABLE
    'RelocTimes' (
        'Id' INTEGER PRIMARY KEY AUTOINCREMENT,
        'time' REAL NOT NULL,
        'traj' INTEGER NOT NULL
    );