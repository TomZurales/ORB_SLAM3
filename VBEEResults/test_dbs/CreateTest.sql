CREATE TABLE
    'Params' (
        'name' TEXT NOT NULL,
        'use_vbee' INTEGER NOT NULL DEFAULT 1,
        'weight_ransac' INTEGER NOT NULL DEFAULT 1,
        'k' INTEGER NOT NULL DEFAULT 150,
        'n' INTEGER NOT NULL DEFAULT 15,
        'a_th' INTEGER NOT NULL DEFAULT 0.52,
        'f_th' REAL NOT NULL DEFAULT 0.1,
        'init_p_e' REAL NOT NULL DEFAULT 0.9,
        'damp_coeff' REAL NOT NULL DEFAULT 0.99,
        'init_obs' REAL NOT NULL DEFAULT 0.5,
        'obs_damp_coeff' REAL NOT NULL DEFAULT 0.4
    );

CREATE TABLE
    'Trajectories' (
        'Order' INTEGER PRIMARY KEY AUTOINCREMENT,
        'Id' INTEGER NOT NULL
    );

INSERT INTO Params VALUES (name, use_vbee, weight_ransac) VALUES ('Test1');