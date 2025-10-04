import sqlite3

class Test:
    def __init__(self, name: str, use_vbee: bool, weight_ransac: bool, trajectories: list[int]):
        self.name = name
        self.use_vbee = use_vbee
        self.weight_ransac = weight_ransac
        self.trajectories = trajectories

tests = [
    Test("Low Clutter Full Apartment No VBEE", False, False, [0, 1, 2]),

    Test("High Clutter Full Apartment No VBEE", False, False, [3, 4, 5]),

    Test("Low Clutter Full Apartment VBEE No RANSAC", True, False, [0, 1, 2]),

    Test("High Clutter Full Apartment VBEE No RANSAC", True, False, [3, 4, 5]),

    Test("Low Clutter Full Apartment VBEE with RANSAC", True, True, [0, 1, 2]),

    Test("High Clutter Full Apartment VBEE with RANSAC", True, True, [3, 4, 5]),

    Test("Low to High Clutter Full Apartment No VBEE", False, False, [0, 1, 2, 3, 4, 5]),

    Test("Low to High Clutter Full Apartment VBEE No RANSAC", False, True, [0, 1, 2, 3, 4, 5]),

    Test("Low to High Clutter Full Apartment VBEE with RANSAC", True, True, [0, 1, 2, 3, 4, 5]),
]

def create_test_db(db_path, schema_path, name, use_vbee, weight_ransac, trajectories):
    # Create the database and apply the schema
    with open(schema_path, 'r') as f:
        schema_sql = f.read()
    conn = sqlite3.connect(db_path)
    try:
        conn.executescript(schema_sql)
        # Insert into Params
        conn.execute(
            "INSERT INTO Params (name, use_vbee, weight_ransac) VALUES (?, ?, ?);",
            (name, int(use_vbee), int(weight_ransac))
        )
        # Insert into Trajectories
        for traj_id in trajectories:
            conn.execute(
                "INSERT INTO Trajectories (Id) VALUES (?);",
                (traj_id,)
            )
        conn.commit()
    finally:
        conn.close()

for i, test in enumerate(tests):
    db_path = f"test{i+1}.db"
    schema_path = "schema.sql"
    create_test_db(db_path, schema_path, test.name, test.use_vbee, test.weight_ransac, test.trajectories)

