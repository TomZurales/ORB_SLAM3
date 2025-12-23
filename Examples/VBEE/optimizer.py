import optuna
import subprocess
import random
import string
import os
from pathlib import Path

# Create a persistent SQLite database for the study
DB_PATH = "optuna_study.db"
STUDY_NAME = "vbee_optimization"

def objective(trial):
    bad_threshold = trial.suggest_float("bad_threshold", 0.01, 0.75)
    init_p_e = trial.suggest_float("init_p_e", bad_threshold + 0.1, 0.99)
    damping_coefficient = trial.suggest_float("damping_coefficient", 0.0, 1.0)
    init_observability = trial.suggest_float("init_observability", 0.001, 0.999)
    observability_damping_coefficient = trial.suggest_float("observability_damping_coefficient", 0.01, 0.99)
    n = trial.suggest_int("n", 2, 1000)
    k = trial.suggest_int("k", 1, n)
    angle_threshold = trial.suggest_float("angle_threshold", 0.01, 1.57)
    feedback_threshold = trial.suggest_float("feedback_threshold", 0.01, 0.99)
    sigmoid_steepness = trial.suggest_float("sigmoid_steepness", 0.01, 5.0)
    sigmoid_midpoint = trial.suggest_float("sigmoid_midpoint", 1.0, 500.0)
    false_negative_rate = trial.suggest_float("false_negative_rate", 0.0, 0.5)
    false_positive_rate = trial.suggest_float("false_positive_rate", 0.0, 0.5)

    random_suffix = ''.join(random.choices(string.ascii_lowercase + string.digits, k=8))
    csv_filename = f"results_{random_suffix}.csv"
    
    # Log the trial number for progress tracking
    print(f"Running trial {trial.number}...")
    
    result = subprocess.run(
        ["./vbee_test_random_accuracy",
         str(bad_threshold),
         str(init_p_e),
         str(damping_coefficient),
         str(init_observability),
         str(observability_damping_coefficient),
         str(k),
         str(n),
         str(angle_threshold),
         str(feedback_threshold),
         str(sigmoid_steepness),
         str(sigmoid_midpoint),
         str(false_negative_rate),
         str(false_positive_rate),
         csv_filename
         ],
        capture_output=True,
        text=True
    )
    
    # Check for errors
    if result.returncode != 0:
        print(f"Error in trial {trial.number}: {result.stderr}")
        raise optuna.TrialPruned()

    values = None
    try:
        with open(csv_filename, 'r') as f:
            line = f.readline().strip()
            values = line.split(',')
        os.remove(csv_filename)
        
        # Convert to float and add some logging
        objective_values = tuple(float(v) for v in values)
        print(f"Trial {trial.number} completed with values: {objective_values}")
        return objective_values
        
    except (FileNotFoundError, ValueError) as e:
        print(f"Error reading results for trial {trial.number}: {e}")
        if os.path.exists(csv_filename):
            os.remove(csv_filename)
        raise optuna.TrialPruned()

def main():
    # Create study with persistent storage
    storage = f"sqlite:///{DB_PATH}"
    
    # Create or load existing study
    study = optuna.create_study(
        study_name=STUDY_NAME,
        storage=storage,
        directions=["minimize", "minimize", "minimize", "minimize"],
        load_if_exists=True  # This allows resuming optimization
    )
    
    print(f"Study created with storage: {storage}")
    print(f"Study name: {STUDY_NAME}")
    print(f"Database path: {Path(DB_PATH).absolute()}")
    
    # Add study-level attributes for better dashboard visualization
    study.set_system_attr("description", "VBEE parameter optimization")
    study.set_system_attr("objective_names", ["error_1", "error_2", "error_3", "error_4"])
    
    # Start optimization
    try:
        # study.optimize(objective, n_trials=1000, n_jobs=10)
        
        print("\nOptimization completed!")
        print(f"Best trials:")
        for i, trial in enumerate(study.best_trials):
            print(f"  Trial {trial.number}: {trial.values}")

        # Print Pareto optimal parameters
        print("\nPareto Optimal Parameters:")
        for i, trial in enumerate(study.best_trials):
            print(f"\nSolution {i+1} (Trial {trial.number}):")
            for param_name, param_value in trial.params.items():
                print(f"  {param_name}: {param_value}")
            
    except KeyboardInterrupt:
        print("\nOptimization interrupted by user.")
        print(f"Completed trials: {len(study.trials)}")

if __name__ == "__main__":
    main()