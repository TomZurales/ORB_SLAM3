import optuna
import subprocess
import random
import string
import os
from pathlib import Path

# Create a persistent SQLite database for the study
DB_PATH = "observability_optuna_study4.db"
STUDY_NAME = "observability_model_optimization"

def objective(trial):
    n = trial.suggest_int("n", 2, 150)
    k = trial.suggest_int("k", 1, n)
    angle_threshold = trial.suggest_float("angle_threshold", 0.01, 1.57)
    distance_threshold = trial.suggest_float("distance_threshold", 0.01, 10.0)
    unknown_psge_value = trial.suggest_float("unknown_psge_value", 0.0, 1.0)
    min_confidence_threshold = trial.suggest_float("min_confidence_threshold", 0.00, 1.0)
    max_error_threshold = trial.suggest_float("max_error_threshold", 0.00, 1.0)

    random_suffix = ''.join(random.choices(string.ascii_lowercase + string.digits, k=8))
    csv_filename = f"results_{random_suffix}.csv"
    
    # Log the trial number for progress tracking
    print(f"Running trial {trial.number}...")
    
    result = subprocess.run(
        ["./characterize_observability_model",
            str(n),
            str(k),
            str(angle_threshold),
            str(distance_threshold),
            str(unknown_psge_value),
            str(min_confidence_threshold),
            str(max_error_threshold),
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
        # single_value = (objective_values[0] * 3) + ((1 - objective_values[1]) * 3) + (objective_values[2]) + (objective_values[3])
        single_value = objective_values[0]
        return single_value
        
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
        directions=["minimize"],
        load_if_exists=True  # This allows resuming optimization
    )
    
    print(f"Study created with storage: {storage}")
    print(f"Study name: {STUDY_NAME}")
    print(f"Database path: {Path(DB_PATH).absolute()}")
    
    # Add study-level attributes for better dashboard visualization
    study.set_system_attr("description", "Observability model parameter optimization")
    study.set_system_attr("objective_names", ["Combined Performance Metric"])
    
    # Start optimization
    try:
        study.optimize(objective, n_trials=100, n_jobs=20)
        
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