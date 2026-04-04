#!/usr/bin/env python3
"""
Optuna Study Analyzer
Analyzes the best trials at different iteration milestones.
"""

import argparse
import optuna
import sys
from typing import Dict, List, Optional


def get_best_trials_at_milestones(study: optuna.Study, milestones: List[int]) -> Dict[int, Optional[optuna.Trial]]:
    """
    Get the best trial at each milestone iteration.
    
    Args:
        study: The Optuna study to analyze
        milestones: List of iteration numbers to check
        
    Returns:
        Dictionary mapping milestone to best trial at that point
    """
    results = {}
    all_trials = study.trials
    
    for milestone in milestones:
        if milestone <= len(all_trials):
            # Get trials up to this milestone
            trials_up_to_milestone = all_trials[:milestone]
            
            # Filter completed trials
            completed_trials = [t for t in trials_up_to_milestone if t.state == optuna.trial.TrialState.COMPLETE]
            
            if completed_trials:
                # Find best trial (assuming minimization)
                best_trial = min(completed_trials, key=lambda t: t.value)
                results[milestone] = best_trial
            else:
                results[milestone] = None
        else:
            results[milestone] = None
    
    return results


def print_trial_info(trial: optuna.Trial, milestone: int):
    """Print formatted trial information."""
    print(f"\n=== Best Trial after {milestone} iterations ===")
    print(f"Trial number: {trial.number}")
    print(f"Value: {trial.value:.6f}")
    print("Parameters:")
    for param_name, param_value in trial.params.items():
        print(f"  {param_name}: {param_value}")
    print(f"Trial state: {trial.state}")


def analyze_optuna_study(database_path: str, milestones: List[int] = [1, 50, 100, 500, 1000]):
    """
    Analyze an Optuna study and report best trials at milestones.
    
    Args:
        database_path: Path to the SQLite database file
        milestones: List of iteration milestones to analyze
    """
    try:
        # Load the study from database
        storage = optuna.storages.RDBStorage(f"sqlite:///{database_path}")
        study_summaries = optuna.get_all_study_summaries(storage)
        
        if not study_summaries:
            print(f"No studies found in database: {database_path}")
            return
        
        # Use the first study (or you could modify this to select a specific study)
        study_name = study_summaries[0].study_name
        study = optuna.load_study(storage=storage, study_name=study_name)
        
        print(f"Loaded study: {study_name}")
        print(f"Total trials: {len(study.trials)}")
        print(f"Study direction: {study.direction}")
        
        # Get best trials at milestones
        best_trials = get_best_trials_at_milestones(study, milestones)
        
        # Print results
        for milestone in sorted(milestones):
            trial = best_trials[milestone]
            if trial is not None:
                print_trial_info(trial, milestone)
            else:
                print(f"\n=== Best Trial after {milestone} iterations ===")
                print("No completed trials available at this milestone")
        
        # Overall best trial
        if study.best_trial is not None:
            print(f"\n=== Overall Best Trial ===")
            print(f"Trial number: {study.best_trial.number}")
            print(f"Best value: {study.best_value:.6f}")
            print("Best parameters:")
            for param_name, param_value in study.best_params.items():
                print(f"  {param_name}: {param_value}")
        
    except Exception as e:
        print(f"Error analyzing study: {e}")
        sys.exit(1)


def main():
    parser = argparse.ArgumentParser(description="Analyze Optuna study progress at different milestones")
    parser.add_argument("database", help="Path to the Optuna SQLite database file")
    parser.add_argument("--milestones", nargs="+", type=int, default=[1, 50, 100, 500, 1000],
                        help="Iteration milestones to analyze (default: 1 50 100 500 1000)")
    
    args = parser.parse_args()
    
    analyze_optuna_study(args.database, args.milestones)


if __name__ == "__main__":
    main()