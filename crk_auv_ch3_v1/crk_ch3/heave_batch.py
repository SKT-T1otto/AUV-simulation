"""Run three fresh Gazebo worlds and export one verified, host-readable archive."""
from .heave import CASES, create_trial
from .heave_report import report
from .surge_batch import main, run_trials


def run(root):
    return run_trials(root, 'heave_force', [(case, case) for case in CASES],
                      create_trial, report, 'heave_response')


if __name__ == '__main__':
    main(run)
