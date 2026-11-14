"""Offline labels and training exports; never initializes ROS or commands robot."""
import argparse
from pathlib import Path

from .storage import recover, relabel, write_csv


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('command', choices=('label', 'invalid', 'recover', 'summarize', 'export'))
    parser.add_argument('session', type=Path)
    parser.add_argument('trial_id', nargs='?')
    parser.add_argument('value', nargs='?')
    args = parser.parse_args()
    if args.command in ('label', 'invalid'):
        if not args.trial_id or args.value is None:
            parser.error('label/invalid 需要 trial_id 和距离/原因')
        relabel(args.session, args.trial_id, distance=args.value if args.command == 'label' else None,
                invalid=args.value if args.command == 'invalid' else None)
    elif args.command == 'recover':
        recover(args.session)
    elif args.command == 'summarize':
        from .analyze import summarize
        summarize(args.session)
    print('训练导出有效样本数：', write_csv(args.session, training_only=True))


if __name__ == '__main__':
    main()
