"""Flatten the images of an existing result directory into one folder.

`viewpoint_sim run` already writes `rgb/` inside every result directory. This
script remains useful for results produced before that, and for the cases the
run does not cover: copying other image kinds, symlinking instead of copying, or
writing to a directory outside the result.
"""
from pathlib import Path
import argparse
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from viewpoint_sim.acquisition import IMAGE_KINDS, collect_images


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Copy the images of one result directory into a single flat folder")
    parser.add_argument('result', nargs='?', default='outputs/demo',
                        help="Result directory produced by `viewpoint_sim run` (default: outputs/demo)")
    parser.add_argument('--output', default=None,
                        help="Destination folder (default: <result>/rgb)")
    parser.add_argument('--kinds', default='rgb',
                        help="Comma separated subset of: "+", ".join(IMAGE_KINDS)+" (default: rgb)")
    parser.add_argument('--link', action='store_true',
                        help="Create relative symlinks instead of copying files")
    parser.add_argument('--force', action='store_true',
                        help="Allow writing into a non-empty destination folder")
    args = parser.parse_args(argv)

    kinds = [k.strip() for k in args.kinds.split(',') if k.strip()]
    unknown = [k for k in kinds if k not in IMAGE_KINDS]
    if unknown:
        parser.error(f"Unknown image kind(s): {', '.join(unknown)}. Choose from: {', '.join(IMAGE_KINDS)}")
    output = Path(args.output) if args.output else Path(args.result)/'rgb'
    try:
        rows, manifest = collect_images(args.result, output, kinds=kinds,
                                        mode='link' if args.link else 'copy',
                                        force=args.force)
    except (ValueError, FileExistsError, FileNotFoundError) as error:
        print(f"Error: {error}", file=sys.stderr)
        return 1
    print(f"Collected {len(rows)} RGB images into {output.resolve()}")
    print(f"Manifest: {manifest.resolve()}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
