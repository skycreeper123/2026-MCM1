"""Independent Q3 V5 entry point: --self-check, --tests, --validate, or practice arguments."""
import importlib.util
from pathlib import Path
import sys
import unittest

sys.dont_write_bytecode = True
HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent


def load_v5():
    sys.path.insert(0, str(REPO))
    dependencies = REPO / '.modeling-deps'
    if dependencies.is_dir():
        sys.path.insert(0, str(dependencies))
    import B
    spec = importlib.util.spec_from_file_location(
        'B.q3', HERE / '__init__.py', submodule_search_locations=[str(HERE)])
    package = importlib.util.module_from_spec(spec)
    sys.modules['B.q3'] = package
    B.q3 = package
    spec.loader.exec_module(package)
    from B.q3 import planner
    assert Path(planner.__file__).resolve() == HERE / 'planner.py'
    assert package.Q3Config().strategy == 'v5_dynamic'
    print(f'Q3 V5 | strategy=v5_dynamic | planner={planner.__file__}', flush=True)
    return package


def main(argv=None):
    args = list(sys.argv[1:] if argv is None else argv)
    load_v5()
    if args and args[0] == '--tests':
        if len(args) > 1:
            raise SystemExit('--tests does not accept additional arguments')
        suite = unittest.defaultTestLoader.discover(str(HERE / 'tests'))
        result = unittest.TextTestRunner(verbosity=2).run(suite)
        return 0 if result.wasSuccessful() else 1
    if args and args[0] == '--validate':
        from B.q3.validation import main as validate
        return validate(args[1:])
    from B.q3.runner import main as run
    return run(args)


if __name__ == '__main__':
    raise SystemExit(main())
