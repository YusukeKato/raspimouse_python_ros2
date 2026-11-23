import pytest

# Skipped when ament is not installed (e.g. standalone use without ROS 2).
main = pytest.importorskip('ament_pep257.main').main


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    rc = main(argv=['.', 'test'])
    assert rc == 0, 'Found code style errors / warnings'
