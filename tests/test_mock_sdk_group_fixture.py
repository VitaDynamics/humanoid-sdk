"""Mock group fixture must exercise the strict SDK group contract, no IPC."""
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace as S
import unittest
from unittest.mock import patch

from tools.production_topic_mock import sdk_group_fixture


class MockSdkGroupFixtureTest(unittest.TestCase):
    def run_fixture(self, factory):
        with tempfile.TemporaryDirectory() as directory:
            with patch.object(sdk_group_fixture, 'validate_client_environment',
                              return_value='mock-sdkmain-fixture'), \
                 patch('locomotion_aorta.PullController', side_effect=factory):
                sdk_group_fixture.run(directory)
            return json.loads((Path(directory)/'sdk-group-fixture.json').read_text())

    def test_matching_case_reads_state_and_both_mismatches_are_rejected(self):
        calls = []
        closed = []
        group = 'mock-sdkmain-fixture'
        def factory(**kwargs):
            calls.append(kwargs)
            requested = kwargs.get('group', 'default')
            if requested != group:
                raise ValueError(f'requested={requested!r}, actual={group!r}; AORTA_GROUP={group!r}')
            return S(transport=S(node=S(group=lambda:group)),
                     get_robot_info=lambda **kw:S(info=S(body=S(slot_count=31))),
                     wait_state=lambda *args:S(state_id=42),close=lambda:closed.append(True))
        result = self.run_fixture(factory)
        self.assertTrue(result['status'].startswith('PASS_'))
        self.assertEqual([row.get('expected_rejection',False) for row in result['cases']],
                         [True,True,False])
        self.assertEqual(result['cases'][-1]['state_id'],42)
        self.assertEqual(calls,[{}, {'group':'sdk-argument-unused'}, {'group':group}])
        self.assertEqual(closed,[True])

    def test_unrelated_ValueError_is_not_an_expected_group_rejection(self):
        with self.assertRaisesRegex(ValueError,'unrelated failure'):
            self.run_fixture(lambda **kwargs: (_ for _ in ()).throw(ValueError('unrelated failure')))

    def test_mismatched_controller_success_is_closed_and_fails_fixture(self):
        closed = []
        with self.assertRaisesRegex(AssertionError,'accepted a mismatched group'):
            self.run_fixture(lambda **kwargs:S(close=lambda:closed.append(True)))
        self.assertEqual(closed,[True])


if __name__ == '__main__':
    unittest.main()
