import math
from pathlib import Path
import tempfile
import unittest
from unittest import mock

from latency import decode_profile as profile
from latency.report import stats
from decode_fixtures import pack_state, state_object


class DecodeProfileTest(unittest.TestCase):
    def test_signed_deltas_accept_negative_while_elapsed_does_not(self):
        result=profile.signed_stats([-3.,-1.,0.,2.])
        self.assertEqual(result['mean_ms'],-.5)
        self.assertEqual(result['p50_ms'],-.5)
        self.assertEqual(result['max_ms'],2.)
        with self.assertRaises(ValueError):stats([-1.,0.])
        for values in ([],[math.inf],[-math.inf],[math.nan]):
            with self.assertRaises(ValueError):profile.signed_stats(values)

    def test_same_contract_oracle_for_all_fixtures_and_cache(self):
        for normal in (False,True):
            for applied in (False,True):
                payload=pack_state(state_object(normal=normal,applied=applied))
                for case in profile.cases():
                    expected=profile.baseline(payload,case)
                    frame=profile.frame(payload)
                    for _ in range(2):
                        result,stages=profile.candidate(payload,case,existing=frame)
                        self.assertTrue(profile.equal(result,expected))
                        self.assertTrue(all(v>=0 for v in stages.values()))
                        if case['name']=='columns31':
                            self.assertNotIn('imu_requested_ns',stages)
                            self.assertNotIn('applied_requested_ns',stages)

    def test_negative_control_changed_output_fails_before_timing(self):
        with tempfile.TemporaryDirectory() as directory:
            with mock.patch.object(profile,'candidate',return_value=('wrong',{})):
                with self.assertRaisesRegex(AssertionError,'oracle mismatch'):
                    profile.run(Path(directory)/'bad',samples=1,warmup=0)

    def test_smoke_records_contracts_orders_counts_and_no_profiler_by_default(self):
        with tempfile.TemporaryDirectory() as directory:
            result=profile.run(Path(directory)/'ok',samples=2,warmup=0)
            self.assertEqual(result['status'],'ORACLE_PASS')
            self.assertEqual(len(result['results']),12)
            self.assertFalse(result['profile'])
            self.assertNotIn('separate_profile',result)
            self.assertEqual(result['request_counts']['first_use_pairs'],24)
            text=(Path(directory)/'ok/samples.csv').read_text()
            self.assertIn('baseline/candidate',text)
            self.assertIn('candidate/baseline',text)


if __name__=='__main__':unittest.main()
