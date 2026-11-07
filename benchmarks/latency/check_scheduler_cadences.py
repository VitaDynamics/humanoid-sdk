"""Finite FakeClock scheduling checks, outside normal unit discovery.

No elapsed-time load, sockets, node, device, compiler or frequency sweep runs.
Reuse the existing example test fixture and only these two original checks.
"""
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / 'tests')]
import test_end_effector_external as example_tests


class IndependentCadenceChecks(example_tests.EndEffectorExampleTest):
    def test_mixed_rates_do_not_couple_endpoints_or_speed_up_body(self):
        c = self.controller()
        clients = self.endpoints(c)
        self.run_case(c,clients,duration=.4,rates={'left':20.,'right':100.},
                      native_hz={'left':200,'right':125})
        self.assertGreater(len(clients[1].commands),len(clients[0].commands)*3)
        for times,period in ((c.command_times,.02),(clients[0].times,.05),(clients[1].times,.01)):
            self.assertTrue(all(b-a >= period-1e-10 for a,b in zip(times,times[1:])))
        for endpoint, frequency in zip(clients, (200, 125)):
            for first, second, a, b in zip(endpoint.commands, endpoint.commands[1:], endpoint.times, endpoint.times[1:]):
                self.assertLessEqual(abs(second.target.pos[0]-first.target.pos[0]), min(b-a, 1/frequency))
            self.assertEqual(endpoint.commands[-1].target.pos, (.0625,))
        self.assertEqual(c.calls.count('enter_external'),1)


    def test_nondividing_body_and_endpoint_deadlines_keep_each_cadence(self):
        c = self.controller()
        clients = self.endpoints(c)
        self.run_case(c, clients, duration=.12, rates={'left': 1/.015, 'right': 1/.025})
        for times, period, minimum in ((c.command_times, .02, 7),
                                       (clients[0].times, .015, 9),
                                       (clients[1].times, .025, 6)):
            self.assertGreaterEqual(len(times), minimum)
            for index, actual in enumerate(times):
                self.assertAlmostEqual(actual-times[0], index*period, places=9)
        self.assertEqual(c.calls.count('exit_external'), 1)



if __name__ == '__main__':
    # Explicit selection avoids rerunning inherited 50 Hz unit methods.
    checks = (
        'test_mixed_rates_do_not_couple_endpoints_or_speed_up_body',
        'test_nondividing_body_and_endpoint_deadlines_keep_each_cadence',
    )
    suite = unittest.TestSuite(IndependentCadenceChecks(name) for name in checks)
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    raise SystemExit(0 if result.wasSuccessful() else 1)
