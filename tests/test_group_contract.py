"""Effective Core group gates run before endpoints; fake nodes only, no IPC."""
import os
from types import SimpleNamespace as S
import unittest
from unittest.mock import patch

from locomotion_aorta import BodyStateSource, PullStateSource
from locomotion_aorta.transport import _AortaTransport


class Node:
    instances = []

    def __init__(self, name, requested):
        self.requested = requested
        self.actual = os.environ.get('AORTA_GROUP', 'default')
        self.endpoints = []
        self.closed = False
        self.group_reads = 0
        self.instances.append(self)

    def group(self):
        self.group_reads += 1
        return self.actual

    def create_subscriber(self, *args, **kwargs):
        self.endpoints.append('callback')
        return S(close=lambda: None)

    def create_subscriber_pull(self, *args, **kwargs):
        self.endpoints.append('pull')
        return S(close=lambda: None)

    def create_publisher_typed(self, *args, **kwargs):
        self.endpoints.append('publisher')
        raise AssertionError('unexpected control publisher')

    def close(self):
        self.closed = True


class GroupContractTest(unittest.TestCase):
    def setUp(self):
        Node.instances = []
        aorta = S(Node=Node, QoS=S(sensor_data=lambda: None, state_update=lambda: None))
        decoder = S(BodyState=S(GetRootAs=lambda value: value))
        p = patch('locomotion_aorta._state_stream.importlib.import_module',
                  side_effect=lambda name: aorta if name=='aorta'else decoder)
        p.start();self.addCleanup(p.stop)
        p = patch('locomotion_aorta.transport._load_aorta_bindings',return_value=S(aorta=aorta))
        p.start();self.addCleanup(p.stop)

    def create(self, kind, group):
        if kind=='callback':
            return BodyStateSource(group=group)
        if kind=='pull':
            return PullStateSource(group=group)
        return _AortaTransport(node_name='test',group=group)

    def test_default_group_matches_unset_environment_for_both_state_apis(self):
        with patch.dict(os.environ,{},clear=True):
            for kind in ('callback','pull'):
                source=self.create(kind,'default')
                self.assertEqual(source.group,'default')
                self.assertEqual(Node.instances[-1].group_reads,1)
                self.assertEqual(len(Node.instances[-1].endpoints),1 if kind=='callback'else 2)
                source.close()

    def test_requested_actual_and_AORTA_GROUP_match_without_environment_mutation(self):
        values={'AORTA_GROUP':'robot-A','SDK_GROUP':'robot-A'}
        with patch.dict(os.environ,values,clear=True):
            for kind in ('callback','pull'):
                source=self.create(kind,'robot-A')
                self.assertEqual(source.group,'robot-A')
                source.close()
            self.assertEqual(dict(os.environ),values)

    def test_override_mismatch_rejects_before_any_subscriber_or_control_publisher(self):
        with patch.dict(os.environ,{'AORTA_GROUP':'robot-B'},clear=True):
            for kind in ('callback','pull','control'):
                with self.subTest(kind=kind),self.assertRaises(ValueError)as failure:
                    self.create(kind,'robot-A')
                message=str(failure.exception)
                self.assertIn("requested='robot-A'",message)
                self.assertIn("actual='robot-B'",message)
                self.assertIn("AORTA_GROUP='robot-B'",message)
                node=Node.instances[-1]
                self.assertTrue(node.closed)
                self.assertFalse(node.endpoints)
                self.assertEqual(node.group_reads,1)

    def test_nondefault_argument_without_AORTA_GROUP_rejects_core_default(self):
        with patch.dict(os.environ,{},clear=True):
            for kind in ('callback','pull'):
                with self.assertRaisesRegex(ValueError,"actual='default'.*AORTA_GROUP=None"):
                    self.create(kind,'robot-A')
                self.assertTrue(Node.instances[-1].closed)
                self.assertFalse(Node.instances[-1].endpoints)

    def test_SDK_GROUP_alone_does_not_override_core(self):
        with patch.dict(os.environ,{'SDK_GROUP':'robot-A'},clear=True):
            with self.assertRaisesRegex(ValueError,'SDK_GROUP is a demo/runner convention'):
                self.create('pull','robot-A')
            self.assertFalse(Node.instances[-1].endpoints)

    def test_mock_runner_mapping_with_explicit_group_passes(self):
        from tools.production_topic_mock.run import child_environment
        group='mock-sdkmain-'+('a'*19)
        values=child_environment({},group,'/tmp/mock-private-peer.json')
        with patch.dict(os.environ,values,clear=True):
            source=self.create('pull',group)
            self.assertEqual(source.group,group)
            self.assertEqual(Node.instances[-1].actual,group)
            self.assertEqual(len(Node.instances[-1].endpoints),2)
            source.close()


if __name__=='__main__':
    unittest.main()
