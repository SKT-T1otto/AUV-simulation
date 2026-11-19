from dataclasses import replace
import unittest
from crk_ch3.progress_commit import PendingSearchCommits,RouteUnavailable
from progress_support import search,rebuild


class QueueTests(unittest.TestCase):
    def queue(self):
        old=search();new=search(goal=(5.,1.,.5),name='new');queue=PendingSearchCommits()
        queue.offer(new,old,old.waypoint,step=20,proposal_hash='native_hash')
        return queue,old,new

    def test_strict_arrival_wait_prepare_ack_and_fresh_start(self):
        q,old,new=self.queue()
        self.assertEqual(q.prepare({0:old},[(2.25,1.,.5)],{0:True},rebuild,step=21),())
        ready=q.prepare({0:old},[(2.3,1.,.5)],{0:True},rebuild,step=22)
        self.assertEqual(ready[0].assignment.path[0],(2.3,1.,.5))
        self.assertEqual(len(q.pending),1)  # Preparation is not installation.
        q.acknowledge(ready,step=22)
        self.assertFalse(q.pending);self.assertEqual(q.committed,1)
        self.assertEqual(q.events[-1]['proposed_step'],20);self.assertEqual(q.events[-1]['committed_step'],22)

    def test_superseded_and_reaffirmed_intents(self):
        q,old,new=self.queue()
        q.offer(replace(new,candidate_id='newer'),old,old.waypoint,step=21,proposal_hash='newer_hash')
        self.assertEqual(q.pending[0].assignment.candidate_id,'newer')
        q.offer(old,old,old.waypoint,step=22,proposal_hash='keep')
        self.assertFalse(q.pending)

    def test_changed_installed_route_and_unsafe_old_path_cancel(self):
        for changed,safe in [(True,True),(False,False)]:
            q,old,_=self.queue()
            q.prepare({0:replace(old,candidate_id='reroute') if changed else old},[(2.5,1.,.5)],{0:safe},rebuild,step=21)
            self.assertFalse(q.pending);self.assertEqual(q.committed,0)

    def test_unreachable_or_stale_rebuild_never_installs_cached_path(self):
        for failed in ('unreachable','old_start'):
            q,old,new=self.queue()
            def bad(intent,position):
                if failed=='unreachable':raise RouteUnavailable('no_connector')
                return intent,{}
            self.assertEqual(q.prepare({0:old},[(2.5,1.,.5)],{0:True},bad,step=21),())
            self.assertFalse(q.pending);self.assertEqual(q.committed,0)

    def test_no_partial_acknowledgement_of_superseded_preparation(self):
        q,old,new=self.queue();ready=q.prepare({0:old},[(2.5,1.,.5)],{0:True},rebuild,step=21)
        q.offer(replace(new,candidate_id='newer'),old,old.waypoint,step=22,proposal_hash='second')
        with self.assertRaises(RuntimeError):q.acknowledge(ready,step=22)
        self.assertEqual(q.committed,0);self.assertEqual(q.pending[0].assignment.candidate_id,'newer')

    def test_phase_stop_cancellation_is_explicit(self):
        q,_,_=self.queue();q.clear(21,'handoff');self.assertFalse(q.pending)
        self.assertEqual(q.events[-1]['reason'],'handoff')

    def test_executor_is_rejected(self):
        q=PendingSearchCommits()
        with self.assertRaises(ValueError):q.offer(search(3),search(3),(3,1,.5),step=20,proposal_hash='x')


if __name__=='__main__':unittest.main()
