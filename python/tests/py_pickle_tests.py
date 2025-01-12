import unittest
import pickle
import numpy as np

from bark.world.agent import *
from bark.models.behavior import *
from bark.world import *
from bark.world.map import *
from bark.geometry import *
from bark.models.dynamic import *
from bark.models.execution import *
from bark.geometry import *
from bark.geometry.standard_shapes import *
from bark.world.goal_definition import *
from modules.runtime.commons.parameters import ParameterServer

def pickle_unpickle(object):
    f = open('temp.pickle','wb')
    pickle.dump(object, f)
    f.close()
    f = open( 'temp.pickle', "rb" )
    object = pickle.load( f)
    f.close()
    return object


class PickleTests(unittest.TestCase):

    def test_geometry_pickle(self):
        # point 2d
        p = Point2d(2 ,3)

        pa = pickle_unpickle(p)
        self.assertEqual(p.x(), pa.x())
        self.assertEqual(p.y(), pa.y())

        # linestring
        l = Line2d()
        l.addPoint(p)
        l.addPoint(Point2d(10,4))
        l.addPoint(Point2d(1.555555, 1.244222))

        la = pickle_unpickle(l)
        self.assertTrue(np.array_equal(l.toArray(), la.toArray()))

        l = Line2d()
        la = pickle_unpickle(l)
        self.assertTrue(np.array_equal(l.toArray(), la.toArray()))

        # polygon
        p = CarLimousine()
        pa = pickle_unpickle(p)
        self.assertTrue(np.array_equal(p.toArray(), pa.toArray()))

    def test_behavior_model_pickle(self):
        
        params = ParameterServer()
        b = BehaviorConstantVelocity(params)

        ba = pickle_unpickle(b)
        self.assertTrue(isinstance(ba, BehaviorConstantVelocity))

    def test_execution_model_pickle(self):
        
        params = ParameterServer()
        e = ExecutionModelInterpolate(params)

        ea = pickle_unpickle(e)
        self.assertTrue(isinstance(ea,ExecutionModelInterpolate))

    def test_dynamic_model_pickle(self):
        
        params = ParameterServer()
        d = SingleTrackModel()

        da = pickle_unpickle(d)
        self.assertTrue(isinstance(da,SingleTrackModel))

    def test_driving_corridor_pickle(self):
        cor = DrivingCorridor()

        l1 = Line2d()
        l1.addPoint(Point2d(10,4))

        l2 = Line2d()
        l2.addPoint(Point2d(10,4))
        l2.addPoint(Point2d(1.555555, 1.244222))

        l3 = Line2d()
        l3.addPoint(Point2d(10,4))
        l3.addPoint(Point2d(1.555555, 1.244222))
        l3.addPoint(Point2d(20, 45))

        cor.inner = l1
        cor.outer = l2
        cor.center = l3

        cor_after = pickle_unpickle(cor)

        self.assertTrue(np.array_equal(cor.inner.toArray(), cor_after.inner.toArray()))
        self.assertTrue(np.array_equal(cor.outer.toArray(), cor_after.outer.toArray()))
        self.assertTrue(np.array_equal(cor.center.toArray(), cor_after.center.toArray()))

    def test_agent_pickle(self):

        params = ParameterServer()
        behavior = BehaviorConstantVelocity(params)
        execution = ExecutionModelInterpolate(params)
        dynamic = SingleTrackModel()
        shape = CarLimousine()
        init_state = np.array([0, 0, 0, 0, 5])
        goal_polygon = Polygon2d([0, 0, 0],[Point2d(-1,-1),Point2d(-1,1),Point2d(1,1), Point2d(1,-1)])
        goal_definition = GoalDefinition(goal_polygon)
        agent = Agent(init_state, behavior, dynamic, execution, shape, params.AddChild("agent"), goal_definition )

        agent_after = pickle_unpickle(agent)

        self.assertEqual(agent_after.id , agent.id)
        self.assertTrue(np.array_equal(agent_after.state, agent.state) )
        self.assertTrue(np.array_equal(agent_after.goal_definition.goal_shape.center, \
                                        agent.goal_definition.goal_shape.center))

        agent_list = []
        agent_list.append(agent)

        agent_list_after = pickle_unpickle(agent_list)

        self.assertEqual(agent_list_after[0].id , agent.id)
        self.assertTrue(np.array_equal(agent_list_after[0].state, agent.state) )






if __name__ == '__main__':
    unittest.main()