######################################################################
# This file copyright the Georgia Institute of Technology
#
# Permission is given to students to use or modify this file (only)
# to work on their assignments.
#
# You may NOT publish this file or make it available to others not in
# the course.
#
######################################################################


from DroneSimulator import DroneSimulator
from MatplotlibVisualizer import MatplotlibVisualizer
from drone_pid import who_am_i
import unittest
import traceback

DEBUGGING_SINGLE_PROCESS = True

DEBUG = False # This will output all timesteps for 
              # all test cases, so use with a single 
              # test case to debug, or output to a file 
              # to analyse separately.
             
VISUALIZE = True

Visualizer = MatplotlibVisualizer # TurtleVisualizer # 

TIME_LIMIT = 10

PART_1_a = False
PART_1_b = False
PART_2_a = False
PART_2_b = False
PART_3   = True

Test_Cases = {'1a': PART_1_a, 
              '1b': PART_1_b, 
              '2a': PART_2_a,
              '2b': PART_2_b,
              '3' : PART_3}


class Part_1_a_TestCase(unittest.TestCase):
    """
    Test Part A: 
    Elevation only, no roll or horizontal displacement.
    Gain values supplied by test case.
    """
    
    @classmethod
    def setUpClass(cls):
        """Setup test class.
        """
        cls.simulator = DroneSimulator()
        if VISUALIZE:
            visualizer = Visualizer()
            cls.simulator.add_listener(visualizer)
        
    def setUp(self):
        self.score = 0
    
    def test_case01(self):
                  
        self.score = self.simulator.test_core(
                test_thrust         = True, 
                test_roll           = False, 
                target_elevation    = 4, 
                target_x            = 0,
                drone_mass          = 100,
                num_steps           = 500, 
                target_hover_time   = 50, 
                supply_params       = True, 
                target_elev_error   = 0.02,
                target_x_error      = 0.00,
                thrust_params       = {'tau_p': 8, 'tau_d': 500, 'tau_i': 0},
                DEBUG               = DEBUG,
                VISUALIZE           = VISUALIZE)
        
        
    def test_case02(self):
        self.score = self.simulator.test_core(
            test_thrust         = True, 
            test_roll           = False, 
            target_elevation    = 5, 
            target_x            = 0,
            drone_mass          = 100,
            num_steps           = 500,
            target_hover_time   = 50, 
            supply_params       = True, 
            target_elev_error   = 0.02,
            target_x_error      = 0.00,
            thrust_params       = {'tau_p': 8, 'tau_d': 500, 'tau_i': 0},
            DEBUG               = DEBUG,
            VISUALIZE           = VISUALIZE)
        
    def test_case03(self):
        self.score = self.simulator.test_core(
            test_thrust         = True, 
            test_roll           = False, 
            target_elevation    = 2, 
            target_x            = 0,
            drone_mass          = 100,
            num_steps           = 500, 
            target_hover_time   = 50, 
            supply_params       = True, 
            target_elev_error   = 0.02,
            target_x_error      = 0.00,
            thrust_params       = {'tau_p': 8, 'tau_d': 500, 'tau_i': 0},
            DEBUG               = DEBUG,
            VISUALIZE           = VISUALIZE)

class Part_1_b_TestCase(unittest.TestCase):
    """ 
    Test Part C: Elevation and roll.
    Gain values supplied by test case
    """

    @classmethod
    def setUpClass(cls):
        """Setup test class.
        """
        cls.simulator = DroneSimulator()
        if VISUALIZE:
            visualizer = Visualizer()
            cls.simulator.add_listener(visualizer)
        
    def setUp(self):
        self.score = 0
        
    def test_case01(self):
        self.score = self.simulator.test_core(
            test_thrust         = True, 
            test_roll           = True, 
            target_elevation    = 4, #20, 
            target_x            = 2, #-10,
            drone_mass          = 100,
            num_steps           = 600, #1000, 
            target_hover_time   = 200, 
            supply_params       = True, 
            target_elev_error   = 0.02,
            target_x_error      = 0.02,
            thrust_params       = {'tau_p': 8, 'tau_d': 700, 'tau_i': 0},
            roll_params         = {'tau_p': 0.35, 'tau_d':50, 'tau_i': 0}, 
            DEBUG               = DEBUG,
            VISUALIZE           = VISUALIZE)

    def test_case02(self):
        self.score = self.simulator.test_core(
            test_thrust         = True, 
            test_roll           = True, 
            target_elevation    = 5, #15, 
            target_x            = 3, #15,
            drone_mass          = 100,
            num_steps           = 600, 
            target_hover_time   = 200, 
            supply_params       = True, 
            target_elev_error   = 0.02,
            target_x_error      = 0.02,
            thrust_params       = {'tau_p': 8, 'tau_d': 700, 'tau_i': 0}, 
             roll_params         = {'tau_p': 0.35, 'tau_d':50, 'tau_i': 0},
            DEBUG               = DEBUG,
            VISUALIZE           = VISUALIZE)

    def test_case03(self):
        self.score = self.simulator.test_core(
            test_thrust         = True, 
            test_roll           = True, 
            target_elevation    = 5.5, #17, 
            target_x            = -3.4, #5,
            drone_mass          = 100,
            num_steps           = 600, 
            target_hover_time   = 200, 
            supply_params       = True, 
            target_elev_error   = 0.02,
            target_x_error      = 0.02,
            thrust_params       = {'tau_p': 8, 'tau_d': 700, 'tau_i': 0},
            roll_params         = {'tau_p': 0.35, 'tau_d':50, 'tau_i': 0},
            DEBUG               = DEBUG,
            VISUALIZE           = VISUALIZE)        
    


class Part_2_a_TestCase(unittest.TestCase):
    """ 
    Test Part 2a: 
    Elevation only. Gain values tuned by student.
    """

    @classmethod
    def setUpClass(cls):
        """Setup test class.
        """
        cls.simulator = DroneSimulator()
        if VISUALIZE:
            visualizer = Visualizer()
            cls.simulator.add_listener(visualizer)
        
        
    def setup(self):
        self.score = 0
        
    def test_case01(self):
        self.score = self.simulator.test_core(
            test_thrust         = True, 
            test_roll           = False, 
            target_elevation    = 20, 
            target_x            = 0,
            num_steps           = 500,
            target_hover_time   = 100, 
            supply_params       = False, 
            target_elev_error   = 0.02,
            target_x_error      = 0.00,
            DEBUG               = DEBUG,
            VISUALIZE           = VISUALIZE)
        

    def test_case02(self):
        self.score = self.simulator.test_core(
            test_thrust         = True, 
            test_roll           = False, 
            target_elevation    = 25, 
            target_x            = 0,
            num_steps           = 600, 
            target_hover_time   = 150, 
            supply_params       = False, 
            target_elev_error   = 0.02,
            target_x_error      = 0.00,
            DEBUG               = DEBUG,
            VISUALIZE           = VISUALIZE)
      
        
    def test_case03(self):
        self.score = self.simulator.test_core(
            test_thrust         = True, 
            test_roll           = False, 
            target_elevation    = 40, 
            target_x            = 0,
            num_steps           = 700, 
            target_hover_time   = 120, 
            supply_params       = False, 
            target_elev_error   = 0.02,
            target_x_error      = 0.00,
            DEBUG               = DEBUG,
            VISUALIZE           = VISUALIZE)

        
        
class Part_2_b_TestCase(unittest.TestCase):
    """ 
    Test Part D: Elevation and roll.
    Gain values tuned by student
    """

    @classmethod
    def setUpClass(cls):
        """Setup test class.
        """
        cls.simulator = DroneSimulator()
        if VISUALIZE:
            visualizer = Visualizer()
            cls.simulator.add_listener(visualizer)
        
    def setUp(self):
        self.score = 0
        
    def test_case01(self):
        self.score = self.simulator.test_core(
            test_thrust         = True, 
            test_roll           = True, 
            target_elevation    = 5, 
            target_x            = 3, 
            num_steps           = 600, 
            target_hover_time   = 250, 
            supply_params       = False, 
            target_elev_error   = 0.02, 
            target_x_error      = 0.02, 
            DEBUG               = DEBUG,
            VISUALIZE           = VISUALIZE)

    def test_case02(self):
        self.score = self.simulator.test_core(
            test_thrust         = True, 
            test_roll           = True, 
            target_elevation    = 6, 
            target_x            = 2.2, 
            num_steps           = 600, 
            target_hover_time   = 250, 
            supply_params       = False, 
            target_elev_error   = 0.02, 
            target_x_error      = 0.02, 
            DEBUG               = DEBUG,
            VISUALIZE           = VISUALIZE)
        
    def test_case03(self):
        self.score = self.simulator.test_core(
            test_thrust         = True, 
            test_roll           = True, 
            target_elevation    = 5.5, 
            target_x            = -4.5, 
            num_steps           = 600, 
            target_hover_time   = 250, 
            supply_params       = False, 
            target_elev_error   = 0.02, 
            target_x_error      = 0.02, 
            DEBUG               = DEBUG,
            VISUALIZE           = VISUALIZE)
        
            
class Part_3_TestCase(unittest.TestCase):
    """ Test elevation and roll with RPM error in drone
    
    """

    @classmethod
    def setUpClass(cls):
        """Setup test class.
        """
        cls.simulator = DroneSimulator()
        if VISUALIZE:
            visualizer = Visualizer()
            cls.simulator.add_listener(visualizer)
        
    def setUp(self):
        self.score = 0
        
    def test_case01(self):
        self.score = self.simulator.test_core(
            test_thrust         = True, 
            test_roll           = False, 
            target_elevation    = 18,
            target_x            = 0, #-10,
            num_steps           = 600, #1000, 
            target_hover_time   = 300,
            supply_params       = False, 
            target_elev_error   = 0.02,
            #target_x_error     = 0.02,
            drone_rpm_error     = 2,           
            test_integral       = True,
            DEBUG               = DEBUG,
            VISUALIZE           = VISUALIZE)
        

    def test_case02(self):
        self.score = self.simulator.test_core(
            test_thrust         = True, 
            test_roll           = False, 
            target_elevation    = 2, 
            target_x            = 0, #12,
            num_steps           = 600, #1000, 
            target_hover_time   = 350,
            supply_params       = False, 
            target_elev_error   = 0.02,
            #target_x_error      = 0.02,
            drone_rpm_error     = 2,          
            test_integral       = True,
            DEBUG               = DEBUG,
            VISUALIZE           = VISUALIZE)
    
    def test_case03(self):
        self.score = self.simulator.test_core(
            test_thrust         = True, 
            test_roll           = False, 
            target_elevation    = 22, 
            target_x            = 0, #-10,
            num_steps           = 600, #1000, 
            target_hover_time   = 300,
            supply_params       = False, 
            target_elev_error   = 0.02,
            #target_x_error     = 0.02,
            drone_rpm_error     = 2,           
            test_integral       = True,
            DEBUG               = DEBUG,
            VISUALIZE           = VISUALIZE)
        
    def test_case04(self):
        self.score = self.simulator.test_core(
            test_thrust         = True, 
            test_roll           = False, 
            target_elevation    = 3, 
            target_x            = 0, #-10,
            num_steps           = 600, #1000, 
            target_hover_time   = 300,
            supply_params       = False, 
            target_elev_error   = 0.02,
            #target_x_error     = 0.02,
            drone_rpm_error     = 2,           
            test_integral       = True,
            DEBUG               = DEBUG,
            VISUALIZE           = VISUALIZE)
        
    def test_case05(self):
        self.score = self.simulator.test_core(
            test_thrust         = True, 
            test_roll           = False, 
            target_elevation    = 35, 
            target_x            = 0, #-10,
            num_steps           = 600, #1000, 
            target_hover_time   = 150, 
            supply_params       = False, 
            target_elev_error   = 0.02,
            #target_x_error     = 0.02,
            drone_rpm_error     = 2,           
            test_integral       = True,
            DEBUG               = DEBUG,
            VISUALIZE           = VISUALIZE)
        
    def test_case06(self):
        self.score = self.simulator.test_core(
            test_thrust         = True, 
            test_roll           = False, 
            target_elevation    = 10, 
            target_x            = 0, #-10,
            num_steps           = 600, #1000, 
            target_hover_time   = 300, 
            supply_params       = False, 
            target_elev_error   = 0.02,
            #target_x_error     = 0.02,
            drone_rpm_error     = 3,           
            test_integral       = True,
            DEBUG               = DEBUG,
            VISUALIZE           = VISUALIZE)
        
        

class PIDTestResult(unittest.TestResult):

    def __init__(self, stream=None, descriptions=None, verbosity=None):
        super(PIDTestResult, self).__init__(stream, verbosity, descriptions)
        self.stream = stream
        self.credit = []

    def stopTest(self, test):
        super(PIDTestResult, self).stopTest(test)
        try:
            self.credit.append(test.score)

        except AttributeError as exp:
            if self.stream != None:
                self.stream.write(str(exp))

    @property
    def avg_credit(self):
        try:
            return sum(self.credit) / len(self.credit)

        except ZeroDivisionError:
            return 0.0

# Only run all of the test automatically if this file was executed from the command line.
# Otherwise, let Nose/py.test do it's own thing with the test cases.
if __name__ == "__main__":
    student_id = who_am_i()
    if student_id:
        cases = []
        if PART_1_a is True: cases.append(Part_1_a_TestCase)
        if PART_1_b is True: cases.append(Part_1_b_TestCase)
        if PART_2_a is True: cases.append(Part_2_a_TestCase)
        if PART_2_b is True: cases.append(Part_2_b_TestCase)
        if PART_3 is True: cases.append(Part_3_TestCase)

        suites = [unittest.TestSuite(unittest.TestLoader().loadTestsFromTestCase(case)) for case in cases]

        total_passes = 0
        average_scores = []

        try:
            for i, suite in zip(Test_Cases.keys(), suites):
                print("====================\nTests for Part {}:".format(i))

                result = PIDTestResult()
                suite.run(result)
                average_scores.append(result.avg_credit)

                print("Average Weighted Score: ", result.avg_credit * 100/5)

            overall_score = (sum(average_scores)/len(average_scores)) * 100
        
        except Exception as e:
            #print(e)
            traceback.print_exc()
            overall_score = 0
        if overall_score > 100:
            print()
            print("Score above 100:", overall_score, " capped to 101!")
            overall_score = 101
        print("====================\nScore: {}".format(overall_score))
    else:
        print("Student ID not specified.  Please fill in 'whoami' variable.")
        print('score: 0')
