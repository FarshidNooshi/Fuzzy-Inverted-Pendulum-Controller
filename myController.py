# -*- coding: utf-8 -*-

# python imports
from math import degrees

import numpy as np


class MyFuzzyController:

    def __init__(self):
        self.system = System()

    def _make_input(self, world):
        return dict(
            cp=world.x,
            cv=world.v,
            pa=degrees(world.theta),
            pv=degrees(world.omega)
        )

    def _make_output(self):
        return dict(
            force=0.
        )

    def decide(self, world):
        output = self._make_output()
        self.system.calculate(self._make_input(world), output)
        return output['force']


def do_fuzzify_and_inference(inputs):
    if inputs['pa'] < 0.0:
        inputs['pa'] = -inputs['pa']
    methods = FuzzificationMethods()
    return methods.calculate_with_rules_2_vars(inputs)


def center_of_gravity(x, y):
    up = 0
    down = 0
    for i in enumerate(x):
        up += y[i[0]] * x[i[0]]
        down += y[i[0]]

    return up / (float(down)) if down != 0.0 else 0.0


def defuzzify(results):
    x = np.arange(-100, 100, 0.05)
    y = np.zeros(len(x))
    methods = FuzzificationMethods()
    for i0, i1 in enumerate(x):
        y[i0] = max(methods.left_fast_force(x=i1, limit=results['left_fast']),
                    methods.left_slow_force(x=i1, limit=results['left_slow']),
                    methods.right_fast_force(x=i1, limit=results['right_fast']),
                    methods.right_slow_force(x=i1, limit=results['right_slow']),
                    methods.stop_force(x=i1, limit=results['stop']))
    return center_of_gravity(x, y)


class System:
    def __init__(self):
        pass

    def calculate(self, inputs, output):
        results = do_fuzzify_and_inference(inputs=inputs)

        output['force'] = defuzzify(results)


def line_equation((x1, y1), (x2, y2), x):
    if 0 == x2 - x1:
        y = 1.0 * max(y1, y2)
    else:
        m = (y2 - y1) / (float(x2 - x1))
        b = y1 - m * x1
        y = m * x + b
    return y


def get_fuzzify(x, points, is_left_free=False, is_right_free=False):
    (x1, y1), (x2, y2), (x3, y3) = points
    if is_left_free:
        if x <= x2:
            return line_equation((x1, y1), (x2, y2), x)
    if is_right_free:
        if x2 <= x:
            return line_equation((x3, y3), (x2, y2), x)
    if x1 <= x <= x2:
        return line_equation((x1, y1), (x2, y2), x)
    elif x2 <= x <= x3:
        return line_equation((x3, y3), (x2, y2), x)
    else:
        return 0


class FuzzificationMethods:
    def __init__(self):
        pass

    # pa
    def up_more_right_pa(self, x):
        points = ((0, 0), (30, 1), (60, 0))
        return get_fuzzify(x, points)

    def up_right_pa(self, x):
        points = ((30, 0), (60, 1), (90, 0))
        return get_fuzzify(x, points)

    def up_pa(self, x):
        points = ((60, 0), (90, 1), (120, 0))
        return get_fuzzify(x, points)

    def up_left_pa(self, x):
        points = ((90, 0), (120, 1), (150, 0))
        return get_fuzzify(x, points)

    def up_more_left_pa(self, x):
        points = ((120, 0), (150, 1), (180, 0))
        return get_fuzzify(x, points)

    def down_more_left_pa(self, x):
        points = ((180, 0), (210, 1), (240, 0))
        return get_fuzzify(x, points)

    def down_left_pa(self, x):
        points = ((210, 0), (240, 1), (270, 0))
        return get_fuzzify(x, points)

    def down_pa(self, x):
        points = ((240, 0), (270, 1), (300, 0))
        return get_fuzzify(x, points)

    def down_right_pa(self, x):
        points = ((270, 0), (300, 1), (330, 0))
        return get_fuzzify(x, points)

    def down_more_right_pa(self, x):
        points = ((300, 0), (330, 1), (360, 0))
        return get_fuzzify(x, points)

    # pv
    def cw_fast_pv(self, x):
        points = ((-200, 0), (-200, 1), (-100, 0))
        return get_fuzzify(x, points, is_left_free=True)

    def cw_slow_pv(self, x):
        points = ((-200, 0), (-100, 1), (0, 0))
        return get_fuzzify(x, points)

    def stop_pv(self, x):
        points = ((-100, 0), (0, 1), (100, 0))
        return get_fuzzify(x, points)

    def ccw_slow_pv(self, x):
        points = ((0, 0), (100, 1), (200, 0))
        return get_fuzzify(x, points)

    def ccw_fast_pv(self, x):
        points = ((100, 0), (200, 1), (200, 0))
        return get_fuzzify(x, points, is_right_free=True)

    @staticmethod
    def return_result(y, limit):
        if limit < y:
            return limit * 1.0
        return y * 1.0

    # force
    def left_fast_force(self, x, limit):
        points = ((-100, 0), (-80, 1), (-60, 0))
        y = get_fuzzify(x, points)
        return FuzzificationMethods.return_result(y, limit=limit)

    def left_slow_force(self, x, limit):
        points = ((-80, 0), (-60, 1), (0, 0))
        y = get_fuzzify(x, points)
        return FuzzificationMethods.return_result(y, limit=limit)

    def stop_force(self, x, limit):
        points = ((-60, 0), (0, 1), (60, 0))
        y = get_fuzzify(x, points)
        return FuzzificationMethods.return_result(y, limit=limit)

    def right_slow_force(self, x, limit):
        points = ((0, 0), (60, 1), (80, 0))
        y = get_fuzzify(x, points)
        return FuzzificationMethods.return_result(y, limit=limit)

    def right_fast_force(self, x, limit):
        points = ((60, 0), (80, 1), (100, 0))
        y = get_fuzzify(x, points)
        return FuzzificationMethods.return_result(y, limit=limit)

    def calculate_with_rules_2_vars(self, inputs):
        pa_list = ["down_left", "down_more_right", "down_right", "up", "down_more_left", "down", "up_left",
                   "up_more_right", "up_right", "up_more_left"]
        pv_list = ["cw_fast", "stop", "cw_slow", "ccw_slow", "ccw_fast"]
        pa_values = dict()
        pv_values = dict()
        methods = FuzzificationMethods()
        for item in pa_list:
            pa_values[item] = getattr(methods, item + '_pa')(inputs['pa'])
        for item in pv_list:
            pv_values[item] = getattr(methods, item + '_pv')(inputs['pv'])

        return self.apply_rules(pa_values=pa_values, pv_values=pv_values)

    def apply_rules(self, pa_values, pv_values):
        results = dict()
        results['stop'] = max(min((pa_values['up'], pv_values['stop'])),
                              min((pa_values['up_right'], pv_values['ccw_slow'])),
                              min((pa_values['up_left'], pv_values['cw_slow'])))
        rules = [("up_more_right", 'ccw_slow'),
                 ('up_more_right', 'cw_slow'),
                 ('up_more_right', 'cw_fast'),
                 ('down_more_right', 'ccw_slow'),
                 ('down_right', 'ccw_slow'),
                 ('down_right', 'cw_slow'),
                 ('up_right', 'cw_slow'),
                 ('up_right', 'stop'),
                 ('up_right', 'cw_fast'),
                 ('up_left', 'cw_fast'),
                 ('down', 'stop'),
                 ('up', 'cw_fast')]
        results['right_fast'] = min(pa_values['up_more_right'], pv_values['ccw_slow'])
        for item_pa, item_pv in rules:
            results['right_fast'] = max(results['right_fast'], min(pa_values[item_pa], pv_values[item_pv]))
        # stop
        rules = [("down_more_right", 'cw_slow'),
                 ('down_more_left', 'ccw_slow'),
                 ('down_more_right', 'cw_fast'),
                 ('down_more_right', 'ccw_fast'),
                 ('down_more_left', 'ccw_fast'),
                 ('down_more_left', 'cw_fast'),
                 ('down_right', 'ccw_fast'),
                 ('down_left', 'cw_fast'),
                 ('down', 'cw_fast'),
                 ('up', 'stop'),
                 ('down', 'ccw_fast')]
        results['stop'] = min(pa_values['down_more_right'], pv_values['cw_slow'])
        for item_pa, item_pv in rules:
            results['stop'] = max(results['stop'], min(pa_values[item_pa], pv_values[item_pv]))
        # left fast
        rules = [('up_more_left', 'cw_slow'),
                 ('up_more_left', 'ccw_slow'),
                 ('up_more_left', 'ccw_fast'),
                 ('down_more_left', 'cw_slow'),
                 ('down_left', 'cw_slow'),
                 ('down_left', 'ccw_slow'),
                 ('up_left', 'ccw_slow'),
                 ('up_left', 'stop'),
                 ('up_right', 'ccw_fast'),
                 ('up_left', 'ccw_fast'),
                 ('up', 'ccw_fast')]
        results['left_fast'] = min(pa_values['up_more_left'], pv_values['cw_slow'])
        for item_pa, item_pv in rules:
            results['left_fast'] = max(results['left_fast'], min(pa_values[item_pa], pv_values[item_pv]))
        # right slow
        rules = [("up_more_left", "cw_fast"),
                 ("down_right", "cw_fast"),
                 ("up_right", "ccw_slow"),
                 ("up", "cw_slow")]
        results['right_slow'] = min(pa_values['up_more_left'], pv_values['cw_fast'])
        for item_pa, item_pv in rules:
            results['right_slow'] = max(results['right_slow'], min(pa_values[item_pa], pv_values[item_pv]))
        # left slow
        rules = [("up_more_right", "ccw_fast"),
                 ("down_left", "ccw_fast"),
                 ("up_left", "cw_slow"),
                 ("up", "ccw_slow")]
        results['left_slow'] = min(pa_values['up_more_right'], pv_values['ccw_fast'])
        for item_pa, item_pv in rules:
            results['left_slow'] = max(results['left_slow'], min(pa_values[item_pa], pv_values[item_pv]))
        return results
