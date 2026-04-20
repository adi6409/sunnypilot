#!/usr/bin/env python3
import argparse
import os
import sys
import unittest # noqa: TID251

from opendbc.car.tests.routes import CarTestRoute
from openpilot.selfdrive.car.tests.test_models import TestCarModel
from openpilot.tools.lib.logreader import LogReader
from openpilot.tools.lib.route import SegmentRange


def create_test_models_suite(routes: list[CarTestRoute]) -> unittest.TestSuite:
  test_suite = unittest.TestSuite()
  for test_route in routes:
    # create new test case and discover tests
    test_case_args = {"platform": test_route.car_model, "test_route": test_route}
    CarModelTestCase = type("CarModelTestCase", (TestCarModel,), test_case_args)
    test_suite.addTest(unittest.TestLoader().loadTestsFromTestCase(CarModelTestCase))
  return test_suite


def create_local_test_models_suite(file_path: str, car_model: str | None) -> unittest.TestSuite:
  test_suite = unittest.TestSuite()

  @classmethod
  def get_testing_data(cls):
    lr = LogReader(file_path, sort_by_time=True)
    return cls.get_testing_data_from_logreader(lr)

  test_case_args = {
    "platform": car_model,
    "test_route": CarTestRoute(os.path.basename(file_path), car_model, segment=0),
    "get_testing_data": get_testing_data,
  }
  CarModelTestCase = type("CarModelTestCase", (TestCarModel,), test_case_args)
  test_suite.addTest(unittest.TestLoader().loadTestsFromTestCase(CarModelTestCase))
  return test_suite


if __name__ == "__main__":
  parser = argparse.ArgumentParser(description="Test any route against common issues with a new car port. " +
                                               "Uses selfdrive/car/tests/test_models.py")
  parser.add_argument("route_or_segment_name", help="Specify route (or local rlog file path) to run tests on")
  parser.add_argument("--car", help="Specify car model for test route")
  args = parser.parse_args()
  if len(sys.argv) == 1:
    parser.print_help()
    sys.exit()

  if os.path.isfile(args.route_or_segment_name):
    test_suite = create_local_test_models_suite(args.route_or_segment_name, args.car)
  else:
    sr = SegmentRange(args.route_or_segment_name)
    test_routes = [CarTestRoute(sr.route_name, args.car, segment=seg_idx) for seg_idx in sr.seg_idxs]
    test_suite = create_test_models_suite(test_routes)

  unittest.TextTestRunner().run(test_suite)
