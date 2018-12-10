import sys
from controller import Controller
from model import Model
from View import View
import argparse

class Main(object):

	
	def __init__(self, algo):
		self.algo = algo
		self.run()

	def run(self):
		model_object = Model()
		controller_object = Controller()
		view_object = View()


		controller_object.set_model(model_object)
		view_object.set_controller(controller_object)

		controller_object.set_view(view_object)
		controller_object.get_input()


		model_object.set_algo(self.algo)
		controller_object.get_route()


if __name__ == '__main__':
	parser = argparse.ArgumentParser(description='Set a strategy [1,2,3] beware 1 is fast but only minimizes, default is 1 though')
	parser.add_argument('--algorithm', help='an integer for algorithm', default='1')

	args = parser.parse_args()
	option = args.algorithm
	mobj = Main(int(option))

