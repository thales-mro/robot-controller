import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl





#def main():

class gtgFuzzyController():

	def __init__(self):
		deltaR = ctrl.Antecedent(np.arange(0, 50, 1), 'distance')
		deltaA = ctrl.Antecedent(np.arange(0, 8, 1), 'angle variation')
		velocity = ctrl.Consequent(np.arange(0, 6, 1), 'velocity')

		deltaR['far'] = fuzz.trapmf(deltaR.universe, [3, 4, 50, 50])
		deltaR['medium'] = fuzz.trapmf(deltaR.universe, [0.75, 2, 3, 4])
		deltaR['close'] = fuzz.trapmf(deltaR.universe, [0, 0, 0.5, 1.0])

		deltaA['high'] = fuzz.trapmf(deltaA.universe, [np.pi/2, 7*np.pi/12, 7.0, 7.0])
		deltaA['medium'] = fuzz.trapmf(deltaA.universe, [np.pi/6, np.pi/4, 5*np.pi/12, 7*np.pi/12])
		deltaA['low'] = fuzz.trapmf(deltaA.universe, [0, 0, np.pi/6, np.pi/4])

		velocity['fast'] = fuzz.trapmf(velocity.universe, [3.5, 5, 5, 5])
		velocity['normal'] = fuzz.trapmf(velocity.universe, [1.5, 2.5, 3.5, 4])
		velocity['slow'] = fuzz.trapmf(velocity.universe, [0, 0, 1.5, 2])


		rule1 = ctrl.Rule(deltaR['far'] & (deltaA['high'] | deltaA['medium']), velocity['normal'])
		rule2 = ctrl.Rule(deltaR['far'] & deltaA['low'], velocity['fast'])

		rule3 = ctrl.Rule(deltaR['medium'] & deltaA['high'], velocity['normal'])
		rule4 = ctrl.Rule(deltaR['medium'] & (deltaA['medium'] | deltaA['low']), velocity['normal'])

		rule5 = ctrl.Rule(deltaR['close'] & deltaA['high'], velocity['normal'])
		rule6 = ctrl.Rule(deltaR['close'] & (deltaA['medium'] | deltaA['low']), velocity['slow'])
		

		gtg_ctrl = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5, rule6])


		self.gtg = ctrl.ControlSystemSimulation(gtg_ctrl)

	def computeVelocity(self, ErrorDistance, ErrorAngle):
		self.gtg.input['distance'] = ErrorDistance
		self.gtg.input['angle variation'] = ErrorAngle

		self.gtg.compute()

		vel = self.gtg.output['velocity']
		return vel

	




#if __name__ == '__main__':
#	main()