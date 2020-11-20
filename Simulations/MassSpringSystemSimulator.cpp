#include "MassSpringSystemSimulator.h"


MassSpringSystemSimulator::MassSpringSystemSimulator(){
	setMass(10);
	setStiffness(40);
}

// UI Functions
const char * MassSpringSystemSimulator::getTestCasesStr() { return "One-step,Simple Spring,Complex scene"; }

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass * DUC){
	this->DUC = DUC;

	// Switch between integrators
	TwType integratorTypes;
	integratorTypes = TwDefineEnum("integratorTypes", integrators, 3);
	TwAddVarRW(DUC->g_pTweakBar, "Integration Method", integratorTypes, &integrator, NULL);


	if (test_case == 2) {
		TwAddVarRW(DUC->g_pTweakBar, "Gravity", TW_TYPE_FLOAT, &gravity, "min=0 max=10 step=0.1");
		TwAddVarRW(DUC->g_pTweakBar, "Wind Force", TW_TYPE_FLOAT, &wind, "min=0 step=0.1");
		TwAddVarRW(DUC->g_pTweakBar, "Spring stiffness", TW_TYPE_FLOAT, &m_fStiffness, "min=10 step=10");
	}
}

void MassSpringSystemSimulator::reset(){
	points.clear();
	springs.clear();
	if (test_case < 2) {
		addMassPoint(Vec3(), Vec3(-1, 0, 0), false);
		addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false);
		addSpring(0, 1, 1);
	}
	if (test_case == 2) {
		float baseLength = 0.3;
		for (int i = 0; i < 10; i++)
			for (int j = 0; j < 10; j++)
				addMassPoint(Vec3(baseLength * i, 1, baseLength * j), Vec3(0, 0, 0), false);
		points[0].isFixed = true;
		points[9].isFixed = true;

		for (int i = 0; i < 10; i++)
			for (int j = 0; j < 10; j++) {
				// Structural
				if(j < 9)	addSpring(10 * i + j, 10 * i + j + 1, baseLength);
				if(i < 9) addSpring(10 * i + j, 10 * (i+1) + j, baseLength);
				// Flexion
				if(j < 8) addSpring(10 * i + j, 10 * i + j + 2, 2 * baseLength);
				if(i < 8)	addSpring(10 * i + j, 10 * (i+2) + j, 2 * baseLength);
				// Shear
				if (i < 9 && j < 9) addSpring(10 * i + j, 10 * (i + 1) + j + 1, 1.414*baseLength);
				if (i < 9 && j > 0) addSpring(10 * i + j, 10 * (i + 1) + j - 1, 1.414*baseLength);
			}
	}
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext){
	if (previous_test_case != test_case) {// test case changed
		// clear old setup and build up new setup
		notifyCaseChanged(test_case);
		previous_test_case = test_case;
		reset();
	}

	for (int i = 0; i < getNumberOfSprings(); i++) {
		DUC->beginLine();
		DUC->drawLine(points[springs[i].p1].position, Vec3(1, 1, 1), points[springs[i].p2].position, Vec3(1, 1, 1));
		DUC->endLine();
	}

	DUC->setUpLighting(Vec3(1,1,1), Vec3(1,1,1), 0.1, Vec3(1,1,1));
	for (int i = 0; i < getNumberOfMassPoints(); i++) {
		if (points[i].isFixed) {
			DUC->setUpLighting(Vec3(1,0,0), Vec3(1,0,0), 0.1, Vec3(1, 0, 0));
			DUC->drawSphere(points[i].position, 0.1);
			DUC->setUpLighting(Vec3(1,1,1), Vec3(1,1,1), 0.1, Vec3(1, 1, 1));
		}
		else { DUC->drawSphere(points[i].position, 0.1); }
	}
}
void MassSpringSystemSimulator::notifyCaseChanged(int testCase) {
	test_case = testCase;
	std::cout << "Welcome to test case " << tests[test_case].Label << '!' << std::endl;
	switch (testCase) {
	case 0:
		std::cout << "Click on the animation to perform an integration step. Feel free to change the integrator !" << std::endl;
		break;
	case 1:
		break;
	case 2:
		std::cout << "Recommended parameters : \n\tGravity = 0.1\n\tWind = 2\n\tStiffness = 140" << std::endl;
		break;
	default:
		std::cout << "How did you even get there ?" << std::endl;
	}
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed){
	if (test_case < 2) return;

	for (int i = 0; i < getNumberOfMassPoints(); i++) {
		// gravity
		points[i].Velocity[1] += -timeElapsed * gravity;
		// wind
		points[i].Velocity[0] += timeElapsed * wind / m_fMass;
	}
}

void MassSpringSystemSimulator::simulateTimestep(float timeStep){
	if (test_case == 0 && notOnCallback) return;
	switch (integrator){
	case 0: // semi-implicit Euler
		externalForcesCalculations(timeStep);
		eulerIntegrator(timeStep);
		break;
	case 1: // LeapFrog
		externalForcesCalculations(timeStep);
		leapfrogIntegrator(timeStep);
		break;
	case 2: // Midpoint
		midpointIntegrator(timeStep);
		break;
	default:
		break;
	}
	collisionCheck();
	if (test_case == 0) {
		std::cout << "Position and speed after last step with integrator " << integrators[integrator].Label << std::endl;
		for (int i = 0; i < getNumberOfMassPoints(); i++)
			std::cout << "Point " << i << " : x(t) = " << points[i].position << " - v(t) = " << points[i].Velocity << std::endl;
		notOnCallback = true;
	}
}

void MassSpringSystemSimulator::collisionCheck(){
	if (test_case < 2) return;

	for(int i = 0; i < getNumberOfMassPoints(); i++)
		if (points[i].position[1] < -0.9) {
			points[i].position[1] = -0.9;
			points[i].Velocity[1] = 0;
		}
}

void MassSpringSystemSimulator::onClick(int x, int y) { if (test_case == 0) { notOnCallback = false;  simulateTimestep(0.1); } }
void MassSpringSystemSimulator::onMouse(int x, int y){}

// Specific Functions
void MassSpringSystemSimulator::setMass(float mass) { m_fMass = mass; }
void MassSpringSystemSimulator::setStiffness(float stiffness) { m_fStiffness = stiffness; }
void MassSpringSystemSimulator::setDampingFactor(float damping) { m_fDamping = damping; }

int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed) {
	int i = getNumberOfMassPoints();
	MassPoint p = MassPoint(position, Velocity, isFixed);
	points.push_back(p);
	return i; 
}

void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength){
	springs.push_back(Spring(masspoint1, masspoint2, initialLength));
}

int MassSpringSystemSimulator::getNumberOfMassPoints() { return points.size(); }
int MassSpringSystemSimulator::getNumberOfSprings() { return springs.size(); }
Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index){ return points[index].position; }
Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index){ return points[index].Velocity; }
void MassSpringSystemSimulator::applyExternalForce(Vec3 force){}


void MassSpringSystemSimulator::eulerIntegrator(float h){ // Implements semi-implicit Euler method
	// 1. Update Velocity
	for (int i = 0; i < getNumberOfSprings(); i++) {
		MassPoint p1 = points[springs[i].p1];
		MassPoint p2 = points[springs[i].p2];
		float il = springs[i].initial_length;
		float l = sqrt(p1.position.squaredDistanceTo(p2.position));
		float force = m_fStiffness * (l - il) / l;
		if (!p1.isFixed) {
			p1.Velocity = p1.Velocity - h * force*(p1.position - p2.position) / m_fMass; 
			points[springs[i].p1] = p1;
		}
		if (!p2.isFixed) {
			p2.Velocity = p2.Velocity - h * force*(p2.position - p1.position) / m_fMass;
			points[springs[i].p2] = p2;
		}
	}
	// 2. Update Position (after velocity since semi-implicit)
	for (int i = 0; i < getNumberOfMassPoints(); i++) {
		if (!points[i].isFixed)
			points[i].position = points[i].position + h * points[i].Velocity;
	}
}

void MassSpringSystemSimulator::midpointIntegrator(float h){
	vector<Vec3> step0positions;
	for (int i = 0; i < getNumberOfMassPoints(); i++)
		step0positions.push_back(points[i].position);

	// 1. Update Velocity & position, timeStep =  h/2
	externalForcesCalculations(h / 2);
	for (int i = 0; i < getNumberOfSprings(); i++) {
		MassPoint p1 = points[springs[i].p1];
		MassPoint p2 = points[springs[i].p2];
		float il = springs[i].initial_length;
		float l = sqrt(p1.position.squaredDistanceTo(p2.position));
		float force = m_fStiffness * (l - il) / l;
		if (!p1.isFixed) {
			p1.Velocity = p1.Velocity - 0.5 * h * force*(p1.position - p2.position) / m_fMass;
			points[springs[i].p1] = p1;
		}
		if (!p2.isFixed) {
			p2.Velocity = p2.Velocity - 0.5 * h * force*(p2.position - p1.position) / m_fMass;
			points[springs[i].p2] = p2;
		}

	}

	for (int i = 0; i < getNumberOfMassPoints(); i++) {
		if (!points[i].isFixed)
			points[i].position = points[i].position + 0.5 * h * points[i].Velocity;
	}

	// 2. Compute final velocity using intermediate position
	externalForcesCalculations(h);
	for (int i = 0; i < getNumberOfSprings(); i++) {
		MassPoint p1 = points[springs[i].p1];
		MassPoint p2 = points[springs[i].p2];
		float il = springs[i].initial_length;
		float l = sqrt(p1.position.squaredDistanceTo(p2.position));
		float force = m_fStiffness * (l - il) / l;
		if (!p1.isFixed) {
			p1.Velocity = p1.Velocity - h * force*(p1.position - p2.position) / m_fMass;
			points[springs[i].p1] = p1;
		}
		if (!p2.isFixed) {
			p2.Velocity = p2.Velocity - h * force*(p2.position - p1.position) / m_fMass;
			points[springs[i].p2] = p2;
		}
	}

	// 3. Update final position
	for (int i = 0; i < getNumberOfMassPoints(); i++) {
		if (!points[i].isFixed)
			points[i].position = step0positions[i] + h * points[i].Velocity;
	}
}

void MassSpringSystemSimulator::leapfrogIntegrator(float h){
	if (LeapfrogFirstStep) { // Compute v(t0 + h/2) with x(t0)
		for (int i = 0; i < getNumberOfSprings(); i++) {
			MassPoint p1 = points[springs[i].p1];
			MassPoint p2 = points[springs[i].p2];
			float il = springs[i].initial_length;
			float l = sqrt(p1.position.squaredDistanceTo(p2.position));
			float force = m_fStiffness * (l - il) / l;
			if (!p1.isFixed) {
				p1.Velocity = p1.Velocity - 0.5 * h * force*(p1.position - p2.position) / m_fMass;
				points[springs[i].p1] = p1;
			}
			if (!p2.isFixed) {
				p2.Velocity = p2.Velocity - 0.5 * h * force*(p2.position - p1.position) / m_fMass;
				points[springs[i].p2] = p2;
			}
		}
		LeapfrogFirstStep = false;
	} else { // We have v(t+h/2) and x(t). First -> x(t+h)
		for (int i = 0; i < getNumberOfMassPoints(); i++) {
			if (!points[i].isFixed)
				points[i].position = points[i].position + h * points[i].Velocity;
		}

		// Prepare next step : compute v(t + 3h/2)
		for (int i = 0; i < getNumberOfSprings(); i++) {
			MassPoint p1 = points[springs[i].p1];
			MassPoint p2 = points[springs[i].p2];
			float il = springs[i].initial_length;
			float l = sqrt(p1.position.squaredDistanceTo(p2.position));
			float force = m_fStiffness * (l - il) / l;
			if (!p1.isFixed) {
				p1.Velocity = p1.Velocity - h * force*(p1.position - p2.position) / m_fMass;
				points[springs[i].p1] = p1;
			}
			if (!p2.isFixed) {
				p2.Velocity = p2.Velocity - h * force*(p2.position - p1.position) / m_fMass;
				points[springs[i].p2] = p2;
			}
		}
	}
}