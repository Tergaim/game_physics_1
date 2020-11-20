#ifndef MASSSPRINGSYSTEMSIMULATOR_h
#define MASSSPRINGSYSTEMSIMULATOR_h
#include "Simulator.h"

// Do Not Change
#define EULER 0
#define LEAPFROG 1
#define MIDPOINT 2
// Do Not Change

struct MassPoint {
	Vec3 position;
	Vec3 Velocity;
	bool isFixed;

	MassPoint(Vec3 position, Vec3 Velocity, bool isFixed): position(position), Velocity(Velocity), isFixed(isFixed){}
};

struct Spring {
	int p1, p2;
	float initial_length;

	Spring(int p1, int p2, float initial_length): p1(p1), p2(p2), initial_length(initial_length) {}
};

class MassSpringSystemSimulator:public Simulator{
public:
	// Construtors
	MassSpringSystemSimulator();
	
	// UI Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void collisionCheck();
	void onClick(int x, int y);
	void onMouse(int x, int y);

	// Specific Functions
	void setMass(float mass);
	void setStiffness(float stiffness);
	void setDampingFactor(float damping);
	int addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed);
	void addSpring(int masspoint1, int masspoint2, float initialLength);
	int getNumberOfMassPoints();
	int getNumberOfSprings();
	Vec3 getPositionOfMassPoint(int index);
	Vec3 getVelocityOfMassPoint(int index);
	void applyExternalForce(Vec3 force);
	
	// Do Not Change
	void setIntegrator(int integrator) {
		m_iIntegrator = integrator;
	}

private:
	// Test Cases
	int previous_test_case = 0;
	int test_case = 0;
	int previous_integrator = 0;
	int integrator = 0;
	bool LeapfrogFirstStep = true;
	void eulerIntegrator(float h);
	void midpointIntegrator(float h);
	void leapfrogIntegrator(float h);


	// Data Attributes
	float m_fMass;
	float m_fStiffness;
	float m_fDamping;
	int m_iIntegrator;
	float gravity = 0;
	float wind = 0;

	vector<MassPoint> points;
	vector<Spring> springs;

	// UI Attributes
	Vec3 m_externalForce;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
	bool notOnCallback = true;
	TwEnumVal tests[3] = { {0, "One-step"}, {1, "Simple Spring"}, {2, "Complex scene"} };
	TwEnumVal integrators[3] = { {0, "Semi-implicit Euler"}, {1, "LeapFrog"}, {2, "Midpoint"} };
};
#endif