// TinkerToy.cpp : Defines the entry point for the console application.
//

#include "Particle.h"
#include "SpringForce.h"
#include "SpringAngleForce.h"
#include "GravityForce.h"
#include "RodConstraint.h"
#include "CircularWireConstraint.h"
#include "LineConstraint.h"
#include "AngularConstraint.h"
#include "ViscousDragForce.h"
#include "imageio.h"

#include <vector>
#include <stdlib.h>
#include <stdio.h>
#include <GL/glut.h>
#include "ParticleSystem.h"
#include "ModelReader.h"

/* external definitions (from solver) */
extern void ExplicitEulerStep(ParticleSystem& particleSystem, float dt);
void ImplicitEulerStep(ParticleSystem& particleSystem, float dt);
extern void MidPointStep(ParticleSystem& particleSystem, float dt);
extern void RungeKutta4Step(ParticleSystem& particleSystem, float dt);

void testParticles();
void clothPoints();
void clothPointLine();
void CreatePuppet();
void clothLineLine();
void CreatePuppetBoxed();

/* global variables */

static int N;
static float dt, d;
static int dsim;
static int dump_frames;
static int frame_number;

static int win_id;
static int win_x, win_y;
static int mouse_down[3];
static int mouse_release[3];
static int mouse_shiftclick[3];
static int omx, omy, mx, my;
static int hmx, hmy;
bool mouse0IsPressed = false;

ParticleSystem particleSystem;
Particle* mouseParticle = nullptr;
int mouseParticleID = -1;
int mouseSpringID = -1;

static void clear_data ( void )
{
	auto& particles = particleSystem.GetParticles();
	for (auto p = particles.begin(); p != particles.end(); p++)
		(*p)->Reset();
}

static void init_system(void)
{
	//testParticles();
	//clothPoints();
	//clothPointLine();
	//clothLineLine();
	//CreatePuppet();
	//clothLineLine();
	//clothLineLine();
	//CreatePuppet();
	CreatePuppetBoxed();
}

void CreatePuppet() {
	ModelReader mr(300.f, 0.85f, true, true, 0.85f);
	mr.ReadModel("puppet.txt", particleSystem);
	particleSystem.AddConstraint(new LineConstraint(particleSystem.GetParticles()[0], Vec2f(-1.0f, 0.3f), Vec2f(1.f, 0.f)));
}

void CreatePuppetBoxed() {
	ModelReader mr(200.f, 0.85f, true, true, 0.85f);
	mr.ReadModel("puppet.txt", particleSystem);
	auto& particles = particleSystem.GetParticles();
	particleSystem.AddConstraint(new LineConstraint(particles[0], Vec2f(0.0,0.4), Vec2f(1.0,0.0)));
	particleSystem.AddCollidableLineSegment(new CollidableLineSegment(Vec2f(-1.0,-1.0),Vec2f(-1.0,1.0),0.8,0.011));
	particleSystem.AddCollidableLineSegment(new CollidableLineSegment(Vec2f(1.0,-1.0),Vec2f(1.0,1.0),0.8,0.011));
}


void testParticles(void){
	const double dist = 0.2;
	const Vec2f center(0.0, 0.0);
	const Vec2f offset(dist, 0.0);
	const Vec2f aoffset(dist, dist);

	// Create three particles, attach them to each other, then add a
	// circular wire constraint to the first.
	particleSystem.AddParticle(new Particle(center + offset));
	particleSystem.AddParticle(new Particle(center + offset + offset));
	particleSystem.AddParticle(new Particle(center + offset + offset + offset));
	particleSystem.AddParticle(new Particle(center + aoffset));

	// Add gravity to all particles
	auto& particles = particleSystem.GetParticles();
	for (auto p = particles.begin(); p != particles.end(); p++)
		particleSystem.AddForce(new GravityForce(*p));

	// Add viscous drag to all particles
	const float drag = 0.1f; // Viscous drag (friction)
	for (auto p = particles.begin(); p != particles.end(); p++)
		particleSystem.AddForce(new ViscousDragForce(*p, drag));

	particleSystem.AddForce(new SpringForce(particles[1], particles[2], dist, 5.0, 1.0));
	particleSystem.AddConstraint(new RodConstraint(particles[0], particles[1], dist));
	particleSystem.AddConstraint(new CircularWireConstraint(particles[0], center, dist));
	particleSystem.AddForce(new SpringAngleForce(particles[3], particles[0], particles[1], 0.5*M_PI, 5.0,1.0));
	particleSystem.AddConstraint(new RodConstraint(particles[0], particles[3], dist));

}

void clothPoints(void)
{
	const double dist = 0.1;
	const Vec2f center(0.0, 0.8);
	const Vec2f offsetx(dist, 0.0);
	const Vec2f offsety(0.0, dist);
	const int particlesx = 6;
	const int particlesy = 6;
	const bool BendingSpring = true;
	const bool TorsionSpring = true;
	const float springStiffness = 1000.f;
	const float bendingStiffness = 30.f;
	const float torsionStiffness = 10.f;
	const float damping = 0.95f;
	const int fixedPoints = 1;
	
	float xdir;
	float ydir;

	for (xdir = 0; xdir < particlesx;xdir++){
		for (ydir = 0; ydir < particlesy;ydir++){
			particleSystem.AddParticle(new Particle(center-offsetx*((float)particlesx-1)/2.f + xdir*offsetx - offsety*ydir));
		}
	}

	auto& particles = particleSystem.GetParticles();
	for (xdir = 1; xdir < particlesx;xdir++){
		particleSystem.AddForce(new SpringForce(particles[(xdir-1)*particlesy], particles[xdir*particlesy], dist, springStiffness, damping));
	}
	for (ydir = 1; ydir < particlesy;ydir++){
		particleSystem.AddForce(new SpringForce(particles[ydir-1], particles[ydir], dist, springStiffness, 1.0));
	}
	for (xdir = 1; xdir < particlesx;xdir++){
		for (ydir = 1; ydir < particlesy;ydir++){
			particleSystem.AddForce(new SpringForce(particles[(xdir-1)*particlesy+ydir], particles[xdir*particlesy+ydir], dist, springStiffness, damping));
			particleSystem.AddForce(new SpringForce(particles[xdir*particlesy+ydir-1], particles[xdir*particlesy+ydir], dist, springStiffness, damping));
			if (BendingSpring){
				particleSystem.AddForce(new SpringForce(particles[(xdir-1)*particlesy+ydir-1], particles[xdir*particlesy+ydir], dist, bendingStiffness, damping));
				particleSystem.AddForce(new SpringForce(particles[xdir*particlesy+ydir-1], particles[(xdir-1)*particlesy+ydir], dist, bendingStiffness, damping));
			}
		}
	}
	if (TorsionSpring){
		for (xdir = 2; xdir < particlesx;xdir++){
			for (ydir = 0; ydir < particlesy;ydir++){
				particleSystem.AddForce(new SpringForce(particles[(xdir-2)*particlesy+ydir], particles[xdir*particlesy+ydir], dist, torsionStiffness, damping));
			}
		}
		for (ydir = 2; ydir < particlesy;ydir++){
			for (xdir = 0; xdir < particlesx;xdir++){
				particleSystem.AddForce(new SpringForce(particles[xdir*particlesy+ydir-2], particles[xdir*particlesy+ydir], dist, torsionStiffness, damping));
			}
		}
	}


	// Add gravity to all particles
	for (auto p = particles.begin(); p != particles.end(); p++)
		particleSystem.AddForce(new GravityForce(*p));

	// Add viscous drag to all particles
	const float drag = 0.1f; // Viscous drag (friction)
	for (auto p = particles.begin(); p != particles.end(); p++)
		particleSystem.AddForce(new ViscousDragForce(*p, drag));

	particleSystem.AddConstraint(new CircularWireConstraint(particles[0], center-offsetx*((float)particlesx-1)/2.f+offsety/2.f, dist/2));
	particleSystem.AddConstraint(new CircularWireConstraint(particles[particlesy*(particlesx-1)], center+offsetx*((float)particlesx-1)/2.f+offsety/2.f, dist/2));
}

void clothPointLine(void)
{
	const double dist = 0.1;
	const Vec2f center(0.0, 0.8);
	const Vec2f offsetx(dist, 0.0);
	const Vec2f offsety(0.0, dist);
	const int particlesx = 9;
	const int particlesy = 9;
	const bool BendingSpring = true;
	const bool TorsionSpring = false;
	const float springStiffness = 1000.f;
	const float bendingStiffness = 80.f;
	const float torsionStiffness = 10.f;
	const float damping = 0.85f;
	const int fixedPoints = 1;
	
	float xdir;
	float ydir;

	for (xdir = 0; xdir < particlesx;xdir++){
		for (ydir = 0; ydir < particlesy;ydir++){
			particleSystem.AddParticle(new Particle(center-offsetx*((float)particlesx-1)/2.f + xdir*offsetx - offsety*ydir));
		}
	}

	auto& particles = particleSystem.GetParticles();
	for (xdir = 1; xdir < particlesx;xdir++){
		particleSystem.AddForce(new SpringForce(particles[(xdir-1)*particlesy], particles[xdir*particlesy], dist, springStiffness, damping));
	}
	for (ydir = 1; ydir < particlesy;ydir++){
		particleSystem.AddForce(new SpringForce(particles[ydir-1], particles[ydir], dist, springStiffness, 1.0));
	}
	for (xdir = 1; xdir < particlesx;xdir++){
		for (ydir = 1; ydir < particlesy;ydir++){
			particleSystem.AddForce(new SpringForce(particles[(xdir-1)*particlesy+ydir], particles[xdir*particlesy+ydir], dist, springStiffness, damping));
			particleSystem.AddForce(new SpringForce(particles[xdir*particlesy+ydir-1], particles[xdir*particlesy+ydir], dist, springStiffness, damping));
			if (BendingSpring){
				particleSystem.AddForce(new SpringForce(particles[(xdir-1)*particlesy+ydir-1], particles[xdir*particlesy+ydir], dist, bendingStiffness, damping));
				particleSystem.AddForce(new SpringForce(particles[xdir*particlesy+ydir-1], particles[(xdir-1)*particlesy+ydir], dist, bendingStiffness, damping));
			}
		}
	}
	if (TorsionSpring){
		for (xdir = 2; xdir < particlesx;xdir++){
			for (ydir = 0; ydir < particlesy;ydir++){
				particleSystem.AddForce(new SpringForce(particles[(xdir-2)*particlesy+ydir], particles[xdir*particlesy+ydir], dist, torsionStiffness, damping));
			}
		}
		for (ydir = 2; ydir < particlesy;ydir++){
			for (xdir = 0; xdir < particlesx;xdir++){
				particleSystem.AddForce(new SpringForce(particles[xdir*particlesy+ydir-2], particles[xdir*particlesy+ydir], dist, torsionStiffness, damping));
			}
		}
	}


	// Add gravity to all particles
	for (auto p = particles.begin(); p != particles.end(); p++)
		particleSystem.AddForce(new GravityForce(*p));

	// Add viscous drag to all particles
	const float drag = 0.1f; // Viscous drag (friction)
	for (auto p = particles.begin(); p != particles.end(); p++)
		particleSystem.AddForce(new ViscousDragForce(*p, drag));

	particleSystem.AddConstraint(new CircularWireConstraint(particles[0], center-offsetx*((float)particlesx-1)/2.f+offsety/2.f, dist/2));
	particleSystem.AddConstraint(new LineConstraint(particles[particlesy*(particlesx-1)], center+offsetx*((float)particlesx-1)/2.f, offsetx));
}


void clothLineLine(void)
{
	const double dist = 0.1;
	const Vec2f center(0.0, 0.8);
	const Vec2f offsetx(dist, 0.0);
	const Vec2f offsety(0.0, dist);
	const int particlesx = 5;
	const int particlesy = 6;
	const bool BendingSpring = true;
	const bool TorsionSpring = false;
	const float springStiffness = 1000.f;
	const float bendingStiffness = 80.f;
	const float torsionStiffness = 10.f;
	const float damping = 0.85f;
	const int fixedPoints = 1;
	
	float xdir;
	float ydir;

	for (xdir = 0; xdir < particlesx;xdir++){
		for (ydir = 0; ydir < particlesy;ydir++){
			particleSystem.AddParticle(new Particle(center-offsetx*((float)particlesx-1)/2.f + xdir*offsetx - offsety*ydir));
		}
	}

	auto& particles = particleSystem.GetParticles();
	for (xdir = 1; xdir < particlesx;xdir++){
		particleSystem.AddForce(new SpringForce(particles[(xdir-1)*particlesy], particles[xdir*particlesy], dist, springStiffness, damping));
	}
	for (ydir = 1; ydir < particlesy;ydir++){
		particleSystem.AddForce(new SpringForce(particles[ydir-1], particles[ydir], dist, springStiffness, 1.0));
	}
	for (xdir = 1; xdir < particlesx;xdir++){
		for (ydir = 1; ydir < particlesy;ydir++){
			particleSystem.AddForce(new SpringForce(particles[(xdir-1)*particlesy+ydir], particles[xdir*particlesy+ydir], dist, springStiffness, damping));
			particleSystem.AddForce(new SpringForce(particles[xdir*particlesy+ydir-1], particles[xdir*particlesy+ydir], dist, springStiffness, damping));
			if (BendingSpring){
				particleSystem.AddForce(new SpringForce(particles[(xdir-1)*particlesy+ydir-1], particles[xdir*particlesy+ydir], dist, bendingStiffness, damping));
				particleSystem.AddForce(new SpringForce(particles[xdir*particlesy+ydir-1], particles[(xdir-1)*particlesy+ydir], dist, bendingStiffness, damping));
			}
		}
	}
	if (TorsionSpring){
		for (xdir = 2; xdir < particlesx;xdir++){
			for (ydir = 0; ydir < particlesy;ydir++){
				particleSystem.AddForce(new SpringForce(particles[(xdir-2)*particlesy+ydir], particles[xdir*particlesy+ydir], dist, torsionStiffness, damping));
			}
		}
		for (ydir = 2; ydir < particlesy;ydir++){
			for (xdir = 0; xdir < particlesx;xdir++){
				particleSystem.AddForce(new SpringForce(particles[xdir*particlesy+ydir-2], particles[xdir*particlesy+ydir], dist, torsionStiffness, damping));
			}
		}
	}


	// Add gravity to all particles
	for (auto p = particles.begin(); p != particles.end(); p++)
		particleSystem.AddForce(new GravityForce(*p));

	// Add viscous drag to all particles
	const float drag = 0.1f; // Viscous drag (friction)
	for (auto p = particles.begin(); p != particles.end(); p++)
		particleSystem.AddForce(new ViscousDragForce(*p, drag));

	particleSystem.AddConstraint(new LineConstraint(particles[0], center-offsetx*((float)particlesx-1)/2.f, -offsetx));
	particleSystem.AddConstraint(new LineConstraint(particles[particlesy*(particlesx-1)], center+offsetx*((float)particlesx-1)/2.f, offsetx));
	particleSystem.AddCollidableLineSegment(new CollidableLineSegment(Vec2f(0.2,-0.8),Vec2f(0.2,0.35),0,0.009));
}

/*
----------------------------------------------------------------------
OpenGL specific drawing routines
----------------------------------------------------------------------
*/
static void pre_display ( void )
{
	glViewport ( 0, 0, win_x, win_y );
	glMatrixMode ( GL_PROJECTION );
	glLoadIdentity ();
	gluOrtho2D ( -1.0, 1.0, -1.0, 1.0 );
	glClearColor ( 0.0f, 0.0f, 0.0f, 1.0f );
	glClear ( GL_COLOR_BUFFER_BIT );
}

static void post_display ( void )
{
	// Write frames if necessary.
	if (dump_frames) {
		const int FRAME_INTERVAL = 4;
		if ((frame_number % FRAME_INTERVAL) == 0) {
			const unsigned int w = glutGet(GLUT_WINDOW_WIDTH);
			const unsigned int h = glutGet(GLUT_WINDOW_HEIGHT);
			unsigned char * buffer = (unsigned char *) malloc(w * h * 4 * sizeof(unsigned char));
			if (!buffer)
				exit(-1);
			// glRasterPos2i(0, 0);
			glReadPixels(0, 0, w, h, GL_RGBA, GL_UNSIGNED_BYTE, buffer);
			static char filename[80];
			sprintf_s(filename, "snapshots/img%.5i.png", frame_number / FRAME_INTERVAL);
			printf("Dumped %s.\n", filename);
			saveImageRGBA(filename, buffer, w, h);
			
			free(buffer);
		}
	}
	frame_number++;
	
	glutSwapBuffers ();
}

static void draw_particles ( void )
{
	auto& particles = particleSystem.GetParticles();
	for (auto p = particles.begin(); p != particles.end(); p++)
		(*p)->Draw();
}

static void draw_forces ( void )
{
	auto& forces = particleSystem.GetForces();
	for (auto f = forces.begin(); f != forces.end(); f++)
		(*f)->Draw();
}

static void draw_constraints ( void )
{
	auto& constraints = particleSystem.GetConstraints();
	for (auto c = constraints.begin(); c != constraints.end(); c++)
		(*c)->Draw();
}

static void draw_collidableLineSegments ( void )
{
	auto& collidableLineSegment = particleSystem.GetCollidableLineSegment();
	for (auto c = collidableLineSegment.begin(); c != collidableLineSegment.end(); c++)
		(*c)->Draw();
}

/*
----------------------------------------------------------------------
relates mouse movements to tinker toy construction
----------------------------------------------------------------------
*/
static void get_from_UI ()
{
	if (!mouse_down[0] && !mouse_down[2] && !mouse_release[0] 
		&& !mouse_shiftclick[0] && !mouse_shiftclick[2]) return;

	// Map mouse position to [-1, 1]
	float x = ((float)mx / (float)win_x) * 2.f - 1.f;
	float y = ((float)my / (float)win_y) * 2.f - 1.f;
	y = -y;

	// If mouse position is outside of viewport
	if (x < -1.f || x > 1.f || y < -1.f || y > 1.f) return;

	// If left mouse button is clicked
	if (mouse_down[0] && !mouse0IsPressed) {
		// Find closest other particle
		Particle* cp = particleSystem.GetClosestParticle(Vec2f(x, y), mouseParticleID);
		const float minSD = 0.2f; // Minimum squared distance
		// If there is a closest particle, closer than minimum distance
		if (cp && norm2(cp->m_Position - Vec2f(x, y)) < minSD) {
			// Add a particle for the mouse
			mouse0IsPressed = true;
			mouseParticle = new Particle(Vec2f(x, y));
			mouseParticle->m_Mass = 50.f;
			mouseParticleID = particleSystem.AddParticle(mouseParticle);

			// Add a spring between mouse particle and closest particle
			mouseSpringID = particleSystem.AddForce(new SpringForce(mouseParticle, cp, 0.1, 100.0, 1.0));
		}
	}
	
	// If left mouse button is being held
	if (mouse_down[0] && mouse0IsPressed) {
		// Update position of mouse particle
		mouseParticle->m_Position = Vec2f(x, y);
	}

	if ( mouse_down[2] ) {
	}

	// If left mouse button is released
	if( mouse_release[0] && mouse0IsPressed) {
		mouse0IsPressed = false;
		// Remove mouse particle and associated spring force
		if (mouseParticleID != -1) {
			particleSystem.RemoveParticle(mouseParticleID);
			mouseParticleID = -1;
			mouseParticle = nullptr;
		}
		if (mouseSpringID != -1) {
			particleSystem.RemoveForce(mouseSpringID);
			mouseSpringID = -1;
		}
	}

	omx = mx;
	omy = my;
}

static void remap_GUI()
{
	/*
	int ii, size = pVector.size();
	for(ii=0; ii<size; ii++)
	{
		pVector[ii]->m_Position[0] = pVector[ii]->m_ConstructPos[0];
		pVector[ii]->m_Position[1] = pVector[ii]->m_ConstructPos[1];
	}
	*/
}

/*
----------------------------------------------------------------------
GLUT callback routines
----------------------------------------------------------------------
*/
static void key_func ( unsigned char key, int x, int y )
{
	switch ( key )
	{
	case 'c':
	case 'C':
		clear_data ();
		break;

	case 'd':
	case 'D':
		dump_frames = !dump_frames;
		break;

	case 'q':
	case 'Q':
		particleSystem.Clear();
		exit ( 0 );
		break;

	case ' ':
		dsim = !dsim;
		break;
	case 27: // Escape key
		particleSystem.Clear();
		exit(0);
		break;
	}
}

static void mouse_func ( int button, int state, int x, int y )
{
	omx = mx = x;
	omx = my = y;

	if(!mouse_down[0]){hmx=x; hmy=y;}
	mouse_release[button] = state == GLUT_UP;
	if(mouse_down[button]) mouse_shiftclick[button] = glutGetModifiers()==GLUT_ACTIVE_SHIFT;
	mouse_down[button] = state == GLUT_DOWN;
}

static void motion_func ( int x, int y )
{
	mx = x;
	my = y;
}

static void reshape_func ( int width, int height )
{
	glutSetWindow ( win_id );
	glutReshapeWindow ( width, height );

	win_x = width;
	win_y = height;
}

static void idle_func ( void )
{
	if (dsim) {
		get_from_UI();
		ImplicitEulerStep(particleSystem, dt);
		remap_GUI();
	}

	glutSetWindow(win_id);
	glutPostRedisplay();
}

static void display_func ( void )
{
	pre_display ();

	draw_forces();
	draw_constraints();
	draw_particles();
	draw_collidableLineSegments();

	post_display();
}


/*
----------------------------------------------------------------------
open_glut_window --- open a glut compatible window and set callbacks
----------------------------------------------------------------------
*/
static void open_glut_window ( void )
{
	glutInitDisplayMode ( GLUT_RGBA | GLUT_DOUBLE );

	glutInitWindowPosition ( 0, 0 );
	glutInitWindowSize ( win_x, win_y );
	win_id = glutCreateWindow ( "Tinkertoys!" );

	glClearColor ( 0.0f, 0.0f, 0.0f, 1.0f );
	glClear ( GL_COLOR_BUFFER_BIT );
	glutSwapBuffers ();
	glClear ( GL_COLOR_BUFFER_BIT );
	glutSwapBuffers ();

	glEnable(GL_LINE_SMOOTH);
	glEnable(GL_POLYGON_SMOOTH);

	pre_display ();

	glutKeyboardFunc ( key_func );
	glutMouseFunc ( mouse_func );
	glutMotionFunc ( motion_func );
	glutReshapeFunc ( reshape_func );
	glutIdleFunc ( idle_func );
	glutDisplayFunc ( display_func );
}


/*
----------------------------------------------------------------------
main --- main routine
----------------------------------------------------------------------
*/
int main ( int argc, char ** argv )
{
	glutInit ( &argc, argv );

	if ( argc == 1 ) {
		N = 64;
		dt = 0.001f;
		d = 5.f;
		fprintf ( stderr, "Using defaults : N=%d dt=%g d=%g\n",
			N, dt, d );
	} else {
		N = atoi(argv[1]);
		dt = atof(argv[2]);
		d = atof(argv[3]);
	}

	printf ( "\n\nHow to use this application:\n\n" );
	printf ( "\t Toggle construction/simulation display with the spacebar key\n" );
	printf ( "\t Dump frames by pressing the 'd' key\n" );
	printf ( "\t Quit by pressing the 'q' key\n" );

	dsim = 0;
	dump_frames = 0;
	frame_number = 0;
	
	init_system();
	
	win_x = 512;
	win_y = 512;
	open_glut_window ();

	glutMainLoop ();

	exit ( 0 );
}

