/**********************************************************************************************
*
*   Physac v1.1 - 2D Physics library for videogames
*
*   DESCRIPTION:
*
*   Physac is a small 2D physics library written in pure C. The engine uses a fixed time-step thread loop
*   to simluate physics. A physics step contains the following phases: get collision information,
*   apply dynamics, collision solving and position correction. It uses a very simple struct for physic
*   bodies with a position vector to be used in any 3D rendering API.
*
*   CONFIGURATION:
*
*   #define PHYSAC_IMPLEMENTATION
*       Generates the implementation of the library into the included file.
*       If not defined, the library is in header only mode and can be included in other headers
*       or source files without problems. But only ONE file should hold the implementation.
*
*   #define PHYSAC_STATIC (defined by default)
*       The generated implementation will stay private inside implementation file and all
*       internal symbols and functions will only be visible inside that file.
*
*   #define PHYSAC_NO_THREADS
*       The generated implementation won't include pthread library and user must create a secondary thread to call PhysicsThread().
*       It is so important that the thread where PhysicsThread() is called must not have v-sync or any other CPU limitation.
*
*   #define PHYSAC_STANDALONE
*       Avoid raylib.h header inclusion in this file. Data types defined on raylib are defined
*       internally in the library and input management and drawing functions must be provided by
*       the user (check library implementation for further details).
*
*   #define PHYSAC_DEBUG
*       Traces log messages when creating and destroying physics bodies and detects errors in physics
*       calculations and reference exceptions; it is useful for debug purposes
*
*   #define PHYSAC_MALLOC()
*   #define PHYSAC_FREE()
*       You can define your own malloc/free implementation replacing stdlib.h malloc()/free() functions.
*       Otherwise it will include stdlib.h and use the C standard library malloc()/free() function.
*
*
*   NOTE 1: Physac requires multi-threading, when InitPhysics() a second thread is created to manage physics calculations.
*   NOTE 2: Physac requires static C library linkage to avoid dependency on MinGW DLL (-static -lpthread)
*
*   Use the following code to compile:
*   gcc -o $(NAME_PART).exe $(FILE_NAME) -s -static -lraylib -lpthread -lopengl32 -lgdi32 -lwinmm -std=c99
*
*   VERY THANKS TO:
*       - raysan5: helped with library design
*       - ficoos: added support for Linux
*       - R8D8: added support for Linux
*       - jubalh: fixed implementation of time calculations
*       - a3f: fixed implementation of time calculations
*       - define-private-public: added support for OSX
*       - pamarcos: fixed implementation of physics steps
*       - noshbar: fixed some memory leaks
*
*
*   LICENSE: zlib/libpng
*
*   Copyright (c) 2016-2020 Victor Fisac (github: @victorfisac)
*
*   This software is provided "as-is", without any express or implied warranty. In no event
*   will the authors be held liable for any damages arising from the use of this software.
*
*   Permission is granted to anyone to use this software for any purpose, including commercial
*   applications, and to alter it and redistribute it freely, subject to the following restrictions:
*
*     1. The origin of this software must not be misrepresented; you must not claim that you
*     wrote the original software. If you use this software in a product, an acknowledgment
*     in the product documentation would be appreciated but is not required.
*
*     2. Altered source versions must be plainly marked as such, and must not be misrepresented
*     as being the original software.
*
*     3. This notice may not be removed or altered from any source distribution.
*
**********************************************************************************************/

#if !defined(PHYSAC_H)
#define PHYSAC_H

// #define PHYSAC_STATIC
// #define  PHYSAC_NO_THREADS
// #define  PHYSAC_STANDALONE
// #define  PHYSAC_DEBUG

#if defined(PHYSAC_STATIC)
    #define PHYSACDEF static            // Functions just visible to module including this file
#else
    #if defined(__cplusplus)
        #define PHYSACDEF extern "C"    // Functions visible from other files (no name mangling of functions in C++)
    #else
        #define PHYSACDEF extern        // Functions visible from other files
    #endif
#endif

//----------------------------------------------------------------------------------
// Defines and Macros
//----------------------------------------------------------------------------------
#define     PHYSAC_MAX_BODIES               64
#define     PHYSAC_MAX_MANIFOLDS            4096
#define     PHYSAC_MAX_VERTICES             24
#define     PHYSAC_CIRCLE_VERTICES          24

#define     PHYSAC_COLLISION_ITERATIONS     100
#define     PHYSAC_PENETRATION_ALLOWANCE    0.05f
#define     PHYSAC_PENETRATION_CORRECTION   0.4f

#define     PHYSAC_PI                       3.14159265358979323846
#define     PHYSAC_DEG2RAD                  (PHYSAC_PI/180.0f)

#define     PHYSAC_MALLOC(size)             malloc(size)
#define     PHYSAC_FREE(ptr)                free(ptr)

//----------------------------------------------------------------------------------
// Types and Structures Definition
// NOTE: Below types are required for PHYSAC_STANDALONE usage
//----------------------------------------------------------------------------------
#if defined(PHYSAC_STANDALONE)
    // Vector2 type
    typedef struct Vector2 {
        float x;
        float y;
    } Vector2;

    // Boolean type
    #if !defined(_STDBOOL_H)
        typedef enum { false, true } bool;
        #define _STDBOOL_H
    #endif
#endif

typedef enum PhysicsShapeType { PHYSICS_CIRCLE, PHYSICS_POLYGON } PhysicsShapeType;

// Previously defined to be used in PhysicsShape struct as circular dependencies
typedef struct PhysicsBodyData *PhysicsBody;

// Mat2 type (used for polygon shape rotation matrix)
typedef struct Mat2 {
    float m00;
    float m01;
    float m10;
    float m11;
} Mat2;

typedef struct PolygonData {
    unsigned int vertexCount;                   // Current used vertex and normals count
    Vector2 positions[PHYSAC_MAX_VERTICES];     // Polygon vertex positions vectors
    Vector2 normals[PHYSAC_MAX_VERTICES];       // Polygon vertex normals vectors
} PolygonData;

typedef struct PhysicsShape {
    PhysicsShapeType type;                      // Physics shape type (circle or polygon)
    PhysicsBody body;                           // Shape physics body reference
    float radius;                               // Circle shape radius (used for circle shapes)
    Mat2 transform;                             // Vertices transform matrix 2x2
    PolygonData vertexData;                     // Polygon shape vertices position and normals data (just used for polygon shapes)
} PhysicsShape;

typedef struct PhysicsBodyData {
    unsigned int id;                            // Reference unique identifier
    bool enabled;                               // Enabled dynamics state (collisions are calculated anyway)
    Vector2 position;                           // Physics body shape pivot
    Vector2 velocity;                           // Current linear velocity applied to position
    Vector2 force;                              // Current linear force (reset to 0 every step)
    float angularVelocity;                      // Current angular velocity applied to orient
    float torque;                               // Current angular force (reset to 0 every step)
    float orient;                               // Rotation in radians
    float inertia;                              // Moment of inertia
    float inverseInertia;                       // Inverse value of inertia
    float mass;                                 // Physics body mass
    float inverseMass;                          // Inverse value of mass
    float staticFriction;                       // Friction when the body has not movement (0 to 1)
    float dynamicFriction;                      // Friction when the body has movement (0 to 1)
    float restitution;                          // Restitution coefficient of the body (0 to 1)
    bool useGravity;                            // Apply gravity force to dynamics
    bool isGrounded;                            // Physics grounded on other body state
    bool freezeOrient;                          // Physics rotation constraint
    PhysicsShape shape;                         // Physics body shape information (type, radius, vertices, normals)
} PhysicsBodyData;

typedef struct PhysicsManifoldData {
    unsigned int id;                            // Reference unique identifier
    PhysicsBody bodyA;                          // Manifold first physics body reference
    PhysicsBody bodyB;                          // Manifold second physics body reference
    float penetration;                          // Depth of penetration from collision
    Vector2 normal;                             // Normal direction vector from 'a' to 'b'
    Vector2 contacts[2];                        // Points of contact during collision
    unsigned int contactsCount;                 // Current collision number of contacts
    float restitution;                          // Mixed restitution during collision
    float dynamicFriction;                      // Mixed dynamic friction during collision
    float staticFriction;                       // Mixed static friction during collision
} PhysicsManifoldData, *PhysicsManifold;

#if defined(__cplusplus)
extern "C" {                                    // Prevents name mangling of functions
#endif

//----------------------------------------------------------------------------------
// Module Functions Declaration
//----------------------------------------------------------------------------------
PHYSACDEF void InitPhysics(void);                                                                           // Initializes physics values, pointers and creates physics loop thread
PHYSACDEF void RunPhysicsStep(void);                                                                        // Run physics step, to be used if PHYSICS_NO_THREADS is set in your main loop
PHYSACDEF void SetPhysicsTimeStep(double delta);                                                            // Sets physics fixed time step in milliseconds. 1.666666 by default
PHYSACDEF bool IsPhysicsEnabled(void);                                                                      // Returns true if physics thread is currently enabled
PHYSACDEF void SetPhysicsGravity(float x, float y);                                                         // Sets physics global gravity force
PHYSACDEF PhysicsBody CreatePhysicsBodyCircle(Vector2 pos, float radius, float density);                    // Creates a new circle physics body with generic parameters
PHYSACDEF PhysicsBody CreatePhysicsBodyRectangle(Vector2 pos, float width, float height, float density);    // Creates a new rectangle physics body with generic parameters
PHYSACDEF PhysicsBody CreatePhysicsBodyPolygon(Vector2 pos, float radius, int sides, float density);        // Creates a new polygon physics body with generic parameters
PHYSACDEF void PhysicsAddForce(PhysicsBody body, Vector2 force);                                            // Adds a force to a physics body
PHYSACDEF void PhysicsAddTorque(PhysicsBody body, float amount);                                            // Adds an angular force to a physics body
PHYSACDEF void PhysicsShatter(PhysicsBody body, Vector2 position, float force);                             // Shatters a polygon shape physics body to little physics bodies with explosion force
PHYSACDEF int GetPhysicsBodiesCount(void);                                                                  // Returns the current amount of created physics bodies
PHYSACDEF PhysicsBody GetPhysicsBody(int index);                                                            // Returns a physics body of the bodies pool at a specific index
PHYSACDEF int GetPhysicsShapeType(int index);                                                               // Returns the physics body shape type (PHYSICS_CIRCLE or PHYSICS_POLYGON)
PHYSACDEF int GetPhysicsShapeVerticesCount(int index);                                                      // Returns the amount of vertices of a physics body shape
PHYSACDEF Vector2 GetPhysicsShapeVertex(PhysicsBody body, int vertex);                                      // Returns transformed position of a body shape (body position + vertex transformed position)
PHYSACDEF void SetPhysicsBodyRotation(PhysicsBody body, float radians);                                     // Sets physics body shape transform based on radians parameter
PHYSACDEF void DestroyPhysicsBody(PhysicsBody body);                                                        // Unitializes and destroy a physics body
PHYSACDEF void ClosePhysics(void);                                                                          // Unitializes physics pointers and closes physics loop thread

#if defined(__cplusplus)
}
#endif

#endif // PHYSAC_H

/***********************************************************************************
*
*   PHYSAC IMPLEMENTATION
*
************************************************************************************/

#if defined(PHYSAC_IMPLEMENTATION)

#if !defined(PHYSAC_NO_THREADS)
    #include <pthread.h>            // Required for: pthread_t, pthread_create()
#endif

#if defined(PHYSAC_DEBUG)
    #include <stdio.h>              // Required for: printf()
#endif

#include <stdlib.h>                 // Required for: malloc(), free(), srand(), rand()
#include <math.h>                   // Required for: cosf(), sinf(), fabs(), sqrtf()
#include <stdint.h>                 // Required for: uint64_t

#if !defined(PHYSAC_STANDALONE)
    #include "raymath.h"            // Required for: Vector2Add(), Vector2Subtract()
#endif

// Time management functionality
#include <time.h>                   // Required for: time(), clock_gettime()
#if defined(_WIN32)
    // Functions required to query time on Windows
    int __stdcall QueryPerformanceCounter(unsigned long long int *lpPerformanceCount);
    int __stdcall QueryPerformanceFrequency(unsigned long long int *lpFrequency);
#elif defined(__linux__)
    #if _POSIX_C_SOURCE < 199309L
        #undef _POSIX_C_SOURCE
        #define _POSIX_C_SOURCE 199309L // Required for CLOCK_MONOTONIC if compiled with c99 without gnu ext.
    #endif
    #include <sys/time.h>           // Required for: timespec
#elif defined(__APPLE__)            // macOS also defines __MACH__
    #include <mach/mach_time.h>     // Required for: mach_absolute_time()
#endif

//----------------------------------------------------------------------------------
// Defines and Macros
//----------------------------------------------------------------------------------
#define     min(a,b)                    (((a)<(b))?(a):(b))
#define     max(a,b)                    (((a)>(b))?(a):(b))
#define     PHYSAC_FLT_MAX              3.402823466e+38f
#define     PHYSAC_EPSILON              0.000001f
#define     PHYSAC_K                    1.0f/3.0f
#define     PHYSAC_VECTOR_ZERO          (Vector2){ 0.0f, 0.0f }

//----------------------------------------------------------------------------------
// Global Variables Definition
//----------------------------------------------------------------------------------
#if !defined(PHYSAC_NO_THREADS)
static pthread_t physicsThreadId;                           // Physics thread id
#endif
static unsigned int usedMemory = 0;                         // Total allocated dynamic memory
static bool physicsThreadEnabled = false;                   // Physics thread enabled state
static double baseTime = 0.0;                               // Offset time for MONOTONIC clock
static double startTime = 0.0;                              // Start time in milliseconds
static double deltaTime = 1.0/60.0/10.0 * 1000;             // Delta time used for physics steps, in milliseconds
static double currentTime = 0.0;                            // Current time in milliseconds
static uint64_t frequency = 0;                              // Hi-res clock frequency

static double accumulator = 0.0;                            // Physics time step delta time accumulator
static unsigned int stepsCount = 0;                         // Total physics steps processed
static Vector2 gravityForce = { 0.0f, 9.81f };              // Physics world gravity force
static PhysicsBody bodies[PHYSAC_MAX_BODIES];               // Physics bodies pointers array
static unsigned int physicsBodiesCount = 0;                 // Physics world current bodies counter
static PhysicsManifold contacts[PHYSAC_MAX_MANIFOLDS];      // Physics bodies pointers array
static unsigned int physicsManifoldsCount = 0;              // Physics world current manifolds counter

//----------------------------------------------------------------------------------
// Module Internal Functions Declaration
//----------------------------------------------------------------------------------
static int FindAvailableBodyIndex();                                                                        // Finds a valid index for a new physics body initialization
static PolygonData CreateRandomPolygon(float radius, int sides);                                            // Creates a random polygon shape with max vertex distance from polygon pivot
static PolygonData CreateRectanglePolygon(Vector2 pos, Vector2 size);                                       // Creates a rectangle polygon shape based on a min and max positions
static void *PhysicsLoop(void *arg);                                                                        // Physics loop thread function
static void PhysicsStep(void);                                                                              // Physics steps calculations (dynamics, collisions and position corrections)
static int FindAvailableManifoldIndex();                                                                    // Finds a valid index for a new manifold initialization
static PhysicsManifold CreatePhysicsManifold(PhysicsBody a, PhysicsBody b);                                 // Creates a new physics manifold to solve collision
static void DestroyPhysicsManifold(PhysicsManifold manifold);                                               // Unitializes and destroys a physics manifold
static void SolvePhysicsManifold(PhysicsManifold manifold);                                                 // Solves a created physics manifold between two physics bodies
static void SolveCircleToCircle(PhysicsManifold manifold);                                                  // Solves collision between two circle shape physics bodies
static void SolveCircleToPolygon(PhysicsManifold manifold);                                                 // Solves collision between a circle to a polygon shape physics bodies
static void SolvePolygonToCircle(PhysicsManifold manifold);                                                 // Solves collision between a polygon to a circle shape physics bodies
static void SolveDifferentShapes(PhysicsManifold manifold, PhysicsBody bodyA, PhysicsBody bodyB);           // Solve collision between two different types of shapes
static void SolvePolygonToPolygon(PhysicsManifold manifold);                                                // Solves collision between two polygons shape physics bodies
static void IntegratePhysicsForces(PhysicsBody body);                                                       // Integrates physics forces into velocity
static void InitializePhysicsManifolds(PhysicsManifold manifold);                                           // Initializes physics manifolds to solve collisions
static void IntegratePhysicsImpulses(PhysicsManifold manifold);                                             // Integrates physics collisions impulses to solve collisions
static void IntegratePhysicsVelocity(PhysicsBody body);                                                     // Integrates physics velocity into position and forces
static void CorrectPhysicsPositions(PhysicsManifold manifold);                                              // Corrects physics bodies positions based on manifolds collision information
static float FindAxisLeastPenetration(int *faceIndex, PhysicsShape shapeA, PhysicsShape shapeB);            // Finds polygon shapes axis least penetration
static void FindIncidentFace(Vector2 *v0, Vector2 *v1, PhysicsShape ref, PhysicsShape inc, int index);      // Finds two polygon shapes incident face
static int Clip(Vector2 normal, float clip, Vector2 *faceA, Vector2 *faceB);                                // Calculates clipping based on a normal and two faces
static bool BiasGreaterThan(float valueA, float valueB);                                                    // Check if values are between bias range
static Vector2 TriangleBarycenter(Vector2 v1, Vector2 v2, Vector2 v3);                                      // Returns the barycenter of a triangle given by 3 points

static void InitTimer(void);                                                                                // Initializes hi-resolution MONOTONIC timer
static uint64_t GetTimeCount(void);                                                                         // Get hi-res MONOTONIC time measure in mseconds
static double GetCurrentTime(void);                                                                         // Get current time measure in milliseconds

// Math functions
static Vector2 MathCross(float value, Vector2 vector);                                                      // Returns the cross product of a vector and a value
static float MathCrossVector2(Vector2 v1, Vector2 v2);                                                      // Returns the cross product of two vectors
static float MathLenSqr(Vector2 vector);                                                                    // Returns the len square root of a vector
static float MathDot(Vector2 v1, Vector2 v2);                                                               // Returns the dot product of two vectors
static inline float DistSqr(Vector2 v1, Vector2 v2);                                                        // Returns the square root of distance between two vectors
static void MathNormalize(Vector2 *vector);                                                                 // Returns the normalized values of a vector
#if defined(PHYSAC_STANDALONE)
static Vector2 Vector2Add(Vector2 v1, Vector2 v2);                                                          // Returns the sum of two given vectors
static Vector2 Vector2Subtract(Vector2 v1, Vector2 v2);                                                     // Returns the subtract of two given vectors
#endif

static Mat2 Mat2Radians(float radians);                                                                     // Creates a matrix 2x2 from a given radians value
static void Mat2Set(Mat2 *matrix, float radians);                                                           // Set values from radians to a created matrix 2x2
static inline Mat2 Mat2Transpose(Mat2 matrix);                                                              // Returns the transpose of a given matrix 2x2
static inline Vector2 Mat2MultiplyVector2(Mat2 matrix, Vector2 vector);                                     // Multiplies a vector by a matrix 2x2
