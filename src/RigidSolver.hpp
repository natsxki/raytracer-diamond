// Rigidsolver.hpp : IGR final project (Fundamentals of Computer Graphics)
//Lylia Mesa

#ifndef _RIGIDSOLVER_HPP_
#define _RIGIDSOLVER_HPP_

#include <glm/ext/matrix_transform.hpp>
#include <vector>
#include <cmath>
#include <iostream>

#include "Vector3.hpp"
#include "Matrix3x3.hpp"

struct Quaternion{ //q=(s,x,y,z)
  tReal s;
  Vec3f v;

  Quaternion():s(1.0),v(0,0,0){}
  Quaternion(tReal _s, Vec3f _v) : s(_s), v(_v) {}
  Quaternion(tReal _s, tReal _x, tReal _y, tReal _z): s(_s), v(Vec3f(_x,_y,_z)){} 

  //multiplication (of two quaternions q1, q2) ->composing rotations
   Quaternion operator*(const Quaternion& q) const {
    //v . q.v 
    tReal dot_val = v[0]*q.v[0] + v[1]*q.v[1] + v[2]*q.v[2];

    //v x q.v
    Vec3f cross_val(
      v[1]*q.v[2]-v[2]*q.v[1], 
      v[2]*q.v[0]-v[0]*q.v[2], 
      v[0]*q.v[1]-v[1]*q.v[0]);

    tReal new_s = s*q.s-dot_val;
    Vec3f new_v = (q.v *s) + (v*q.s) + cross_val;

    return Quaternion(new_s, new_v);
  }

  //normalization
    void normalize() {
    tReal norm = s*s + v[0]*v[0] + v[1]*v[1] + v[2]*v[2];
    if(norm > 0) {
      tReal inv = 1.0 / std::sqrt(norm);
      s *= inv;
      v *= inv; 
    }
  }


  //conversion from quaternion to rotation matrix 
  Mat3f toMatrix3x3() const {
    tReal x = v.x; tReal y = v.y ; tReal z = v.z;
    tReal xx = x*x; tReal yy = y*y; tReal zz = z*z;
    tReal xy = x*y; tReal xz = x*z; tReal yz = y*z;
    tReal sx = s*x; tReal sy = s*y; tReal sz = s*z;

    return Mat3f(
      1-2*yy-2*zz,2*xy-2*sz,2*xz+2*sy,
      2*xy+2*sz,1-2*xx-2*zz,2*yz-2*sx,
      2*xz-2*sy,2*yz+2*sx,1-2*xx-2*yy
    );
    }
};

struct BodyAttributes {
  BodyAttributes() :
    X(0, 0, 0), R(Mat3f::I()), P(0, 0, 0), L(0, 0, 0),
    V(0, 0, 0), omega(0, 0, 0), F(0, 0, 0), tau(0, 0, 0), 
    q(1.0, 0, 0, 0)
    {}

  glm::mat4 worldMat() const
  {
    return glm::mat4(           // column-major
      R(0,0), R(1,0), R(2,0), 0,
      R(0,1), R(1,1), R(2,1), 0,
      R(0,2), R(1,2), R(2,2), 0,
      X[0],   X[1],   X[2],   1);
  }

  tReal M;                      // mass
  Mat3f I0, I0inv;              // inertia tensor and its inverse in body space
  Mat3f Iinv;                   // inverse of inertia tensor

  // rigid body state
  Vec3f X;                      // position
  Mat3f R;                      // rotation
  Vec3f P;                      // linear momentum
  Vec3f L;                      // angular momentum

  // auxiliary quantities
  Vec3f V;                      // linear velocity
  Vec3f omega;                  // angular velocity

  // force and torque
  Vec3f F;                      // force
  Vec3f tau;                    // torque

  Quaternion q;

  // mesh's vertices in body space
  std::vector<Vec3f> vdata0;
};


class Box : public BodyAttributes {
public:
  explicit Box(
    const tReal w=1.0, const tReal h=1.0, const tReal d=1.0, const tReal dens=10.0,
    const Vec3f v0=Vec3f(0, 0, 0), const Vec3f omega0=Vec3f(0, 0, 0)) :
    width(w), height(h), depth(d)
  {
    V = v0;                     // initial velocity
    omega = omega0;             // initial angular velocity

    M = w*h*d*dens;

    //diamond is like a sphere
tReal r = w * 0.5;
tReal I_val = (2.0 / 5.0) * M * r * r;

// A sphere has the same inertia on all axes
I0 = Mat3f(I_val, 0, 0, 0, I_val, 0, 0, 0, I_val);
I0inv = Mat3f(1.0/I_val, 0, 0, 0, 1.0/I_val, 0, 0, 0, 1.0/I_val);
    
    Iinv = I0inv; 

    //Angular momentum
    L = I0*omega; 

    // vertices data (8 vertices)
    vdata0.push_back(Vec3f(-0.5*w, -0.5*h, -0.5*d));
    vdata0.push_back(Vec3f( 0.5*w, -0.5*h, -0.5*d));
    vdata0.push_back(Vec3f( 0.5*w,  0.5*h, -0.5*d));
    vdata0.push_back(Vec3f(-0.5*w,  0.5*h, -0.5*d));

    vdata0.push_back(Vec3f(-0.5*w, -0.5*h,  0.5*d));
    vdata0.push_back(Vec3f( 0.5*w, -0.5*h,  0.5*d));
    vdata0.push_back(Vec3f( 0.5*w,  0.5*h,  0.5*d));
    vdata0.push_back(Vec3f(-0.5*w,  0.5*h,  0.5*d));
  }

  // rigid body property
  tReal width, height, depth;
};

// Cone class for Diamond
class Cone : public BodyAttributes {
public:
  explicit Cone(
    const tReal r=1.0, const tReal h=1.0, const tReal m=1.0,
    const Vec3f x0=Vec3f(0,0,0), const Vec3f omega0=Vec3f(0,0,0)) :
    radius(r), height(h)
  {
    X = x0;         // Initial Position
    omega = omega0; // Initial Angular Velocity
    M = m;          // Mass

    // Cone inertia tensor
    tReal Iy = (3.0/10.0) * M * r * r;
    tReal Ixz = (3.0/80.0) * M * (4.0*r*r + h*h);

    I0 = Mat3f(Ixz, 0, 0, 0, Iy, 0, 0, 0, Ixz);
    I0inv = Mat3f(1.0/Ixz, 0, 0, 0, 1.0/Iy, 0, 0, 0, 1.0/Ixz);
    Iinv = I0inv; 

    L = I0 * omega; 

    vdata0.push_back(Vec3f(0, h/2.0, 0));  //top point
    vdata0.push_back(Vec3f(0, -h/2.0, 0)); //bottom point
  }

  tReal radius, height;
};

class RigidSolver {
public:
  explicit RigidSolver(
    BodyAttributes *body0=nullptr, const Vec3f g=Vec3f(0, 0, 0)) :
    body(body0), _g(g), _step(0), _sim_t(0) {}

  void init(BodyAttributes *body0)
  {
    body = body0;
    _step = 0;
    _sim_t = 0;
  }

  void step(const tReal dt)
  {
    // std::cout << "t=" << _sim_t << " (dt=" << dt << ")" << std::endl;

    computeForceAndTorque();

    //momentum 
    body->P+= body->F*dt;

    // velocity
    body->V = body->P/body->M;

    // position
    body->X += body->V * dt; 

    // Angular momentum
    body->L += body->tau * dt; 
    // omega = Iinv * L
    body->omega = body->Iinv * body-> L;
    
    //Quaternion
    Quaternion w_q(0, body->omega[0], body->omega[1], body->omega[2]);

    //derivative dq/dt=0.5* w * q
    Quaternion dq = w_q * body->q; 
    dq.s *= 0.5;
    dq.v *= 0.5;
    body->q.s += dq.s * dt;
    body->q.v += dq.v * dt;

    body->q.normalize();

    body->R = body->q.toMatrix3x3();

    //inertia tensor 
    body->Iinv = body->R * body->I0inv * body->R.transpose();

    // Clear forces
    body->F = Vec3f(0,0,0);
    body->tau = Vec3f(0,0,0);

    handleFloorCollision(-1.0f, 0.6, 0.9); // Floor at Y=0, Restitution=0.6, Friction=0.9

    ++_step;
    _sim_t += dt;
  }

  BodyAttributes *body;

private:

  void computeForceAndTorque()
  {
    //Force and torque calculation
    body->F = body->M *_g;

    if(_step==1) {
      //force
      Vec3f force0 = Vec3f(4.0,0.40,2.0);
      body->F += force0; //adding to the gravitaional force

      //torque
      Vec3f vertexBody = body->vdata0[0]; //first corner in body space to apply the force on it
      //lever arm
      Vec3f r = body->R * vertexBody; 

      //torque=cross product between lever arm vector and force
      body->tau += Vec3f(r.y * force0.z-r.z*force0.y, r.z * force0.x - r.x * force0.z, r.x*force0.y - r.y*force0.x);

    }
  }

  void handleFloorCollision(tReal floorY, tReal restitution, tReal friction) {
    // Bounding sphere approximation
    tReal radius = 1.0; 
      
    // collision check 
    if (body->X.y - radius < floorY) {
          
      // Hard constraint against sinking
      body->X.y = floorY + radius;
    
      if (body->V.y < 0) {
              
      //Only bounce if significant impact
      if (body->V.y > -0.1f) {
        body->V.y = 0.0f; 
      } else {
        body->P.y *= -restitution; 
        body->V.y = body->P.y / body->M;
      }
              
      // Apply Linear Friction (Sliding)
      body->P.x *= friction; body->P.z *= friction;
      body->V.x *= friction; body->V.z *= friction;

      // Only tumble on high impact, random torque is assigned to make the diamond spin 
      if (std::abs(body->V.y) > 2.0f) {
        Vec3f randomTorque((float(rand())/RAND_MAX - 0.5f) * 10.0f,0.0f,(float(rand())/RAND_MAX - 0.5f) * 10.0f);
        body->L += randomTorque;
      } 
      // Otherwise, if rolling/resting, angular friction 
      else {
        body->L *= 0.75f; 
        body->omega *= 0.75f;
      }
    }
  }
}

  Vec3f _g;                     // gravity
  tIndex _step;                 // step count
  tReal _sim_t;                 // simulation time
};

#endif  /* _RIGIDSOLVER_HPP_ */