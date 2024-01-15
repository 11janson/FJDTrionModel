//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 or later of the License.      #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#ifndef CC_INCLUDE_GL_HEADER
#define CC_INCLUDE_GL_HEADER

#include <cmath>

#include "CCMath.h"

//Local
#include "ccGLMatrix.h"

//Qt
#include <QOpenGLFunctions_2_1>

#include "Eigen\Dense"
#include "Eigen\Eigen"
#include "Eigen\src\Geometry\AlignedBox.h"
//! Shortcuts to OpenGL commands independent on the input type
class ccGL
{
public:

	//type-less glVertex3Xv call (X=f,d)
	static inline void Vertex3v(QOpenGLFunctions_2_1* glFunc, const float* v) { glFunc->glVertex3fv(v); }
	static inline void Vertex3v(QOpenGLFunctions_2_1* glFunc, const double* v) { glFunc->glVertex3dv(v); }

	//type-less glVertex3X call (X=f,d)
	static inline void Vertex3(QOpenGLFunctions_2_1* glFunc, float x, float y, float z) { glFunc->glVertex3f(x, y, z); }
	static inline void Vertex3(QOpenGLFunctions_2_1* glFunc, double x, double y, double z) { glFunc->glVertex3d(x, y, z); }

	//type-less glScaleX call (X=f,d)
	static inline void Scale(QOpenGLFunctions_2_1* glFunc, float x, float y, float z) { glFunc->glScalef(x, y, z); }
	static inline void Scale(QOpenGLFunctions_2_1* glFunc, double x, double y, double z) { glFunc->glScaled(x, y, z); }

	//type-less glNormal3Xv call (X=f,d)
	static inline void Normal3v(QOpenGLFunctions_2_1* glFunc, const float* v) { glFunc->glNormal3fv(v); }
	static inline void Normal3v(QOpenGLFunctions_2_1* glFunc, const double* v) { glFunc->glNormal3dv(v); }

	//type-less glRotateX call (X=f,d)
	static inline void Rotate(QOpenGLFunctions_2_1* glFunc, float a, float x, float y, float z) { glFunc->glRotatef(a, x, y, z); }
	static inline void Rotate(QOpenGLFunctions_2_1* glFunc, double a, double x, double y, double z) { glFunc->glRotated(a, x, y, z); }

	//type-less glTranslateX call (X=f,d)
	static inline void Translate(QOpenGLFunctions_2_1* glFunc, float x, float y, float z) { glFunc->glTranslatef(x, y, z); }
	static inline void Translate(QOpenGLFunctions_2_1* glFunc, double x, double y, double z) { glFunc->glTranslated(x, y, z); }

	//type-less glColor3Xv call (X=f,ub)
	static inline void Color3v(QOpenGLFunctions_2_1* glFunc, const unsigned char* v) { glFunc->glColor3ubv(v); }
	static inline void Color3v(QOpenGLFunctions_2_1* glFunc, const float* v) { glFunc->glColor3fv(v); }

	//type-less glColor4Xv call (X=f,ub)
	static inline void Color4v(QOpenGLFunctions_2_1* glFunc, const unsigned char* v) { glFunc->glColor4ubv(v); }
	static inline void Color4v(QOpenGLFunctions_2_1* glFunc, const float* v) { glFunc->glColor4fv(v); }

public: //GLU equivalent methods

	static ccGLMatrixd Frustum(double left, double right, double bottom, double top, double znear, double zfar)
	{
		// invalid for: n<=0, f<=0, l=r, b=t, or n=f
		assert(znear > 0);
		assert(zfar > 0);
		assert(left != right);
		assert(bottom != top);
		assert(znear != zfar);

		ccGLMatrixd outMatrix;
		{
			double* matrix = outMatrix.data();

			double dX = right - left;
			double dY = top - bottom;
			double dZ = zfar - znear;

			matrix[0]  =  (2 * znear) / dX;
			matrix[1]  =  0.0;
			matrix[2]  =  0.0;
			matrix[3]  =  0.0;

			matrix[4]  =  0.0;
			matrix[5]  =  (2 * znear) / dY;
			matrix[6]  =  0.0;
			matrix[7]  =  0.0;

			matrix[8]  =  (right + left) / dX;
			matrix[9]  =  (top + bottom) / dY;
			matrix[10] = -(zfar + znear) / dZ;
			matrix[11] = -1.0;

			matrix[12] =  0.0;
			matrix[13] =  0.0;
			matrix[14] =  (-2 * zfar*znear) / dZ;
			matrix[15] =  0.0;
		}

		return outMatrix;
	}

	static ccGLMatrixd Ortho(	double left,    double right,
								double bottom,  double top,
								double nearVal, double farVal )
	{
		ccGLMatrixd matrix;
		double dx = (right - left);
		double dy = (top - bottom);
		double dz = (farVal - nearVal);
		if (dx != 0 && dy != 0 && dz != 0)
		{
			double* mat = matrix.data();
			// set OpenGL perspective projection matrix
			mat[0] = 2.0 / dx;
			mat[1] = 0;
			mat[2] = 0;
			mat[3] = 0;

			mat[4] = 0;
			mat[5] = 2.0 / dy;
			mat[6] = 0;
			mat[7] = 0;

			mat[8] = 0;
			mat[9] = 0;
			mat[10] = -2.0 / dz;
			mat[11] = 0;

			mat[12] = -(right + left) / dx;
			mat[13] = -(top + bottom) / dy;
			mat[14] = -(farVal + nearVal) / dz;
			mat[15] = 1.0;
		}
		else
		{
			matrix.toIdentity();
		}

		return matrix;
	}

	//inspired from http://www.songho.ca/opengl/gl_projectionmatrix.html
	static ccGLMatrixd Ortho(double w, double h, double d)
	{
		ccGLMatrixd matrix;
		if (w != 0 && h != 0 && d != 0)
		{
			double* mat = matrix.data();
			mat[0]  = 1.0 / w;
			mat[1]  = 0.0;
			mat[2]  = 0.0;
			mat[3]  = 0.0;

			mat[4]  = 0.0;
			mat[5]  = 1.0 / h;
			mat[6]  = 0.0;
			mat[7]  = 0.0;

			mat[8]  = 0.0;
			mat[9]  = 0.0;
			mat[10] = - 1.0 / d;
			mat[11] = 0.0;

			mat[12] = 0.0;
			mat[13] = 0.0;
			mat[14] = 0.0;
			mat[15] = 1.0;
		}
		else
		{
			matrix.toIdentity();
		}

		return matrix;
	}

	template <typename iType, typename oType>
	static bool Project(const Vector3Tpl<iType>& input3D, const oType* modelview, const oType* projection, const int* viewport, Vector3Tpl<oType>& output2D, bool* inFrustum = nullptr)
	{
		//Modelview transform
		Tuple4Tpl<oType> Pm;
		{
			Pm.x = static_cast<oType>(modelview[0]*input3D.x + modelview[4]*input3D.y + modelview[ 8]*input3D.z + modelview[12]);
			Pm.y = static_cast<oType>(modelview[1]*input3D.x + modelview[5]*input3D.y + modelview[ 9]*input3D.z + modelview[13]);
			Pm.z = static_cast<oType>(modelview[2]*input3D.x + modelview[6]*input3D.y + modelview[10]*input3D.z + modelview[14]);
			Pm.w = static_cast<oType>(modelview[3]*input3D.x + modelview[7]*input3D.y + modelview[11]*input3D.z + modelview[15]);
		};

		//Projection transform
		Tuple4Tpl<oType> Pp;
		{
			Pp.x = static_cast<oType>(projection[0]*Pm.x + projection[4]*Pm.y + projection[ 8]*Pm.z + projection[12]*Pm.w);
			Pp.y = static_cast<oType>(projection[1]*Pm.x + projection[5]*Pm.y + projection[ 9]*Pm.z + projection[13]*Pm.w);
			Pp.z = static_cast<oType>(projection[2]*Pm.x + projection[6]*Pm.y + projection[10]*Pm.z + projection[14]*Pm.w);
			Pp.w = static_cast<oType>(projection[3]*Pm.x + projection[7]*Pm.y + projection[11]*Pm.z + projection[15]*Pm.w);
		};
		
		//The result normalizes between -1 and 1
		if (Pp.w == 0.0)
		{
			return false;
		}

		if (inFrustum)
		{
			//Check if the point is inside the frustum
			*inFrustum = (std::abs(Pp.x) <= Pp.w && std::abs(Pp.y) <= Pp.w && std::abs(Pp.z) <= Pp.w);
		}

		//Perspective division
		Pp.x /= Pp.w;
		Pp.y /= Pp.w;
		Pp.z /= Pp.w;
		//Window coordinates
		//Map x, y to range 0-1
		output2D.x = (1.0 + Pp.x) / 2 * viewport[2] + viewport[0];
		output2D.y = (1.0 + Pp.y) / 2 * viewport[3] + viewport[1];
		//This is only correct when glDepthRange(0.0, 1.0)
		output2D.z = (1.0 + Pp.z) / 2;	//Between 0 and 1

		return true;
	}

	
	inline static double MAT(const double* m, int r, int c) { return m[c*4+r]; }
	inline static float MAT(const float* m, int r, int c) { return m[c*4+r]; }

	inline static double& MAT(double* m, int r, int c) { return m[c*4+r]; }
	inline static float& MAT(float* m, int r, int c) { return m[c*4+r]; }
	
	template <typename Type>
	static bool InvertMatrix(const Type* m, Type* out)
	{
		Type wtmp[4][8];
		Type m0, m1, m2, m3, s;
		Type *r0, *r1, *r2, *r3;
		r0 = wtmp[0], r1 = wtmp[1], r2 = wtmp[2], r3 = wtmp[3];

		r0[0] = MAT(m, 0, 0), r0[1] = MAT(m, 0, 1),
		r0[2] = MAT(m, 0, 2), r0[3] = MAT(m, 0, 3),
		r0[4] = 1.0, r0[5] = r0[6] = r0[7] = 0.0,
		r1[0] = MAT(m, 1, 0), r1[1] = MAT(m, 1, 1),
		r1[2] = MAT(m, 1, 2), r1[3] = MAT(m, 1, 3),
		r1[5] = 1.0, r1[4] = r1[6] = r1[7] = 0.0,
		r2[0] = MAT(m, 2, 0), r2[1] = MAT(m, 2, 1),
		r2[2] = MAT(m, 2, 2), r2[3] = MAT(m, 2, 3),
		r2[6] = 1.0, r2[4] = r2[5] = r2[7] = 0.0,
		r3[0] = MAT(m, 3, 0), r3[1] = MAT(m, 3, 1),
		r3[2] = MAT(m, 3, 2), r3[3] = MAT(m, 3, 3),
		r3[7] = 1.0, r3[4] = r3[5] = r3[6] = 0.0;
		
		//choose pivot - or die
		if (std::abs(r3[0]) > std::abs(r2[0]))
			std::swap(r3, r2);
		if (std::abs(r2[0]) > std::abs(r1[0]))
			std::swap(r2, r1);
		if (std::abs(r1[0]) > std::abs(r0[0]))
			std::swap(r1, r0);
		if (0.0 == r0[0])
			return false;
		
		//eliminate first variable
		m1 = r1[0] / r0[0];
		m2 = r2[0] / r0[0];
		m3 = r3[0] / r0[0];
		s = r0[1];
		r1[1] -= m1 * s;
		r2[1] -= m2 * s;
		r3[1] -= m3 * s;
		s = r0[2];
		r1[2] -= m1 * s;
		r2[2] -= m2 * s;
		r3[2] -= m3 * s;
		s = r0[3];
		r1[3] -= m1 * s;
		r2[3] -= m2 * s;
		r3[3] -= m3 * s;
		s = r0[4];
		if (s != 0.0)
		{
			r1[4] -= m1 * s;
			r2[4] -= m2 * s;
			r3[4] -= m3 * s;
		}
		s = r0[5];
		if (s != 0.0)
		{
			r1[5] -= m1 * s;
			r2[5] -= m2 * s;
			r3[5] -= m3 * s;
		}
		s = r0[6];
		if (s != 0.0)
		{
			r1[6] -= m1 * s;
			r2[6] -= m2 * s;
			r3[6] -= m3 * s;
		}
		s = r0[7];
		if (s != 0.0)
		{
			r1[7] -= m1 * s;
			r2[7] -= m2 * s;
			r3[7] -= m3 * s;
		}
		
		//choose pivot - or die
		if (std::abs(r3[1]) > std::abs(r2[1]))
			std::swap(r3, r2);
		if (std::abs(r2[1]) > std::abs(r1[1]))
			std::swap(r2, r1);
		if (0.0 == r1[1])
			return false;
		
		//eliminate second variable
		m2 = r2[1] / r1[1];
		m3 = r3[1] / r1[1];
		r2[2] -= m2 * r1[2];
		r3[2] -= m3 * r1[2];
		r2[3] -= m2 * r1[3];
		r3[3] -= m3 * r1[3];
		s = r1[4];
		if (0.0 != s)
		{
			r2[4] -= m2 * s;
			r3[4] -= m3 * s;
		}
		s = r1[5];
		if (0.0 != s)
		{
			r2[5] -= m2 * s;
			r3[5] -= m3 * s;
		}
		s = r1[6];
		if (0.0 != s)
		{
			r2[6] -= m2 * s;
			r3[6] -= m3 * s;
		}
		s = r1[7];
		if (0.0 != s)
		{
			r2[7] -= m2 * s;
			r3[7] -= m3 * s;
		}
		
		//choose pivot - or die
		if (std::abs(r3[2]) > std::abs(r2[2]))
			std::swap(r3, r2);
		if (0.0 == r2[2])
			return false;
		
		//eliminate third variable
		m3 = r3[2] / r2[2];
		r3[3] -= m3 * r2[3], r3[4] -= m3 * r2[4],
		r3[5] -= m3 * r2[5], r3[6] -= m3 * r2[6], r3[7] -= m3 * r2[7];
		
		//last check
		if (0.0 == r3[3])
			return false;
		
		s = 1.0 / r3[3]; //now back substitute row 3
		r3[4] *= s;
		r3[5] *= s;
		r3[6] *= s;
		r3[7] *= s;
		m2 = r2[3]; //now back substitute row 2
		s = 1.0 / r2[2];
		r2[4] = s * (r2[4] - r3[4] * m2), r2[5] = s * (r2[5] - r3[5] * m2),
		r2[6] = s * (r2[6] - r3[6] * m2), r2[7] = s * (r2[7] - r3[7] * m2);
		m1 = r1[3];
		r1[4] -= r3[4] * m1, r1[5] -= r3[5] * m1,
		r1[6] -= r3[6] * m1, r1[7] -= r3[7] * m1;
		m0 = r0[3];
		r0[4] -= r3[4] * m0, r0[5] -= r3[5] * m0,
		r0[6] -= r3[6] * m0, r0[7] -= r3[7] * m0;
		m1 = r1[2]; //now back substitute row 1
		s = 1.0 / r1[1];
		r1[4] = s * (r1[4] - r2[4] * m1), r1[5] = s * (r1[5] - r2[5] * m1),
		r1[6] = s * (r1[6] - r2[6] * m1), r1[7] = s * (r1[7] - r2[7] * m1);
		m0 = r0[2];
		r0[4] -= r2[4] * m0, r0[5] -= r2[5] * m0,
		r0[6] -= r2[6] * m0, r0[7] -= r2[7] * m0;
		m0 = r0[1]; //now back substitute row 0
		s = 1.0 / r0[0];
		r0[4] = s * (r0[4] - r1[4] * m0), r0[5] = s * (r0[5] - r1[5] * m0),
		r0[6] = s * (r0[6] - r1[6] * m0), r0[7] = s * (r0[7] - r1[7] * m0);
		
		MAT(out, 0, 0) = r0[4];
		MAT(out, 0, 1) = r0[5], MAT(out, 0, 2) = r0[6];
		MAT(out, 0, 3) = r0[7], MAT(out, 1, 0) = r1[4];
		MAT(out, 1, 1) = r1[5], MAT(out, 1, 2) = r1[6];
		MAT(out, 1, 3) = r1[7], MAT(out, 2, 0) = r2[4];
		MAT(out, 2, 1) = r2[5], MAT(out, 2, 2) = r2[6];
		MAT(out, 2, 3) = r2[7], MAT(out, 3, 0) = r3[4];
		MAT(out, 3, 1) = r3[5], MAT(out, 3, 2) = r3[6];
		MAT(out, 3, 3) = r3[7];
		
		return true;
	}

	template <typename iType, typename oType>
	static bool Unproject(const Vector3Tpl<iType>& input2D, const oType* modelview, const oType* projection, const int* viewport, Vector3Tpl<oType>& output3D)
	{
		//compute projection x modelview
		ccGLMatrixTpl<oType> A = ccGLMatrixTpl<oType>(projection) * ccGLMatrixTpl<oType>(modelview);
		ccGLMatrixTpl<oType> m;

		if (!InvertMatrix(A.data(), m.data()))
		{
			return false;
		}

		ccGLMatrixTpl<oType> mA = m * A;

		//Transformation of normalized coordinates between -1 and 1
		Tuple4Tpl<oType> in;
		in.x = static_cast<oType>((input2D.x - static_cast<iType>(viewport[0])) / viewport[2] * 2 - 1);
		in.y = static_cast<oType>((input2D.y - static_cast<iType>(viewport[1])) / viewport[3] * 2 - 1);
		in.z = static_cast<oType>(2*input2D.z - 1);
		in.w = 1;

		//Objects coordinates
		Tuple4Tpl<oType> out = m * in;
		if (out.w == 0)
		{
			return false;
		}

		output3D = Vector3Tpl<oType>(out.u) / out.w;

		return true;
	}

	template <typename iType, typename oType>
	static bool Unprojectx(const Vector3Tpl<iType>& input2D, const oType* modelview, const oType* projection, const int* viewport, Vector3Tpl<oType>& output3D, double x)
	{
		double ppx = 2 * (input2D.x - viewport[0]) / viewport[2] - 1.0;
		double ppy = 2 * (input2D.y - viewport[1]) / viewport[3] - 1.0;
		//double ppz = 2 * z - 1.0;
		double ppw = 1.0;

		Eigen::Matrix4d Mat1;
		Mat1 << projection[0], projection[4], projection[8], projection[12],
			projection[1], projection[5], projection[9], projection[13],
			projection[2], projection[6], projection[10], projection[14],
			projection[3], projection[7], projection[11], projection[15];

		Eigen::Matrix4d Mat11 = Mat1.inverse();

		double k1 = 2 * Mat11(0, 2);
		double k2 = 2 * Mat11(1, 2);
		double k3 = 2 * Mat11(2, 2);
		double k4 = 2 * Mat11(3, 2);
		double b1 = Mat11(0, 0) * ppx + Mat11(0, 1) * ppy - Mat11(0, 2) + Mat11(0, 3) * ppw;
		double b2 = Mat11(1, 0) * ppx + Mat11(1, 1) * ppy - Mat11(1, 2) + Mat11(1, 3) * ppw;
		double b3 = Mat11(2, 0) * ppx + Mat11(2, 1) * ppy - Mat11(2, 2) + Mat11(2, 3) * ppw;
		double b4 = Mat11(3, 0) * ppx + Mat11(3, 1) * ppy - Mat11(3, 2) + Mat11(3, 3) * ppw;

		Eigen::Matrix4d Mat2;
		Mat2 << modelview[0], modelview[4], modelview[8], modelview[12],
			modelview[1], modelview[5], modelview[9], modelview[13],
			modelview[2], modelview[6], modelview[10], modelview[14],
			modelview[3], modelview[7], modelview[11], modelview[15];

		Eigen::Matrix4d Mat21 = Mat2.inverse();

		double k5 = Mat21(0, 0) * k1 + Mat21(0, 1) * k2 + Mat21(0, 2) * k3 + Mat21(0, 3) * k4;
		double k6 = Mat21(1, 0) * k1 + Mat21(1, 1) * k2 + Mat21(1, 2) * k3 + Mat21(1, 3) * k4;
		double k7 = Mat21(2, 0) * k1 + Mat21(2, 1) * k2 + Mat21(2, 2) * k3 + Mat21(2, 3) * k4;
		double k8 = Mat21(3, 0) * k1 + Mat21(3, 1) * k2 + Mat21(3, 2) * k3 + Mat21(3, 3) * k4;
		double b5 = Mat21(0, 0) * b1 + Mat21(0, 1) * b2 + Mat21(0, 2) * b3 + Mat21(0, 3) * b4;
		double b6 = Mat21(1, 0) * b1 + Mat21(1, 1) * b2 + Mat21(1, 2) * b3 + Mat21(1, 3) * b4;
		double b7 = Mat21(2, 0) * b1 + Mat21(2, 1) * b2 + Mat21(2, 2) * b3 + Mat21(2, 3) * b4;
		double b8 = Mat21(3, 0) * b1 + Mat21(3, 1) * b2 + Mat21(3, 2) * b3 + Mat21(3, 3) * b4;
		double p2dz = (x - b5) / k5;
		output3D.x = x;
		output3D.y = k6 * p2dz + b6;
		output3D.z = k7 * p2dz + b7;
		return true;
	}

	template <typename iType, typename oType>
	static bool Unprojecty(const Vector3Tpl<iType>& input2D, const oType* modelview, const oType* projection, const int* viewport, Vector3Tpl<oType>& output3D,double y)
	{
		double ppx = 2 * (input2D.x - viewport[0]) / viewport[2] - 1.0;
		double ppy = 2 * (input2D.y - viewport[1]) / viewport[3] - 1.0;
		//double ppz = 2 * z - 1.0;
		double ppw = 1.0;

		Eigen::Matrix4d Mat1;
		Mat1 << projection[0], projection[4], projection[8], projection[12],
			projection[1], projection[5], projection[9], projection[13],
			projection[2], projection[6], projection[10], projection[14],
			projection[3], projection[7], projection[11], projection[15];

		Eigen::Matrix4d Mat11 = Mat1.inverse();

		double k1 = 2 * Mat11(0, 2);
		double k2 = 2 * Mat11(1, 2);
		double k3 = 2 * Mat11(2, 2);
		double k4 = 2 * Mat11(3, 2);
		double b1 = Mat11(0, 0) * ppx + Mat11(0, 1) * ppy - Mat11(0, 2) + Mat11(0, 3) * ppw;
		double b2 = Mat11(1, 0) * ppx + Mat11(1, 1) * ppy - Mat11(1, 2) + Mat11(1, 3) * ppw;
		double b3 = Mat11(2, 0) * ppx + Mat11(2, 1) * ppy - Mat11(2, 2) + Mat11(2, 3) * ppw;
		double b4 = Mat11(3, 0) * ppx + Mat11(3, 1) * ppy - Mat11(3, 2) + Mat11(3, 3) * ppw;

		Eigen::Matrix4d Mat2;
		Mat2 << modelview[0], modelview[4], modelview[8], modelview[12],
			modelview[1], modelview[5], modelview[9], modelview[13],
			modelview[2], modelview[6], modelview[10], modelview[14],
			modelview[3], modelview[7], modelview[11], modelview[15];

		Eigen::Matrix4d Mat21 = Mat2.inverse();

		double k5 = Mat21(0, 0) * k1 + Mat21(0, 1) * k2 + Mat21(0, 2) * k3 + Mat21(0, 3) * k4;
		double k6 = Mat21(1, 0) * k1 + Mat21(1, 1) * k2 + Mat21(1, 2) * k3 + Mat21(1, 3) * k4;
		double k7 = Mat21(2, 0) * k1 + Mat21(2, 1) * k2 + Mat21(2, 2) * k3 + Mat21(2, 3) * k4;
		double k8 = Mat21(3, 0) * k1 + Mat21(3, 1) * k2 + Mat21(3, 2) * k3 + Mat21(3, 3) * k4;
		double b5 = Mat21(0, 0) * b1 + Mat21(0, 1) * b2 + Mat21(0, 2) * b3 + Mat21(0, 3) * b4;
		double b6 = Mat21(1, 0) * b1 + Mat21(1, 1) * b2 + Mat21(1, 2) * b3 + Mat21(1, 3) * b4;
		double b7 = Mat21(2, 0) * b1 + Mat21(2, 1) * b2 + Mat21(2, 2) * b3 + Mat21(2, 3) * b4;
		double b8 = Mat21(3, 0) * b1 + Mat21(3, 1) * b2 + Mat21(3, 2) * b3 + Mat21(3, 3) * b4;
		double p2dz = (y - b6) / k6;
		output3D.x = k5 * p2dz + b5;
		output3D.y = y;
		output3D.z = k7 * p2dz + b7;
		return true;
	}

	template <typename iType, typename oType>
	static bool Unprojectz(const Vector3Tpl<iType>& input2D, const oType* modelview, const oType* projection, const int* viewport, Vector3Tpl<oType>& output3D, double z)
	{
		double ppx = 2 * (input2D.x - viewport[0]) / viewport[2] - 1.0;
		double ppy = 2 * (input2D.y - viewport[1]) / viewport[3] - 1.0;
		double ppz = 2 * z - 1.0;
		double ppw = 1.0;

		Eigen::Matrix4d Mat1;
		Mat1 << projection[0], projection[4], projection[8], projection[12],
			projection[1], projection[5], projection[9], projection[13],
			projection[2], projection[6], projection[10], projection[14],
			projection[3], projection[7], projection[11], projection[15];

		Eigen::Matrix4d Mat11 = Mat1.inverse();

		//Projection transform
		Tuple4Tpl<oType> Pp;
		{
			Pp.x = static_cast<oType>(Mat11(0, 0) * ppx + Mat11(0, 1) * ppy + Mat11(0, 2) * ppz + Mat11(0, 3) * ppw);
			Pp.y = static_cast<oType>(Mat11(1, 0) * ppx + Mat11(1, 1) * ppy + Mat11(1, 2) * ppz + Mat11(1, 3) * ppw);
			Pp.z = static_cast<oType>(Mat11(2, 0) * ppx + Mat11(2, 1) * ppy + Mat11(2, 2) * ppz + Mat11(2, 3) * ppw);
			Pp.w = static_cast<oType>(Mat11(3, 0) * ppx + Mat11(3, 1) * ppy + Mat11(3, 2) * ppz + Mat11(3, 3) * ppw);
		};


		Eigen::Matrix4d Mat2;
		Mat2 << modelview[0], modelview[4], modelview[8], modelview[12],
			modelview[1], modelview[5], modelview[9], modelview[13],
			modelview[2], modelview[6], modelview[10], modelview[14],
			modelview[3], modelview[7], modelview[11], modelview[15];

		Eigen::Matrix4d Mat21 = Mat2.inverse();
		Tuple4Tpl<oType> Pm;
		{
			Pm.x = static_cast<oType>(Mat21(0, 0) * Pp.x + Mat21(0, 1) * Pp.y + Mat21(0, 2) * Pp.z + Mat21(0, 3) * Pp.w);
			Pm.y = static_cast<oType>(Mat21(1, 0) * Pp.x + Mat21(1, 1) * Pp.y + Mat21(1, 2) * Pp.z + Mat21(1, 3) * Pp.w);
			Pm.z = static_cast<oType>(Mat21(2, 0) * Pp.x + Mat21(2, 1) * Pp.y + Mat21(2, 2) * Pp.z + Mat21(2, 3) * Pp.w);
			Pm.w = static_cast<oType>(Mat21(3, 0) * Pp.x + Mat21(3, 1) * Pp.y + Mat21(3, 2) * Pp.z + Mat21(3, 3) * Pp.w);
		};

		output3D.x = Pm.x;
		output3D.y = Pm.y;
		output3D.z = Pm.z;
		return true;
	}

	static void PickMatrix(double x, double y, double width, double height, int viewport[4], double m[16])
	{
		double sx = viewport[2] / width;
		double sy = viewport[3] / height;
		double tx = (viewport[2] + 2.0 * (viewport[0] - x)) / width;
		double ty = (viewport[3] + 2.0 * (viewport[1] - y)) / height;

		MAT(m, 0, 0) = sx;
		MAT(m, 0, 1) = 0.0;
		MAT(m, 0, 2) = 0.0;
		MAT(m, 0, 3) = tx;
		MAT(m, 1, 0) = 0.0;
		MAT(m, 1, 1) = sy;
		MAT(m, 1, 2) = 0.0;
		MAT(m, 1, 3) = ty;
		MAT(m, 2, 0) = 0.0;
		MAT(m, 2, 1) = 0.0;
		MAT(m, 2, 2) = 1.0;
		MAT(m, 2, 3) = 0.0;
		MAT(m, 3, 0) = 0.0;
		MAT(m, 3, 1) = 0.0;
		MAT(m, 3, 2) = 0.0;
		MAT(m, 3, 3) = 1.0;
	}
};

#endif //CC_INCLUDE_GL_HEADER
