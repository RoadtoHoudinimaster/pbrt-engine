#pragma once
#include<iostream>
#include<vector>
#include<algorithm>
#include<string>
#include"Quaternion.h"
#include"pbrt.h"
#include"Geometry.h"
namespace pbrt {
	struct Matrix4x4 {
		Matrix4x4(){
			m[0][0] = m[1][1] = m[2][2] = m[3][3] = 1.0f;
			m[0][1] = m[0][2] = m[0][3] = m[1][0] = m[1][2] = m[1][3] = m[2][0] = m[2][1] = m[2][3] = m[3][0] = m[3][1] = m[3][2] = 0.f;
		}
		Matrix4x4(float mat[4][4]) {
			m[0][0] = mat[0][0]; m[0][1] = mat[0][1]; m[0][2] = mat[0][2]; m[0][3] = mat[0][3];
			m[1][0] = mat[1][0]; m[1][1] = mat[1][1]; m[1][2] = mat[1][2]; m[1][3] = mat[1][3];
			m[2][0] = mat[2][0]; m[2][1] = mat[2][1]; m[2][2] = mat[2][2]; m[2][3] = mat[2][3];
			m[3][0] = mat[3][0]; m[3][1] = mat[3][1]; m[3][2] = mat[3][2]; m[3][3] = mat[3][3];
		}
		Matrix4x4(float t00, float t01, float t02, float t03, float t10, float t11, float t12, float t13,
			float t20, float t21, float t22, float t23, float t30, float t31, float t32, float t33)
		{
			m[0][0] = t00;m[0][1] = t01;m[0][2] = t02;m[0][3] = t03;
			m[1][0] = t10;m[1][1] = t11;m[1][2] = t12;m[1][3] = t13;
			m[2][0] = t20;m[2][1] = t21;m[2][2] = t22;m[2][3] = t23;
			m[3][0] = t30;m[3][1] = t31;m[3][2] = t32;m[3][3] = t33;
		}
		
		bool operator ==(const Matrix4x4& m2)const
		{
			for (int i = 0; i < 4; ++i)
			{
				for (int j = 0; j < 4; ++j)
				{
					if (m[i][j] != m2.m[i][j])return false;
				}
			}
			return true;
		}
		bool operator !=(const Matrix4x4& m3)const
		{
			return !(*this == m3);
		}
		//These two function need a better or a good explanation
		friend Matrix4x4 Transpose(const Matrix4x4& m);
		void Print(FILE* file)const
		{
			fprintf(file, "[");
			for (int i = 0; i < 4; ++i)
			{
				fprintf(file, "[");
				for (int i = 0; i < 4; ++i)
				{
					for (int j = 0; j < 4; ++j)
					{
						fprintf(file, "%f", m[i][j]);
						if (j != 3)fprintf(file, ",");
					}
				}
				fprintf(file, "]");
			}
			fprintf(file, "]");
		}
		static Matrix4x4 Mul(const Matrix4x4& m1, const Matrix4x4& m2) {
			Matrix4x4 r;
			for (int i = 0; i < 4; ++i)
				for (int j = 0; j < 4; ++j)
					r.m[i][j] = m1.m[i][0] * m2.m[0][j] + m1.m[i][1] * m2.m[1][j] +
					m1.m[i][2] * m2.m[2][j] + m1.m[i][3] * m2.m[3][j];
			return r;
		}
		friend Matrix4x4 Inverse(const Matrix4x4&);

		friend std::ostream& operator<<(std::ostream& os, const Matrix4x4& m) {
			
			os << "[ [" << m.m[0][0] << "," << m.m[0][1] << "," << m.m[0][2] << "," << m.m[0][3] << "],"
				"[" << m.m[1][0] << "," << m.m[1][1] << "," << m.m[1][2] << "," << m.m[1][3] << "],"
				"[" << m.m[2][0] << "," << m.m[2][1] << "," << m.m[2][2] << "," << m.m[2][3] << "],"
				"[" << m.m[3][0] << "," << m.m[3][1] << "," << m.m[3][2] << "," << m.m[3][3] << "] ]";
			return os;
		}

		float m[4][4];
	};
	class Transform {
	public:
		Transform() {}
		Transform(const float mat[4][4]) {
			m = Matrix4x4(mat[0][0], mat[0][1], mat[0][2], mat[0][3],
				mat[1][0], mat[1][1], mat[1][2], mat[1][3],
				mat[2][0], mat[2][1], mat[2][2], mat[2][3],
				mat[3][0], mat[3][1], mat[3][2], mat[3][3]);
			mInv = Inverse(this->m);
		}
		Transform(const Matrix4x4 &m) :m(m), mInv(Inverse(m)) {}
		Transform(const Matrix4x4& m, const Matrix4x4& mInv) :m(m), mInv(mInv) {}
		void Print(FILE* file)const;
		//Why these two function need to use friend function!
		friend Transform Inverse(const Transform& t)
		{
			return Transform(t.mInv, t.m);
		}
		friend Transform Transpose(const Transform& t)
		{
			return Transform(Transpose(t.m), Transpose(t.mInv));
		}
		bool operator ==(const Transform& t)const {
			return t.m == m && t.mInv == mInv;
		}
		bool operator !=(const Transform& t)const {
			return t.m != m || t.mInv != mInv;
		}

		bool operator <(const Transform& t2)const {
			for (int i = 0; i < 4; ++i)
			{
				for (int j = 0; j < 4; ++j)
				{
					if (m.m[i][j] < t2.m.m[i][j])return true;
					if (m.m[i][j] > t2.m.m[i][j])return false;
				}
			}
			return false;
		}
		bool IsIdentity() const {
			return (m.m[0][0] == 1.f && m.m[0][1] == 0.f && m.m[0][2] == 0.f &&
				m.m[0][3] == 0.f && m.m[1][0] == 0.f && m.m[1][1] == 1.f &&
				m.m[1][2] == 0.f && m.m[1][3] == 0.f && m.m[2][0] == 0.f &&
				m.m[2][1] == 0.f && m.m[2][2] == 1.f && m.m[2][3] == 0.f &&
				m.m[3][0] == 0.f && m.m[3][1] == 0.f && m.m[3][2] == 0.f &&
				m.m[3][3] == 1.f);
		}
		const Matrix4x4& GetMatrix() const { return m; }
		const Matrix4x4& GetInverseMatrix()const { return mInv; }
		bool HasScale()const {
			float la2 = (Vector3f(1, 0, 0)).LengthSquared();
			float lb2 = (Vector3f(0, 1, 0)).LengthSquared();
			float lc2 = (Vector3f(0, 0, 1)).LengthSquared();
			//set the difference between the length squared ways!
			//why here define then undefine the not_one macro
#define NOT_ONE(x) (x<.999f||x>1.001f)
			return (NOT_ONE(la2) || NOT_ONE(lb2) || NOT_ONE(lc2));
#undef NOT_ONE
		}
		//Here I want to use the * ,but the sequence of the multiplication cannot sure!
		//Here the sequence looks this way!
		template<typename T>
		inline Point3<T> operator()(const Point3<T>& p)const;
		template<typename T>
		inline Vector3<T> operator()(const Vector3<T>& v)const;
		template<typename T>
		inline Normal3<T> operator()(const Normal3<T>& v)const;
		inline Ray operator()(const Ray& r)const;
		inline RayDifferential operator()(const RayDifferential& r)const;
		Bounds3f operator()(const Bounds3f& b)const;
		Transform operator *(const Transform& t2)const;
		bool SwapHandedness() const;
		//This one needs more study
		//SurfaceInteraction operator()(const SurfaceInteraction& si)const;
		template <typename T>
		inline Point3<T> operator()(const Point3<T>& pt,
			Vector3<T>* absError) const;
		template <typename T>
		inline Point3<T> operator()(const Point3<T>& p, const Vector3<T>& pError,
			Vector3<T>* pTransError) const;
		template <typename T>
		inline Vector3<T> operator()(const Vector3<T>& v,
			Vector3<T>* vTransError) const;
		template <typename T>
		inline Vector3<T> operator()(const Vector3<T>& v, const Vector3<T>& vError,
			Vector3<T>* vTransError) const;
		inline Ray operator()(const Ray& r, Vector3f* oError,
			Vector3f* dError) const;
		inline Ray operator()(const Ray& r, const Vector3f& oErrorIn,
			const Vector3f& dErrorIn, Vector3f* oErrorOut,
			Vector3f* dErrorOut) const;

		friend std::ostream& operator<<(std::ostream& os, const Transform& t) {
			os << "t=" << t.m << ", inv = " << t.mInv;
			return os;
		}
	private:
		Matrix4x4 m, mInv;
		friend class AnimatedTransform;
		friend struct Quaternion;
	};
	
	Transform Translate(const Vector3f& delta);
	Transform Scale(float x, float y, float z);
	Transform RotateX(float delta);
	Transform RotateY(float delta);
	Transform RotateZ(float delta);
	Transform Rotate(float theta, const Vector3f& axis);
	Transform LookAt(const Point3f& pos, const Point3f& look, const Vector3f& up);
	Transform Orthographic(float znear, float zfar);
	Transform Perspective(float fov, float znear, float zfar);
	bool SolveLinearSystem2x2(const float A[2][2], const float B[2], float* x0, float* x1);

	template<typename T>
	inline Point3<T> Transform::operator()(const Point3<T>& p)const {
		T x = p.x, y = p.y, z = p.z;
		T xp = m.m[0][0] * x + m.m[0][1] * y + m.m[0][2] * z + m.m[0][3];
		T yp = m.m[1][0] * x + m.m[1][1] * y + m.m[1][2] * z + m.m[1][3];
		T zp = m.m[2][0] * x + m.m[2][1] * y + m.m[2][2] * z + m.m[2][3];
		T wp = m.m[3][0] * x + m.m[3][1] * y + m.m[3][2] * z + m.m[3][3];
		//Transform to the orthogonal clip space 
		assert(wp != 0);
		if (wp == 1)
			return Point3<T>(xp, yp, zp);
		else
			return Point3<T>(xp, yp, zp) / wp;
	}
	template <typename T>
	inline Point3<T> Transform::operator()(const Point3<T>& p,
		Vector3<T>* pError) const {
		T x = p.x, y = p.y, z = p.z;
		// Compute transformed coordinates from point _pt_
		T xp = (m.m[0][0] * x + m.m[0][1] * y) + (m.m[0][2] * z + m.m[0][3]);
		T yp = (m.m[1][0] * x + m.m[1][1] * y) + (m.m[1][2] * z + m.m[1][3]);
		T zp = (m.m[2][0] * x + m.m[2][1] * y) + (m.m[2][2] * z + m.m[2][3]);
		T wp = (m.m[3][0] * x + m.m[3][1] * y) + (m.m[3][2] * z + m.m[3][3]);

		// Compute absolute error for transformed point
		T xAbsSum = (std::abs(m.m[0][0] * x) + std::abs(m.m[0][1] * y) +
			std::abs(m.m[0][2] * z) + std::abs(m.m[0][3]));
		T yAbsSum = (std::abs(m.m[1][0] * x) + std::abs(m.m[1][1] * y) +
			std::abs(m.m[1][2] * z) + std::abs(m.m[1][3]));
		T zAbsSum = (std::abs(m.m[2][0] * x) + std::abs(m.m[2][1] * y) +
			std::abs(m.m[2][2] * z) + std::abs(m.m[2][3]));
		*pError = gamma(3) * Vector3<T>(xAbsSum, yAbsSum, zAbsSum);
		assert(wp != 0);
		if (wp == 1)
			return Point3<T>(xp, yp, zp);
		else
			return Point3<T>(xp, yp, zp) / wp;
	}

	inline void Transform::Print(FILE* file) const
	{
		assert(file);
		fprintf(file, "[");
		for (int i = 0; i < 4; ++i)
		{
			fprintf(file, "[");
			for (int i = 0; i < 4; ++i)
			{
				for (int j = 0; j < 4; ++j)
				{
					fprintf(file, "%f", m.m[i][j]);
					if (j != 3)fprintf(file, ",");
				}
			}
			fprintf(file, "]");
		}
		fprintf(file, "]");
	}

	inline Ray Transform::operator()(const Ray& r)const {
		Vector3f oError;
		Point3f o = (*this)(r.o, &oError);
		Vector3f d = (*this)(r.d);
		float lengthSquared = d.LengthSquared();
		float tMax = r.tMax;
		if (lengthSquared > 0) {
			float dt = Dot(Abs(d), oError) / lengthSquared;
			o += d * dt;
			tMax -= dt;
		}
		return Ray(o, d, tMax, r.time, r.medium);
	}
	inline RayDifferential Transform::operator()(const RayDifferential& r)const {
		Ray tr = (*this)(Ray(r));
		RayDifferential ret(tr.o, tr.d, tr.tMax, tr.time, tr.medium);
		ret.hasDifferentials = r.hasDifferentials;
		ret.rxOrigin = (*this)(r.rxOrigin);
		ret.ryOrigin = (*this)(r.ryOrigin);
		ret.rxDirection = (*this)(r.rxDirection);
		ret.ryDirection = (*this)(r.ryDirection);
		return ret;
	}
	template<typename T>
	inline Point3<T> Transform::operator()(const Point3<T>& pt, const Vector3<T>& ptError, Vector3<T>* absError)const {
		T x = pt.x, y = pt.y, z = pt.z;
		T xp = (m.m[0][0] * x + m.m[0][1] * y) + (m.m[0][2] * z + m.m[0][3]);
		T yp = (m.m[1][0] * x + m.m[1][1] * y) + (m.m[1][2] * z + m.m[1][3]);
		T zp = (m.m[2][0] * x + m.m[2][1] * y) + (m.m[2][2] * z + m.m[2][3]);
		T wp = (m.m[3][0] * x + m.m[3][1] * y) + (m.m[3][2] * z + m.m[3][3]);
		absError->x =
			(gamma(3) + (T)1) *
			(std::abs(m.m[0][0]) * ptError.x + std::abs(m.m[0][1]) * ptError.y +
				std::abs(m.m[0][2]) * ptError.z) +
			gamma(3) * (std::abs(m.m[0][0] * x) + std::abs(m.m[0][1] * y) +
				std::abs(m.m[0][2] * z) + std::abs(m.m[0][3]));
		absError->y =
			(gamma(3) + (T)1) *
			(std::abs(m.m[1][0]) * ptError.x + std::abs(m.m[1][1]) * ptError.y +
				std::abs(m.m[1][2]) * ptError.z) +
			gamma(3) * (std::abs(m.m[1][0] * x) + std::abs(m.m[1][1] * y) +
				std::abs(m.m[1][2] * z) + std::abs(m.m[1][3]));
		absError->z =
			(gamma(3) + (T)1) *
			(std::abs(m.m[2][0]) * ptError.x + std::abs(m.m[2][1]) * ptError.y +
				std::abs(m.m[2][2]) * ptError.z) +
			gamma(3) * (std::abs(m.m[2][0] * x) + std::abs(m.m[2][1] * y) +
				std::abs(m.m[2][2] * z) + std::abs(m.m[2][3]));
		assert(wp != 0);
		if (wp == 1.)
			return Point3<T>(xp, yp, zp);
		else
			return Point3<T>(xp, yp, zp) / wp;
	}
	template <typename T>
	inline Vector3<T> Transform::operator()(const Vector3<T>& v,
		Vector3<T>* absError) const {
		T x = v.x, y = v.y, z = v.z;
		//Why need to calculate the error of the diff?
		absError->x =
			gamma(3) * (std::abs(m.m[0][0] * v.x) + std::abs(m.m[0][1] * v.y) +
				std::abs(m.m[0][2] * v.z));
		absError->y =
			gamma(3) * (std::abs(m.m[1][0] * v.x) + std::abs(m.m[1][1] * v.y) +
				std::abs(m.m[1][2] * v.z));
		absError->z =
			gamma(3) * (std::abs(m.m[2][0] * v.x) + std::abs(m.m[2][1] * v.y) +
				std::abs(m.m[2][2] * v.z));
		return Vector3<T>(m.m[0][0] * x + m.m[0][1] * y + m.m[0][2] * z,
			m.m[1][0] * x + m.m[1][1] * y + m.m[1][2] * z,
			m.m[2][0] * x + m.m[2][1] * y + m.m[2][2] * z);
	}
	template <typename T>
	inline Vector3<T> Transform::operator()(const Vector3<T>& v,
		const Vector3<T>& vError,
		Vector3<T>* absError) const {
		T x = v.x, y = v.y, z = v.z;
		absError->x =
			(gamma(3) + (T)1) *
			(std::abs(m.m[0][0]) * vError.x + std::abs(m.m[0][1]) * vError.y +
				std::abs(m.m[0][2]) * vError.z) +
			gamma(3) * (std::abs(m.m[0][0] * v.x) + std::abs(m.m[0][1] * v.y) +
				std::abs(m.m[0][2] * v.z));
		absError->y =
			(gamma(3) + (T)1) *
			(std::abs(m.m[1][0]) * vError.x + std::abs(m.m[1][1]) * vError.y +
				std::abs(m.m[1][2]) * vError.z) +
			gamma(3) * (std::abs(m.m[1][0] * v.x) + std::abs(m.m[1][1] * v.y) +
				std::abs(m.m[1][2] * v.z));
		absError->z =
			(gamma(3) + (T)1) *
			(std::abs(m.m[2][0]) * vError.x + std::abs(m.m[2][1]) * vError.y +
				std::abs(m.m[2][2]) * vError.z) +
			gamma(3) * (std::abs(m.m[2][0] * v.x) + std::abs(m.m[2][1] * v.y) +
				std::abs(m.m[2][2] * v.z));
		return Vector3<T>(m.m[0][0] * x + m.m[0][1] * y + m.m[0][2] * z,
			m.m[1][0] * x + m.m[1][1] * y + m.m[1][2] * z,
			m.m[2][0] * x + m.m[2][1] * y + m.m[2][2] * z);
	}
	inline Ray Transform::operator()(const Ray& r, Vector3f* oError,
		Vector3f* dError) const {
		Point3f o = (*this)(r.o, oError);
		Vector3f d = (*this)(r.d, dError);
		float tMax = r.tMax;
		float lengthSquared = d.LengthSquared();
		if (lengthSquared > 0) {
			float dt = Dot(Abs(d), *oError) / lengthSquared;
			o += d * dt;
			//        tMax -= dt;
		}
		return Ray(o, d, tMax, r.time, r.medium);
	}

	inline Ray Transform::operator()(const Ray& r, const Vector3f& oErrorIn,
		const Vector3f& dErrorIn, Vector3f* oErrorOut,
		Vector3f* dErrorOut) const {
		Point3f o = (*this)(r.o, oErrorIn, oErrorOut);
		Vector3f d = (*this)(r.d, dErrorIn, dErrorOut);
		float tMax = r.tMax;
		float lengthSquared = d.LengthSquared();
		if (lengthSquared > 0) {
			float dt = Dot(Abs(d), *oErrorOut) / lengthSquared;
			o += d * dt;
			//        tMax -= dt;
		}
		return Ray(o, d, tMax, r.time, r.medium);
	}

	template<typename T>
	inline Vector3<T>
		Transform::operator()(const Vector3<T>& v)const 
	{
		T x = v.x, y = v.y, z = v.z;
		return Vector3<T>(m.m[0][0] * x + m.m[0][1] * y + m.m[0][2] * z,
			m.m[1][0] * x + m.m[1][1] * y + m.m[1][2] * z,
			m.m[2][0] * x + m.m[2][1] * y + m.m[2][2] * z);
	}

	// AnimatedTransform Declarations
	class AnimatedTransform {
	public:
		// AnimatedTransform Public Methods
		AnimatedTransform(const Transform* startTransform, float startTime,
			const Transform* endTransform, float endTime);
		static void Decompose(const Matrix4x4& m, Vector3f* T, Quaternion* R,
			Matrix4x4* S);
		void Interpolate(float time, Transform* t) const;
		Ray operator()(const Ray& r) const;
		RayDifferential operator()(const RayDifferential& r) const;
		Point3f operator()(float time, const Point3f& p) const;
		Vector3f operator()(float time, const Vector3f& v) const;
		bool HasScale() const {
			return startTransform->HasScale() || endTransform->HasScale();
		}
		Bounds3f MotionBounds(const Bounds3f& b) const;
		Bounds3f BoundPointMotion(const Point3f& p) const;

	private:
		// AnimatedTransform Private Data
		const Transform* startTransform, * endTransform;
		const float startTime, endTime;
		const bool actuallyAnimated;
		Vector3f T[2];
		Quaternion R[2];
		Matrix4x4 S[2];
		bool hasRotation;
		struct DerivativeTerm {
			//DerivativeTerm() {}
			DerivativeTerm(float c, float x, float y, float z)
				: kc(c), kx(x), ky(y), kz(z) {}
			float kc, kx, ky, kz;
			float Eval(const Point3f& p) const {
				return kc + kx * p.x + ky * p.y + kz * p.z;
			}
		};
		DerivativeTerm c1[3], c2[3], c3[3], c4[3], c5[3];
	};
}